/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define DT_DRV_COMPAT mecha_pivot_pip

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <mecha/input/pivot_pip.h>

#include "pip_core.h"

LOG_MODULE_REGISTER(pivot_pip, CONFIG_INPUT_LOG_LEVEL);

/* Longest time between two samples that is taken as it is, for example after a suspend. */
#define MAX_ELAPSED_US 100000

struct pivot_pip_config {
	struct adc_dt_spec adc_x;
	struct adc_dt_spec adc_y;
	struct pip_params params;
	uint32_t poll_period_ms;
	uint32_t max_speed;
	uint16_t rel_code_x;
	uint16_t rel_code_y;
	uint16_t abs_code_x;
	uint16_t abs_code_y;
	enum pivot_pip_mode initial_mode;
	bool inverted_x;
	bool inverted_y;
	bool swapped_xy;
};

struct pivot_pip_data {
	struct k_thread thread;

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_INPUT_MECHA_PIVOT_PIP_THREAD_STACK_SIZE);

	struct k_timer timer;
	int64_t last_ticks;
	struct pip_config pip;
	struct pip_axis x;
	struct pip_axis y;
	int32_t remainder_x;
	int32_t remainder_y;
	int32_t last_abs_x;
	int32_t last_abs_y;
	enum pivot_pip_mode last_mode;
	atomic_t mode;
	atomic_t recalibrate;
	bool adc_failing;
#ifdef CONFIG_PM_DEVICE
	struct k_sem wakeup;
	atomic_t suspended;
#endif
};

int pivot_pip_set_mode(const struct device *dev, enum pivot_pip_mode mode)
{
	struct pivot_pip_data *data = dev->data;

	if (mode != PIVOT_PIP_MODE_ABSOLUTE && mode != PIVOT_PIP_MODE_RELATIVE) {
		return -EINVAL;
	}

	atomic_set(&data->mode, mode);

	return 0;
}

enum pivot_pip_mode pivot_pip_get_mode(const struct device *dev)
{
	struct pivot_pip_data *data = dev->data;

	return atomic_get(&data->mode);
}

int pivot_pip_recalibrate(const struct device *dev)
{
	struct pivot_pip_data *data = dev->data;

	/* The polling thread owns the state of the axes, so it does the measuring. */
	atomic_set(&data->recalibrate, 1);

	return 0;
}

static void report_abs(const struct device *dev, int32_t x, int32_t y)
{
	const struct pivot_pip_config *cfg = dev->config;
	struct pivot_pip_data *data = dev->data;

	input_report_abs(dev, cfg->abs_code_x, x, false, K_FOREVER);
	input_report_abs(dev, cfg->abs_code_y, y, true, K_FOREVER);

	data->last_abs_x = x;
	data->last_abs_y = y;
}

static void report_rel(const struct device *dev, int32_t dx, int32_t dy)
{
	const struct pivot_pip_config *cfg = dev->config;

	if (dx != 0) {
		input_report_rel(dev, cfg->rel_code_x, dx, dy == 0, K_FOREVER);
	}
	if (dy != 0) {
		input_report_rel(dev, cfg->rel_code_y, dy, true, K_FOREVER);
	}
}

/* Report the deflection of both axes, as events of the current mode. */
static void report(const struct device *dev, int32_t x, int32_t y, uint32_t elapsed_us)
{
	const struct pivot_pip_config *cfg = dev->config;
	struct pivot_pip_data *data = dev->data;
	enum pivot_pip_mode mode = atomic_get(&data->mode);

	if (mode != data->last_mode) {
		/* Do not leave a deflection behind that is no longer updated. */
		if (data->last_mode == PIVOT_PIP_MODE_ABSOLUTE &&
		    (data->last_abs_x != 0 || data->last_abs_y != 0)) {
			report_abs(dev, 0, 0);
		}

		data->remainder_x = 0;
		data->remainder_y = 0;
		data->last_mode = mode;
	}

	if (mode == PIVOT_PIP_MODE_ABSOLUTE) {
		if (x != data->last_abs_x || y != data->last_abs_y) {
			report_abs(dev, x, y);
		}
		return;
	}

	report_rel(dev, pip_motion(&data->remainder_x, x, cfg->max_speed, elapsed_us),
		   pip_motion(&data->remainder_y, y, cfg->max_speed, elapsed_us));
}

/* Orient the axes: swap them first, then invert. */
static void orient(const struct pivot_pip_config *cfg, int32_t *x, int32_t *y)
{
	if (cfg->swapped_xy) {
		int32_t tmp = *x;

		*x = *y;
		*y = tmp;
	}
	if (cfg->inverted_x) {
		*x = -*x;
	}
	if (cfg->inverted_y) {
		*y = -*y;
	}
}

/* As in the ADC API, the samples of a single-ended channel are unsigned and those of a differential
 * channel are signed.
 */
static int32_t sample_value(const struct adc_dt_spec *spec, uint16_t raw)
{
	return spec->channel_cfg.differential ? (int16_t)raw : raw;
}

static void poll(const struct device *dev)
{
	const struct pivot_pip_config *cfg = dev->config;
	struct pivot_pip_data *data = dev->data;
	uint16_t samples[2];
	struct adc_sequence sequence = {
		.buffer = samples,
		.buffer_size = sizeof(samples),
	};
	/* The ADC stores the samples in order of channel ID. */
	bool x_first = cfg->adc_x.channel_id < cfg->adc_y.channel_id;
	int64_t now;
	uint32_t elapsed_us;
	int32_t raw_x;
	int32_t raw_y;
	int32_t x;
	int32_t y;
	int err;

	adc_sequence_init_dt(&cfg->adc_x, &sequence);
	sequence.channels |= BIT(cfg->adc_y.channel_id);

	err = adc_read_dt(&cfg->adc_x, &sequence);
	if (err < 0) {
		if (!data->adc_failing) {
			LOG_ERR("ADC read failed (%d)", err);
			data->adc_failing = true;
		}
		return;
	}
	data->adc_failing = false;

	/* Speed and timing follow the measured time, not the requested period. */
	now = k_uptime_ticks();
	elapsed_us = MIN(k_ticks_to_us_floor64(now - data->last_ticks), MAX_ELAPSED_US);
	data->last_ticks = now;

	/* Measure the centre again, as at start-up. */
	if (atomic_cas(&data->recalibrate, 1, 0)) {
		pip_axis_init(&data->x);
		pip_axis_init(&data->y);
		data->remainder_x = 0;
		data->remainder_y = 0;
	}

	raw_x = sample_value(&cfg->adc_x, samples[x_first ? 0 : 1]);
	raw_y = sample_value(&cfg->adc_y, samples[x_first ? 1 : 0]);

	x = pip_axis_update(&data->pip, &data->x, raw_x, elapsed_us);
	y = pip_axis_update(&data->pip, &data->y, raw_y, elapsed_us);

	orient(cfg, &x, &y);
	LOG_DBG("%s: raw x=%d y=%d, deflection x=%d y=%d", dev->name, raw_x, raw_y, x, y);
	report(dev, x, y, elapsed_us);
}

static void pivot_pip_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	const struct pivot_pip_config *cfg = dev->config;
	struct pivot_pip_data *data = dev->data;
	int err;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (!adc_is_ready_dt(&cfg->adc_x)) {
		LOG_ERR("ADC controller device not ready");
		return;
	}

	err = adc_channel_setup_dt(&cfg->adc_x);
	if (err < 0) {
		LOG_ERR("Could not setup the X channel (%d)", err);
		return;
	}

	err = adc_channel_setup_dt(&cfg->adc_y);
	if (err < 0) {
		LOG_ERR("Could not setup the Y channel (%d)", err);
		return;
	}

	data->last_ticks = k_uptime_ticks();

	while (true) {
#ifdef CONFIG_PM_DEVICE
		/*
		 * Park while suspended. A resume that comes before the thread has noticed the
		 * suspend leaves a token in the semaphore, so check the flag again after taking it.
		 */
		while (atomic_get(&data->suspended) == 1) {
			k_sem_take(&data->wakeup, K_FOREVER);
			data->last_ticks = k_uptime_ticks();
		}
#endif

		poll(dev);
		k_timer_status_sync(&data->timer);
	}
}

static int pivot_pip_init(const struct device *dev)
{
	const struct pivot_pip_config *cfg = dev->config;
	struct pivot_pip_data *data = dev->data;
	k_tid_t tid;
	int err;

	if (cfg->adc_x.dev != cfg->adc_y.dev || cfg->adc_x.channel_id == cfg->adc_y.channel_id ||
	    cfg->adc_x.resolution != cfg->adc_y.resolution ||
	    cfg->adc_x.oversampling != cfg->adc_y.oversampling) {
		LOG_ERR("X and Y must be different channels of one ADC with the same settings");
		return -EINVAL;
	}

	/* The samples are read into 16-bit values. */
	if (cfg->adc_x.resolution > 16) {
		LOG_ERR("ADC resolutions above 16 bits are not supported");
		return -ENOTSUP;
	}

	err = pip_config_init(&data->pip, &cfg->params);
	if (err < 0) {
		LOG_ERR("Invalid stick parameters");
		return err;
	}

	pip_axis_init(&data->x);
	pip_axis_init(&data->y);
	atomic_set(&data->mode, cfg->initial_mode);
	data->last_mode = cfg->initial_mode;

	k_timer_init(&data->timer, NULL, NULL);

#ifdef CONFIG_PM_DEVICE
	k_sem_init(&data->wakeup, 0, 1);
#endif
#ifdef CONFIG_PM_DEVICE_RUNTIME
	/* The device stays suspended until a consumer asks for it with pm_device_runtime_get(). */
	atomic_set(&data->suspended, 1);
#endif

	tid = k_thread_create(&data->thread, data->thread_stack,
			      K_KERNEL_STACK_SIZEOF(data->thread_stack), pivot_pip_thread,
			      (void *)dev, NULL, NULL, CONFIG_INPUT_MECHA_PIVOT_PIP_THREAD_PRIORITY,
			      0, K_NO_WAIT);
	if (!tid) {
		LOG_ERR("Thread creation failed");
		return -ENODEV;
	}

	k_thread_name_set(&data->thread, dev->name);

#ifndef CONFIG_PM_DEVICE_RUNTIME
	k_timer_start(&data->timer, K_MSEC(cfg->poll_period_ms), K_MSEC(cfg->poll_period_ms));
#else
	pm_device_init_suspended(dev);
	err = pm_device_runtime_enable(dev);
	if (err < 0) {
		LOG_ERR("Failed to enable runtime power management");
		return err;
	}
#endif

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int pivot_pip_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct pivot_pip_config *cfg = dev->config;
	struct pivot_pip_data *data = dev->data;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		atomic_set(&data->suspended, 1);
		k_timer_stop(&data->timer);
		break;
	case PM_DEVICE_ACTION_RESUME:
		k_timer_start(&data->timer, K_MSEC(cfg->poll_period_ms),
			      K_MSEC(cfg->poll_period_ms));
		atomic_set(&data->suspended, 0);
		k_sem_give(&data->wakeup);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif

#define PIVOT_PIP_INIT(inst)                                                                       \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, io_channels) == 2,                                     \
		     "io-channels must list the X and Y channels");                                \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, rel_codes) == 2,                                       \
		     "rel-codes must list the X and Y codes");                                     \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, abs_codes) == 2,                                       \
		     "abs-codes must list the X and Y codes");                                     \
	BUILD_ASSERT(DT_INST_PROP(inst, full_scale) > 0 &&                                         \
			     DT_INST_PROP(inst, full_scale) <= UINT16_MAX,                         \
		     "full-scale must be between 1 and 65535");                                    \
	BUILD_ASSERT(DT_INST_PROP(inst, poll_period_ms) > 0, "poll-period-ms must not be zero");   \
	BUILD_ASSERT(DT_INST_PROP(inst, deadzone_percent) >= 0 &&                                  \
			     DT_INST_PROP(inst, deadzone_percent) < 100,                           \
		     "deadzone-percent must be between 0 and 99");                                 \
	BUILD_ASSERT(DT_INST_PROP(inst, recenter_window_percent) >= 0 &&                           \
			     DT_INST_PROP(inst, recenter_window_percent) <= 100,                   \
		     "recenter-window-percent must be between 0 and 100");                         \
	BUILD_ASSERT(DT_INST_PROP(inst, recenter_limit_percent) >= 0 &&                            \
			     DT_INST_PROP(inst, recenter_limit_percent) <= 100,                    \
		     "recenter-limit-percent must be between 0 and 100");                          \
	BUILD_ASSERT(DT_INST_PROP(inst, max_speed) >= 0 &&                                         \
			     DT_INST_PROP(inst, filter_time_constant_ms) >= 0 &&                   \
			     DT_INST_PROP(inst, recenter_delay_ms) >= 0,                           \
		     "max-speed and the times must not be negative");                              \
                                                                                                   \
	static const struct pivot_pip_config pivot_pip_cfg_##inst = {                              \
		.adc_x = ADC_DT_SPEC_INST_GET_BY_NAME(inst, x),                                    \
		.adc_y = ADC_DT_SPEC_INST_GET_BY_NAME(inst, y),                                    \
		.params =                                                                          \
			{                                                                          \
				.full_scale = DT_INST_PROP(inst, full_scale),                      \
				.filter_time_constant_ms =                                         \
					DT_INST_PROP(inst, filter_time_constant_ms),               \
				.deadzone_percent = DT_INST_PROP(inst, deadzone_percent),          \
				.recenter_window_percent =                                         \
					DT_INST_PROP(inst, recenter_window_percent),               \
				.recenter_limit_percent =                                          \
					DT_INST_PROP(inst, recenter_limit_percent),                \
				.recenter_delay_ms = DT_INST_PROP(inst, recenter_delay_ms),        \
			},                                                                         \
		.poll_period_ms = DT_INST_PROP(inst, poll_period_ms),                              \
		.max_speed = DT_INST_PROP(inst, max_speed),                                        \
		.rel_code_x = DT_INST_PROP_BY_IDX(inst, rel_codes, 0),                             \
		.rel_code_y = DT_INST_PROP_BY_IDX(inst, rel_codes, 1),                             \
		.abs_code_x = DT_INST_PROP_BY_IDX(inst, abs_codes, 0),                             \
		.abs_code_y = DT_INST_PROP_BY_IDX(inst, abs_codes, 1),                             \
		.initial_mode = UTIL_CAT(PIVOT_PIP_MODE_, DT_INST_STRING_UPPER_TOKEN(inst, mode)), \
		.inverted_x = DT_INST_PROP(inst, inverted_x),                                      \
		.inverted_y = DT_INST_PROP(inst, inverted_y),                                      \
		.swapped_xy = DT_INST_PROP(inst, swapped_x_y),                                     \
	};                                                                                         \
                                                                                                   \
	static struct pivot_pip_data pivot_pip_data_##inst;                                        \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, pivot_pip_pm_action);                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, pivot_pip_init, PM_DEVICE_DT_INST_GET(inst),                   \
			      &pivot_pip_data_##inst, &pivot_pip_cfg_##inst, POST_KERNEL,          \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PIVOT_PIP_INIT)
