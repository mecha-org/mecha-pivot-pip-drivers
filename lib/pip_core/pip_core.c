/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include "pip_core.h"

/* Fractional bits kept in the filtered value and the centre. */
#define FRAC_BITS     4
/* Time constant of the centre while it follows an idle stick. */
#define CENTRE_TAU_US 64000
/* Time over which the filter settles and the centre is measured at start-up. */
#define WARMUP_US     100000
/* One count of motion in the remainder. */
#define MOTION_ONE    65536

static int32_t iabs(int32_t v)
{
	return v < 0 ? -v : v;
}

/* Part of a difference covered in the given time by something with the given time constant. */
static int32_t approach(int32_t diff, uint32_t elapsed_us, uint32_t tau_us)
{
	int64_t den = (int64_t)tau_us + elapsed_us;
	int64_t step = ((int64_t)iabs(diff) * elapsed_us + den / 2) / den;

	return diff < 0 ? -(int32_t)step : (int32_t)step;
}

int pip_config_init(struct pip_config *cfg, const struct pip_params *params)
{
	int64_t full;

	if (params->full_scale == 0 || params->full_scale > (INT32_MAX >> FRAC_BITS) ||
	    params->deadzone_percent >= 100 || params->recenter_window_percent > 100 ||
	    params->recenter_limit_percent > 100 || params->recenter_delay_ms > UINT32_MAX / 1000 ||
	    params->filter_time_constant_ms > UINT32_MAX / 1000) {
		return -EINVAL;
	}

	full = (int64_t)params->full_scale * (1 << FRAC_BITS);

	cfg->filter_tau_us = params->filter_time_constant_ms * 1000;
	cfg->full_q = (int32_t)full;
	cfg->deadzone_q = (int32_t)(full * params->deadzone_percent / 100);
	cfg->window_q = (int32_t)(full * params->recenter_window_percent / 100);
	cfg->limit_q = (int32_t)(full * params->recenter_limit_percent / 100);
	cfg->delay_us = params->recenter_delay_ms * 1000;

	return 0;
}

void pip_axis_init(struct pip_axis *axis)
{
	axis->filtered_q = 0;
	axis->centre_q = 0;
	axis->origin_q = 0;
	axis->idle_us = 0;
	axis->warmup_us = WARMUP_US;
	axis->warmup_sum_q = 0;
	axis->warmup_count = 0;
	axis->primed = false;
}

/* Let the centre follow a stick that has been idle for long enough. */
static void follow_centre(const struct pip_config *cfg, struct pip_axis *axis, uint32_t elapsed_us)
{
	int32_t deviation = axis->filtered_q - axis->centre_q;

	if (iabs(deviation) > cfg->window_q) {
		axis->idle_us = 0;
		return;
	}

	if (axis->idle_us < cfg->delay_us) {
		axis->idle_us += elapsed_us;
		if (axis->idle_us > cfg->delay_us) {
			axis->idle_us = cfg->delay_us;
		}
		return;
	}

	axis->centre_q += approach(deviation, elapsed_us, CENTRE_TAU_US);

	if (axis->centre_q > axis->origin_q + cfg->limit_q) {
		axis->centre_q = axis->origin_q + cfg->limit_q;
	} else if (axis->centre_q < axis->origin_q - cfg->limit_q) {
		axis->centre_q = axis->origin_q - cfg->limit_q;
	}
}

int32_t pip_axis_update(const struct pip_config *cfg, struct pip_axis *axis, int32_t raw,
			uint32_t elapsed_us)
{
	int32_t sample_q = raw * (1 << FRAC_BITS);
	int32_t deviation;
	int64_t n;

	if (!axis->primed || cfg->filter_tau_us == 0) {
		axis->filtered_q = sample_q;
		axis->primed = true;
	} else {
		axis->filtered_q +=
			approach(sample_q - axis->filtered_q, elapsed_us, cfg->filter_tau_us);
	}

	/* The first moments settle the filter and measure the centre as the mean of the samples. */
	if (axis->warmup_us > 0) {
		axis->warmup_sum_q += sample_q;
		axis->warmup_count++;
		axis->warmup_us = elapsed_us < axis->warmup_us ? axis->warmup_us - elapsed_us : 0;
		if (axis->warmup_us == 0) {
			axis->centre_q = (int32_t)(axis->warmup_sum_q / axis->warmup_count);
			axis->origin_q = axis->centre_q;
		}
		return 0;
	}

	follow_centre(cfg, axis, elapsed_us);

	deviation = axis->filtered_q - axis->centre_q;
	if (iabs(deviation) <= cfg->deadzone_q) {
		return 0;
	}

	n = (int64_t)(iabs(deviation) - cfg->deadzone_q) * PIP_ONE /
	    (cfg->full_q - cfg->deadzone_q);
	if (n > PIP_ONE) {
		n = PIP_ONE;
	}

	return deviation < 0 ? -(int32_t)n : (int32_t)n;
}

int32_t pip_curve(int32_t n)
{
	int64_t magnitude = (int64_t)iabs(n) * iabs(n) / PIP_ONE;

	return n < 0 ? -(int32_t)magnitude : (int32_t)magnitude;
}

int32_t pip_motion(int32_t *remainder, int32_t n, uint32_t max_speed, uint32_t elapsed_us)
{
	int64_t speed_q;
	int64_t total;
	int32_t counts;

	if (n == 0) {
		*remainder = 0;
		return 0;
	}

	speed_q = (int64_t)pip_curve(n) * max_speed * MOTION_ONE / PIP_ONE;
	total = *remainder + speed_q * elapsed_us / 1000000;
	counts = (int32_t)(total / MOTION_ONE);
	*remainder = (int32_t)(total - (int64_t)counts * MOTION_ONE);

	return counts;
}
