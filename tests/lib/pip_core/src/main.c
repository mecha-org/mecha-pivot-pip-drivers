/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include <zephyr/ztest.h>

#include "pip_core.h"

#define FULL_SCALE 1000
#define DEADZONE   60
#define PERIOD_US  8000

static const struct pip_params params = {
	.full_scale = FULL_SCALE,
	.filter_time_constant_ms = 12,
	.deadzone_percent = 6,
	.recenter_window_percent = 3,
	.recenter_limit_percent = 15,
	.recenter_delay_ms = 500,
};

static struct pip_config cfg;
static struct pip_axis axis;

static int32_t feed_every(int32_t raw, int samples, uint32_t elapsed_us)
{
	int32_t out = 0;

	for (int i = 0; i < samples; i++) {
		out = pip_axis_update(&cfg, &axis, raw, elapsed_us);
	}

	return out;
}

static int32_t feed(int32_t raw, int samples)
{
	return feed_every(raw, samples, PERIOD_US);
}

/* Expected deflection of a given raw offset from the centre, once the filter has settled. */
static int32_t expected(int32_t counts)
{
	return (int32_t)((int64_t)(counts - DEADZONE) * PIP_ONE / (FULL_SCALE - DEADZONE));
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_ok(pip_config_init(&cfg, &params));
	pip_axis_init(&axis);
	/* Measure the centre at 0. */
	feed(0, 40);
}

ZTEST_SUITE(pip_core, NULL, NULL, before, NULL, NULL);

ZTEST(pip_core, test_config)
{
	struct pip_params bad;

	zassert_equal(cfg.filter_tau_us, 12000);
	zassert_equal(cfg.full_q, 16000);
	zassert_equal(cfg.deadzone_q, 960);
	zassert_equal(cfg.window_q, 480);
	zassert_equal(cfg.limit_q, 2400);
	zassert_equal(cfg.delay_us, 500000);

	bad = params;
	bad.full_scale = 0;
	zassert_equal(pip_config_init(&cfg, &bad), -EINVAL);

	bad = params;
	bad.deadzone_percent = 100;
	zassert_equal(pip_config_init(&cfg, &bad), -EINVAL);

	bad = params;
	bad.recenter_window_percent = 101;
	zassert_equal(pip_config_init(&cfg, &bad), -EINVAL);

	bad = params;
	bad.recenter_limit_percent = 101;
	zassert_equal(pip_config_init(&cfg, &bad), -EINVAL);

	/* Times are converted to microseconds, which must not overflow. */
	bad = params;
	bad.filter_time_constant_ms = UINT32_MAX / 1000 + 1;
	zassert_equal(pip_config_init(&cfg, &bad), -EINVAL);

	bad = params;
	bad.recenter_delay_ms = UINT32_MAX / 1000 + 1;
	zassert_equal(pip_config_init(&cfg, &bad), -EINVAL);

	bad = params;
	bad.recenter_delay_ms = UINT32_MAX / 1000;
	zassert_ok(pip_config_init(&cfg, &bad));
}

ZTEST(pip_core, test_filter_smooths_a_step)
{
	int32_t first = pip_axis_update(&cfg, &axis, FULL_SCALE, PERIOD_US);

	/* The filter has only covered part of the way after the first sample. */
	zassert_true(first > 0 && first < PIP_ONE, "first %d", first);
	zassert_within(feed(FULL_SCALE, 80), PIP_ONE, 20);
}

ZTEST(pip_core, test_longer_time_constant_is_slower)
{
	struct pip_params slow = params;
	int32_t fast_out = feed(FULL_SCALE, 3);
	int32_t slow_out;

	slow.filter_time_constant_ms = 100;
	zassert_ok(pip_config_init(&cfg, &slow));
	pip_axis_init(&axis);
	feed(0, 40);
	slow_out = feed(FULL_SCALE, 3);

	zassert_true(slow_out < fast_out, "slow %d, fast %d", slow_out, fast_out);
}

ZTEST(pip_core, test_filter_off_follows_the_sample_at_once)
{
	struct pip_params off = params;

	off.filter_time_constant_ms = 0;
	zassert_ok(pip_config_init(&cfg, &off));
	zassert_equal(cfg.filter_tau_us, 0);
	pip_axis_init(&axis);
	feed(0, 40);

	zassert_equal(pip_axis_update(&cfg, &axis, FULL_SCALE, PERIOD_US), PIP_ONE);
	zassert_equal(pip_axis_update(&cfg, &axis, 0, PERIOD_US), 0);
	/* A time of zero between two samples is handled. */
	zassert_equal(pip_axis_update(&cfg, &axis, FULL_SCALE, 0), PIP_ONE);
}

ZTEST(pip_core, test_centre_is_the_mean_of_the_samples_at_start_up)
{
	const uint32_t time_constants[] = {12, 0};

	for (int t = 0; t < ARRAY_SIZE(time_constants); t++) {
		struct pip_params p = params;

		p.filter_time_constant_ms = time_constants[t];
		zassert_ok(pip_config_init(&cfg, &p));
		pip_axis_init(&axis);

		/* Ten samples in 100 ms that alternate around 102, so that their mean is 102. */
		for (int i = 0; i < 10; i++) {
			zassert_equal(pip_axis_update(&cfg, &axis, i % 2 ? 104 : 100, 10000), 0);
		}

		zassert_equal(axis.centre_q, 102 * 16, "time constant %u ms", time_constants[t]);
	}
}

ZTEST(pip_core, test_no_output_while_measuring_centre)
{
	pip_axis_init(&axis);

	for (int i = 0; i < 16; i++) {
		zassert_equal(pip_axis_update(&cfg, &axis, 700, PERIOD_US), 0, "sample %d", i);
	}

	/* The centre is the measured rest value, not zero. */
	zassert_equal(feed(700, 50), 0);
	zassert_true(feed(700 + 500, 50) > 0);
}

ZTEST(pip_core, test_deadzone)
{
	zassert_equal(feed(DEADZONE - 5, 60), 0);
	zassert_equal(feed(-(DEADZONE - 5), 60), 0);
	zassert_true(feed(DEADZONE + 40, 60) > 0);
}

ZTEST(pip_core, test_scaling_is_linear_to_full_travel)
{
	zassert_within(feed(530, 80), expected(530), 20);
	zassert_within(feed(800, 80), expected(800), 20);
	zassert_within(feed(FULL_SCALE, 80), PIP_ONE, 20);
}

ZTEST(pip_core, test_clamped_beyond_full_travel)
{
	zassert_equal(feed(3 * FULL_SCALE, 80), PIP_ONE);
	zassert_equal(feed(-3 * FULL_SCALE, 80), -PIP_ONE);
}

ZTEST(pip_core, test_negative_mirrors_positive)
{
	int32_t positive = feed(530, 80);
	int32_t negative;

	feed(0, 80);
	negative = feed(-530, 80);

	zassert_within(positive, -negative, 20);
}

ZTEST(pip_core, test_slow_drift_is_followed)
{
	/* One count every 20 samples: far slower than the centre follows. */
	for (int raw = 1; raw <= 100; raw++) {
		for (int i = 0; i < 20; i++) {
			zassert_equal(pip_axis_update(&cfg, &axis, raw, PERIOD_US), 0, "raw %d",
				      raw);
		}
	}
}

ZTEST(pip_core, test_held_deflection_is_kept)
{
	/* Measured before the idle time has passed, so the centre cannot have moved yet. */
	int32_t first = feed(300, 40);
	int32_t later = feed(300, 20000);

	zassert_true(first > 0);
	zassert_within(later, first, 20, "the centre must not follow a held deflection");
}

ZTEST(pip_core, test_drift_beyond_limit_is_not_absorbed)
{
	int32_t out = 0;

	for (int raw = 1; raw <= 300; raw++) {
		for (int i = 0; i < 20; i++) {
			out = pip_axis_update(&cfg, &axis, raw, PERIOD_US);
		}
		if (raw == 100) {
			zassert_equal(out, 0, "inside the drift limit");
		}
	}

	/* The centre stops 150 counts from where it started, so 150 counts remain. */
	zassert_true(out > 0);
	zassert_within(out, expected(150), 40);
}

ZTEST(pip_core, test_curve)
{
	int32_t previous = 0;

	zassert_equal(pip_curve(0), 0);
	zassert_equal(pip_curve(PIP_ONE), PIP_ONE);
	zassert_equal(pip_curve(-PIP_ONE), -PIP_ONE);
	zassert_within(pip_curve(PIP_ONE / 2), PIP_ONE / 4, 2);

	for (int32_t n = 1000; n <= PIP_ONE; n += 1000) {
		zassert_true(pip_curve(n) > previous, "not increasing at %d", n);
		zassert_equal(pip_curve(-n), -pip_curve(n));
		previous = pip_curve(n);
	}
}

ZTEST(pip_core, test_motion_at_full_deflection)
{
	int32_t remainder = 0;

	/* 1000 counts per second over 8 ms is exactly 8 counts. */
	for (int i = 0; i < 10; i++) {
		zassert_equal(pip_motion(&remainder, PIP_ONE, 1000, PERIOD_US), 8);
		zassert_equal(pip_motion(&remainder, -PIP_ONE, 1000, PERIOD_US), -8);
	}
}

ZTEST(pip_core, test_slow_motion_keeps_its_fractions)
{
	int32_t remainder = 0;
	int32_t total = 0;

	/* About 100 counts per second, which is 0.8 of a count per sample. */
	for (int i = 0; i < 125; i++) {
		total += pip_motion(&remainder, 10361, 1000, PERIOD_US);
	}

	zassert_between_inclusive(total, 99, 100);
}

ZTEST(pip_core, test_motion_forgets_the_fraction_when_idle)
{
	int32_t remainder = 0;

	zassert_equal(pip_motion(&remainder, 3000, 1000, PERIOD_US), 0);
	zassert_not_equal(remainder, 0);

	zassert_equal(pip_motion(&remainder, 0, 1000, PERIOD_US), 0);
	zassert_equal(remainder, 0);
}

ZTEST(pip_core, test_motion_follows_elapsed_time)
{
	int32_t remainder = 0;
	int32_t total = 0;

	/* 1000 counts per second: a late sample covers more distance. */
	zassert_equal(pip_motion(&remainder, PIP_ONE, 1000, 4000), 4);
	zassert_equal(pip_motion(&remainder, PIP_ONE, 1000, 16000), 16);

	for (int i = 0; i < 4; i++) {
		total += pip_motion(&remainder, PIP_ONE, 1000, 4000);
	}
	zassert_equal(total, 16, "four short samples equal one long one");
}

ZTEST(pip_core, test_idle_time_is_time_not_samples)
{
	int32_t soon;
	int32_t late;

	/* Many samples, but only 200 ms: the centre has not started to follow. */
	feed_every(25, 100, 2000);
	soon = feed_every(95, 40, 2000);
	zassert_within(soon, expected(95), 60);

	/* Few samples, but 2 s: the centre has moved to the new rest value. */
	before(NULL);
	feed_every(25, 5, 400000);
	late = feed_every(95, 40, 2000);
	zassert_within(late, expected(70), 100);
	zassert_true(late < soon);
}
