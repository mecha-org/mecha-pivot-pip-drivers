/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include <mecha/input/pivot_pip.h>

#define PIP        DEVICE_DT_GET(DT_NODELABEL(pivot_pip))
#define PIP_CUSTOM DEVICE_DT_GET(DT_NODELABEL(pivot_pip_custom))
#define PIP_WIDE   DEVICE_DT_GET(DT_NODELABEL(pivot_pip_wide))
#define PIP_16BIT  DEVICE_DT_GET(DT_NODELABEL(pivot_pip_16bit))
#define ADC        DEVICE_DT_GET(DT_NODELABEL(adc0))
#define ADC_16BIT  DEVICE_DT_GET(DT_NODELABEL(adc2))

#define REST       2048
#define FULL_SCALE 1000
/* Rest value of the stick on the 16 bit ADC, just below the 32767 that a signed value reaches. */
#define REST_16BIT 32000

/* Events of the default stick, which uses INPUT_REL_X, INPUT_REL_Y, INPUT_ABS_X and INPUT_ABS_Y. */
static atomic_t dx;
static atomic_t dy;
static atomic_t abs_x;
static atomic_t abs_y;
static atomic_t rel_events;
static atomic_t abs_events;
static atomic_t other_codes;

/* Events of the custom stick, which uses the RX and RY codes. */
static atomic_t c_rel_x;
static atomic_t c_rel_y;
static atomic_t c_abs_x;
static atomic_t c_abs_y;
static atomic_t c_events;
static atomic_t c_other_codes;

/* Pointer motion of the stick on the 16 bit ADC. */
static atomic_t dx_16bit;

/* Mode of both sticks when they had just started. */
static enum pivot_pip_mode initial_mode;
static enum pivot_pip_mode initial_mode_custom;

static void input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_REL && (evt->code == INPUT_REL_X || evt->code == INPUT_REL_Y)) {
		atomic_inc(&rel_events);
		atomic_add(evt->code == INPUT_REL_X ? &dx : &dy, evt->value);
	} else if (evt->type == INPUT_EV_ABS &&
		   (evt->code == INPUT_ABS_X || evt->code == INPUT_ABS_Y)) {
		atomic_inc(&abs_events);
		atomic_set(evt->code == INPUT_ABS_X ? &abs_x : &abs_y, evt->value);
	} else {
		atomic_inc(&other_codes);
	}
}
INPUT_CALLBACK_DEFINE(PIP, input_cb, NULL);

static void custom_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	atomic_inc(&c_events);

	if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_RX) {
		atomic_add(&c_rel_x, evt->value);
	} else if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_RY) {
		atomic_add(&c_rel_y, evt->value);
	} else if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_RX) {
		atomic_set(&c_abs_x, evt->value);
	} else if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_RY) {
		atomic_set(&c_abs_y, evt->value);
	} else {
		atomic_inc(&c_other_codes);
	}
}
INPUT_CALLBACK_DEFINE(PIP_CUSTOM, custom_input_cb, NULL);

static void input_cb_16bit(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_REL && evt->code == INPUT_REL_X) {
		atomic_add(&dx_16bit, evt->value);
	}
}
INPUT_CALLBACK_DEFINE(PIP_16BIT, input_cb_16bit, NULL);

static void deflect(int32_t x, int32_t y)
{
	adc_emul_const_raw_value_set(ADC, 0, REST + x);
	adc_emul_const_raw_value_set(ADC, 1, REST + y);
}

/* The driver measures the idle centre as soon as it starts, so the stick must already be at rest.
 */
static int stick_at_rest(void)
{
	deflect(0, 0);
	adc_emul_const_raw_value_set(ADC_16BIT, 0, REST_16BIT);
	adc_emul_const_raw_value_set(ADC_16BIT, 1, REST_16BIT);

	return 0;
}
SYS_INIT(stick_at_rest, POST_KERNEL, 60);

static int remember_initial_modes(void)
{
	initial_mode = pivot_pip_get_mode(PIP);
	initial_mode_custom = pivot_pip_get_mode(PIP_CUSTOM);

	return 0;
}
SYS_INIT(remember_initial_modes, APPLICATION, 0);

static void clear_events(void)
{
	atomic_clear(&dx);
	atomic_clear(&dy);
	atomic_clear(&abs_x);
	atomic_clear(&abs_y);
	atomic_clear(&rel_events);
	atomic_clear(&abs_events);
	atomic_clear(&other_codes);
	atomic_clear(&c_rel_x);
	atomic_clear(&c_rel_y);
	atomic_clear(&c_abs_x);
	atomic_clear(&c_abs_y);
	atomic_clear(&c_events);
	atomic_clear(&c_other_codes);
}

/* Let the stick rest and forget what it reported. */
static void settle(void)
{
	deflect(0, 0);
	k_msleep(200);
	clear_events();
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	zassert_ok(pivot_pip_set_mode(PIP, PIVOT_PIP_MODE_RELATIVE));
	zassert_ok(pivot_pip_set_mode(PIP_CUSTOM, PIVOT_PIP_MODE_ABSOLUTE));
	settle();
}

ZTEST_SUITE(pivot_pip, NULL, NULL, before, NULL, NULL);

ZTEST(pivot_pip, test_idle_reports_nothing)
{
	k_msleep(400);

	zassert_equal(atomic_get(&rel_events), 0);
	zassert_equal(atomic_get(&abs_events), 0);
	zassert_equal(atomic_get(&c_events), 0);
}

ZTEST(pivot_pip, test_relative_motion_follows_deflection)
{
	/* 1000 counts per second at full deflection: about 400 counts in 400 ms. */
	deflect(FULL_SCALE, 0);
	k_msleep(400);
	zassert_between_inclusive(atomic_get(&dx), 360, 405);
	zassert_equal(atomic_get(&dy), 0);

	settle();

	deflect(-FULL_SCALE, 0);
	k_msleep(400);
	zassert_between_inclusive(atomic_get(&dx), -405, -360);
	zassert_equal(atomic_get(&dy), 0);
	zassert_equal(atomic_get(&other_codes), 0);
}

ZTEST(pivot_pip, test_response_curve_slows_small_deflections)
{
	deflect(FULL_SCALE, -FULL_SCALE / 2);
	k_msleep(400);

	zassert_between_inclusive(atomic_get(&dx), 360, 405);
	zassert_between_inclusive(atomic_get(&dy), -100, -70);
}

ZTEST(pivot_pip, test_motion_stops_at_rest)
{
	deflect(FULL_SCALE, FULL_SCALE);
	k_msleep(200);
	zassert_true(atomic_get(&dx) > 0);

	settle();

	k_msleep(300);
	zassert_equal(atomic_get(&rel_events), 0);
}

ZTEST(pivot_pip, test_absolute_mode_reports_stick_position)
{
	zassert_ok(pivot_pip_set_mode(PIP, PIVOT_PIP_MODE_ABSOLUTE));
	zassert_equal(pivot_pip_get_mode(PIP), PIVOT_PIP_MODE_ABSOLUTE);

	deflect(FULL_SCALE, 0);
	k_msleep(200);
	zassert_between_inclusive(atomic_get(&abs_x), 32000, 32767);
	zassert_equal(atomic_get(&abs_y), 0);
	zassert_equal(atomic_get(&rel_events), 0);

	/* Linear: 500 counts, less the 60 counts of deadzone, out of 940. */
	deflect(500, 0);
	k_msleep(200);
	zassert_within(atomic_get(&abs_x), 15336, 300);

	deflect(0, 0);
	k_msleep(200);
	zassert_equal(atomic_get(&abs_x), 0);
	zassert_equal(atomic_get(&other_codes), 0);
}

ZTEST(pivot_pip, test_leaving_absolute_mode_releases_the_axes)
{
	zassert_ok(pivot_pip_set_mode(PIP, PIVOT_PIP_MODE_ABSOLUTE));
	deflect(FULL_SCALE, -FULL_SCALE);
	k_msleep(200);
	zassert_true(atomic_get(&abs_x) > 0);
	zassert_true(atomic_get(&abs_y) < 0);

	zassert_ok(pivot_pip_set_mode(PIP, PIVOT_PIP_MODE_RELATIVE));
	k_msleep(100);

	zassert_equal(atomic_get(&abs_x), 0);
	zassert_equal(atomic_get(&abs_y), 0);
}

ZTEST(pivot_pip, test_invalid_mode_is_rejected)
{
	zassert_equal(pivot_pip_set_mode(PIP, 42), -EINVAL);
	zassert_equal(pivot_pip_get_mode(PIP), PIVOT_PIP_MODE_RELATIVE);
}

ZTEST(pivot_pip, test_mode_at_start_follows_the_devicetree)
{
	zassert_equal(initial_mode, PIVOT_PIP_MODE_RELATIVE, "the default is relative");
	zassert_equal(initial_mode_custom, PIVOT_PIP_MODE_ABSOLUTE);
}

ZTEST(pivot_pip, test_custom_absolute_codes_swap_and_invert)
{
	/* The X channel is swapped to Y, which is not inverted. */
	deflect(FULL_SCALE, 0);
	k_msleep(200);
	zassert_between_inclusive(atomic_get(&c_abs_y), 32000, 32767);
	zassert_equal(atomic_get(&c_abs_x), 0);

	settle();

	/* The Y channel is swapped to X, which is inverted. */
	deflect(0, FULL_SCALE);
	k_msleep(200);
	zassert_between_inclusive(atomic_get(&c_abs_x), -32767, -32000);
	zassert_equal(atomic_get(&c_abs_y), 0);

	/* Only the codes of the devicetree were used. */
	zassert_equal(atomic_get(&c_other_codes), 0);
}

ZTEST(pivot_pip, test_custom_relative_codes)
{
	zassert_ok(pivot_pip_set_mode(PIP_CUSTOM, PIVOT_PIP_MODE_RELATIVE));

	deflect(FULL_SCALE, 0);
	k_msleep(400);

	zassert_between_inclusive(atomic_get(&c_rel_y), 360, 405);
	zassert_equal(atomic_get(&c_rel_x), 0);
	zassert_equal(atomic_get(&c_other_codes), 0);
}

ZTEST(pivot_pip, test_deadzone_percent_is_applied)
{
	/* 8 percent is outside the default deadzone of 6 percent, but inside the 10 percent. */
	zassert_ok(pivot_pip_set_mode(PIP, PIVOT_PIP_MODE_ABSOLUTE));

	deflect(80, 0);
	k_msleep(200);

	zassert_true(atomic_get(&abs_x) > 300, "default deadzone: %ld", (long)atomic_get(&abs_x));
	zassert_equal(atomic_get(&c_abs_y), 0, "10 percent deadzone");
	zassert_equal(atomic_get(&c_events), 0);
}

ZTEST(pivot_pip, test_adc_wider_than_16_bits_is_rejected)
{
	zassert_true(device_is_ready(PIP));
	zassert_false(device_is_ready(PIP_WIDE));
}

/*
 * The centre is measured when the driver starts. A stick that is held then drifts once it is
 * let go. Holding the stick while measuring again is the same situation.
 */
ZTEST(pivot_pip, test_recalibrate_removes_the_drift_of_a_held_stick)
{
	zassert_ok(pivot_pip_set_mode(PIP, PIVOT_PIP_MODE_ABSOLUTE));

	/* The centre is measured while the stick is held... */
	deflect(400, 0);
	zassert_ok(pivot_pip_recalibrate(PIP));
	k_msleep(300);
	zexpect_equal(atomic_get(&abs_x), 0, "the held stick should be the new centre");

	/* ...so letting go looks like a push the other way. */
	deflect(0, 0);
	k_msleep(300);
	zexpect_within(atomic_get(&abs_x), -11851, 400, "no drift: %ld", (long)atomic_get(&abs_x));

	/* Measuring again with the stick at rest removes it. */
	zassert_ok(pivot_pip_recalibrate(PIP));
	k_msleep(300);
	zexpect_equal(atomic_get(&abs_x), 0, "still drifting: %ld", (long)atomic_get(&abs_x));
}

ZTEST(pivot_pip, test_recalibrate_stops_the_motion_of_a_drifting_pointer)
{
	deflect(400, 0);
	zassert_ok(pivot_pip_recalibrate(PIP));
	k_msleep(300);

	/* Letting go of the stick moves the pointer, as nothing is pushing it. */
	deflect(0, 0);
	k_msleep(300);
	clear_events();
	k_msleep(200);
	zexpect_true(atomic_get(&rel_events) > 0, "no drift to remove");

	zassert_ok(pivot_pip_recalibrate(PIP));
	k_msleep(300);
	clear_events();
	k_msleep(300);
	zexpect_equal(atomic_get(&rel_events), 0, "the pointer still drifts");
}

ZTEST(pivot_pip, test_single_ended_samples_above_32767_are_unsigned)
{
	/* A push from 32000 to 33500 goes across 32767, where a signed value turns negative. */
	k_msleep(200);
	atomic_clear(&dx_16bit);
	adc_emul_const_raw_value_set(ADC_16BIT, 0, REST_16BIT + 1500);
	k_msleep(200);

	zassert_true(atomic_get(&dx_16bit) > 0, "dx %ld", (long)atomic_get(&dx_16bit));

	adc_emul_const_raw_value_set(ADC_16BIT, 0, REST_16BIT);
}
