/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * The driver polls the ADC from its own thread. These tests count the reads of the emulated
 * ADC to see whether the thread polls when it should, and that it stops when the device is
 * suspended, however quickly suspend and resume follow each other.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#define PIP  DEVICE_DT_GET(DT_NODELABEL(pivot_pip))
#define ADC  DEVICE_DT_GET(DT_NODELABEL(adc0))
#define REST 2048

/* Suspend and resume cycles of the stress tests. */
#define CYCLES 150

static atomic_t reads;

/* The emulated ADC calls this for every sample, which counts how often the driver polls. */
static int count_read(const struct device *dev, unsigned int chan, void *data, uint32_t *result)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	if (chan == 0) {
		atomic_inc(&reads);
	}
	*result = REST;

	return 0;
}

static int count_the_reads(void)
{
	adc_emul_raw_value_func_set(ADC, 0, count_read, NULL);
	adc_emul_raw_value_func_set(ADC, 1, count_read, NULL);

	return 0;
}
SYS_INIT(count_the_reads, POST_KERNEL, 60);

/* Number of ADC reads during the next @p ms milliseconds. */
static long reads_in(int ms)
{
	long start = atomic_get(&reads);

	k_msleep(ms);

	return atomic_get(&reads) - start;
}

/* A pause of 100 us to 9 ms, different for every cycle. */
static void pause(int cycle, int scale)
{
	k_usleep(100 + (cycle * 7919 * scale) % 9000);
}

#ifndef CONFIG_PM_DEVICE_RUNTIME

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* The device may be running already, which is fine. */
	pm_device_action_run(PIP, PM_DEVICE_ACTION_RESUME);
	k_msleep(50);
}

ZTEST_SUITE(pivot_pip_pm, NULL, NULL, before, NULL, NULL);

ZTEST(pivot_pip_pm, test_suspend_stops_polling_and_resume_restarts_it)
{
	zassert_true(reads_in(200) > 10, "not polling at start-up");

	zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_SUSPEND));
	k_msleep(100);
	zassert_equal(reads_in(400), 0, "polled while suspended");

	zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_RESUME));
	zassert_true(reads_in(200) > 10, "did not restart");
}

/*
 * A resume that comes before the thread has noticed the suspend leaves a token in its
 * semaphore. The thread must not take that as a reason to poll while it is suspended.
 */
ZTEST(pivot_pip_pm, test_resume_before_the_thread_noticed_the_suspend)
{
	const int rounds = 6;
	int priority = k_thread_priority_get(k_current_get());
	long extra = 0;

	for (int round = 0; round < rounds; round++) {
		/* Nothing else runs between the suspend and the resume. */
		k_thread_priority_set(k_current_get(), K_PRIO_COOP(1));
		zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_SUSPEND));
		zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_RESUME));
		k_thread_priority_set(k_current_get(), priority);
		k_msleep(50);

		/* Now suspend for real, and count what the thread does on its way to sleep. */
		long start = atomic_get(&reads);

		zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_SUSPEND));
		k_msleep(100);
		extra += atomic_get(&reads) - start;

		zassert_equal(reads_in(300), 0, "polled while suspended");

		zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_RESUME));
		zassert_true(reads_in(200) > 10, "did not restart");
	}

	/* One read can be in flight when the suspend comes. A poll in every round is the bug. */
	zassert_true(extra <= rounds / 3, "polled after the suspend in %ld of %d rounds", extra,
		     rounds);
}

ZTEST(pivot_pip_pm, test_rapid_suspend_and_resume)
{
	for (int i = 0; i < CYCLES; i++) {
		pm_device_action_run(PIP, PM_DEVICE_ACTION_SUSPEND);
		pause(i, 1);
		pm_device_action_run(PIP, PM_DEVICE_ACTION_RESUME);
		pause(i, 3);
	}

	zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_SUSPEND));
	k_msleep(100);
	zassert_equal(reads_in(400), 0, "polled while suspended");

	zassert_ok(pm_device_action_run(PIP, PM_DEVICE_ACTION_RESUME));
	zassert_true(reads_in(200) > 10, "did not restart");
}

#else /* CONFIG_PM_DEVICE_RUNTIME */

static enum pm_device_state state_at_start_up;

static int remember_the_state(void)
{
	pm_device_state_get(PIP, &state_at_start_up);

	return 0;
}
SYS_INIT(remember_the_state, APPLICATION, 0);

ZTEST_SUITE(pivot_pip_pm, NULL, NULL, NULL, NULL, NULL);

/* Every test leaves the device suspended, so that the order of the tests does not matter. */

ZTEST(pivot_pip_pm, test_starts_suspended)
{
	zassert_equal(state_at_start_up, PM_DEVICE_STATE_SUSPENDED);
	zassert_equal(reads_in(300), 0, "polled before it was asked to");
}

ZTEST(pivot_pip_pm, test_get_starts_and_put_stops)
{
	zassert_ok(pm_device_runtime_get(PIP));
	zassert_true(reads_in(200) > 10, "did not start");

	zassert_ok(pm_device_runtime_put(PIP));
	k_msleep(100);
	zassert_equal(reads_in(400), 0, "polled while suspended");
}

ZTEST(pivot_pip_pm, test_rapid_get_and_put)
{
	for (int i = 0; i < CYCLES; i++) {
		pm_device_runtime_get(PIP);
		pause(i, 1);
		pm_device_runtime_put(PIP);
		pause(i, 3);
	}

	k_msleep(100);
	zassert_equal(reads_in(400), 0, "polled while suspended");

	zassert_ok(pm_device_runtime_get(PIP));
	zassert_true(reads_in(200) > 10, "did not restart");
	zassert_ok(pm_device_runtime_put(PIP));
}

#endif /* CONFIG_PM_DEVICE_RUNTIME */
