/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file
 * @brief Signal processing of the Mecha Pivot Pip
 *
 * Plain C with no operating system dependencies, so it can be unit tested on its own and
 * reused outside Zephyr. The functions never read a clock: the caller passes the time
 * between two samples, so filtering and timing do not depend on how fast samples are taken.
 *
 * Every axis goes through a low pass filter that can be turned off, a measured and slowly
 * followed idle centre, a deadzone, and a linear scaling to @ref PIP_ONE. Relative motion adds a
 * response curve and keeps the fraction of a count left over from earlier samples.
 */

#ifndef MECHA_PIP_CORE_H_
#define MECHA_PIP_CORE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Normalized deflection at full stick travel. */
#define PIP_ONE 32767

/** Parameters of one stick, in physical units. */
struct pip_params {
	/** Deflection from the centre, in raw counts, at full travel. */
	uint32_t full_scale;
	/** Time constant of the low pass filter of the samples, in milliseconds. 0 turns it off. */
	uint32_t filter_time_constant_ms;
	/** Deadzone around the centre, in percent of @a full_scale. */
	uint8_t deadzone_percent;
	/** The stick counts as idle within this distance of the centre, in percent. */
	uint8_t recenter_window_percent;
	/** The centre never moves further than this from its start-up value, in percent. */
	uint8_t recenter_limit_percent;
	/** Time the stick must stay idle before the centre follows it. */
	uint32_t recenter_delay_ms;
};

/** Parameters converted to the units used while processing. */
struct pip_config {
	uint32_t filter_tau_us;
	int32_t full_q;
	int32_t deadzone_q;
	int32_t window_q;
	int32_t limit_q;
	uint32_t delay_us;
};

/** State of one axis. */
struct pip_axis {
	int32_t filtered_q;
	int32_t centre_q;
	int32_t origin_q;
	uint32_t idle_us;
	uint32_t warmup_us;
	/* Sum and number of the samples that measure the centre at start-up. */
	int64_t warmup_sum_q;
	uint32_t warmup_count;
	bool primed;
};

/**
 * @brief Convert stick parameters.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the parameters are inconsistent.
 */
int pip_config_init(struct pip_config *cfg, const struct pip_params *params);

/** @brief Reset the state of an axis, so that its centre is measured again. */
void pip_axis_init(struct pip_axis *axis);

/**
 * @brief Process one raw sample of an axis.
 *
 * @param cfg Converted parameters.
 * @param axis State of the axis.
 * @param raw Raw sample.
 * @param elapsed_us Time since the previous sample.
 *
 * @return Deflection from -PIP_ONE to PIP_ONE, with the deadzone removed. Zero while the
 *         centre is being measured.
 */
int32_t pip_axis_update(const struct pip_config *cfg, struct pip_axis *axis, int32_t raw,
			uint32_t elapsed_us);

/** @brief Response curve of relative motion, for a deflection from -PIP_ONE to PIP_ONE. */
int32_t pip_curve(int32_t n);

/**
 * @brief Motion produced since the previous sample.
 *
 * @param remainder Fraction of a count left over from earlier samples, updated here.
 * @param n Deflection from -PIP_ONE to PIP_ONE.
 * @param max_speed Speed at full deflection, in counts per second.
 * @param elapsed_us Time since the previous sample.
 *
 * @return Whole counts of motion.
 */
int32_t pip_motion(int32_t *remainder, int32_t n, uint32_t max_speed, uint32_t elapsed_us);

#ifdef __cplusplus
}
#endif

#endif /* MECHA_PIP_CORE_H_ */
