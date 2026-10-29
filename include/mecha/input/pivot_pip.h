/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file
 * @brief Mecha Pivot Pip input driver
 */

#ifndef MECHA_INPUT_PIVOT_PIP_H_
#define MECHA_INPUT_PIVOT_PIP_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Kind of input events reported by a Pivot Pip. */
enum pivot_pip_mode {
	/** Pointer motion, as relative events. */
	PIVOT_PIP_MODE_RELATIVE,
	/** Stick position, as absolute axis events. */
	PIVOT_PIP_MODE_ABSOLUTE,
};

/**
 * @brief Select the kind of events reported by a Pivot Pip.
 *
 * Leaving the absolute mode releases the axes, so a consumer does not keep a deflection that
 * is no longer updated.
 *
 * @param dev Pivot Pip device.
 * @param mode New mode.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p mode is not a valid mode.
 */
int pivot_pip_set_mode(const struct device *dev, enum pivot_pip_mode mode);

/**
 * @brief Get the kind of events reported by a Pivot Pip.
 *
 * @param dev Pivot Pip device.
 *
 * @return The current mode.
 */
enum pivot_pip_mode pivot_pip_get_mode(const struct device *dev);

/**
 * @brief Measure the idle centre of a Pivot Pip again.
 *
 * The centre is measured when the driver starts. A stick that is pressed at that moment is
 * taken for the centre, so it reports a push the other way, for as long as it is at rest, once
 * it is let go. Let go of the stick and call this function to remove that.
 *
 * The measurement runs in the polling thread and takes about 100 ms. The stick must not be
 * touched meanwhile, and no events are reported. In absolute mode the axes are released.
 *
 * @param dev Pivot Pip device.
 *
 * @retval 0 on success.
 */
int pivot_pip_recalibrate(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* MECHA_INPUT_PIVOT_PIP_H_ */
