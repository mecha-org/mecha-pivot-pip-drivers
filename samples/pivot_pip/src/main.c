/*
 * Copyright (c) 2026 Mecha Systems
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * Push a simulated Pivot Pip to the right and print the motion that the driver reports.
 * A real application reads the same events from the input subsystem, for example through
 * a zmk,input-listener in ZMK.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/init.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define PIVOT_PIP DEVICE_DT_GET(DT_NODELABEL(pivot_pip))
#define ADC       DEVICE_DT_GET(DT_NODELABEL(adc0))

#define REST 2048
#define PUSH 1000

static void on_input(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type == INPUT_EV_REL) {
		printk("pointer moved %s by %d\n", evt->code == INPUT_REL_X ? "x" : "y",
		       evt->value);
	}
}
INPUT_CALLBACK_DEFINE(PIVOT_PIP, on_input, NULL);

/* The driver measures the idle centre as soon as it starts, so the emulated stick starts at rest.
 */
static int stick_at_rest(void)
{
	adc_emul_const_raw_value_set(ADC, 0, REST);
	adc_emul_const_raw_value_set(ADC, 1, REST);

	return 0;
}
SYS_INIT(stick_at_rest, POST_KERNEL, 60);

int main(void)
{
	printk("Pivot Pip sample: stick at rest\n");
	k_msleep(500);

	printk("Pivot Pip sample: pushing the stick to the right\n");
	adc_emul_const_raw_value_set(ADC, 0, REST + PUSH);
	k_msleep(500);

	printk("Pivot Pip sample: releasing the stick\n");
	adc_emul_const_raw_value_set(ADC, 0, REST);
	k_msleep(500);

	printk("Pivot Pip sample: done\n");

	return 0;
}
