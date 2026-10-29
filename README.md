# Mecha Pivot Pip - Zephyr Module

Zephyr module for the **Mecha Pivot Pip**, an analog pointing stick used by the Mecha Comet
keyboard and gamepad extensions. The driver reads the stick through two ADC channels and reports
both relative and absolute input events through the Zephyr input subsystem. It works in Zephyr
applications and in ZMK.

It needs Zephyr 4.1 or newer and an ADC that reads two channels in one request, with a
resolution of 16 bits at most. Full documentation is coming. Until then, the devicetree
[binding](dts/bindings/input/mecha,pivot-pip.yaml) describes every property, and
[`samples/pivot_pip`](samples/pivot_pip) shows the driver at work on `native_sim`.

Licensed under the [BSD 3-Clause license](LICENSE).
