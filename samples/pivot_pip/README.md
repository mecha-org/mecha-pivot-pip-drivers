# Pivot Pip sample

Pushes an emulated Pivot Pip to the right, lets go of it, and prints the pointer motion that
the driver reports. It runs on `native_sim`, so it needs no hardware:

```sh
west build -b native_sim/native/64 -d build/sample mecha-pivot-pip-zephyr/samples/pivot_pip
./build/sample/zephyr/zephyr.exe
```

The output looks like this. The sample does not exit by itself, so stop it with Ctrl+C.

```
Pivot Pip sample: stick at rest
Pivot Pip sample: pushing the stick to the right
pointer moved x by 1
pointer moved x by 5
pointer moved x by 7
...
pointer moved x by 10
Pivot Pip sample: releasing the stick
pointer moved x by 2
pointer moved x by 1
Pivot Pip sample: done
```

`boards/native_sim.overlay` shows the devicetree of a Pivot Pip. On a board, point
`io-channels` at the ADC channels that the stick is wired to, as in the
[binding](../../dts/bindings/input/mecha,pivot-pip.yaml).
