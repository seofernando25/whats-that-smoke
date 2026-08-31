# COMMANDS

`cwd:=/home/advanced/whats-that-smoke`

```text
snapshot := uv run wts snapshot
snapshot@path := uv run wts snapshot --output <file.jpg>
servo.pan := uv run wts servo-test pan
servo.tilt := uv run wts servo-test tilt
wheel.each := uv run wts wheel-test
wheel.one := uv run wts wheel-test <front-left|rear-left|front-right|rear-right>
led.each.rgb := uv run wts led-test
led.each.rgb.gpio18 := sudo .venv/bin/wts led-test-gpio18
```

## semantics

```text
snapshot: OV5647; wait=500ms; hflip+vflip=180deg; mkdir(parent); print(abs-path)
servo-test(axis): require i2c-1 + PCA9685@0x40
  axis=pan  -> PCA channel 8 only
  axis=tilt -> PCA channel 9 only
  pulse(us)=1500 -> 1400 -> 1600 -> 1500; dwell=150ms/step
  finally => selected channel full-off/released
wheel-test: PCA9685 ch0..7; default duty=900/4095; dwell=200ms
  order=front-left>rear-left>front-right>rear-right
  between/finally => all motor channels brake=4095
led-test: 8×WS2812/GRB via SPI0-MOSI/GPIO10; brightness=32/255
  order=LED1..8 × red>green>blue; off between/finally
led-test-gpio18: same sequence via PWM/DMA GPIO18; requires root
```

`servo gate:=car power✓ + clear linkage✓ + one command at time`
`wheel gate:=chassis-lifted✓ + clear-zone✓ + human-present✓`
