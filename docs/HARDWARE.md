# HARDWARE

## graph

```text
Pi4B
├─ CSI -> OV5647 camera ✓ [physical image inverted; transform=rot180]
├─ 40-pin GPIO -> Freenove connector PCB V3.0 ✓
│  ├─ motor driver ? -> 4×DC motor
│  ├─ PWM/servo controller ? -> pan/tilt
│  ├─ ADC ? -> light/battery sensing
│  ├─ ultrasonic ?
│  ├─ line tracking ?
│  └─ LEDs ?
└─ Wi-Fi -> mini-router
```

## known-vendor baseline; not-v3-proof

```text
FNK0043 public docs: PCA9685@0x40 + ADC@0x48 via i2c-1
observed:=0x40+0x48+0x70 ✓; 0x70=PCA9685 all-call
docs rule: PCB v1=>SPI off; PCB v2=>SPI on
PCB here=v3.0 => do-not-transfer v1/v2 SPI rule without evidence
```

## camera

```text
detect := rpicam-hello --list-cameras
capture := rpicam-still --nopreview --timeout 1000 -o <file>
orientation := downstream rotate=180; physical reseat not required
```

## electrical invariants

```text
Pi logic=3.3V; GPIO overvoltage!; motor rail≠GPIO rail
power-off before ribbon/GPIO topology change
motors require chassis lifted/clear before first actuation
never infer board compatibility from connector fit
```

## installed power topology

```text
source := 2×3.7V cells in Freenove holder; observed pack≈8.85V charged
S1 := car-board master / Pi supply via connection header
S2 := motor+servo load switch
mobile operation := battery+S1; USB-C physically inaccessible after assembly
bench docs permit USB-C while switches on, but this is not the mobile power path
Pi PWR LED red + ACT green are not voltage proof; truth:=vcgencmd get_throttled
2026-08-29 after second cell installed: throttled=0x0; no current/historical undervoltage
```
