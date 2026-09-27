# HARDWARE

## graph

```text
Pi4B
├─ CSI -> Arducam UC-788 Rev.B / OV9281 mono global-shutter
│  ├─ controller=Pivariety@0x0c fw=0x10002; sensor=mono global-shutter
│  ├─ boot:=camera_auto_detect=0 + dtoverlay=arducam-pivariety
│  └─ modes=1280x720|1600x1080|1600x1300@30; stream=1280x720@30 MJPEG
├─ 40-pin GPIO -> Freenove connector PCB V3.0 ✓
│  ├─ motor driver ? -> 4×DC motor
│  ├─ PWM/servo controller ? -> pan/tilt
│  ├─ ADC ? -> light/battery sensing
│  ├─ ultrasonic ?
│  ├─ line tracking ?
│  └─ LEDs ?
├─ USB -> Arduino Nano 33 BLE Sense Rev2@/dev/ttyACM0
│  └─ BMI270 IMU -> accel[g]+gyro[°/s]@~100Hz serial JSON
└─ Wi-Fi -> mini-router
```

## known-vendor baseline; not-v3-proof

```text
FNK0043 public docs: PCA9685@0x40 + ADC@0x48 via i2c-1
observed:=0x40+0x48+0x70 ✓; 0x70=PCA9685 all-call
imu := Nano33BLESenseRev2/USB; firmware=firmware/nano_imu; reconnect=by-id glob
docs rule: PCB v1=>SPI off; PCB v2=>SPI on
PCB here=v3.0 => do-not-transfer v1/v2 SPI rule without evidence
```

## camera

`stream := 1280x720@30 mono GS; shutter=8000us; gain=1.5; denoise=off; transform=none`

```text
detect := rpicam-hello --list-cameras
capture := rpicam-still --nopreview --timeout 1000 -o <file>
boot backup := /boot/firmware/config.txt.pre-uc788
userspace := Arducam libcamera+rpicam trixie packages; capture✓ 2026-09-27
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
# Chassis geometry / traction observation (2026-09-27)

`wheelbase=210mm; track=149mm; wheel/contact centers; user-confirmed`

`CCW pivot: rear-left + front-right slip; all wheels touch at rest; cause unverified`

`gyro measures chassis yaw, not per-wheel slip; ordinary wheels require lateral scrub in pivot turns`
