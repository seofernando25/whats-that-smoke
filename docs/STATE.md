# STATE

`snapshot:=2026-08-29; mode:=observe-only`

## vision update · 2026-09-27

```text
deployed := optimized aruco.py + camera/vision telemetry + UI; service restarted; ArUco ON/FOLLOW OFF restored
camera current := UC-788/Arducam mono; 1280x720/30fps; capture fresh during validation
observed := native ID2 decoding now succeeds; fallback retained; 28.3 processed fps median over15s; 28.9ms median compute
rollback := Pi /home/advanced/wts-vision-backup-bqgZIM/{aruco.py,web.py,app.js,index.html}
detail := docs/ARUCO.md#performance
shallow-view followup := corner refinement before retry decode + narrow cell-center sampling + bounded thresholds; live16s/40polls: ID2 visible40/40; fresh resampled33 + flow7; processed22.7fps median; compute33.8ms median/p95=59ms; camera30fps
shallow-view rollback := Pi /home/advanced/wts-angle-backup-VqDnUW/aruco.py
```

## ✓

```text
host.model = Raspberry Pi 4 Model B Rev 1.5
host.arch = aarch64
host.os = Debian GNU/Linux 13/trixie
host.kernel = 6.18.34+rpt-rpi-v8
host.ram ~= 2GiB
host.disk.root ~= 14GiB; used~=49%
host.temp = 37..41C
host.throttle = 0x0
power.mobile = Freenove 2-cell pack -> S1 -> Pi GPIO/header; verified boot + throttle=0x0
host.failed_system_units = 0
ssh.wifi = advanced@192.168.8.170:22 ✓
ssh.ethernet = advanced@192.168.8.169:22 ✓ when cabled
wifi.profile = FillMeUpLink; autoconnect=yes; gateway=192.168.8.1
uv = /usr/local/bin/uv@0.12.7 ✓
camera.sensor = ov5647@0x36 ✓
camera.capture = 2592x1944/JPEG ✓
camera.mount_rotation = hflip+vflip = 180deg
pcb.label = V3.0
```

## ×|?

```text
i2c.header = /dev/i2c-1 ✓
i2c.car = {0x40:PCA9685,0x48:ADC,0x70:PCA9685-all-call} ✓
i2c.aux = {0,10,20,21,22}; camera/display domain
spi = ×; /dev/spidev* absent
car.controllers = detected/read-address-only ✓
motors+servos = dashboard-controlled; polarity corrected empirically
ultrasonic/line = untested
battery.pack ~= 8.85V observed; two-cell operation confirmed
led.spi-gpio10 = ×; 8×RGB sequence sent; no visible output
led.pwm-gpio18 = ×; 8×RGB sequence sent; no visible output
led.status = unsupported/unresolved on PCB V3; dashboard scope∅
Freenove software = absent at initial audit
PCB-v3 protocol/pin map = ?; public FNK0043 docs describe v1/v2 only
```

## next

```text
1 identify PCB-v3 authoritative schematic/protocol
2 establish STOP primitive + wheel-off-ground test fixture
3 test sensors -> servo -> motors, one subsystem/step
4 each effect -> verify + append snapshot
```

## 2026-09-27 Arduino companion import

```text
arduino = Nano 33 BLE Sense Rev2; firmware + receiver + dashboard @ ../arduino/
accel+gyro = desktop BLE streaming observed (~98Hz normal trial)
magnetometer = unavailable on this board; no absolute heading
BLE = 256-sample retention + cumulative ACK/retry; desktop fault-injection reports committed
Pi BLE/runtime = unverified; separate uv environment, Python>=3.14
car integration = standalone observation only; no controller or actuation changes
position = experimental XY dead reckoning; not navigation truth
```

## 2026-09-27 car-dashboard USB integration

```text
nano := Nano33BLESenseRev2@USB/by-id; sensor=BMI270
firmware := firmware/nano_imu; transport=115200/newline-json; observed≈100Hz
dashboard := accel[g]+gyro[°/s]+rate+sample-age in /ws + top-right overlay
reconnect := USB unplug/replug auto-retry; actuator coupling=∅
BLE companion := arduino/ remains independent experiment; main dashboard uses USB
```

## 2026-09-27 relative-heading HUD

```text
compass := browser HUD; relative gyro-yaw integrated on Pi; zero=still pose; clockwise-positive
calibration := continuous 3s still; rest debounce=500ms; gyro bias adapts only after rest
gaps>150ms := preserve heading; do-not-integrate unknown interval
limitations := no north reference; no translation/ground-track from IMU alone; skid needs external visual/encoder reference
```

## 2026-09-27 gyro manual drive

```text
geometry := user-confirmed wheelbase=210mm; track=149mm; center-to-center
observation := all wheels contact at rest; CCW => rear-left + front-right consistently slip
cause := unverified; tire scrub/load transfer plausible; gyro cannot identify individual wheel traction
Q/E := closed-loop yaw rate; A/D := heading-anchored side-step; W/S := ramped duty
limits := no position feedback; no cliff protection; no traction guarantee; gains require floor trial
verification := simulated plant + mocked hardware only; no physical maneuver during deployment
regression := user reports pivot failure + sidestep-turn-timeout; hidden1500cap + low near-target duty plausible, not proven
revision := cap restored to slider≤1800; stationary-yaw breakaway90%; W/S+A/D steering arcs
verification-new := synthetic80%-deadzone plant + keyboard mappings; real traction/gains still require floor test
```

## 2026-09-27 short floor arc tuning

```text
permission := user confirmed floor + clear area; three bounded forward arc trials, stop/disarm between
baseline35% := left .7s@1800; yaw∆=-1.59°; peak=7.11°/s; median=-.78°/s
candidate65% := left .7s@1800; yaw∆=-4.38°; peak=18.58°/s; median=-5.17°/s
deployed := ARC_CORRECTION=.65; same-direction wheels; existing gyro/input watchdogs retained
right65% := aborted by trial-only |accel.z|<.8g check; finally STOP; verified disarmed+wheels0
interpretation := stronger left arc measured; right/reverse/repeatability unverified; rocking vs vibration unknown
next := user inspect/report right-trial motion before further automated runs; no retry after safety abort
tool := tools/tune_arc.py; max .8s; no autonomous retries; fresh camera+IMU required
camera := stale before testing; service restart restored≈30fps; no camera recovery code change
```
