# STATE

`snapshot:=2026-08-29; mode:=observe-only`

## vision update · 2026-09-27

```text
deployed := optimized aruco.py + camera/vision telemetry + UI; service restarted; ArUco ON/FOLLOW OFF restored
camera current := UC-788/Arducam mono; 1280x720/30fps; capture fresh during validation
observed := native ID2 decoding now succeeds; fallback retained; 28.3 processed fps median over15s; 28.9ms median compute
rollback := Pi /home/advanced/ugv-vision-backup-bqgZIM/{aruco.py,web.py,app.js,index.html}
detail := docs/ARUCO.md#performance
shallow-view followup := corner refinement before retry decode + narrow cell-center sampling + bounded thresholds; live16s/40polls: ID2 visible40/40; fresh resampled33 + flow7; processed22.7fps median; compute33.8ms median/p95=59ms; camera30fps
shallow-view rollback := Pi /home/advanced/ugv-angle-backup-VqDnUW/aruco.py
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

## 2026-10-13 dashboard service verification

- Existing systemd dashboard unit matches the repository service; active and enabled at boot.
- Restarted service to recover a stale camera feed; observed camera at 29.8 fps with 7 ms frame age after restart.
- LAN UI HTTP 200 at http://192.168.8.170:8765; WebSocket state handshake verified; USB IMU streaming at 100 Hz.
- Robot remained disarmed, follow disabled, and wheel commands zero; no motion tests performed.

## 2026-10-13 simple manual controls

- Manual drive now uses direct differential wheel commands; gyro control, ramps, breakaway and side-step disabled for manual driving.
- W/S forward/reverse; A/D and Q/E direct left/right turning. Input watchdog and STOP brake without disarming the connected owner; explicit DISARM, disconnect and shutdown still release ownership.
- Seven mocked controller regression checks passed, including no-gyro turning, watchdog braking/resume, stop vs disarm, disconnect and disabled side-step. No physical movement tests.
- Service restarted; UI/static assets/WebSocket verified and camera fresh after deployment.
- Rollback files: /home/advanced/ugv-simple-controls-backup-JwLzjy/{web.py,app.js,DASHBOARD.md,SAFETY.md,test_drive.py}.

## 2026-10-13 curled ArUco detection completed

- Added bounded correction of curled paper borders and exact, unambiguous cell-pixel decoding. Original live ID0: baseline 0/50 visible polls; correction 27/27 unobstructed captured frames and 268/268 stationary live target polls over 30 seconds.
- Ten detector regression functions passed, covering the cropped real tag, brightness shifts, 200 bent ID/rotation cases, 569 invalid codes, blank and image-edge rejection, tracker expiry, and stale-frame braking.
- Added camera capture timeout/recovery with 5s startup grace, 2s frame timeout, graceful termination and bounded kill fallback. Three camera regression functions and seven simple-control checks passed; capture currently fresh at 30fps.
- Stationary validation was follow-off and stopped. The user subsequently operated follow with multiple tags and accepted the detection goal as complete; no additional agent movement or camera fault injection was performed.
- Source matches tested candidates; dashboard service remains active. Rollback: /home/advanced/ugv-curved-tag-backup-WzCwZj/{aruco.py,web.py,test_aruco.py,ARUCO.md,STATE.md}; new tests/fixtures/curled-id0.png and tests/test_camera.py.

## 2026-10-13 follow centering improvement

- Prior follow turn was proportional to image error with a hidden1300 PWM cap; small corrections could command far less torque than manual Q/E. Physical breakaway threshold remains uncalibrated.
- Follow now aligns using full-strength Q/E-style pivots at the selected speed, then approaches straight. It also centers when already close. Bounded40..180ms pulses, independent braking deadlines, camera settling and visual response adaptation limit overshoot and repeated blind commands.
- Speed slider updates the owner's active follow speed; STOP disables follow and brakes without automatically disarming. Fresh-frame/decode, ownership and disconnect gates remain.
- Verified with controller mocks, a simulated80% motor dead zone at three response rates in both directions, pulse expiry without camera processing, and UI speed/ownership contracts. Physical centering has not been tested by the agent.
- Validation: 49 Python tests and keyboard/follow-slider contract checks passed. Pi checks confirmed close-tag full-strength pivots and independent pulse braking with mocked hardware; UI/API served correctly and capture was fresh at30.1fps. Service restarted with follow off and robot disarmed.
- Rollback: /home/advanced/ugv-follow-center-backup-obbyd7zl/{aruco.py,web.py,app.js,ARUCO.md,DASHBOARD.md,SAFETY.md,STATE.md,.gitignore}.

## 2026-10-13 drone-parking prototype and outside-gear survey

- User confirmed powered-off drone, clear floor, human beside the car, and explicitly authorized autonomous controlled exploration and recovery.
- Drone tags9/12/13/14 form the allowed rigid landmark set; live survey mapped9 and12 together. Car position is currently a camera proxy in a provisional first-tag frame.
- Added map panel, raw marker poses, short static-scene visual bridge, swept full-footprint planner and bounded parking feedback. Exact entry requires verified envelope/gear/camera transforms and empirically validated error bounds; current values remain unverified, so entry is disabled.
- Physical120ms pivots measured approximately11° left and15° right. Seven100ms forward steps reached approximately1.11m nominal tag range; nine reverse steps returned beyond1.5m. Every motion was bounded and braked. Initial/median translation response differed substantially; no precise odometry claim.
- Camera≈30fps and USB IMU≈100Hz remained live. Final state after each survey was stopped and disarmed. See PARKING.md for actual limits and calibration conventions. No parking between the legs has been demonstrated.

- Second outside viewpoint: clockwise≈70°, six80ms tangent-biased forward pulses, then visual/IMU rotation back toward drone. Observed short scene bridge:12 unique fresh poses, up to104 PnP inliers; bridge subsequently expired. A fixed-jump recovery defect was corrected and the recorded survey replay relocalized automatically with three consistent tags. Camera proxy≈x0.419m,z1.495m from initial tag9 frame; nominal uncertainty≈0.237m, not parking precision.
- Final automated validation:62 Python tests;8 keyboard mappings and follow slider/ownership contracts; recorded survey replay excluding incidentalID17. Calibration remains unverified and no exact parking claim is made.

## Parking goal continuation: demonstrated fit and ground observer

- Operator confirmed the actual car was between the legs in the final reference image and fits; approximate dimensions6×8in are retained as empirical estimates, not certified envelope values.
- Stationary camera1400/1500/1600µs sweep showed≈0.19px static-scene motion; camera view effectively unchanged. Restored original servo values and disarmed after the sweep.
- Added independent skid-ground pose/tracking observer and two meaningful geometry/tracker tests. Initial seed≈35° oblique approach; systematic geometry error remains unverified.
- Pi SSH and dashboard subsequently became unreachable while the router remained in the local ARP table. The next short motor probe was not deployed or executed; last agent-controlled state was stopped/disarmed. Goal remains physical parking, not prototype completion.

## 2026-10-13 motor trials suspended on wired supply

- Operator confirmed the LiPo was depleted and the car is now on a wire. Current instruction: do not run any motors; commit and push progress only. Resume physical motion only after a later operator instruction authorizes it.
- Refined skid-plane pose candidates with LM before geometric validation. The recorded seed's reprojection RMS decreased from approximately5.7px to4.4px; systematic geometry and mounting error remain unverified.
- An outside-only alignment route to a camera-proxy staging point0.55m before the drone centre was computed and all42 sampled poses/sweeps checked against the nominal skids, using a200×280mm envelope inflated100mm. This route was not executed and does not demonstrate parking.
- Prior Pi connection loss is explained by the depleted LiPo. No motor probe was issued after the outage. Existing raw captures and telemetry remain in ignored local work storage.

## 2026-10-17 UGV migration and dynamics campaign

Canonical source is `uOAeronautics/ugv`; laptop `~/Projects/ugv`, Pi
`/home/advanced/ugv`, package/CLI `ugv`, system service `ugv-dashboard.service`.
SSH alias `advanced` uses a dedicated public key; secrets are not in the repo.
Pi package rebuilt in its new path; service active, dashboard HTTP 200;
74 robot-side tests passed after the rename.

Measured mass 577.4 g. Recorded 116 floor trials and 44,282 100 Hz IMU/PWM
samples without telemetry loss. Longer trials last two seconds at PWM 1800;
Q/E and curved motions show material asymmetry. Raw measurements remain on
both machines; calibrated response and held-out simulator comparison are in
`simulation/gazebo/real_robot_profile.json` and `calibration_validation.json`.
See `DYNAMICS.md` for rest-window bias correction, conditional translation
estimates and limitations. Absolute position, speed scale and stopping distance
remain unverified by an independent measurement. The car finished stopped and
disarmed; calibration did not change the real controller's ownership/watchdog.

## 2026-10-17 Reset feedback and steering correction

A repeated-reset browser regression reproduced W+A travelling almost straight
while raw wheel feedback remained zero. Reset now removes/recreates joint
velocity requests and clears cached joint entities; missing feedback never
gets treated as zero measured speed for powered actuation. Bounded zero-input
gearbox/brake resistance also brakes the inside wheels of a steering arc.
Refitted effort tables preserve the measured yaw response. Native and browser
regressions each passed six resets, checking full steering geometry, pivot
translation and prompt release braking. The tire friction coefficients were
not changed. No new real-car driving trials were required for this correction.
