# DASHBOARD

```text
url := http://192.168.8.170:8765
transport := browser ↔ WebSocket(/ws) ↔ Pi-authoritative controller ↔ PCA9685
keys := W/S forward/reverse; A/D strafe-left/right; Q/E rotate-left/right; arrows=pan/tilt; Space STOP
input := keyboard-only movement; UI keycaps=status indicators, not controls
touch := directional pad + STOP
slider := speed_limit 500..1800; default=1800
video := GET /stream.mjpg; MJPEG 640x480@20; rot180; latest-frame-only
vision := optional browser-only Ultralytics YOLO11n@320 ONNX + ORT-Web/WASM; Pi inference∅
boxes := canvas overlay; confidence=.38; class-aware NMS=.45; cadence≈3.3Hz
stabilize := optional browser-only 64x48 translational registration; max shift=24x18px
arm := explicit/session-local; reconnect=>disarmed
```

## state

```text
server := owner+vector+speed+wheels+reason+revision
one-controller; observers=N
drive.mix := left=f+r; right=f-r; normalize≤1
side-step ordinary/skid-steer := turn(d·θ) > forward(l) > turn(-d·2θ) > reverse(l) > turn(d·θ)
side-step := gyro heading ±12° from initial heading; translation=.35s@.40; brake=.12s; return initial heading; repeat while A|D held
result ideal := longitudinal≈0; heading∆≈0; lateral≈2·l*sin(θ); translation remains timed, slip=>position drift
Q/E := gyro yaw-rate target ±40°/s; cap=user slider≤1800PWM; ramp=3500PWM/s; reversal crosses zero
pivot breakaway := yaw<3°/s + nonzero target => ≥90% selected PWM before ramp; drops when rotating; uncalibrated hardware threshold
W+A/D := forward steering arcs; S+A/D := reverse steering arcs (yaw sign reversed); Q/E precedence
arc := differential correction≤65% travel command; both sides retain travel direction; yaw target may be unattainable at this bound
A/D alone := experimental side-step; releasing W/S while A/D held starts side-step; release all keys to stop
gyro safety := sample age≤150ms + calibrated; wrong-direction>.30s or no-response>1.2s => brake+disarm
side-step turn deadline=3s; translation heading error>10° => brake+disarm
manual input lease=450ms; independent browser-command watchdog=600ms; internal ticks cannot renew lease
motor polarity := forward+rotation intent inverted at output (physical chassis correction)
camera := pan ch8 + tilt ch9; center=1500us; step=25us; bounds=1000..2000us
camera.pan polarity := ArrowLeft=>+25us; ArrowRight=>-25us [physical correction]
```

## fail-safe

```text
drive-refresh=150ms while held; watchdog=600ms independent hardware thread
heartbeat=200ms state-only; heartbeat does NOT renew motion; arm-required
keyup|blur|hidden|disconnect|shutdown|STOP => wheel0..7 brake=4095
client raw-PWM∅; server clamps vector±1 + speed[500,1800]
camera worker ∥ motor guard; camera stall/failure cannot stall braking
LAN-only; auth∅ => trusted-private-network only
LED := unsupported PCB-v3; UI∅
heading := game compass, relative to 3s stationary calibration pose; clockwise gyro integration
heading-rest := controls neutral + quiet IMU 500ms; bias adaptation only at rest; gaps>150ms => hold heading
heading-limit := no magnetometer => no north; gyro yaw drifts; IMU cannot recover translation from wheel skid
```
