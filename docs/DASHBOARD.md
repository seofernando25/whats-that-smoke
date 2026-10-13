# DASHBOARD

```text
url := http://192.168.8.170:8765
transport := browser ↔ WebSocket(/ws) ↔ Pi-authoritative controller ↔ PCA9685
keys := W/S forward/reverse; A/D turn-left/right; Q/E rotate-left/right; arrows=pan/tilt; Space STOP
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
manual := direct differential wheel commands; no gyro requirement, yaw-rate loop, ramp, breakaway, or side-step
W/S := forward/reverse; A/D := left/right pivot or steering combined with W/S; Q/E := left/right turn (precedence)
mix := left=-forward+turn; right=-forward-turn; normalize max magnitude to selected speed 500..1800
watchdog := brake after 600ms without fresh drive; remain armed for same connected controller
keyup|blur|hidden|STOP := disable follow + brake without disarming; DISARM button explicitly brakes+releases
owner-disconnect|shutdown := brake+release session; reconnect requires ARM
camera := pan ch8 + tilt ch9; center=1500us; step=25us; bounds=1000..2000us
camera.pan polarity := ArrowLeft=>+25us; ArrowRight=>-25us [physical correction]
```

## fail-safe

```text
drive-refresh=150ms while held; watchdog=600ms independent hardware thread
heartbeat=200ms state-only; heartbeat does NOT renew motion; arm-required
keyup|blur|hidden|disconnect|shutdown|STOP => wheel0..7 brake=4095; STOP preserves arm, explicit DISARM releases
client raw-PWM∅; server clamps vector±1 + speed[500,1800]
camera worker ∥ motor guard; camera stall/failure cannot stall braking
LAN-only; auth∅ => trusted-private-network only
LED := unsupported PCB-v3; UI∅
heading := game compass, relative to 3s stationary calibration pose; clockwise gyro integration
heading-rest := controls neutral + quiet IMU 500ms; bias adaptation only at rest; gaps>150ms => hold heading
heading-limit := no magnetometer => no north; gyro yaw drifts; IMU cannot recover translation from wheel skid
```

## Follow centering · 2026-10-13

`FOLLOW := center with Q/E-style full selected-speed pivots > approach straight > stop near tag; pivot impulse40..180ms; settle>=120ms; fresh post-pivot image required; Speed slider applies live; STOP disables follow while preserving ARM.`
