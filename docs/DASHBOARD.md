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
side-step timing := θ-phase=.16s@.65; translation=.20s@.55; repeat while A|D held
result ideal := longitudinal≈0; heading∆≈0; lateral≈2·l*sin(θ); open-loop drift=>calibration required
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
```
