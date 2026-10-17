# UGV

`UGV := FNK0043/Freenove-4WD control+observe repo`

Canonical repository: https://github.com/uOAeronautics/ugv. Local checkout:
`~/Projects/ugv`; Pi checkout: `/home/advanced/ugv`. Connect with `ssh advanced`.
The Pi runs `ugv-dashboard.service`; commands are `uv run ugv` and
`uv run ugv-dashboard`. See [dynamics calibration](docs/DYNAMICS.md) for measured
keyboard responses, sensor drift treatment and the Gazebo fit.

The initial audit below is historical; current observations are in `docs/STATE.md`.

## ∆0

```text
target := Pi4B@adv034; os:=Debian13/aarch64; user:=advanced
net := router@192.168.8.1 -> pi@192.168.8.169/DHCP
pcb := Freenove connection PCB v3.0
cam := OV5647✓; native=2592x1944; mount=rot180
i2c.car := ×/disabled; spi := ×/disabled; motion := forbidden-until-preflight
```

## map

```text
docs/INDEX.md    -> load order + truth rules
docs/STATE.md    -> observed state / unknowns / next probes
docs/HARDWARE.md -> components + interfaces + orientation
docs/SAFETY.md   -> actuation gates + stop invariants
docs/NETWORK.md  -> topology + access
docs/PROTOCOL.md -> compact agent coordination grammar
docs/COMMANDS.md -> bounded snapshot + one-servo diagnostics
docs/DASHBOARD.md -> WebSocket control/state/dead-man contract
```

## ops

```text
env  := uv sync
run  := uv run ugv --help
dash := uv run ugv-dashboard # http://192.168.8.170:8765

deploy := Pi-local hardware service; source mirror=GitHub
cloud-runtime := ∅ by design; GPIO/I2C/camera live on Pi
browser-vision := local model/runtime assets; inference runs on dashboard client
aruco := robot-side DICT_4X4_50 IDs0..4; optional detect/follow; print=docs/aruco-tags-50mm.pdf
rule := observe≠actuate; ambiguity⇒q; secrets∉repo
```

Refs: [Freenove FNK0043](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi) · [official docs](https://docs.freenove.com/projects/fnk0043/en/latest/)

## Arduino Sense companion

See [arduino/README.md](arduino/README.md) for Nano 33 BLE Sense Rev2 firmware, acknowledged BLE streaming, a VQF orientation dashboard on port 8766, and transport stress scripts. Run it independently on a laptop or provision it on the Pi; it does not actuate the car.

Drone parking observation and prototype: [docs/PARKING.md](docs/PARKING.md). Open the dashboard and press **MAP**; exact entry awaits verified geometry and calibration.

## Gazebo car model

See [simulation/gazebo/README.md](simulation/gazebo/README.md) for the photo-informed
Freenove model, editable dimensions, close camera configuration, bounded drive
demo, and Blender preview renderer. The simulation has no hardware connection.

For local WASD/QE keyboard controls, open
`simulation/gazebo/launch_control.command` or run it from Terminal. It opens a
browser console at `http://127.0.0.1:8767` alongside Gazebo.
