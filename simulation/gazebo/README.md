# UGV Gazebo simulation

The local drive console now uses the real car's 500–1800 PWM scale, matching
WASDQE mixing and combined inputs. The real-car effort/contact fit comes from
116 floor trials; see [dynamics calibration](../../docs/DYNAMICS.md) and
`real_robot_profile.json`. Turn response is gyro-measured; translation remains
a provisional short-burst inertial estimate. Release or command expiry brakes
the simulated car. This console has no connection to real hardware.

## Freenove photo-informed model

Revised 2026-10-17 from the user's four photographs and explicit correction:
**1 inch is the motor-bottom ground clearance; 1.75 inches is the main board
height.** The old solid chassis envelope was incorrect and has been removed.

The visual assembly includes a thin black main PCB, four yellow TT gearmotors
with silver cans, black treaded tires and yellow spoke hubs, an underside
estimated low central LiPo battery, a raised green Raspberry Pi, a blue Nano on
headers, a front servo/horn, a lower forward-facing camera, an orange ribbon,
and approximate screws, switches, LEDs and connectors. It represents the
photographed configuration, with no ultrasonic turret attached. Loose cables
outside the car are omitted.

| Measurement | Inches | Millimetres |
|---|---:|---:|
| Tire/chassis overall length, excluding camera protrusion | 8 | 203.2 |
| Main board length | 7.5 | 190.5 |
| Main board width | 3.75 | 95.25 |
| Tire outer width | 5.75 | 146.05 |
| Wheel diameter | 2.5 | 63.5 |
| Motor bottom above ground | 1 | 25.4 |
| Main board top surface above ground | 1.75 | 44.45 |

The board thickness is provisionally 1.6mm; the 1.75in reference is interpreted
as its top surface. The heights no longer define a 0.75in thick chassis block.
Motor housings reach down to 25.4mm. The battery pack and camera can extend
below the motor bottoms; the 1in measurement is not a universal clearance.

Assumptions retained from the original sketch: symmetric axles, overall length
measured tire-to-tire, and tires directly beside the board. These imply a
5.5in / 139.7mm wheelbase, 4.75in / 120.65mm wheel-center track, and 1in / 25.4mm
tire width. Tire core is slightly smaller than the measured radius, with visual
tread reaching the nominal envelope; collision cylinders use the full radius.

**Unmeasured component sizes and locations are approximate.** Photos guide
appearance, not calibrated photogrammetry. Board corner shape, circuitry,
wiring, fasteners, rim/spoke geometry, lens, and brackets are simplified.
The camera extends beyond the 8in tire envelope. No accessory dimensions or
mass distribution should be treated as measured clearance geometry.

Existing `docs/HARDWARE.md` records a 210mm wheelbase and 149mm track from
2026-09-27. Those cannot fit this new overall envelope and are preserved as
historical measurements. This model uses the new sketch separately.

## Generate and run

Edit `dimensions.json`, then use Python 3:

```sh
cd simulation/gazebo  # from the repository root
python3 generate.py
gz sdf -k freenove_world.sdf
python3 build_motor_plugin.py
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/plugins/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
gz sim -r --gui-config car_gui.config freenove_world.sdf
```

`car_gui.config` starts with a fixed, elevated view of the car and test pad,
with editing panels hidden. The pale 1.6m square mat has a 10cm grid, marked
home position, +X/front and +Y/left arrows, and a 50cm scale ruler. The floor
texture and markings are visual only, so they do not introduce bumps or alter
wheel contact. `environment.py` defines the floor, texture placement and lights.
The car starts at the center facing +X. The fixed camera keeps the room's
directions stable while the car turns; use the mouse to orbit or zoom.

The checked-in `assets/test_pad.png` loads relative to the world file; no
internet assets are required at runtime. To redraw this optional texture:

```sh
uv run --no-project --with pillow python make_test_pad.py
```

In a second terminal, run
a bounded simulation-only drive demo:

```sh
python3 simulation/gazebo/demo_drive.py --seconds 60
```

Run the second command from the repository root. The demo sends a slow forward
and turning velocity, then a zero velocity when the duration ends or it is
interrupted with Ctrl+C. The simulation must already be running. The GUI's
pause control can pause the whole simulation at any time.

## Local keyboard control panel

Open `launch_control.command` in Finder, or from the repository root run:

```sh
./simulation/gazebo/launch_control.command
```

This opens a local browser panel at `http://127.0.0.1:8767`. Keep the panel
focused beside the Gazebo window. Hold W/S for forward/reverse, A/D for steering,
or Q/E to pivot; release to stop. W+A and W+D make curved turns. This mapping
matches the existing car dashboard; the four-wheel model cannot strafe.
Space, Escape and the Stop button release all controls. The panel also has
pointer/touch buttons, a 0.03–0.30 m/s speed slider, live world-position map,
and Open Gazebo / Play / Pause / Reset buttons. Reset stops the car and leaves
the world paused; click Play before driving again.

The launcher uses the installed Gazebo Python bindings and creates an ignored
`.control-venv` with the matching Protobuf runtime using `uv`. On this Mac,
Gazebo's Homebrew Python 3.15 bindings require Protobuf 7.36.2 or later.
It does not install the Raspberry Pi package or import its hardware services.
Native Gazebo Transport publishes velocity at 20Hz; expired browser heartbeats
(400ms), blur, a hidden tab, key release, pause and server shutdown stop motion.
The HTTP server binds only to loopback. Stop other demo publishers before using
the panel, since Gazebo does not arbitrate multiple velocity publishers.

Controller checks (standard library only):

```sh
python3 -m unittest discover -s simulation/gazebo -p test_controls.py
```

`assembly.py` defines the parts used by both SDF generation and the Blender
preview. The Gazebo model has four torque-driven wheel joints, four small equivalent
radial tire springs, visual-only tread, and simplified component collisions.
See [MOTOR_MODEL.md](MOTOR_MODEL.md) for the reference TT motor specs, torque-speed
model, rubber-contact assumptions, native plugin build and validation details.
All electronics and motors are rigidly attached to the chassis; the servo and
camera are visual approximations, not articulated sensor systems.

To recreate the editable Blender file and the assembled/underside previews:

```sh
blender -b --python render_blender.py -- /absolute/path/to/output-folder
```

The output folder defaults to `preview/` beside the script when omitted.

Requires Gazebo Sim 10 with Physics, UserCommands, SceneBroadcaster and
OdometryPublisher systems, plus the local `ugv_tt_motor` plugin. CMake and a C++17
compiler are needed for the first plugin build. The controller applies bounded
DC motor torque to each wheel; it no longer uses ideal DiffDrive actuation.

Simulation-only velocity: `/model/freenove/cmd_vel` (`gz.msgs.Twist`).
Odometry: `/model/freenove/odometry`. A 400 ms timeout is configured; send a zero Twist
to stop after experimenting. This model has no connection to real hardware.

Total mass is measured at **577.4 g**. `mass_distribution.json` assigns estimated
component masses and positions: 190 g LiPo, 122.4 g motors, 128 g wheels,
133 g boards/electronics/wiring and 4 g mounting carriers. The rigid link is
445.4 g. `mass_properties.py` computes its center of mass and full inertia tensor
using component envelopes and the parallel-axis theorem; wheel inertia combines
an annular rubber tire (20 g) and plastic hub (12 g). The nominal full assembly
center of mass is (-2.70, 0.026, 36.39) mm relative to the ground-level model
origin, before tire compression. Distribution and tire parameters remain estimates. The torque
and speed reference values come from a documented similar TT motor.
Visual detail does not establish calibrated physics, traction, or skid behavior.

Validation: generated XML parsed; link and joint references, thin-board height,
motor-bottom clearance and nominal wheel envelope checked. Blender views
rendered from the same assembly definition. On macOS arm64 with Gazebo Sim
10.5.0, `gz sdf -k` passed, the GUI loaded the model, and a bounded drive test
changed both the published odometry and the simulated model's world pose.
This verifies loading and basic movement; the physics parameters remain
uncalibrated.
