# Real-car PWM calibration

The browser now uses the real dashboard's normalized wheel PWM, not an ideal
keyboard-to-Twist conversion. See `../../docs/DYNAMICS.md` for the 116-trial
campaign, bias/drift handling, observed asymmetries and validation limits.
`real_robot_profile.json` supplies an aggregate effective-effort map and fitted
lateral slip (0.30 m/(s·N)). The reference parameters below still describe the
motor torque law and assumptions; they are not identified electrical constants.
The 0.03 lateral slip value below was the original reference assumption.
`cmd_vel` retains its velocity servo for diagnostics. Both command paths expire
after 400 ms without a new command; zero duty dynamically brakes the motor.

## TT motors and rubber tires

The appearance follows the supplied dual-shaft yellow TT motor photo. The
documented reference is [Adafruit 3777](https://www.adafruit.com/product/3777):
3–6V, 48:1, nominal 70×22×18mm body, 30.6g, and 0.8kgf·cm stall torque at 6V.
Their measured 6V no-load speed is 250rpm; the catalog also gives a nominal
200rpm minimum-speed specification. This model uses the measured 250rpm point.
These values describe a similar motor, not an identified Freenove part or the
unidentified marketplace listing. Different TT motors can have different gears,
windings, torque and current. No exact motor or tire material datasheet was
provided for the user's car.

## Geometry

`tt_geometry.py` adds a split yellow shell, seam, raised cover, shaft bosses,
white flat output shafts on both sides, two case screws, a flattened metal can,
brush cap, bearing, retaining clips, terminals, and short leads. Subpart sizes
and placement are estimated; this is a recognizable parametric model rather
than a vendor CAD reproduction. The 1in nominal motor-bottom clearance and
1.75in main-board height are retained. Motor housings stay attached to the
chassis and the wheels rotate independently. Tires have tread blocks and matte
rubber material; yellow wheel hubs remain plastic.

## Motor dynamics

`plugins/TTMotorSystem.cc` applies joint **torque**, replacing the previous ideal
DiffDrive velocity actuator. Four motors receive independent torques. At the
assumed 6V terminal voltage, the reference output-shaft limits are:

- Stall torque: `0.8 × 0.0980665 = 0.0784532 N·m` per motor.
- No-load angular speed: `250 × 2π / 60 = 26.18 rad/s`.
- Wheel no-load surface speed at 31.75mm radius: about `0.831 m/s`.

The panel's 0.30m/s bound remains a command limit, not a motor specification.
Twist commands become left/right wheel speed targets. A simulated PI velocity
servo chooses effective voltage fraction `u`, bounded to [-1,1]. The motor law
is the quasi-static linear DC torque-speed approximation:

`tau = clamp(tau_stall × (u - omega / omega_no_load), ±tau_stall)`.

The PI gains are controller assumptions, not datasheet parameters. The real
motors have no built-in feedback; this virtual servo does not imply that the
physical car has wheel encoders. Zero command shorts the virtual terminals for
dynamic braking; it does not instantaneously zero wheel speed. Reset clears
commands and the servo integrators. The original velocity actuator is absent,
so it cannot silently override the motor torques.

Gear ratio is informational because speed/torque are specified at the gearbox
output. It is not multiplied into the output torque again. There is no battery
sag, driver voltage drop, current or temperature simulation, gear backlash, or
identified rotor inertia. The actual Freenove motor voltage remains unmeasured.

## Rubber contact approximation

`motor_profile.json` separates reference values from explicitly named assumptions:
isotropic friction coefficient 0.8, zero restitution, and lateral/longitudinal
force-dependent slip of 0.03/0.0005 m/(s·N). The wheel/link Y axis is the axle; the rotated cylinder collision's local Z
axis supplies the lateral friction direction. These are tunable rubber-on-test-mat
assumptions, not values established by the motor specification. Four ordinary
wheels must scrub sideways in pivot turns; commanded and actual yaw can differ.

Each wheel has an equivalent radial spring/damper carrier with ±2mm travel,
5000N/m stiffness and 10N·s/m damping. This approximates a small amount of tire
compression and lets four contacts share load instead of rocking on a rigid
diagonal. It is a contact-model approximation, not actual suspension or a
finite-element rubber tire. Under load the nominal chassis clearances decrease
slightly. Carrier mass is 1g each and counts within the measured **577.4g** total.
The chassis rigid link is 445.4g; each complete wheel is 32g, estimated as
20g rubber annulus plus 12g plastic hub. All four 30.6g reference motors are
included in the chassis mass. A 190g LiPo is the heaviest estimated component,
placed low and central at (-2, 0, 30)mm. Its 85×44×20mm visual envelope is an
assumption, not a measured pack specification.

`mass_distribution.json` records the estimated component budget. Composite
box inertias and the parallel-axis theorem produce the chassis COM and full
inertia tensor; wheel inertia uses the annular rubber and plastic hub geometry.
The nominal complete assembly COM is (-2.70, 0.026, 36.39)mm, before settling.
These estimates give approximately 48.1% front / 51.9% rear static load on level
ground. Only the total mass is measured; battery mass/position and wheel masses
can be refined without changing that constraint.

Gazebo's DART backend reads friction and force-dependent slip from the SDF
`surface/friction/ode` fields; that tag does not select the ODE engine.
[DART SDF importer](https://github.com/gazebosim/gz-physics/blob/gz-physics9/dartsim/src/SDFFeatures.cc).
The tire collision is smooth and round; tread blocks are visual-only to avoid
artificial bumping from individual boxes. Friction, radial stiffness, component
mass distribution, center of mass and motor gains require measurements to calibrate.

## Build, launch and inspect

The control-panel launcher builds the plugin automatically when its source
changes. For a standalone Gazebo launch from `simulation/gazebo`:

```sh
python3 build_motor_plugin.py
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/plugins/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
gz sim -r --gui-config car_gui.config freenove_world.sdf
```

Requires CMake, a C++17 compiler and Gazebo Sim 10 development libraries. Local
build products are ignored; source and CMake files are portable. Changing
`motor_profile.json` requires regenerating the SDF and reopening the world.

Regression checks:

```sh
python3 -m unittest discover -s simulation/gazebo -p 'test_*.py'
simulation/gazebo/.control-venv/bin/python simulation/gazebo/check_motor_physics.py
```

The live check launches a separate headless world in a unique Gazebo partition.
It checks forward displacement, braking, positive pivot rotation, stable floor
support and the per-wheel torque bound without commanding the user's open world.

`/model/freenove/motor_state` (`gz.msgs.Double_V`) publishes four values for each
wheel in front-left, front-right, rear-left, rear-right order:
actual rad/s, target rad/s, output torque in N·m, effective voltage fraction.
`/model/freenove/odometry` now reports simulated body/world odometry through
Gazebo's OdometryPublisher, not ideal wheel-encoder integration.

## Reset-safe feedback and zero-input braking

Reset must recreate joint velocity requests; a retained empty/zero component
can otherwise silently disable back EMF and braking. Powered torque is skipped
while feedback is unavailable. A zero wheel command applies 0.06 N·m·s/rad
damping plus 0.01 N·m estimated brake/geartrain resistance, limited by the
reference stall torque and a one-step wheel-inertia cap. These are response
estimates rather than measured electrical brake constants. The cap avoids
a damping update reversing the wheel at low speed. Turn effort is refitted
in `brake_effort_corrections.json`; steering holds its inside wheels while
pivoting drives the two sides in opposite directions.
