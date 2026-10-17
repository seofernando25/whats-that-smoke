# UGV real-car dynamics and Gazebo calibration

Canonical repository: `uOAeronautics/ugv`. Laptop: `~/Projects/ugv`.
Pi: `/home/advanced/ugv`, reached with `ssh advanced`.
Hardware service: `ugv-dashboard.service`; Python package and CLI: `ugv`.

## Measurements on 2026-10-17

With the operator confirming a clear floor and authorizing all trials, the Pi
recorded 116 completed movement trials and 44,282 raw IMU/control samples at
approximately 100 Hz, with no telemetry buffer drops:

- 86 short trials at dashboard PWM 600, 800, 1300 and 1800, including repeats
  and aliases. Lower PWM often translates but cannot overcome pivot/arc scrub.
- 30 two-second full-dashboard-power trials: three independent repeats of
  W, S, Q, E, W+A, W+D, S+A and S+D, plus A/D and pivot-key aliases.
- The first long session stopped before its next trial because rest was not
  stable. Its 16 completed trials were retained; the remaining 14 were resumed
  after the operator released another controller session. No moving trial was
  resumed from an interrupted command.

Sessions are named `2026-10-17-floor-01`, `2026-10-17-floor-long-01` and
`2026-10-17-floor-long-03`. Raw `samples.jsonl`, `events.jsonl` and manifests
are retained under `calibration/runs/` on both machines and ignored by Git.
Compact measured response tables, sensor statistics and held-out comparisons
are versioned in `simulation/gazebo/real_robot_profile.json` and
`calibration_validation.json`. Source timestamps use the **Pi monotonic clock**;
the Pi wall clock was not used to synchronize samples or command transitions.

## Keys and motor commands

`tools/calibration/commands.py` enumerates every one of the 64 key subsets.
Forward is W−S. Turn is (Q−E), falling back to (A−D) if the pivot keys cancel.
A and Q alone are equivalent; D and E alone are equivalent. Opposing controls
cancel according to these rules. There are eight distinct nonzero movement
vectors, so repeated equivalent combinations do not need separate physics.

The real hardware mixes PWM as `left=-forward+turn` and
`right=-forward-turn`, dividing both by their maximum magnitude or 1.
Negative hardware PWM corresponds to physical wheel-forward rotation.
Consequently W+A holds the left wheels at zero and drives the right wheels
forward; it is not a sideways translation. Two keys do not double motor power.
The simulation uses this same mapping and normalization.

PWM 1800 means full **dashboard** power. PCA9685's numerical range is 0–4095;
these trials did not extend the hardware dashboard's allowed power range.

## Drift and observable quantities

`DynamicsTelemetry` keeps a bounded read-only history of raw acceleration,
raw gyro, actual applied per-wheel PWM, input vector, revision and Pi monotonic
time. `/api/dynamics?after=<sequence>` returns new samples and reports overflow.
The recorder never opens the serial device a second time or bypasses the
controller's hardware lock, ownership, release behavior or 600 ms watchdog.

Every trial uses quiet windows before and after motion. Analysis estimates
three-axis zero-rate bias in those windows, interpolates the bias across the
trial and projects gyro onto the measured gravity direction. This handles the
Nano's tilted mounting and avoids treating stationary gyro offsets as yaw.
Integration starts fresh per trial; it does not chain accumulated heading or
position over the whole experiment. A synthetic regression checks tilted-sensor
projection and a changing gyro bias against a known response.

Planar acceleration is rotated using short-window yaw, with an assumed IMU
lever arm and zero-velocity endpoint constraints. This produces **provisional
short-burst speed estimates**, not an independent measurement of translation.
Mounting flex, floor tilt, vibration and accelerometer drift can bias those
estimates substantially, especially during pivots. An external measured floor
reference, calibrated fiducials or encoders are still needed for absolute speed,
travel distance, skid and stopping-distance calibration. Short quiet captures
also do not establish long-term bias instability or a bias random-walk model.
[Sensor integration errors](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs).

## Gazebo model

The 577.4 g measured mass and composite inertia are retained. The native
`ugv::TTMotorSystem` applies finite DC output-shaft torque to each wheel, with
back EMF, bounded zero-input gearbox/brake resistance and a 400 ms command timeout. Browser controls publish
normalized wheel duties on `/model/freenove/wheel_duty`; the original
`cmd_vel` path remains available for SI-velocity diagnostics.

An empirical lookup maps `(forward, turn, PWM)` to **effective drive effort**,
then the motor/contact simulation determines the motion. This is an aggregate
behavior fit, not a measurement of motor voltage, motor electrical constants,
individual motor torque or rubber friction. Per-command first-order spin-up lags reproduce the measured turn transients;
release requests the bridge brake state immediately. Brake resistance is an
estimated finite wheel torque, capped to prevent reversing a stopped wheel
in a single physics step. Separate curves preserve the real
left/right and forward/reverse arc asymmetries and low-power dead zones.
Lateral force-dependent slip is fitted at 0.30 m/(s·N); longitudinal slip stays
at its reference assumption. These are effective contact-model parameters.

Full-power turn curves were fitted using the first two two-second repetitions;
the third repetition was held out. Lower-power response points use short-pulse
first-order extrapolations and are less certain. Straight-drive effort uses
unverified short-burst inertial speed estimates. The fit is specific to this
floor, battery condition and robot configuration; it is not a universal motion
or clearance guarantee. Full comparison results are in
`simulation/gazebo/calibration_validation.json`.

The Gazebo console uses the same 500–1800 PWM scale as the real dashboard.
It controls simulation only. It does not connect to or arm the real Pi.

## Reproduce

Install the normal Pi environment with `uv sync --locked`. Read-only capture
and fitting code is under `tools/calibration/`. For a newly authorized,
supervised session on a clear floor, with other controllers disarmed:

```sh
ssh advanced
cd ~/ugv
uv run python tools/calibration/run_trials.py --floor-clear --output calibration/runs/NEW-SESSION
uv run python tools/calibration/run_trials.py --floor-clear --long --output calibration/runs/NEW-LONG-SESSION
uv run python tools/calibration/analyze.py calibration/runs/NEW-SESSION
```

These commands move the real car. The recorder checks fresh IMU data and
ownership, waits for a stable baseline, sends repeated drive intent and always
requests disarm on exit. The controller watchdog is unchanged.
`--start-index` resumes a plan with fresh rest windows; it does not resume
motion. New sessions must use new output directory names.

On the laptop, the Gazebo Python environment also provides transport bindings:

```sh
simulation/gazebo/.control-venv/bin/python tools/calibration/evaluate_gazebo.py --profile --output calibration/runs/sim-validation.json
python3 tools/calibration/fit_profile.py
python3 simulation/gazebo/generate.py
uv run --no-project --with numpy --with matplotlib python tools/calibration/report.py
```

`fit_profile.py` reproduces this recorded campaign; adapt its session inputs for
a new campaign. `evaluate_gazebo.py` uses an isolated transport partition and
only starts/stops its own headless simulation. Omitting `--profile` measures the
reference effective-voltage probe curve used for fitting; `--profile` validates
the generated empirical map. The report tool renders a curve comparison and
records the held-out yaw-rate error. It requires all named raw sessions and
simulator probe JSON files; raw data is deliberately not bundled into Git.

Checks:

```sh
uv run pytest -q tests
python3 -m unittest discover -s tools/calibration -p 'test_*.py'
python3 -m unittest discover -s simulation/gazebo -p 'test_*.py'
gz sdf -k simulation/gazebo/freenove_world.sdf
```

## Reset and release regression (2026-10-17)

The first browser smoke test only required a small heading change and missed a
reset defect: Gazebo cleared its joint-velocity request bookkeeping while the
custom motor system kept the old zero-valued velocity components. Back EMF
and the brake on the stationary inside wheels then used zero instead of the
actual wheel speed. After reset, W+A could travel almost straight and keep
coasting even though both keys reached the server correctly. Fresh-world yaw
fits alone did not detect this failure.

The motor system now removes the velocity request components during Reset,
clears cached joint entities and recreates the requests on the next update.
It refuses powered torque until wheel-speed feedback is available. Zero input
also applies bounded estimated gearbox/brake resistance, including the held
inside wheels of W+A. Accepted browser commands publish immediately instead
of waiting for the next 50 ms transport tick. Static panel assets disable
caching so refreshed tabs load the current control code.

`brake_effort_corrections.json` records the refit of effective effort after this
brake-model change; `fit_profile.py` applies it reproducibly. Tire Coulomb
coefficients are unchanged. This correction addresses motor feedback and
braking rather than treating all sliding as a reason to raise tire friction.

The stronger regressions exercise repeated resets, actual wheel feedback,
forward travel, a near-centre Q pivot, tight translating W+A/W+D/W+Q arcs,
and actual post-release displacement:

```sh
simulation/gazebo/.control-venv/bin/python simulation/gazebo/check_reset_physics.py
node simulation/gazebo/check_browser_controls.cjs
```

The browser check needs Playwright and Chromium plus the local panel running.
`PLAYWRIGHT_MODULE` and `CHROME_PATH` optionally select installed runtimes.
It fails if W+A turns less than 120 degrees in its two-second hold, if a pivot
translates excessively, or if the car still drifts 200 ms after release.
The native check measures the turn centre and requires forward braking below
0.02 m/s within 0.18 s. Neither test commands the real robot.
