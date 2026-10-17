# Noisy Nano IMU and shared heading estimator

The local Gazebo console now runs the same `ugv.heading.RelativeHeading` class
as the Pi. The production estimator is unchanged. It receives noisy acceleration
in g, gyro in degrees/second, simulation timestamps and command intent. World
pose is used only for the comparison display; it never corrects the estimator.

## Recorded fit

`imu_noise_profile.json` is fitted by `tools/calibration/fit_imu_noise.py` from
four existing floor/lap recordings. It contains recording hashes and selection
rules. There are 21,911 samples in 120 settled rest spans, totaling approximately
219 seconds. Samples must follow at least one second of zero-PWM rest, have
plausible gravity and a quiet gyro; telemetry gaps split spans.

| Sensor-axis quantity | X | Y | Z |
|---|---:|---:|---:|
| Mean recorded gyro offset (°/s) | −0.0112 | 0.8118 | −0.0131 |
| Within-rest gyro sample standard deviation (°/s) | 0.0391 | 0.0413 | 0.0432 |
| Within-rest accelerometer sample standard deviation (g) | 0.000617 | 0.000543 | 0.000717 |

The model uses the full residual covariance of each three-axis sensor, rather
than independent white noise on each axis. Gravity fixes mounting tilt; the
previous launch-acceleration fit supplies the forward direction. The approximate
sensor location is the visual Nano board centre, not a measured mounting lever
arm. Native Gazebo acceleration accounts for this position during turns.

Each reset draws an offset from the observed rest-window mean spread and adds
a small, time-varying bias. Its variance is estimated from half-second rest
means after removing the expected independent sample-noise contribution. An
Ornstein–Uhlenbeck approximation uses measured block correlation where resolved;
otherwise its time constant is set to the 0.5-second block resolution and is
explicitly marked unresolved in the profile. This is not a long-term drift fit.

The deterministic seed defaults to 5774. Each world rewind starts a new epoch
using seed + epoch. Restarting the adapter reproduces the same sequence for the
same input samples. Manual heading zero recalibrates without resampling noise.

## Pipeline and units

1. The world loads `gz::sim::systems::Imu`. An ideal chassis-frame IMU at the Nano
   location publishes `/model/freenove/imu/ideal` at the 1000 Hz physics rate.
2. `ImuDownsampler` applies an assumed two-pole 25 Hz low-pass and averages
   10 ms of ideal samples before producing 100 Hz output. This prevents physics/contact chatter from aliasing into the sensor
   stream. The cutoff approximates sensor bandwidth; it is not an identified BMI270
   digital filter. Gaps and resets discard partial windows.
3. `RecordedImu` rotates into the recorded Nano axes, converts units and adds
   measured offsets, covariance and short-term bias variation.
4. `RelativeHeading.update` receives those readings at simulation time. It
   performs its normal three-second rest calibration and rest-bias tracking.
5. The adapter publishes two equivalent noisy streams and exposes the estimate
   and truth comparison in `/api/status` under `imu`.

| Topic | Message | Units/frame |
|---|---|---|
| `/model/freenove/imu/ideal` | `gz.msgs.IMU` | m/s², rad/s, chassis axes, includes ideal orientation |
| `/model/freenove/imu/noisy` | `gz.msgs.IMU` | m/s², rad/s, Nano sensor axes; **orientation absent** |
| `/model/freenove/imu/nano` | `gz.msgs.StringMsg` | JSON `{type:"imu", a:[…], g:[…], t_sim_s:…}`; g, °/s |

No ideal orientation is copied into the noisy topic. The Nano JSON has the same
`type`, `a` and `g` fields accepted by `ugv.imu.parse_imu_line`. Its timestamp is
explicitly simulation seconds, not the firmware's serial clock. Serial hardware
is neither opened nor emulated. The local console adapter must be running to
publish noisy streams; the native ideal IMU exists independently in the world.

```sh
gz topic -e -t /model/freenove/imu/noisy
gz topic -e -t /model/freenove/imu/nano
```

The console shows clockwise estimated heading, clockwise true heading relative
to the same zero, and wrapped signed error in [−180°, 180°). Its existing world
yaw display is counterclockwise. Hold still for three simulation seconds after
starting, resetting or pressing **Zero heading**. Pausing freezes sensor time
and noise evolution. World reset waits for Gazebo to confirm its paused, rewound clock before
allowing Play, then automatically restarts the estimator and bias sequence; reconnecting after a sample gap uses the estimator's existing gap rule.

## Verification

```sh
python3 tools/calibration/fit_imu_noise.py
python3 simulation/gazebo/generate.py
gz sdf -k simulation/gazebo/freenove_world.sdf
python3 -m unittest discover -s simulation/gazebo -p 'test_*.py'
simulation/gazebo/.control-venv/bin/python simulation/gazebo/check_imu_pipeline.py
node simulation/gazebo/check_browser_imu.cjs
```

Fitting requires NumPy and the four raw sessions under `calibration/runs/`.
The checked-in profile runs without NumPy or raw files. The isolated native
check starts its own headless world and records `imu_validation.json`. It checks
100 Hz output, measured rest gyro noise, a W600ms→W+D arc, a Q pivot, heading
against ideal orientation, equivalent SI/JSON values, pause, reset and manual
zero. The browser check needs the running GUI/console, Playwright and Chromium;
`PLAYWRIGHT_MODULE`, `CHROME_PATH` and `IMU_SCREENSHOT` configure those paths.
Neither check connects to the real robot.

## Limits

Rest residuals combine sensor noise, firmware filtering and physical mounting
vibration. The Gaussian model approximates their distribution; it does not
identify intrinsic noise density, exact quantization or full temporal spectrum.
Rest-window variability is from one campaign, not independent power cycles.
Accelerometer bias along gravity cannot be separated from scale/tilt with these
recordings. Temperature effects, scale-factor errors and long-term Allan
variance/random walk remain uncalibrated. Motor-on vibration cannot be isolated
from true motion using these recordings alone, so no invented movement noise is
added. Simulated physical vibration still reaches the ideal sensor.

The real noise is small. The existing estimator removes static bias and suppresses
rest drift, so a stable displayed heading is expected; noise is not amplified
for visibility. Small error on short simulated turns verifies wiring, units and
frames, not long-term accuracy on the real floor.

Native sensor setup follows the [Gazebo Jetty IMU documentation](https://gazebosim.org/docs/jetty/sensors/). The recorded-noise adapter is repository code, not an additional Gazebo Gaussian noise layer.
