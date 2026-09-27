# Arduino motion companion

Nano 33 BLE Sense Rev2 firmware and standalone BLE/VQF dashboard, imported from the working desktop experiment on 2026-09-27. This is an optional sensor companion to the Raspberry Pi car. It does not send motor commands or feed the car controller yet.

## Start on a laptop or Raspberry Pi

```sh
cd arduino
uv sync
uv run python server.py
```

Open http://127.0.0.1:8766 . Port 8766 avoids the car dashboard on 8765. `ARDUINO_HTTP_PORT` overrides the port. This companion has its own locked environment and requires Python 3.14 (uv can install it); the root car environment is unchanged. Desktop operation is observed; installation and BLE operation on the Pi have not yet been verified.

Linux needs a running BlueZ service, an enabled Bluetooth adapter, and permission for the user to access system Bluetooth. Power the Nano through USB; data travels over BLE. Only one receiver can connect at a time: stop the laptop playground before connecting from the Pi. Service discovery does not depend on a hardcoded Bluetooth address or pairing.

To view a Pi-hosted companion securely from a laptop, keep the server bound to localhost and forward its port:

```sh
ssh -N -L 8766:127.0.0.1:8766 advanced@<pi-address>
```

Then open http://127.0.0.1:8766 on the laptop. Leave that local port free for the tunnel.

Hold the mounted board still and click Calibrate for three seconds. A level surface is not required. Calibration also resets the visual reference and position. Zero orientation changes the display reference; Reset position clears the integrated velocity and trail. Reconnect/restart requires calibration again.

## Firmware

The board already has this firmware from the desktop session. For another board or future changes, stop the BLE receiver and connect USB:

```sh
arduino-cli core update-index
arduino-cli core install arduino:mbed_nano@4.6.0
arduino-cli lib install 'Arduino_BMI270_BMM150@1.2.4' 'ArduinoBLE@2.1.0'
arduino-cli compile -b arduino:mbed_nano:nano33ble firmware/MotionStream
arduino-cli upload -b arduino:mbed_nano:nano33ble -p /dev/ttyACM0 firmware/MotionStream
```

Adjust the USB port from `arduino-cli board list`. The firmware starts without a USB serial connection. A charger or power bank can supply power. No changes to the car's GPIO wiring are needed for BLE.

## Sensor and estimation status

- Working: BMI270 accelerometer and gyroscope, nominal 100 Hz, ±4 g and ±2000 degrees/s. Native right-handed chip axes are used.
- This particular board's BMM150 magnetometer did not respond during diagnostics. No magnetic heading is used; yaw is relative and drifts.
- Other Sense sensors are not streamed or validated by this firmware.
- VQF estimates orientation with gyro-bias and rest detection. Calibration estimates stationary bias and gravity magnitude, not a full accelerometer scale/offset calibration.
- Position is experimental horizontal XY integration, with Z fixed to zero. Stabilize applies rest updates, a soft deadband and damping. It is not reliable odometry or a steering input.
- Sensor timestamps, rather than Bluetooth arrival times, drive fusion. Gaps over 50 ms clear velocity and skip the unknown interval while retaining calibration. Unknown motion cannot be reconstructed.

For future car integration, mount the Nano rigidly, record the chip-to-chassis axis transform, and compare timestamps and orientation against camera observations. Use a single BLE receiver and consume its `/api/state` or `/ws` stream locally. The current `q` is a display-relative quaternion in SciPy x/y/z/w order; do not treat calibration zero as an absolute chassis/world frame. Bluetooth retries can delay delivery, so add explicit sample-age handling before any control integration. Existing motor safety gates still apply.

## BLE protocol

Service `cfb00001-7c42-4cb8-9ac0-27d96c562e11` (MotionLab-Nano).

- Data notify/read: `cfb00002-7c42-4cb8-9ac0-27d96c562e11`.
- Packet: 20 bytes, little endian `<I6hI`: uint32 microsecond timestamp, accel XYZ int16, gyro XYZ int16, uint32 sequence. Timestamp and sequence wrap modulo 2^32.
- Scale: accel /8192 in g; gyro /16.384 in degrees/s.
- Cumulative ACK: `cfb00003-7c42-4cb8-9ac0-27d96c562e11`, uint32 last contiguous sequence, write without response.
- Firmware retains 256 packets (~2.5 seconds), sends a 32-packet window, and retries after 40 ms without ACK advancement. Host orders and deduplicates samples. Overflow is counted and resynchronized.
- The prototype has no BLE authentication. Firmware polls registers rather than using an acquisition FIFO, so retries cannot recover samples never captured.

## Record and replay

The Record test button writes raw timestamped CSVs to `recordings/` (gitignored). Remain still at the start for at least three seconds, move, and finish at rest.

```sh
uv run python replay.py recordings/NAME.csv
```

Replay uses the first 300 samples for calibration; UI actions are not recorded.

## Reliability experiment

Stop all BLE dashboard receivers before running:

```sh
uv run python analysis/ble_stress.py
```

The script performs three 20-second trials and overwrites `analysis/ble-stress.json`. The committed report is from the desktop and board on 2026-09-26, not the Raspberry Pi:

| Scenario | Acquisition rate | Unrecoverable sequence gaps |
|---|---:|---:|
| Normal | 98 Hz | 0 |
| Discard every tenth notification | 86 Hz | 0 |
| Pause ACKs for one second | 93 Hz | 0 |

This is application fault injection, not a radio-range test. Acquisition intervals reached 26–30 ms and host delivery pauses reached ~0.9 seconds. Zero missing sequences does not mean every physical sensor sample was captured or delivery was immediate. `ble-baseline.json` records the earlier unacknowledged implementation for comparison.

Three.js is vendored under its included MIT license in `static/vendor/THREE-LICENSE.txt`. Fusion: https://github.com/dlaidig/vqf .
