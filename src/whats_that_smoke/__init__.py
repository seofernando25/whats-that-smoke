from __future__ import annotations

import argparse
import subprocess
import time
from datetime import datetime
from pathlib import Path


SERVO_CHANNELS = {"pan": 8, "tilt": 9}
WHEEL_CHANNELS = {
    "front-left": (0, 1),
    "rear-left": (3, 2),
    "front-right": (6, 7),
    "rear-right": (4, 5),
}


def snapshot(output: Path | None) -> int:
    target = output or Path("snapshots") / f"snapshot-{datetime.now():%Y%m%d-%H%M%S}.jpg"
    target.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "rpicam-still",
            "--nopreview",
            "--timeout",
            "500",
            "--rotation",
            "180",
            "--output",
            str(target),
        ],
        check=True,
    )
    print(target.resolve())
    return 0


def _write(bus: object, address: int, register: int, value: int) -> None:
    bus.write_byte_data(address, register, value)  # type: ignore[attr-defined]


def _configure_pca9685(bus: object, address: int = 0x40, frequency: int = 50) -> None:
    mode1 = bus.read_byte_data(address, 0x00)  # type: ignore[attr-defined]
    awake_mode = mode1 & ~0x10
    prescale = round(25_000_000 / (4096 * frequency) - 1)
    _write(bus, address, 0x00, (awake_mode & 0x7F) | 0x10)
    _write(bus, address, 0xFE, prescale)
    _write(bus, address, 0x00, awake_mode)
    time.sleep(0.005)
    _write(bus, address, 0x00, awake_mode | 0xA0)


def _set_pulse(bus: object, channel: int, pulse_us: int, address: int = 0x40) -> None:
    count = round(pulse_us * 4096 / 20_000)
    base = 0x06 + 4 * channel
    _write(bus, address, base, 0)
    _write(bus, address, base + 1, 0)
    _write(bus, address, base + 2, count & 0xFF)
    _write(bus, address, base + 3, count >> 8)


def _release(bus: object, channel: int, address: int = 0x40) -> None:
    base = 0x06 + 4 * channel
    _write(bus, address, base + 3, 0x10)  # full-off; selected channel only


def _set_pwm(bus: object, channel: int, duty: int, address: int = 0x40) -> None:
    base = 0x06 + 4 * channel
    _write(bus, address, base, 0)
    _write(bus, address, base + 1, 0)
    _write(bus, address, base + 2, duty & 0xFF)
    _write(bus, address, base + 3, duty >> 8)


def _stop_wheels(bus: object) -> None:
    for channel in range(8):
        _set_pwm(bus, channel, 4095)


def servo_test(axis: str, delta_us: int, dwell: float) -> int:
    bus_path = Path("/dev/i2c-1")
    if not bus_path.exists():
        raise SystemExit("refused: /dev/i2c-1 absent; enable GPIO-header I2C first")

    from smbus2 import SMBus

    channel = SERVO_CHANNELS[axis]
    center = 1500
    with SMBus(1) as bus:
        try:
            bus.read_byte_data(0x40, 0x00)
        except OSError as exc:
            raise SystemExit("refused: PCA9685@0x40 not detected") from exc
        _configure_pca9685(bus)
        try:
            for pulse in (center, center - delta_us, center + delta_us, center):
                _set_pulse(bus, channel, pulse)
                time.sleep(dwell)
        finally:
            _release(bus, channel)
    print(f"ok axis={axis} pca_channel={channel} released=true")
    return 0


def wheel_test(wheel: str, duty: int, dwell: float) -> int:
    if not Path("/dev/i2c-1").exists():
        raise SystemExit("refused: /dev/i2c-1 absent")

    from smbus2 import SMBus

    targets = list(WHEEL_CHANNELS) if wheel == "all" else [wheel]
    with SMBus(1) as bus:
        try:
            bus.read_byte_data(0x40, 0x00)
        except OSError as exc:
            raise SystemExit("refused: PCA9685@0x40 not detected") from exc
        _configure_pca9685(bus)
        _stop_wheels(bus)
        try:
            for name in targets:
                reverse_channel, forward_channel = WHEEL_CHANNELS[name]
                _set_pwm(bus, reverse_channel, 0)
                _set_pwm(bus, forward_channel, duty)
                print(f"nudge wheel={name} duty={duty} dwell={dwell:.2f}s", flush=True)
                time.sleep(dwell)
                _stop_wheels(bus)
                time.sleep(0.25)
        finally:
            _stop_wheels(bus)
    print("ok all_wheels=stopped")
    return 0


def _ws2812_spi_frame(pixels: list[tuple[int, int, int]]) -> list[int]:
    encoded: list[int] = []
    for red, green, blue in pixels:
        for value in (green, red, blue):  # Freenove connector V2 sequence=GRB
            for bit in range(7, -1, -1):
                encoded.append(0xF8 if value & (1 << bit) else 0x80)
    return encoded


def led_test(brightness: int, dwell: float) -> int:
    if not Path("/dev/spidev0.0").exists():
        raise SystemExit("refused: /dev/spidev0.0 absent; enable SPI first")

    import spidev

    off = [(0, 0, 0)] * 8
    colors = {"red": (brightness, 0, 0), "green": (0, brightness, 0), "blue": (0, 0, brightness)}
    spi = spidev.SpiDev()
    spi.open(0, 0)
    spi.mode = 0
    spi.max_speed_hz = 6_400_000
    try:
        spi.xfer3(_ws2812_spi_frame(off))
        time.sleep(0.001)
        for index in range(8):
            for name, color in colors.items():
                pixels = off.copy()
                pixels[index] = color
                spi.xfer3(_ws2812_spi_frame(pixels))
                print(f"light led={index + 1} color={name}", flush=True)
                time.sleep(dwell)
                spi.xfer3(_ws2812_spi_frame(off))
                time.sleep(0.10)
    finally:
        spi.xfer3(_ws2812_spi_frame(off))
        time.sleep(0.001)
        spi.close()
    print("ok leds=off")
    return 0


def led_test_gpio18(brightness: int, dwell: float) -> int:
    import os

    if os.geteuid() != 0:
        raise SystemExit("refused: GPIO18 WS2812 driver requires root; use sudo .venv/bin/wts led-test-gpio18")

    from rpi_ws281x import Color, PixelStrip

    strip = PixelStrip(8, 18, 800_000, 10, False, brightness, 0)
    strip.begin()
    colors = {"red": Color(255, 0, 0), "green": Color(0, 255, 0), "blue": Color(0, 0, 255)}

    def all_off() -> None:
        for pixel in range(8):
            strip.setPixelColor(pixel, Color(0, 0, 0))
        strip.show()

    try:
        all_off()
        for index in range(8):
            for name, color in colors.items():
                strip.setPixelColor(index, color)
                strip.show()
                print(f"light-gpio18 led={index + 1} color={name}", flush=True)
                time.sleep(dwell)
                strip.setPixelColor(index, Color(0, 0, 0))
                strip.show()
                time.sleep(0.10)
    finally:
        all_off()
    print("ok gpio18-leds=off")
    return 0


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(prog="wts")
    commands = root.add_subparsers(dest="command", required=True)

    snap = commands.add_parser("snapshot", help="capture a rotation-corrected JPEG")
    snap.add_argument("--output", type=Path)

    servo = commands.add_parser("servo-test", help="briefly nudge one head servo")
    servo.add_argument("axis", choices=SERVO_CHANNELS)
    servo.add_argument("--delta-us", type=int, default=100, metavar="25..200")
    servo.add_argument("--dwell", type=float, default=0.15)

    wheel = commands.add_parser("wheel-test", help="briefly nudge wheels one by one")
    wheel.add_argument("wheel", nargs="?", choices=["all", *WHEEL_CHANNELS], default="all")
    wheel.add_argument("--duty", type=int, default=900, metavar="500..1500")
    wheel.add_argument("--dwell", type=float, default=0.20)

    led = commands.add_parser("led-test", help="test each RGB pixel and component")
    led.add_argument("--brightness", type=int, default=32, metavar="1..96")
    led.add_argument("--dwell", type=float, default=0.25)

    led18 = commands.add_parser("led-test-gpio18", help="test RGB pixels via legacy GPIO18")
    led18.add_argument("--brightness", type=int, default=32, metavar="1..96")
    led18.add_argument("--dwell", type=float, default=0.25)
    return root


def main() -> int:
    args = parser().parse_args()
    if args.command == "snapshot":
        return snapshot(args.output)
    if args.command == "wheel-test":
        if not 500 <= args.duty <= 1500:
            raise SystemExit("--duty must be within 500..1500")
        if not 0.05 <= args.dwell <= 0.5:
            raise SystemExit("--dwell must be within 0.05..0.5 seconds")
        return wheel_test(args.wheel, args.duty, args.dwell)
    if args.command == "led-test":
        if not 1 <= args.brightness <= 96:
            raise SystemExit("--brightness must be within 1..96")
        if not 0.10 <= args.dwell <= 1.0:
            raise SystemExit("--dwell must be within 0.10..1.0 seconds")
        return led_test(args.brightness, args.dwell)
    if args.command == "led-test-gpio18":
        if not 1 <= args.brightness <= 96:
            raise SystemExit("--brightness must be within 1..96")
        if not 0.10 <= args.dwell <= 1.0:
            raise SystemExit("--dwell must be within 0.10..1.0 seconds")
        return led_test_gpio18(args.brightness, args.dwell)
    if not 25 <= args.delta_us <= 200:
        raise SystemExit("--delta-us must be within 25..200 microseconds")
    if not 0.05 <= args.dwell <= 0.5:
        raise SystemExit("--dwell must be within 0.05..0.5 seconds")
    return servo_test(args.axis, args.delta_us, args.dwell)
