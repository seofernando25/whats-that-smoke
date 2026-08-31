from __future__ import annotations

import asyncio

from whats_that_smoke.web import RobotController


def drive_vector(forward: float, strafe: float, turn: float) -> dict[str, int]:
    controller = RobotController()
    controller.state.owner = "test"
    controller.state.armed = True
    written: dict[str, int] = {}
    controller._write = lambda wheels: written.update(wheels)  # type: ignore[method-assign]
    asyncio.run(controller.drive("test", forward, strafe, turn, 1000))
    return written


def test_mecanum_strafe_right_pattern() -> None:
    assert drive_vector(0, 1, 0) == {
        "front-left": 1000,
        "rear-left": -1000,
        "front-right": -1000,
        "rear-right": 1000,
    }


def test_rotate_left_pattern() -> None:
    assert drive_vector(0, 0, 1) == {
        "front-left": 1000,
        "rear-left": 1000,
        "front-right": -1000,
        "rear-right": -1000,
    }
