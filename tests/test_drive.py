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


def test_forward_uses_classical_four_wheel_drive() -> None:
    assert drive_vector(1, 0, 0) == {
        "front-left": -1000,
        "rear-left": -1000,
        "front-right": -1000,
        "rear-right": -1000,
    }


def test_rotate_left_pattern() -> None:
    assert drive_vector(0, 0, 1) == {
        "front-left": 1000,
        "rear-left": 1000,
        "front-right": -1000,
        "rear-right": -1000,
    }


def test_sidestep_cycle_restores_heading_and_longitude_commands() -> None:
    phases = RobotController.sidestep_phases(1)
    assert abs(sum(turn * duration for _, turn, duration in phases)) < 1e-9
    assert abs(sum(forward * duration for forward, _, duration in phases)) < 1e-9
