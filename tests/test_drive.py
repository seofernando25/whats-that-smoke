from __future__ import annotations

import asyncio
import math
import time
from unittest.mock import patch
from ugv.motion import GyroMotion, MotionFault, angle_error, ramp_wheels

from ugv.web import RobotController


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


def mock_controller():
    controller = RobotController()
    controller.state.owner = "test"
    controller.state.armed = True
    controller._write = lambda wheels: None
    return controller


def test_direct_turn_without_gyro():
    controller = mock_controller()
    asyncio.run(controller.drive("test", 0, 0, 1, 1000))
    assert not controller.state.imu_connected
    assert controller.state.wheels["front-left"] == 1000
    assert controller.state.wheels["front-right"] == -1000
    assert controller.motion.mode is None


def test_watchdog_brakes_without_disarming():
    controller = mock_controller()
    asyncio.run(controller.drive("test", 1, 0, 0, 1000))
    controller._guard_check(controller.last_drive + 0.7)
    assert controller.state.stopped
    assert not any(controller.state.wheels.values())
    assert controller.state.armed and controller.state.owner == "test"
    asyncio.run(controller.drive("test", -1, 0, 0, 1000))
    assert controller.state.wheels["front-left"] == 1000


def test_key_release_and_stop_preserve_arm():
    controller = mock_controller()
    asyncio.run(controller.drive("test", 1, 0, 0, 1000))
    asyncio.run(controller.drive("test", 0, 0, 0, 1000))
    assert controller.state.stopped and controller.state.armed
    asyncio.run(controller.stop("client-stop"))
    assert controller.state.armed and controller.state.owner == "test"
    asyncio.run(controller.stop("client-stop", True))
    assert not controller.state.armed and controller.state.owner is None


def test_disconnect_releases_session_and_brakes():
    controller = mock_controller()
    asyncio.run(controller.drive("test", 1, 0, 0, 1000))
    asyncio.run(controller.disconnect("test"))
    assert controller.state.stopped and not controller.state.armed
    assert controller.state.owner is None


def test_sidestep_is_disabled():
    controller = mock_controller()
    try:
        asyncio.run(controller.sidestep("test", 1, 1000))
    except PermissionError:
        pass
    else:
        raise AssertionError("side-step must be disabled")
    assert controller.state.stopped


def test_rate_loop_converges_with_both_turn_directions():
    for direction in (-1, 1):
        motion = GyroMotion()
        yaw = rate = 0.0
        for tick in range(200):
            now = tick * .02
            if tick % 5 == 0: motion.drive(0, direction, 1800, now)
            out = motion.step(now, yaw, rate, True)
            assert out.limit <= 1800 and abs(out.turn) <= 1
            rate += (-out.turn * 65 - rate) * .02 / .15
            yaw = (yaw + rate * .02) % 360
        assert abs(rate + direction * motion.TURN_RATE) < 4


def test_sidestep_uses_actual_heading_and_returns_to_origin():
    for direction in (-1, 1):
        motion = GyroMotion()
        yaw, rate, seen = 359.0, 0.0, set()
        for tick in range(500):
            now = tick * .02
            if tick % 5 == 0: motion.sidestep(direction, 1400, now, yaw)
            out = motion.step(now, yaw, rate, True)
            seen.add(motion.phase)
            rate += (-out.turn * 65 - rate) * .02 / .10
            yaw = (yaw + rate * .02) % 360
            if 4 in seen and motion.phase == 0:
                assert abs(angle_error(359, yaw)) < 2.5
                break
        else:
            raise AssertionError("sidestep never completed")
        assert seen == {0, 1, 2, 3, 4}


def test_expired_input_cannot_be_renewed_by_internal_steps():
    motion = GyroMotion(); motion.sidestep(1, 1000, 0, 0)
    for tick in range(22): motion.step(tick * .02, 0, 0, True)
    try: motion.step(.46, 0, 0, True)
    except MotionFault as error: assert str(error) == "input-expired"
    else: raise AssertionError("stale input remained active")


def test_missing_gyro_stall_and_wrong_direction_stop():
    for case, measured, ready, expected in (("missing", 0, False, "gyro-unavailable"),
                                           ("stuck", 0, True, "turn-no-response"),
                                           ("wrong", 15, True, "gyro-direction-mismatch")):
        motion = GyroMotion()
        try:
            for tick in range(150):
                now = tick * .02
                motion.drive(0, 1, 1800, now)
                motion.step(now, 0, measured, ready)
        except MotionFault as error: assert str(error) == expected, (case, str(error))
        else: raise AssertionError(case)


def test_pwm_ramp_bounds_and_reversal_crosses_zero():
    assert ramp_wheels({"a": 700}, {"a": -1000}, .02)["a"] == 630
    assert ramp_wheels({"a": 50}, {"a": -1000}, .02)["a"] == 0
    assert ramp_wheels({"a": 0}, {"a": -1000}, .02)["a"] == -70


def test_arc_never_reverses_inside_wheels_and_respects_slider():
    for forward in (-1, 1):
        for turn in (-1, 1):
            motion = GyroMotion()
            motion.drive(forward, turn, 1200, 0)
            out = motion.step(.02, 0, 0, True)
            assert out.limit == 1200
            assert abs(out.turn) <= .65
            wheels = drive_vector(out.forward, 0, out.turn)
            assert all(duty * forward < 0 for duty in wheels.values())


def test_pivot_breakaway_overcomes_dead_zone_without_unbounded_power():
    for direction in (-1, 1):
        motion = GyroMotion()
        rate = yaw = applied = 0.0
        for tick in range(120):
            now = tick * .02
            motion.drive(0, direction, 1800, now)
            out = motion.step(now, yaw, rate, True)
            applied = ramp_wheels({'wheel': applied}, {'wheel': out.turn * out.limit}, .02)['wheel']
            # Synthetic high-friction motor: no torque below 80% duty.
            effort = max(0, abs(applied) / 1800 - .80) / .20
            target = -math.copysign(effort * 80, applied)
            rate += (target - rate) * .02 / .10
            yaw += rate * .02
            assert abs(applied) <= 1800
        assert yaw * direction < -5, 'failed to break away'


def test_small_heading_error_still_gets_breakaway_power():
    motion = GyroMotion(); motion.sidestep(1, 1800, 0, 0)
    out = motion.step(.02, -9, 0, True)
    out = motion.step(.04, -9, 0, True)
    assert abs(out.turn) >= .9 and out.limit == 1800


def ready_controller():
    c = RobotController(); c.state.owner = "test"; c.state.armed = True
    c.state.imu_connected = c.state.heading_calibrated = True
    c.imu_sample_at = time.monotonic()
    c.bus = object()
    c._write = lambda wheels: None
    return c


def test_independent_watchdog_uses_browser_lease_even_when_motor_loop_fresh():
    c = ready_controller(); now = time.monotonic()
    c.motion.sidestep(1, 1200, now - .7, 0)
    c.last_operator_command = now - .7
    c.last_drive = now
    c.state.stopped = False
    with patch("ugv.web._stop_wheels") as brake:
        c._guard_check(now)
        brake.assert_called_once()
    assert c.state.armed and c.state.reason == "input-watchdog"
    assert c.motion.mode is None


def test_foreign_command_cannot_cancel_motion_and_disarmed_loop_cannot_write():
    async def scenario():
        c = ready_controller(); c.motion.sidestep(1, 1200, time.monotonic(), 0)
        try: await c.drive("other", 0, 0, 0, 1200)
        except PermissionError: pass
        else: raise AssertionError("ownership not enforced")
        assert c.motion.mode == "sidestep"
        c.state.armed = False
        with patch.object(c, "_write") as write:
            c._apply_vector(0, 1, 1200)
            write.assert_not_called()
    asyncio.run(scenario())
