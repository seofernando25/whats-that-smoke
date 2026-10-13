import asyncio
import math
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

import pytest

from whats_that_smoke.aruco import ArucoFollower, Detection, FollowSteering
from whats_that_smoke.web import RobotController, RobotState


@pytest.mark.parametrize('error,turn', [(-.2, 1), (.2, -1)])
def test_close_tag_gets_full_qe_pivot_with_no_forward_motion(error, turn):
    command = FollowSteering().step(error, .24, 0)
    assert command.forward == 0 and command.turn == turn
    assert .04 <= command.duration <= .18


def test_centered_close_tag_brakes_and_distant_tag_approaches():
    steering = FollowSteering()
    assert steering.step(0, .24, 0).reason == 'aruco-target-reached'
    command = steering.step(0, .6, 1)
    assert command.forward > 0 and command.turn == 0 and command.duration is None


def test_pulse_is_not_extended_and_waits_for_post_pulse_image():
    steering = FollowSteering()
    command = steering.step(.3, .25, 0)
    assert steering.step(.3, .25, .02) is None
    assert steering.step(.3, .25, command.duration + .05).turn == 0
    assert steering.step(.3, .25, 1, captured_at=.01).turn == 0
    assert steering.step(.3, .25, 1, captured_at=1).turn == -1


def test_no_visual_response_increases_next_pulse_duration():
    steering = FollowSteering()
    first = steering.step(.12, .24, 0)
    second = steering.step(.12, .24, 1)
    assert second.duration > first.duration
    assert second.turn == -1


def test_pivot_deadline_brakes_independently_of_camera_processing():
    controller = RobotController()
    controller.state.owner = 'test'; controller.state.armed = True
    controller._write = lambda wheels: None
    asyncio.run(controller.drive('test', 0, 0, 1, 1800, autonomous=True, duration=.1))
    assert controller.state.wheels['front-left'] == 1800
    assert controller.state.wheels['front-right'] == -1800
    deadline = controller.drive_deadline
    controller._guard_check(deadline - .01)
    assert not controller.state.stopped
    controller._guard_check(deadline)
    assert controller.state.stopped and not any(controller.state.wheels.values())
    assert controller.state.armed and controller.state.owner == 'test'
    assert controller.drive_deadline is None


@pytest.mark.parametrize('rate', [1.2, 2., 3.])
def test_visual_pivots_center_a_high_friction_plant_both_directions(rate):
    for initial in (-.75, -.35, -.12, .12, .35, .75):
        steering = FollowSteering()
        error = initial; velocity = 0.; power = 0.; deadline = 0.
        errors = []; max_power = 0.
        for tick in range(800):
            now = tick * .01
            if tick % 10 == 0:
                command = steering.step(error, .24, now)
                if command is not None:
                    assert command.forward == 0
                    power = command.turn
                    deadline = now + command.duration if command.duration else now
                    max_power = max(max_power, abs(power))
            if now >= deadline:
                power = 0.
            # Motors produce no torque below 80% selected PWM.
            effort = math.copysign(max(0., abs(power) - .8) / .2, power)
            velocity += (effort * rate - velocity) * .01 / .08
            error += velocity * .01
            if tick > 700:
                errors.append(abs(error))
        assert max_power == 1
        assert max(errors) <= steering.CENTER_START, (initial, rate, error)


def test_follow_speed_is_clamped_and_only_its_owner_can_change_it():
    async def scenario():
        controller = SimpleNamespace(state=RobotState(owner='test', armed=True),
                                     stop=AsyncMock(), broadcast=AsyncMock())
        follower = ArucoFollower(None, controller)
        await follower.configure('test', True, True, 1800)
        assert follower.follow_speed_limit == 1800
        await follower.configure('test', True, speed_limit=1100)
        assert follower.follow_speed_limit == 1100
        with pytest.raises(PermissionError):
            await follower.configure('other', True, speed_limit=1800)
        await follower.configure('test', True, speed_limit=9999)
        assert follower.follow_speed_limit == 1800
    asyncio.run(scenario())


def test_follow_loop_uses_selected_speed_and_finite_close_pivot():
    async def scenario():
        state = RobotState(owner='test', armed=True)
        controller = SimpleNamespace(state=state, stop=AsyncMock(), drive=AsyncMock(), broadcast=AsyncMock())
        sequence = 0
        def sample():
            nonlocal sequence
            sequence += 1
            return b'frame', sequence, time.monotonic()
        follower = ArucoFollower(SimpleNamespace(latest_sample=sample), controller)
        follower.enabled = follower.follow = True; follower.owner = 'test'
        follower.follow_speed_limit = 1600
        detection = Detection(0, [[1., 1.]] * 4, 512., 300., 200., .24)
        with patch.object(follower, 'track_all', return_value=[detection]):
            task = asyncio.create_task(follower.run())
            try:
                await asyncio.sleep(.09)
                args, kwargs = controller.drive.call_args_list[0]
                assert args == ('test', 0., 0., 1., 1600)
                assert kwargs['autonomous'] and .04 <= kwargs['duration'] <= .18
            finally:
                task.cancel()
                try: await task
                except asyncio.CancelledError: pass
    asyncio.run(scenario())


def test_stop_turns_follow_off_without_disarming_connected_owner():
    import json
    from fastapi import WebSocketDisconnect
    from whats_that_smoke import web
    async def scenario():
        controller = RobotController()
        controller.state.owner = 'testabcd'; controller.state.armed = True
        follower = ArucoFollower(None, controller)
        follower.enabled = follower.follow = True; follower.owner = 'testabcd'
        websocket = SimpleNamespace(accept=AsyncMock(), send_json=AsyncMock())
        calls = 0
        async def receive():
            nonlocal calls
            calls += 1
            if calls == 1:
                return json.dumps({'type':'stop'})
            assert not follower.follow
            assert controller.state.stopped and controller.state.armed
            assert controller.state.owner == 'testabcd'
            raise WebSocketDisconnect()
        websocket.receive_text = receive
        with patch.object(web, 'controller', controller), patch.object(web, 'aruco', follower), patch.object(web.uuid, 'uuid4', return_value=SimpleNamespace(hex='testabcd1234')):
            await web.websocket_endpoint(websocket)
        assert not follower.follow
    asyncio.run(scenario())
