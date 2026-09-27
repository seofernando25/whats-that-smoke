"""One bounded physical arc trial. Floor/clearance confirmation required before use.

Run on Pi; normal WS owner, gyro safety and motor watchdog remain active.
No autonomous retries; stops/disarms in finally. Does not establish travel clearance.
"""
import argparse
import asyncio
import json
import statistics
import time
import websockets


async def trial(args):
    async with websockets.connect('ws://127.0.0.1:8765/ws') as ws:
        state = json.loads(await asyncio.wait_for(ws.recv(), 1))['state']
        assert state['stopped'] and not state['armed'], 'controller busy'
        assert state['heading_calibrated'] and state['imu_age_ms'] < 100
        assert state['camera_frame_age_ms'] is not None and state['camera_frame_age_ms'] < 300
        origin = state['heading_deg']
        samples = []
        async def receive():
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get('type') == 'error': raise RuntimeError(msg)
                s = msg['state']
                samples.append((time.monotonic(), s))
        reader = asyncio.create_task(receive())
        started = None
        try:
            await ws.send(json.dumps({'type':'arm'}))
            await asyncio.sleep(.08)
            started = time.monotonic()
            while time.monotonic() - started < args.seconds:
                if reader.done(): await reader
                assert samples and time.monotonic() - samples[-1][0] < .2, 'state stale'
                s = samples[-1][1]
                assert s['armed'] and s['you_are_owner'], s['reason']
                assert s['imu_connected'] and s['imu_age_ms'] < 100, 'IMU stale'
                assert s['camera_frame_age_ms'] < 300, 'camera stale'
                assert abs(s['imu_accel_g']['z']) > .8, 'tilt/acceleration abort'
                assert abs((s['heading_deg'] - origin + 180) % 360 - 180) < 25, 'angle limit'
                await ws.send(json.dumps({'type':'drive','forward':args.forward,'turn':args.turn,'speed_limit':args.limit}))
                await asyncio.sleep(.07)
        finally:
            await ws.send(json.dumps({'type':'stop'}))
            await asyncio.sleep(.15)
            reader.cancel()
            try: await reader
            except asyncio.CancelledError: pass
        assert samples[-1][1]['stopped'] and not samples[-1][1]['armed'], 'stop not confirmed'
        active = [s for t,s in samples if started and t >= started and not s['stopped']]
        print(json.dumps({'args':vars(args), 'samples':len(active),
            'yaw_delta':round((samples[-1][1]['heading_deg']-origin+180)%360-180,2),
            'median_rate':round(statistics.median(s['heading_rate_dps'] for s in active),2) if active else None,
            'peak_rate':round(max((abs(s['heading_rate_dps']) for s in active),default=0),2),
            'last_wheels':active[-1]['wheels'] if active else {}, 'stopped':True}))


if __name__ == '__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--forward',type=int,choices=(-1,1),default=1)
    p.add_argument('--turn',type=int,choices=(-1,1),default=1)
    p.add_argument('--limit',type=int,default=1500)
    p.add_argument('--seconds',type=float,default=.6)
    a=p.parse_args()
    assert 500 <= a.limit <= 1800 and 0 < a.seconds <= .8
    asyncio.run(trial(a))
