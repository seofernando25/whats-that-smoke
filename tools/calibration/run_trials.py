"""Short, supervised real-car trials through the existing ownership/watchdog API.

Requires explicit --floor-clear. Never accesses I2C directly or changes watchdog.
Raw samples use the Pi monotonic clock; do not mix with the laptop clock.
"""
import argparse
import asyncio
import json
import math
from pathlib import Path
import statistics
import time
from urllib.request import urlopen
import websockets
from commands import vector, combinations, wheel_duties


def read_json(url):
    with urlopen(url, timeout=2) as response:
        return json.load(response)


async def run(args):
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    base=args.url.rstrip('/')
    initial=await asyncio.to_thread(read_json,base+'/api/state')
    s=initial['state']
    if s['armed'] or s['owner'] or s['aruco_follow'] or s['parking']['active']:
        raise RuntimeError('Robot must be stopped, disarmed, unowned, with follow/parking off')
    if not s['imu_connected'] or s['imu_age_ms'] is None or s['imu_age_ms']>100:
        raise RuntimeError('Fresh IMU required')
    start=await asyncio.to_thread(read_json,base+'/api/dynamics?after=0')
    seq=start['latest_seq']
    raw=open(out/'samples.jsonl','w',buffering=1)
    events=open(out/'events.jsonl','w',buffering=1)
    manifest=dict(clock='Pi monotonic seconds',schema_version=1,started_utc=time.time(),
        operator='clear floor and supervised; all trials authorized',surface='unspecified',
        measured_mass_kg=.5774,initial_state=initial,combinations=combinations(),
        translational_scale='unverified: no externally measured distance',status='running')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    samples=[]; state={};errors=[];live=True;dropped=0
    async with websockets.connect(base.replace('http','ws',1)+'/ws',open_timeout=5) as ws:
        async def receive():
            async for message in ws:
                p=json.loads(message)
                if p.get('type')=='error':errors.append(p['error'])
                if p.get('type')=='state':state.clear();state.update(p['state'])
        async def collect():
            nonlocal seq,dropped
            while live:
                p=await asyncio.to_thread(read_json,base+f'/api/dynamics?after={seq}')
                if p['latest_seq']<seq:raise RuntimeError('Dashboard restarted during capture')
                dropped+=p['dropped']
                if dropped:raise RuntimeError('Raw IMU buffer overflow')
                for sample in p['samples']:
                    raw.write(json.dumps(sample,separators=(',',':'))+'\n'); samples.append(sample)
                seq=p['latest_seq']
                await asyncio.sleep(.08)
        receiver=asyncio.create_task(receive()); collector=asyncio.create_task(collect())
        async def send(p):await ws.send(json.dumps(p))
        def check(moving=False):
            if errors:raise RuntimeError(errors[-1])
            if collector.done():collector.result();raise RuntimeError('Telemetry collector stopped')
            if receiver.done():receiver.result();raise RuntimeError('Control connection ended')
            if not samples or time.monotonic()-samples[-1]['t']>.18:
                raise RuntimeError('IMU stale')
            if moving and (not state.get('armed') or not state.get('you_are_owner')):
                raise RuntimeError('Control ownership lost')
            if max(abs(v) for v in samples[-1]['gyro_dps'])>240:
                raise RuntimeError('Angular-rate limit exceeded')
        async def rest(seconds):
            until=time.monotonic()+seconds
            while time.monotonic()<until:
                await send(dict(type='heartbeat'));await asyncio.sleep(.08)
                check()
        def mark(kind,**values):
            event=dict(event=kind,t=time.monotonic(),**values)
            events.write(json.dumps(event)+'\n');return event['t']
        def quiet():
            recent=[p for p in samples if p['t']>time.monotonic()-.6]
            if len(recent)<40:return False
            return all(statistics.pstdev(p['gyro_dps'][i] for p in recent)<1.5
                       and statistics.pstdev(p['accel_g'][i] for p in recent)<.035 for i in range(3))
        try:
            await asyncio.sleep(.4)
            mark('baseline_start');await rest(12);mark('baseline_end')
            if not quiet():raise RuntimeError('Robot not resting steadily')
            await send(dict(type='arm'));await asyncio.sleep(.2);check(True)
            # Pair each command with its inverse to limit accumulated displacement.
            pairs=['W','S','Q','E','WA','SD','WD','SA']
            plan=[(k,600,0) for k in pairs]
            for pwm in (800,1300,1800):
                for repeat in range(3):plan.extend((k,pwm,repeat) for k in pairs)
            plan.extend((k,1300,0) for k in ('A','D','WQ','SE','WE','SQ'))
            if args.long:
                plan=[(k,1800,r) for r in range(3) for k in pairs]
                plan.extend((k,1800,0) for k in ('A','D','WQ','SE','WE','SQ'))
            if args.probe:plan=plan[:8]
            for i,(keys,pwm,repeat) in enumerate(plan):
                if i<args.start_index:continue
                await rest(.8)
                until_quiet=time.monotonic()+10
                while not quiet() and time.monotonic()<until_quiet:await rest(.5)
                if not quiet():raise RuntimeError('No stable pre-trial baseline after 10 seconds')
                f,t=vector(keys)
                duration=2.0 if args.long else (.60 if pwm<=800 else .45)
                mark('trial_start',trial=i,keys=keys,pwm=pwm,repeat=repeat,
                     duration_s=duration,expected_wheels=wheel_duties(keys,pwm))
                until=time.monotonic()+duration
                while time.monotonic()<until:
                    check(True)
                    await send(dict(type='drive',forward=f,turn=t,strafe=0,speed_limit=pwm))
                    await asyncio.sleep(.06)
                await send(dict(type='drive',forward=0,turn=0,strafe=0,speed_limit=pwm))
                mark('drive_end',trial=i)
                await rest(2.4 if args.long else 1.2)
                mark('trial_end',trial=i)
                print(f'{i+1}/{len(plan)} {keys} PWM{pwm} stopped',flush=True)
            await send(dict(type='stop',reason='disarm'))
            mark('final_baseline_start');await rest(12);mark('final_baseline_end')
            manifest['status']='complete'
        except BaseException as error:
            manifest['status']='aborted';manifest['error']=repr(error);raise
        finally:
            try:await send(dict(type='stop',reason='disarm'))
            except Exception:pass
            live=False
            receiver.cancel()
            try:await collector
            except Exception as error:manifest['collector_error']=repr(error)
            raw.close();events.close()
            manifest.update(sample_count=len(samples),dropped_samples=dropped,ended_utc=time.time())
            (out/'manifest.json').write_text(json.dumps(manifest,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:8765')
    p.add_argument('--output',required=True);p.add_argument('--floor-clear',action='store_true')
    p.add_argument('--probe',action='store_true');p.add_argument('--long',action='store_true')
    p.add_argument('--start-index',type=int,default=0)
    args=p.parse_args()
    if not args.floor_clear:p.error('--floor-clear required: supervised, clear floor')
    asyncio.run(run(args))
