"""Live headless validation of Gazebo IMU -> recorded noise -> production heading."""
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time
import uuid

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
os.environ['GZ_PARTITION']='ugv-imu-check-'+uuid.uuid4().hex
from build_motor_plugin import plugin_environment
from gazebo_bridge import GazeboBridge
from gz.msgs.imu_pb2 import IMU
from gz.msgs.stringmsg_pb2 import StringMsg

bridge=GazeboBridge();raw=[];noisy=[];nano=[]
def imu_row(m):
    return dict(t=m.header.stamp.sec+m.header.stamp.nsec*1e-9,
        a=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],
        g=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],orientation=m.HasField('orientation'))
bridge.node.subscribe(IMU,'/model/freenove/imu/ideal',lambda m:raw.append(imu_row(m)))
bridge.node.subscribe(IMU,'/model/freenove/imu/noisy',lambda m:noisy.append(imu_row(m)))
bridge.node.subscribe(StringMsg,'/model/freenove/imu/nano',lambda m:nano.append(json.loads(m.data)))

def wait_for(predicate,timeout=20):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        state=bridge.status()
        if predicate(state):return state
        time.sleep(.03)
    raise AssertionError('IMU pipeline timeout: '+json.dumps(bridge.status()))

seq=0
samples=[]
def drive(keys,duration):
    global seq
    start=bridge.imu.snapshot()['t_sim_s'];deadline=time.monotonic()+10
    while bridge.imu.snapshot()['t_sim_s']-start<duration:
        assert time.monotonic()<deadline,'Simulation stalled during motion'
        seq+=1;bridge.control.drive('imu-validation',seq,keys,.12,1800);bridge.publish_command()
        samples.append(bridge.status()['imu']);time.sleep(.025)

log=(HERE/'.control-logs/imu-pipeline.log').open('w')
proc=subprocess.Popen(['gz','sim','-s','-r',str(HERE/'freenove_world.sdf')],env=plugin_environment(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    initial=wait_for(lambda s:s['imu'].get('heading_calibrated'))
    end=initial['imu']['t_sim_s']+2
    wait_for(lambda s:s['imu'].get('t_sim_s',0)>=end)
    rest=list(noisy[-150:]);assert len(rest)>=100
    gyro_mean=[statistics.mean(math.degrees(r['g'][i]) for r in rest) for i in range(3)]
    gyro_std=[statistics.pstdev(math.degrees(r['g'][i]) for r in rest) for i in range(3)]
    assert .7<gyro_mean[1]<.95,gyro_mean
    assert all(.025<s<.08 for s in gyro_std),gyro_std
    rate=(len(rest)-1)/(rest[-1]['t']-rest[0]['t']);assert 98<rate<102,rate
    assert not any(r['orientation'] for r in rest),'Noisy topic leaked perfect orientation'
    drive(['KeyW'],.6);drive(['KeyW','KeyD'],1.9)
    right=bridge.status()['imu'];assert 140<right['truth_heading_deg']<220,right
    bridge.control.stop();bridge.publish_command()
    wait_for(lambda s:s['imu'].get('heading_stationary'))
    drive(['KeyQ'],1.2)
    bridge.control.stop();bridge.publish_command()
    settled=wait_for(lambda s:s['imu'].get('heading_stationary'))
    errors=[abs(s['heading_error_deg']) for s in samples if s['heading_error_deg'] is not None]
    assert max(errors)<4,('Heading diverges from native orientation',max(errors))
    assert nano and nano[-1]['type']=='imu' and len(nano[-1]['a'])==len(nano[-1]['g'])==3
    # Stamps and values must agree between the Nano JSON and noisy SI topics.
    matching={round(r['t'],6):r for r in noisy}
    record=next(r for r in reversed(nano) if round(r['t_sim_s'],6) in matching)
    match=matching[round(record['t_sim_s'],6)]
    assert max(abs(record['g'][i]-math.degrees(match['g'][i])) for i in range(3))<1e-9
    bridge.action('pause');time.sleep(.3)
    stamp=bridge.imu.snapshot()['t_sim_s'];count=bridge.imu.snapshot()['sample_count']
    time.sleep(.4)
    assert bridge.imu.snapshot()['t_sim_s']==stamp and bridge.imu.snapshot()['sample_count']==count,'Pause advanced sensor time'
    bridge.action('reset');bridge.action('play')
    reset=wait_for(lambda s:s['imu'].get('epoch',0)>0 and s['imu'].get('heading_calibrated'))
    assert abs(reset['imu']['heading_error_deg'])<.1
    bridge.action('imu-zero');assert not bridge.imu.snapshot()['heading_calibrated']
    wait_for(lambda s:s['imu'].get('heading_calibrated'))
    report=dict(sensor_rate_hz=rate,rest_gyro_mean_dps=gyro_mean,rest_gyro_std_dps=gyro_std,
        max_heading_error_deg=max(errors),mean_heading_error_deg=statistics.mean(errors),
        received_ideal=len(raw),received_noisy=len(noisy),received_nano_json=len(nano),
        heading_code='src/ugv/heading.py RelativeHeading (unchanged)',
        checks=['W600ms -> WD steer right','Q pivot left','SI/Nano JSON equivalence','No truth orientation on noisy topic','Pause freezes simulation time','Reset auto-recalibrates','Manual heading zero'])
    (HERE/'imu_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));print('PASS: native 100Hz IMU, recorded noise and real heading pipeline')
finally:
    bridge.close();os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM)
    log.close()
