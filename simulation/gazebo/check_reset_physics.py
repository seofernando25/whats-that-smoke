"""Live regression for reset feedback, steering geometry and release braking."""
import argparse
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/calibration'))
from evaluate_gazebo import run

output=HERE/'.control-logs/reset-regression.json'
args=argparse.Namespace(profile=True,reset_between=True,keys='W,WA,Q,WA,WD,WQ',
    pwm=1800,duration=2.,output=str(output),left=1.475,right=1.475,
    slip=None,mu=None,brake_damping=None)
run(args)
report=json.loads(output.read_text())
for row in report['results']:
    key=row['keys'];m=row['metrics'];state=row['motor_state_at_end']
    assert any(abs(state[i])>1 for i in (0,4,8,12)),f'{key}: wheel-speed feedback lost after reset'
    if key=='W':
        assert m['body_forward_mps']>.4,'Forward motion failed'
        assert m['stop_time_to_002_mps_s'] is not None and m['stop_time_to_002_mps_s']<.18,'Release braking too slow'
        assert m['stopping_path_m']<.06,'Forward release coasts too far'
    elif key=='Q':
        assert row['yaw_dps']>100,'Pivot must turn decisively'
        assert abs(m['body_forward_mps'])<.05,'Pivot must stay near chassis centre'
        assert abs(m['instantaneous_center_body_xy_m'][1])<.025,'Pivot centre moved away from chassis'
    else:
        assert abs(row['yaw_dps'])>55,'Combined steering is almost straight'
        assert m['body_forward_mps']>.05,'Combined steering must also translate'
        assert .04<abs(m['instantaneous_center_body_xy_m'][1])<.16,'Incorrect steering radius'
        inside,outer=(0,4) if key!='WD' else (4,0)
        assert abs(state[inside+3])<1e-9,'Inside wheels must receive zero-drive brake command'
        assert abs(state[inside])<.30*abs(state[outer]),'Inside wheels are freewheeling instead of braking'
        assert m['stopping_path_m']<.015,'Arc keeps sliding after release'
print('PASS: six resets preserve feedback; W brakes promptly; Q pivots; W+A/W+D/W+Q follow tight translating arcs')
