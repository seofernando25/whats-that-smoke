"""Replay six continuous W600ms -> WD180 floor command intervals, without resets."""
import argparse
import json
from pathlib import Path
import statistics
import sys

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/calibration'))
from evaluate_gazebo import run

reference=json.loads((HERE/'lap_reference.json').read_text())
output=HERE/'.control-logs/lap-regression.json'
args=argparse.Namespace(profile=True,reset_between=False,keys='',pwm=1800,duration=2.,
    output=str(output),left=1.475,right=1.475,slip=None,mu=None,brake_damping=None,
    lap_events=str(HERE/'lap_reference.json'))
run(args)
report=json.loads(output.read_text())
errors=[];straight_errors=[]
for real,sim in zip(reference['phases'],report['phases'],strict=True):
    assert real['keys']==sim['keys'] and real['leg']==sim['leg']
    assert abs(sim['sim_duration_s']-real['duration_s'])<.035,'Replay command interval changed'
    error=abs(sim['yaw_angle_deg']-real['yaw_angle_deg'])
    if real['keys']=='WD':
        errors.append(error)
        assert error<20,f"Leg {real['leg']}: half-turn differs by {error:.1f} degrees"
        assert 85<abs(sim['steady_yaw_dps'])<115,'Wrong full-power steering rate'
        # Conditional inertial estimate guides this bracket; no measured real path.
        assert .11<sim['radius_m']<.19,'Steering reverted to overly tight or near-straight arc'
    elif real['leg']>0:
        straight_errors.append(error)
        assert -17<sim['yaw_angle_deg']<-3,'Lost turn-exit response while W stays held'
assert statistics.mean(errors)<10,'Average continuous half-turn error too large'
assert statistics.mean(straight_errors)<4,'Turn-exit rotation differs from floor sequence'
assert report['stopping_path_m']<.025,'Still coasting after releasing the arc'
summary=dict(source_session=reference['source_session'],translation_calibrated=False,
    half_turn_mean_abs_error_deg=statistics.mean(errors),half_turn_max_abs_error_deg=max(errors),
    turn_exit_mean_abs_error_deg=statistics.mean(straight_errors),stopping_path_m=report['stopping_path_m'],
    phases=report['phases'])
(HERE/'lap_validation.json').write_text(json.dumps(summary,indent=2)+'\n')
print('PASS: six continuous half-turns, entry/exit response, wider translating arcs and final release')
print(json.dumps({k:v for k,v in summary.items() if k!='phases'},indent=2))
