"""Summarize continuous floor laps without claiming inertial ground truth position."""
import argparse
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]

def analyze(folder):
    folder=Path(folder)
    samples=[json.loads(line) for line in (folder/'samples.jsonl').read_text().splitlines()]
    events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
    t=np.array([s['t'] for s in samples]);acc=np.array([s['accel_g'] for s in samples]);gyro=np.array([s['gyro_dps'] for s in samples])
    start=next(e['t'] for e in events if e['event']=='phase_start')
    end=max(e['t'] for e in events if e['event']=='phase_end')
    before=(t>start-5)&(t<start-.1);after=(t>end+2)&(t<end+10)
    g=np.median(acc[before],axis=0);up=g/np.linalg.norm(g)
    bias0=np.median(gyro[before],axis=0);bias1=np.median(gyro[after],axis=0)
    fraction=np.clip((t-start)/(end-start),0,1)
    bias=bias0+(bias1-bias0)*fraction[:,None]
    w=(gyro-bias)@up
    profile=json.loads((ROOT/'simulation/gazebo/real_robot_profile.json').read_text())
    forward=np.array(profile['sensor']['body_forward_axis_sensor']);forward-=np.dot(forward,up)*up;forward/=np.linalg.norm(forward)
    left=np.cross(up,forward)
    ay=(acc-g)@left*9.80665
    phases=[]
    for i,event in enumerate(events):
        if event['event']!='phase_start':continue
        finish=next(e for e in events[i+1:] if e['event']=='phase_end')
        lo,hi=event['t'],finish['t'];mask=(t>=lo)&(t<=hi)
        active=t[mask];rate=w[mask]
        steady=(t>lo+min(.4,.5*(hi-lo)))&(t<hi-.15)
        row=dict(keys=event['keys'],leg=event['leg'],duration_s=hi-lo,
                 yaw_angle_deg=float(np.trapezoid(rate,active)),steady_yaw_dps=float(np.median(w[steady])))
        if event['keys']=='WD':
            # Steady planar acceleration a_left = omega*v_forward - omega^2*r_left.
            # Assumed IMU lever arm relative to estimated COM; no alpha term in steady window.
            omega=np.deg2rad(w[steady]);valid=np.abs(omega)>.7
            velocity=(ay[steady][valid]+omega[valid]**2*.002974)/omega[valid]
            row['conditional_forward_mps']=float(np.median(velocity))
            row['conditional_radius_m']=abs(row['conditional_forward_mps']/np.deg2rad(row['steady_yaw_dps']))
            row['sample_velocity_p20_p80_mps']=np.quantile(velocity,[.2,.8]).tolist()
        phases.append(row)
    result=dict(source_session=folder.name,phases=phases,gyro_bias_before_dps=bias0.tolist(),gyro_bias_after_dps=bias1.tolist(),
        method='Interpolate pre/post rest gyro bias, project onto measured gravity; integrate each continuous phase.',
        translation_calibrated=False,
        translation_limits='Conditional v=a_left/omega with flat rigid floor, fixed tilt, negligible steady lateral acceleration derivative and estimated IMU lever arm. Vibration and tilt contaminate acceleration; radius is a model-fitting hypothesis, not measured path or confidence interval.')
    (folder/'lap_analysis.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');args=p.parse_args()
    result=analyze(args.folder)
    print(json.dumps(result,indent=2))
