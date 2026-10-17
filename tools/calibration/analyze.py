"""Local drift-bracketed response estimates; linear IMU integration is provisional.

Requires numpy. No motor torque/friction is claimed identifiable from IMU alone.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import numpy as np


def median_vectors(rows, field):
    return np.median([s[field] for s in rows],axis=0)


def integrate(values,t):
    v=np.asarray(values)
    return np.concatenate([np.zeros_like(v[:1]),np.cumsum((v[1:]+v[:-1])*.5*np.diff(t).reshape((-1,)+(1,)*(v.ndim-1)),axis=0)])


def first_order(t,rate):
    best=None
    for delay in np.arange(0,.151,.01):
        for tau in np.arange(.02,.501,.01):
            shape=1-np.exp(-np.maximum(0,t-delay)/tau)
            gain=float(np.dot(rate,shape)/max(np.dot(shape,shape),1e-9))
            mse=float(np.mean((rate-gain*shape)**2))
            if best is None or mse<best['mse_dps2']:
                best=dict(steady_dps=gain,delay_s=float(delay),time_constant_s=float(tau),mse_dps2=mse)
    return best


def analyze(folder):
    folder=Path(folder)
    samples=[json.loads(l) for l in (folder/'samples.jsonl').read_text().splitlines()]
    events=[json.loads(l) for l in (folder/'events.jsonl').read_text().splitlines()]
    times=np.array([p['t'] for p in samples]); output=[]
    starts=[e for e in events if e['event']=='trial_start']
    # Identify the sensor's body-forward axis from forward/reverse launch acceleration.
    forward_candidates=[]
    for event in starts:
        if event['keys'] not in ('W','S') or event['pwm']<800:continue
        before=[s for s in samples if event['t']-.65<s['t']<event['t']-.1]
        launch=[s for s in samples if event['t']+.02<s['t']<event['t']+.16]
        if len(before)<30 or len(launch)<5:continue
        g=median_vectors(before,'accel_g');up=g/np.linalg.norm(g)
        a=np.mean([s['accel_g'] for s in launch],axis=0)-g
        a-=np.dot(a,up)*up
        forward_candidates.append(a*(1 if event['keys']=='W' else -1))
    f=np.median(forward_candidates,axis=0) if forward_candidates else np.array([1.,0,0])
    forward_signal_g=float(np.linalg.norm(f));f/=max(np.linalg.norm(f),1e-9)
    for event in starts:
        end=next((e for e in events if e['event']=='trial_end' and e['trial']==event['trial']),None)
        drive_end=next((e for e in events if e['event']=='drive_end' and e['trial']==event['trial']),None)
        if not end or not drive_end:continue
        pre=[s for s in samples if event['t']-.65<s['t']<event['t']-.1]
        post=[s for s in samples if end['t']-.5<s['t']<end['t']]
        rows=[s for s in samples if event['t']-.08<s['t']<end['t']]
        if min(len(pre),len(post))<30:continue
        b0=median_vectors(pre,'gyro_dps');b1=median_vectors(post,'gyro_dps')
        g0=median_vectors(pre,'accel_g');g1=median_vectors(post,'accel_g')
        up=g0/np.linalg.norm(g0);left=np.cross(up,f);left/=max(np.linalg.norm(left),1e-9)
        forward=np.cross(left,up)
        t=np.array([s['t'] for s in rows]);frac=np.clip((t-event['t'])/(end['t']-event['t']),0,1)
        biases=b0+(b1-b0)*frac[:,None]
        raw=np.array([s['gyro_dps'] for s in rows])
        yaw=(raw-biases)@up
        active=np.array([any(s['wheels'].values()) for s in rows])
        if not active.any():continue
        begin=t[np.where(active)[0][0]];finish=t[np.where(active)[0][-1]]+.01
        moving=(t>=begin)&(t<finish)
        plateau=(t>begin+.65*(finish-begin))&moving
        steady=float(np.median(yaw[plateau]))
        fitted=first_order(t[moving]-begin,yaw[moving]) if abs(steady)>3 else None
        gravity=g0+(g1-g0)*frac[:,None]
        accel=(np.array([s['accel_g'] for s in rows])-gravity)*9.80665
        body=np.column_stack((accel@forward,accel@left))
        w=np.radians(yaw);theta=integrate(w,t)
        # Remove approximate IMU lever-arm acceleration relative to the chassis COM.
        smooth=np.convolve(np.pad(w,(2,2),mode='edge'),np.ones(5)/5,mode='valid')
        alpha=np.gradient(smooth,t);rx,ry=-.051302,.002974
        body[:,0]+=alpha*ry+w*w*rx
        body[:,1]+=-alpha*rx+w*w*ry
        c,s=np.cos(theta),np.sin(theta)
        world=np.column_stack((body[:,0]*c-body[:,1]*s,body[:,0]*s+body[:,1]*c))
        velocity=integrate(world,t)
        # Both endpoints are observed rest windows; do not carry velocity between trials.
        residual=velocity[-1].copy()
        velocity-=((t-t[0])/(t[-1]-t[0]))[:,None]*residual
        position=integrate(velocity,t)
        v_body=velocity[:,0]*c+velocity[:,1]*s
        angular_stop=integrate(np.abs(yaw[t>=finish]),t[t>=finish])
        std0=np.std([s['gyro_dps'] for s in pre],axis=0)
        std1=np.std([s['gyro_dps'] for s in post],axis=0)
        output.append(dict(trial=event['trial'],keys=event['keys'],pwm=event['pwm'],repeat=event['repeat'],
            actual_start_s=begin,actual_drive_duration_s=finish-begin,
            yaw_plateau_dps=steady,first_order=fitted,
            yaw_angle_deg=float(integrate(yaw,t)[-1]),
            angular_coast_abs_deg=float(angular_stop[-1]) if len(angular_stop) else None,
            gyro_bias_before_dps=b0.tolist(),gyro_bias_after_dps=b1.tolist(),
            gravity_before_g=g0.tolist(),gravity_after_g=g1.tolist(),
            baseline_gyro_std_dps=np.maximum(std0,std1).tolist(),
            baseline_gravity_change_g=float(np.linalg.norm(g1-g0)),
            short_burst_linear_mps_UNVERIFIED=float(np.median(v_body[plateau])),
            short_burst_displacement_xy_m_UNVERIFIED=position[-1].tolist(),
            zero_velocity_correction_mps=residual.tolist(),max_sample_gap_s=float(np.max(np.diff(t)))))
    groups=[]
    for keys,pwm in sorted({(r['keys'],r['pwm']) for r in output}):
        rows=[r for r in output if r['keys']==keys and r['pwm']==pwm]
        rates=[r['yaw_plateau_dps'] for r in rows];med=statistics.median(rates)
        groups.append(dict(keys=keys,pwm=pwm,repetitions=len(rows),yaw_dps_median=med,
            yaw_dps_min=min(rates),yaw_dps_max=max(rates),yaw_dps_MAD=statistics.median(abs(r-med) for r in rates),
            rise_time_constant_s_median=statistics.median(r['first_order']['time_constant_s'] for r in rows if r['first_order']) if any(r['first_order'] for r in rows) else None,
            linear_mps_median_UNVERIFIED=statistics.median(r['short_burst_linear_mps_UNVERIFIED'] for r in rows),
            angular_coast_abs_deg_median=statistics.median(r['angular_coast_abs_deg'] for r in rows)))
    baseline=samples[:min(1000,len(samples))]
    gyro=np.array([s['gyro_dps'] for s in baseline]);acc=np.array([s['accel_g'] for s in baseline])
    report=dict(schema_version=1,source=str(folder),sample_count=len(samples),
        sample_rate_hz=float(1/np.median(np.diff(times))),max_sample_gap_s=float(np.max(np.diff(times))),
        method='Bracket each short trial with rest bias/gravity; interpolate gyro bias; project gyro onto measured gravity. Linear estimates use planar rotation, assumed IMU lever arm and endpoint zero-velocity constraints.',
        limits=['Absolute translation and tire slip remain uncalibrated without external distance/pose reference.',
                'No wheel encoders/current measurements: torque constant, motor friction and tire friction are not separately identifiable.',
                'Short pulses do not guarantee steady speed; reported yaw plateau is the last 35% of each pulse.',
                'Rest windows assume the operator did not move the robot; long-term bias instability needs a longer stationary capture.',
                'Accelerometer-derived speed is a provisional short-burst estimate, not navigation truth.'],
        sensor=dict(gravity_g=np.median(acc,axis=0).tolist(),gyro_bias_dps=np.median(gyro,axis=0).tolist(),
            gyro_sample_std_dps=np.std(gyro,axis=0).tolist(),accel_sample_std_g=np.std(acc,axis=0).tolist(),
            body_forward_axis_sensor=f.tolist(),forward_axis_launch_signal_g=forward_signal_g),
        groups=groups,trials=output)
    (folder/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    for row in groups:print(f"{row['keys']:3} PWM {row['pwm']:4}: yaw {row['yaw_dps_median']:7.2f}°/s [{row['yaw_dps_min']:.1f}, {row['yaw_dps_max']:.1f}], v~{row['linear_mps_median_UNVERIFIED']:.3f} m/s UNVERIFIED")
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args();analyze(a.folder)
