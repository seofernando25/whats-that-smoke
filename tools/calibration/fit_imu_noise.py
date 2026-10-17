"""Fit an empirical rest-noise approximation from recorded Nano BMI270 samples."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
SESSIONS=('2026-10-17-floor-01','2026-10-17-floor-long-01','2026-10-17-floor-long-03','2026-10-17-laps-right-01')

def fit(data, output):
    spans=[];sources=[]
    for name in SESSIONS:
        path=data/name/'samples.jsonl';blob=path.read_bytes();rows=[json.loads(l) for l in blob.splitlines()]
        run=[];rest_start=None;previous=None;count=0
        def save():
            nonlocal count
            if len(run)>=75 and run[-1]['t']-run[0]['t']>=.7:
                spans.append(run.copy());count+=len(run)
        for row in rows:
            gap=previous is not None and not 0<row['t']-previous<.05
            quiet=row['stopped'] and not any(row['wheels'].values()) and np.linalg.norm(row['gyro_dps'])<4 and abs(np.linalg.norm(row['accel_g'])-1)<.1
            if gap or not quiet:
                save();run=[];rest_start=None
            if quiet:
                if rest_start is None:rest_start=row['t']
                if row['t']-rest_start>=1.:run.append(row)
            previous=row['t']
        save();sources.append(dict(session=name,sha256=hashlib.sha256(blob).hexdigest(),rest_samples=count))
    if not spans:raise ValueError('No settled rest spans')
    means={};channels={}
    for field in ('accel_g','gyro_dps'):
        arrays=[np.array([r[field] for r in span]) for span in spans]
        means[field]=np.array([a.mean(axis=0) for a in arrays])
        residual=np.concatenate([a-a.mean(axis=0) for a in arrays])
        # Adjacent increments suppress offsets; residual covariance keeps any
        # fast correlated vibration in the measured per-sample variance.
        covariance=np.cov(residual,rowvar=False)
        bins=[];pairs=[]
        for span,array in zip(spans,arrays):
            time=np.array([r['t']-span[0]['t'] for r in span]);block=[]
            for k in range(int(time[-1]/.5)):
                selected=array[(time>=k*.5)&(time<(k+1)*.5)]
                if len(selected)>=30:block.append(selected.mean(axis=0))
            if len(block)>3:
                block=np.array(block);block-=block.mean(axis=0);bins.extend(block)
                pairs.extend(zip(block[:-1],block[1:]))
        bins=np.array(bins);first=np.array([a for a,b in pairs]);second=np.array([b for a,b in pairs])
        rho=np.sum(first*second,axis=0)/np.maximum(np.sum(first**2,axis=0),1e-20)
        rho=np.clip(rho,0,.99)
        # A half-second block contains about 50 samples. Remove independent
        # sample noise before treating low-frequency variance as dynamic bias.
        drift_var=np.maximum(0,np.var(bins,axis=0)-np.diag(covariance)/50)
        tau=np.where(rho>.05,-.5/np.log(np.maximum(rho,1e-6)),.5)
        channels[field]=dict(mean=np.mean(means[field],axis=0).tolist(),
            white_covariance=covariance.tolist(),white_std=np.sqrt(np.diag(covariance)).tolist(),
            rest_window_mean_std=np.std(means[field],axis=0).tolist(),
            short_term_bias_std=np.sqrt(drift_var).tolist(),short_term_bias_tau_s=tau.tolist(),
            half_second_lag1=rho.tolist(),
            tau_note='Fitted from lag-1 of 0.5s rest means when rho>0.05; otherwise unresolved and set to the 0.5s block resolution, not measured long-term drift.')
    gravity=np.array(channels['accel_g']['mean']);up=gravity/np.linalg.norm(gravity)
    drive=json.loads((ROOT/'simulation/gazebo/real_robot_profile.json').read_text())
    forward=np.array(drive['sensor']['body_forward_axis_sensor']);forward-=forward.dot(up)*up;forward/=np.linalg.norm(forward)
    left=np.cross(up,forward)
    matrix=np.column_stack((forward,left,up))
    accel_bias=gravity-up
    profile=dict(schema_version=1,sensor='Nano 33 BLE Sense Rev2 BMI270',seed=5774,update_rate_hz=100,ideal_physics_rate_hz=1000,anti_alias_cutoff_hz_assumed=25,
        bandwidth_note='Two-pole 25Hz low-pass and 10ms average of physics-rate ideal samples before adding 100Hz recorded noise; anti-alias approximation, not an identified BMI270 filter.',
        source_recordings=sources,rest_spans=len(spans),rest_samples=sum(len(s) for s in spans),
        settled_rest_s=sum(s[-1]['t']-s[0]['t'] for s in spans),
        selection='Zero wheel PWM, stopped, >=1s after stop, |gyro|<4 dps, acceleration norm within 0.1g; gaps >=50ms split spans.',
        channels=channels,body_to_sensor_matrix=matrix.tolist(),
        accel_bias_g=accel_bias.tolist(),gyro_bias_dps=channels['gyro_dps']['mean'],
        position_body_m=[-.054,.003,.08245],
        position_note='Approximate Nano board centre from visual assembly, not a measured sensor lever arm.',
        bias_model='Mean measured rest gyro offset; per-reset spread from observed rest-window means. Independent per-axis Ornstein-Uhlenbeck short-term bias fitted to 0.5s rest-block statistics; sample noise uses full measured residual covariance.',
        limits=['Window-to-window spread comes from one campaign, not independent power cycles.',
                'Dynamic bias is a short-rest approximation; long-term random walk, temperature dependence and Allan variance are uncalibrated.',
                'Rest residuals include mounting vibration and firmware filtering; modeled Gaussian innovations are not identified intrinsic sensor noise density.',
                'Gravity fixes tilt, launch acceleration estimates forward orientation. Accelerometer offset along gravity is not separately identifiable from scale/tilt.',
                'Motor-on vibration is not added from moving records because true acceleration and sensor error cannot be separated without an external reference.'])
    output.write_text(json.dumps(profile,indent=2)+'\n')
    print(json.dumps({k:profile[k] for k in ('rest_spans','rest_samples','settled_rest_s','gyro_bias_dps')}))
    for k,v in channels.items():print(k,'white std',v['white_std'],'bias std',v['short_term_bias_std'],'tau',v['short_term_bias_tau_s'])
    return profile

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--data',type=Path,default=ROOT/'calibration/runs');p.add_argument('--output',type=Path,default=ROOT/'simulation/gazebo/imu_noise_profile.json');a=p.parse_args();fit(a.data,a.output)
