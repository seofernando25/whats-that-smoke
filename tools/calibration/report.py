"""Render measured response curves and holdout validation, with clear limits."""
import argparse
import json
import math
from pathlib import Path
import statistics
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
DATA=ROOT/'calibration/runs'
OUT=ROOT/'calibration/reports'

def main():
    profile=json.loads((ROOT/'simulation/gazebo/real_robot_profile.json').read_text())
    simulation=json.loads((DATA/'sim-calibrated-validation.json').read_text())
    sim={r['keys']:r for r in simulation['results']}
    held={r['keys']:r for r in profile['held_out_trials']}
    keys=('Q','E','WA','WD','SD','SA')
    comparison=[]
    for key in keys:
        real=held[key]['yaw_plateau_dps'];pred=sim[key]['yaw_dps']
        comparison.append(dict(keys=key,held_out_yaw_dps=real,gazebo_yaw_dps=pred,
            absolute_error_dps=abs(pred-real),relative_error_percent=100*abs(pred-real)/abs(real)))
    summary=dict(source_sessions=profile['source_sessions'],completed_trials=116,raw_samples=44282,
        dropped_samples=0,held_out_comparison=comparison,
        angular_MAE_dps=statistics.mean(r['absolute_error_dps'] for r in comparison),
        angular_MAPE_percent=statistics.mean(r['relative_error_percent'] for r in comparison),
        motion_geometry_and_release={r['keys']:r.get('metrics') for r in simulation['results']},
        reset_feedback_fix=True,
        max_motor_torque_nm=simulation['max_abs_torque_nm'],translation_calibrated=False,
        long_term_drift_calibrated=False)
    (ROOT/'simulation/gazebo/calibration_validation.json').write_text(json.dumps(summary,indent=2))
    records=[]
    for session in profile['source_sessions'][1:]:
        p=DATA/session
        analysis=json.loads((p/'analysis.json').read_text())
        samples=[json.loads(l) for l in (p/'samples.jsonl').read_text().splitlines()]
        for trial in analysis['trials']:
            if trial['keys'] not in keys:continue
            begin=trial['actual_start_s'];finish=begin+trial['actual_drive_duration_s']
            end=finish+1.0
            selected=[s for s in samples if begin<=s['t']<=end]
            t=np.array([s['t']-begin for s in selected]);fraction=np.clip(t/(end-begin),0,1)
            b0=np.array(trial['gyro_bias_before_dps']);b1=np.array(trial['gyro_bias_after_dps'])
            up=np.array(trial['gravity_before_g']);up/=np.linalg.norm(up)
            yaw=(np.array([s['gyro_dps'] for s in selected])-b0-(b1-b0)*fraction[:,None])@up
            records.append((trial['keys'],t,yaw,trial['repeat']))
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,3,figsize=(13,7),sharex=True)
    titles={'Q':'Q / A · left pivot','E':'E / D · right pivot','WA':'W + A · forward left',
            'WD':'W + D · forward right','SD':'S + D · reverse right','SA':'S + A · reverse left'}
    for ax,key in zip(axes.flat,keys):
        for k,t,yaw,repeat in records:
            if k==key:ax.plot(t,yaw,color='#176b8c',alpha=.45 if repeat<2 else .8,lw=1,
                             label='Real car (3 repeats)' if not ax.lines else None)
        p=sim[key]['trajectory'];t=np.array([v['t'] for v in p]);yaw=np.unwrap([v['yaw'] for v in p])
        rate=np.degrees(np.diff(yaw)/np.diff(t));tt=t[1:]-t[0]
        ax.plot(tt,rate,color='#e57927',lw=2,ls='--',label='Gazebo fit')
        ax.axvline(2,color='#777',lw=.8,ls=':');ax.axhline(0,color='#aaa',lw=.5)
        ax.set_title(titles[key],loc='left',fontweight='bold');ax.set_xlim(0,3);ax.grid(alpha=.15)
        ax.set_xlabel('Seconds from applied command');ax.set_ylabel('Yaw rate (°/s)')
    axes[0,0].legend(frameon=False,fontsize=9,loc='lower right')
    fig.suptitle('UGV · real floor trials versus Gazebo at PWM 1800',fontsize=17,fontweight='bold',x=.06,ha='left')
    fig.text(.06,.015,'Bias corrected per trial; turn rate measured by 100 Hz IMU. Dotted line: release. Translation and long-term sensor drift remain unverified.',fontsize=9,color='#555')
    fig.tight_layout(rect=(.02,.045,1,.94));OUT.mkdir(parents=True,exist_ok=True)
    fig.savefig(OUT/'ugv-dynamics-calibration.png',dpi=160);plt.close(fig)
    lines=['# UGV dynamics calibration','',
        '116 completed floor trials; 44,282 raw 100 Hz IMU/PWM samples; no telemetry drops.',
        '86 short sweeps and 30 two-second full-dashboard-power trials. One long session stopped before a trial when its rest baseline was unstable; the remaining trials were resumed.','',
        '| Input | Held-out real yaw °/s | Gazebo °/s | Error |','|---|---:|---:|---:|']
    for r in comparison:lines.append(f"| {r['keys']} | {r['held_out_yaw_dps']:.1f} | {r['gazebo_yaw_dps']:.1f} | {r['relative_error_percent']:.1f}% |")
    lines+=['',f"Held-out mean absolute yaw-rate error: {summary['angular_MAE_dps']:.1f}°/s ({summary['angular_MAPE_percent']:.1f}% mean relative error).",'',
        'The effective effort map is an aggregate behavioral fit, not a measurement of motor voltage, torque constants or tire friction. The underlying Gazebo motor torque remains finite.',
        'Each trial brackets motion with stationary gyro bias and gravity estimates, projects yaw onto the measured up direction, and starts fresh. Long-term bias instability is not characterized by these short rest windows.',
        'Accelerometer-derived translation is only a provisional short-burst estimate. Absolute speed, distance, skid and stopping distance need a measured floor reference, fiducials or encoders before they can be called calibrated.','',
        'Canonical source: https://github.com/uOAeronautics/ugv; local ~/Projects/ugv; Pi /home/advanced/ugv; service ugv-dashboard.service.']
    (OUT/'ugv-calibration-report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=OUT)
    OUT=parser.parse_args().output;main()
