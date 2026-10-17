"""Fit aggregate simulated drive effort to observed yaw, not motor electrical constants."""
import json
from pathlib import Path
import statistics
import numpy as np
from commands import vector, combinations

ROOT=Path(__file__).resolve().parents[2]
DATA=ROOT/'calibration/runs'


def fit():
    short=json.loads((DATA/'2026-10-17-floor-01/analysis.json').read_text())
    long1=json.loads((DATA/'2026-10-17-floor-long-01/analysis.json').read_text())
    long3=json.loads((DATA/'2026-10-17-floor-long-03/analysis.json').read_text())
    probes=[]
    for name in ('sim-gain025-slip030','sim-gain05-slip030','sim-gain08-slip030',
                 'sim-gain12-slip030','sim-gain15-slip030','sim-gain18-slip030','sim-gain22-slip030'):
        probes.append(json.loads((DATA/(name+'.json')).read_text()))
    inverse={}
    for key in ('Q','WA'):
        points=[(0.,0.)]+[(abs(next(r for r in d['results'] if r['keys']==key)['yaw_dps']),d['left_gain']*1800/4095) for d in probes]
        points=sorted(points)
        inverse[key]=points
    training=[r for r in long1['trials'] if r['repeat']<2]
    validation=[r for r in long3['trials'] if r['repeat']==2]
    maps=[]
    for keys in ('W','S','Q','E','WA','WD','SA','SD'):
        f,t=vector(keys)
        for pwm in (0,500,600,800,1300,1800):
            rows=([r for r in training if r['keys']==keys] if pwm==1800 else
                  [r for r in short['trials'] if r['keys']==keys and r['pwm']==pwm])
            source='assumed below measured control range'
            rate=0.
            if not rows:effort=0.
            elif t==0:
                speed=abs(statistics.median(r['short_burst_linear_mps_UNVERIFIED'] for r in rows))
                effort=min(.98,speed/(250*2*np.pi/60*.03175))
                source='UNVERIFIED short-burst accelerometer estimate; no external distance reference'
            else:
                if pwm==1800:
                    rate=statistics.median(r['yaw_plateau_dps'] for r in rows)
                    source='median of two held two-second floor trials; third repeat held out'
                else:
                    # Estimate asymptote from the short pulse, and flag the extrapolation.
                    rate=statistics.median(r['first_order']['steady_dps'] if r['first_order'] else r['yaw_plateau_dps'] for r in rows)
                    source='short-pulse first-order asymptote estimate; not a sustained measurement'
                points=inverse['Q' if not f else 'WA']
                effort=float(np.interp(abs(rate),[p[0] for p in points],[p[1] for p in points])) if abs(rate)>3 else 0.
            maps.append(dict(keys=keys,forward=f,turn=t,pwm=pwm,effective_duty=effort,
                             target_yaw_dps=rate,source=source))
    correction_path=ROOT/'simulation/gazebo/brake_effort_corrections.json'
    correction=json.loads(correction_path.read_text()) if correction_path.exists() else None
    if correction:
        factors={(r['forward'],r['turn']):r['factor'] for r in correction['corrections']}
        for row in maps:
            row['effective_duty']=min(1.,row['effective_duty']*factors.get((row['forward'],row['turn']),1.))
    response_tau=[]
    for keys in ("Q","E","WA","WD","SA","SD"):
        fits=[r["first_order"] for r in training if r["keys"]==keys and r["first_order"]]
        f,t=vector(keys)
        response_tau.append(dict(forward=f,turn=t,keys=keys,tau_s=max(.01,statistics.median(r["time_constant_s"] for r in fits)-.025)))
    if correction:
        for override in correction.get("pwm_rise_tau_overrides",[]):
            response_tau=[r for r in response_tau if (r["forward"],r["turn"])!=(override["forward"],override["turn"])]
            response_tau.append(override)
    profile=dict(brake_effort_refit=correction,pwm_rise_tau=response_tau,schema_version=1,measured_total_mass_kg=.5774,hardware_pwm_max=1800,
        lateral_slip_m_per_s_per_n=.30,
        meaning='Aggregate empirical effort/contact fit; effective duty is not a measured motor voltage or friction coefficient.',
        angular_calibration='gyro rest-bias corrected; projected onto measured gravity; short-pulse and two-second trials',
        translation_calibrated=False,
        translation_note='Short-burst IMU estimates only; independent distance reference is still required.',
        long_term_drift_calibrated=False,
        training_trial_count=len(training),held_out_trial_count=len(validation),
        sensor=long1['sensor'],key_combinations=combinations(),pwm_map=maps,
        training_trials=training,held_out_trials=validation,
        source_sessions=['2026-10-17-floor-01','2026-10-17-floor-long-01','2026-10-17-floor-long-03'])
    dest=ROOT/'simulation/gazebo/real_robot_profile.json';dest.write_text(json.dumps(profile,indent=2))
    return profile

if __name__=='__main__':
    p=fit()
    for r in p['pwm_map']:
        if r['pwm']==1800:print(r['keys'],round(r['effective_duty'],3),round(r['target_yaw_dps'],1))
