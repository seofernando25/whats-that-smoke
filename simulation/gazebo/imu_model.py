"""Recorded Nano-frame noise and the production UGV heading estimator.

Gazebo supplies ideal SI readings in chassis axes. Only acceleration, gyro,
simulation time and command intent enter RelativeHeading; pose is comparison only.
"""
import json
import math
from pathlib import Path
import random
import sys
import threading

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'src'))
from ugv.heading import RelativeHeading


def transform(matrix, vector):
    return [sum(a*b for a,b in zip(row,vector)) for row in matrix]


def cholesky(matrix):
    result=[[0.]*3 for _ in range(3)]
    for i in range(3):
        for j in range(i+1):
            value=matrix[i][j]-sum(result[i][k]*result[j][k] for k in range(j))
            result[i][j]=math.sqrt(max(0.,value)) if i==j else value/result[j][j] if result[j][j] else 0.
    return result


class RecordedImu:
    def __init__(self, profile=None, seed=None):
        self.profile=profile or json.loads((HERE/'imu_noise_profile.json').read_text())
        self.seed=self.profile['seed'] if seed is None else seed
        self.lock=threading.RLock()
        self.epoch=-1
        self.reset()

    def reset(self):
        with self.lock:
            self.epoch+=1
            self.random=random.Random(self.seed+self.epoch)
            self.heading=RelativeHeading()
            self.last_t=None
            self.truth_zero=None
            self.sample=None
            self.count=0
            self.offset={};self.drift={};self.factor={}
            for key,bias in (('accel_g',self.profile['accel_bias_g']),('gyro_dps',self.profile['gyro_bias_dps'])):
                p=self.profile['channels'][key]
                # Rest-window variability is empirical within-session spread,
                # not a claimed characterization of power-on bias repeatability.
                self.offset[key]=[b+self.random.gauss(0,s) for b,s in zip(bias,p['rest_window_mean_std'])]
                self.drift[key]=[self.random.gauss(0,s) for s in p['short_term_bias_std']]
                self.factor[key]=cholesky(p['white_covariance'])

    def request_calibration(self):
        with self.lock:
            self.heading.request_calibration()
            self.truth_zero=None

    def _channel(self,key,ideal,dt):
        p=self.profile['channels'][key]
        for i,(std,tau) in enumerate(zip(p['short_term_bias_std'],p['short_term_bias_tau_s'])):
            decay=math.exp(-dt/tau)
            self.drift[key][i]=decay*self.drift[key][i]+self.random.gauss(0,std*math.sqrt(1-decay*decay))
        white=transform(self.factor[key],[self.random.gauss(0,1) for _ in range(3)])
        return [v+b+d+n for v,b,d,n in zip(ideal,self.offset[key],self.drift[key],white)]

    def update(self,t,acceleration_mps2,angular_velocity_rps,body_yaw_rad,moving):
        with self.lock:
            values=[t,*acceleration_mps2,*angular_velocity_rps,body_yaw_rad]
            if not all(math.isfinite(v) for v in values):return None
            if self.last_t is not None and t<self.last_t-1e-9:self.reset()
            if self.last_t is not None and t<=self.last_t:return None
            dt=t-self.last_t if self.last_t is not None else 1/self.profile['update_rate_hz']
            matrix=self.profile['body_to_sensor_matrix']
            acceleration=transform(matrix,[v/9.80665 for v in acceleration_mps2])
            gyro=transform(matrix,[math.degrees(v) for v in angular_velocity_rps])
            noisy_a=self._channel('accel_g',acceleration,dt)
            noisy_g=self._channel('gyro_dps',gyro,dt)
            estimate=self.heading.update(noisy_a,noisy_g,t,moving=moving)
            # Pose is never used to correct the production estimator.
            if estimate['heading_calibrated'] and self.truth_zero is None:self.truth_zero=body_yaw_rad
            truth=(-math.degrees(body_yaw_rad-self.truth_zero))%360 if self.truth_zero is not None else None
            error=(estimate['heading_deg']-truth+180)%360-180 if truth is not None else None
            self.count+=1;self.last_t=t
            self.sample=dict(t_sim_s=t,accel_g=noisy_a,gyro_dps=noisy_g,
                ideal_accel_g=acceleration,ideal_gyro_dps=gyro,truth_heading_deg=truth,
                heading_error_deg=error,truth_yaw_rad=body_yaw_rad,sample_count=self.count,
                seed=self.seed,epoch=self.epoch,**estimate)
            return dict(self.sample)

    def snapshot(self):
        with self.lock:
            return dict(self.sample or {},**self.heading.snapshot())


class ImuDownsampler:
    """Integrate ideal physics-rate samples before 100Hz output to avoid aliasing.

    This is a two-pole low-pass plus 10ms boxcar approximation, not an identified
    BMI270 filter transfer function. Never average across gaps or world resets.
    """
    def __init__(self,rate=100,cutoff_hz=25):
        self.period=1/rate
        self.cutoff_hz=cutoff_hz
        self.filtered=None
        self.filtered2=None
        self.last_t=None
        self.elapsed=0.
        self.total=[0.]*6

    def update(self,t,acceleration,gyro):
        dt=t-self.last_t if self.last_t is not None else 0.
        self.last_t=t
        if dt<=0 or dt>.05:
            self.elapsed=0.;self.total=[0.]*6
            self.filtered=[*acceleration,*gyro];self.filtered2=list(self.filtered)
            return None
        self.elapsed+=dt
        alpha=1-math.exp(-2*math.pi*self.cutoff_hz*dt)
        for i,value in enumerate([*acceleration,*gyro]):
            self.filtered[i]+=alpha*(value-self.filtered[i])
            self.filtered2[i]+=alpha*(self.filtered[i]-self.filtered2[i])
            self.total[i]+=self.filtered2[i]*dt
        if self.elapsed+1e-9<self.period:return None
        averaged=[v/self.elapsed for v in self.total]
        self.elapsed=0.;self.total=[0.]*6
        return averaged[:3],averaged[3:]
