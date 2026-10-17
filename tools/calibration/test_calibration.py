import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from commands import combinations,vector,wheel_duties
from analyze import analyze
spec=importlib.util.spec_from_file_location('dynamics_telemetry',HERE.parents[1]/'src/ugv/dynamics_telemetry.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class CalibrationTest(unittest.TestCase):
    def test_all_key_subsets_preserve_real_mixing_and_aliases(self):
        self.assertEqual(len(combinations()),64)
        for keys in combinations():
            self.assertEqual(vector(keys.replace('A','Q')),vector(keys)) if not set('DEQ') & set(keys) else None
            self.assertLessEqual(max(abs(v) for v in wheel_duties(keys,1800).values()),1800)
        self.assertEqual(wheel_duties('WA',1800),{'front-left':0,'rear-left':0,'front-right':-1800,'rear-right':-1800})
        self.assertEqual(vector('WAE'),(1,-1))
        self.assertEqual(vector('WSADQE'),(0,0))

    def test_telemetry_is_bounded_and_reports_losses(self):
        d=m.DynamicsTelemetry(capacity=2)
        state=types.SimpleNamespace(forward=0,turn=0,speed_limit=1800,wheels={'left':0},stopped=True,armed=False,revision=0)
        for i in range(4):d.append(i,[0,0,1],[0,0,0],state)
        p=d.snapshot(after=1)
        self.assertEqual([s['seq'] for s in p['samples']],[3,4]);self.assertEqual(p['dropped'],1)
        state.wheels['left']=100
        self.assertEqual(p['samples'][0]['wheels']['left'],0)
        d.append(5,[float('nan'),0,1],[0,0,0],state)
        self.assertEqual(d.sequence,4)

    def test_tilted_sensor_and_bias_drift_do_not_become_yaw_response(self):
        up=np.array([-.3,.1,np.sqrt(.9)])
        rows=[]
        for seq,t in enumerate(np.arange(0,5,.01),1):
            rate=60*(1-np.exp(-(t-2)/.1)) if 2<=t<3 else 0
            bias=np.array([2,-3,7])+up*.3*t
            rows.append(dict(seq=seq,t=float(t),accel_g=up.tolist(),gyro_dps=(bias+up*rate).tolist(),
                wheels={'left':-1800,'right':1800} if 2<=t<3 else {'left':0,'right':0}))
        events=[dict(event='trial_start',t=2,trial=0,keys='Q',pwm=1800,repeat=0),
                dict(event='drive_end',t=3,trial=0),dict(event='trial_end',t=4.8,trial=0)]
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'samples.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in rows))
            (p/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in events))
            report=analyze(p);result=report['trials'][0]
            self.assertAlmostEqual(result['yaw_plateau_dps'],60,delta=.5)
            self.assertAlmostEqual(result['first_order']['steady_dps'],60,delta=1)
            self.assertAlmostEqual(result['first_order']['time_constant_s'],.1,delta=.03)
            self.assertGreater(np.linalg.norm(np.array(result['gyro_bias_after_dps'])-result['gyro_bias_before_dps']),.5)

if __name__=='__main__':unittest.main()
