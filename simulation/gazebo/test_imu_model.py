"""Checks for the recorded sensor model and the unchanged production estimator."""
import math
from pathlib import Path
import statistics
import unittest
from imu_model import RecordedImu, ImuDownsampler, transform

HERE=Path(__file__).resolve().parent

class ImuTest(unittest.TestCase):
    def test_reproducible_covariance_and_measured_bias(self):
        a=RecordedImu(seed=1);b=RecordedImu(seed=1)
        rows=[]
        for index in range(5000):
            args=(index*.01,[0,0,9.80665],[0,0,0],0,False)
            x=a.update(*args);self.assertEqual(x,b.update(*args));rows.append(x)
        p=a.profile
        for axis in range(3):
            measured=statistics.pstdev(r['gyro_dps'][axis] for r in rows)
            expected=math.hypot(p['channels']['gyro_dps']['white_std'][axis],p['channels']['gyro_dps']['short_term_bias_std'][axis])
            self.assertAlmostEqual(measured,expected,delta=.1*expected)
            self.assertAlmostEqual(statistics.mean(r['gyro_dps'][axis] for r in rows),a.offset['gyro_dps'][axis],delta=.005)
        self.assertTrue(rows[-1]['heading_calibrated'])
        self.assertLess(abs(rows[-1]['heading_error_deg']),.1)

    def test_frame_rotation_and_existing_heading_on_both_turn_directions(self):
        imu=RecordedImu(seed=9)
        for index in range(350):imu.update(index*.01,[0,0,9.80665],[0,0,0],0,False)
        for direction in (-1,1):
            for i in range(1,201):
                t=3.49+i*.01+(0 if direction==-1 else 2)
                angle=-math.pi*i/200 if direction==-1 else -math.pi+math.pi*i/200
                sample=imu.update(t,[0,0,9.80665],[0,0,direction*math.pi/2],angle,True)
            self.assertLess(abs(sample['heading_error_deg']),1)
        matrix=imu.profile['body_to_sensor_matrix']
        up=transform(matrix,[0,0,1]);self.assertAlmostEqual(sum(v*v for v in up),1)

    def test_pose_does_not_correct_the_heading(self):
        a=RecordedImu(seed=2);b=RecordedImu(seed=2)
        for i in range(500):
            x=a.update(i*.01,[0,0,9.80665],[0,0,.1 if i>350 else 0],0,i>350)
            y=b.update(i*.01,[0,0,9.80665],[0,0,.1 if i>350 else 0],2 if i>350 else 0,i>350)
            self.assertEqual(x['heading_deg'],y['heading_deg'])
        self.assertNotEqual(x['heading_error_deg'],y['heading_error_deg'])

    def test_pause_duplicate_time_reset_and_manual_rezero(self):
        imu=RecordedImu()
        for i in range(350):imu.update(i*.01,[0,0,9.80665],[0,0,0],0,False)
        self.assertIsNone(imu.update(3.49,[0,0,9.80665],[0,0,0],0,False))
        early=RecordedImu();early.update(.02,[0,0,9.80665],[0,0,0],0,False)
        early.update(.01,[0,0,9.80665],[0,0,0],0,False)
        self.assertEqual(early.epoch,1)
        epoch=imu.epoch
        sample=imu.update(.01,[0,0,9.80665],[0,0,0],0,False)
        self.assertEqual(imu.epoch,epoch+1);self.assertFalse(sample['heading_calibrated'])
        for i in range(2,352):imu.update(i*.01,[0,0,9.80665],[0,0,0],0,False)
        imu.request_calibration();self.assertFalse(imu.snapshot()['heading_calibrated'])

    def test_physics_rate_filter_does_not_alias_alternating_chatter(self):
        sampler=ImuDownsampler();outputs=[]
        for i in range(1001):
            result=sampler.update(i*.001,[100*(-1)**i,0,9.80665],[0,0,2])
            if result:outputs.append(result)
        self.assertEqual(len(outputs),100)
        for accel,gyro in outputs[-10:]:
            self.assertAlmostEqual(accel[0],0,places=8)
            self.assertAlmostEqual(accel[2],9.80665)
            self.assertAlmostEqual(gyro[2],2)
        self.assertIsNone(sampler.update(.01,[0,0,9.80665],[0,0,0]))
        self.assertIsNone(sampler.update(1,[0,0,9.80665],[0,0,0]))

if __name__=='__main__':unittest.main()
