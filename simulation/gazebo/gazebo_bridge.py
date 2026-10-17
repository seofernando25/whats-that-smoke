"""Persistent Gazebo transport publisher; never imports the real car hardware service."""
from pathlib import Path
import math
import json
import subprocess
import threading
import time
from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.double_v_pb2 import Double_V
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.world_stats_pb2 import WorldStatistics
from gz.msgs.world_control_pb2 import WorldControl
from gz.msgs.boolean_pb2 import Boolean
from control_state import ControlState
from imu_model import RecordedImu, ImuDownsampler
from gz.msgs.imu_pb2 import IMU
from gz.msgs.stringmsg_pb2 import StringMsg
from build_motor_plugin import plugin_environment

HERE = Path(__file__).resolve().parent

class GazeboBridge:
    def __init__(self):
        self.control = ControlState()
        self.imu = RecordedImu()
        self.imu_downsampler = ImuDownsampler(self.imu.profile['update_rate_hz'],self.imu.profile['anti_alias_cutoff_hz_assumed'])
        self.imu_received_at = 0.
        self.node = Node()
        self.publisher = self.node.advertise('/model/freenove/cmd_vel', Twist)
        self.duty_publisher = self.node.advertise('/model/freenove/wheel_duty', Double_V)
        self.lock = threading.Lock()
        self.last_stats = 0
        self.sim_time_s = 0.
        self.real_time_factor = 0.
        self.paused = True
        self.pose = {'x': 0, 'y': 0, 'yaw': 0}
        self.process = None
        self.closed = threading.Event()
        self.noisy_imu_publisher = self.node.advertise('/model/freenove/imu/noisy', IMU)
        self.nano_publisher = self.node.advertise('/model/freenove/imu/nano', StringMsg)
        self.node.subscribe(IMU, '/model/freenove/imu/ideal', self.imu_sample)
        self.node.subscribe(WorldStatistics, '/world/freenove_lab/stats', self.stats)
        self.node.subscribe(Pose_V, '/world/freenove_lab/dynamic_pose/info', self.poses)
        self.worker = threading.Thread(target=self.publish_loop, daemon=True)
        self.worker.start()

    def imu_sample(self, message):
        t=message.header.stamp.sec+message.header.stamp.nsec*1e-9
        a=message.linear_acceleration;g=message.angular_velocity;q=message.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        v,w=self.control.command()
        averaged=self.imu_downsampler.update(t,[a.x,a.y,a.z],[g.x,g.y,g.z])
        if averaged is None:return
        sample=self.imu.update(t,*averaged,yaw,moving=bool(v or w))
        if sample is None:return
        self.imu_received_at=time.monotonic()
        noisy=IMU();noisy.header.CopyFrom(message.header)
        # No perfect orientation is copied to the noisy sensor stream.
        for index,axis in enumerate(('x','y','z')):
            setattr(noisy.linear_acceleration,axis,sample['accel_g'][index]*9.80665)
            setattr(noisy.angular_velocity,axis,math.radians(sample['gyro_dps'][index]))
        noisy.entity_name='nano_noisy_imu'
        noisy.header.ClearField('data')
        frame=noisy.header.data.add();frame.key='frame_id';frame.value.append('nano_sensor')
        self.noisy_imu_publisher.publish(noisy)
        nano=StringMsg();nano.data=json.dumps(dict(type='imu',a=sample['accel_g'],g=sample['gyro_dps'],t_sim_s=t))
        self.nano_publisher.publish(nano)

    def stats(self, message):
        with self.lock:
            self.last_stats = time.monotonic()
            self.paused = message.paused
            self.sim_time_s = message.sim_time.sec + message.sim_time.nsec*1e-9
            self.real_time_factor = message.real_time_factor
        if message.paused:
            self.control.stop()

    def poses(self, message):
        for pose in message.pose:
            if pose.name == 'freenove':
                q = pose.orientation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                with self.lock:
                    self.pose = {'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw}
                break

    def status(self):
        with self.lock:
            online = time.monotonic() - self.last_stats < 3
            paused, pose = self.paused, dict(self.pose)
            sim_time_s, rtf = self.sim_time_s, self.real_time_factor
        imu=dict(self.imu.snapshot(), streaming=online and not paused and time.monotonic()-self.imu_received_at<1)
        v, w = self.control.command(online and not paused)
        return dict(online=online, paused=paused, pose=pose, imu=imu, sim_time_s=sim_time_s, real_time_factor=rtf, linear=v if self.control.pwm is None else None, angular=w if self.control.pwm is None else None, driving=any(self.control.duty_command(online and not paused)) if self.control.pwm is not None else bool(v or w), pwm=self.control.pwm, drive_mode="real-car PWM" if self.control.pwm is not None else "SI velocity")

    def publish_command(self):
        state = self.status()
        if self.control.pwm is not None:
            message=Double_V()
            message.data.extend(self.control.duty_command(state['online'] and not state['paused']))
            self.duty_publisher.publish(message)
        else:
            message = Twist()
            message.linear.x, message.angular.z = state['linear'], state['angular']
            self.publisher.publish(message)

    def publish_loop(self):
        while not self.closed.wait(.05):
            self.publish_command()

    def action(self, action):
        self.control.stop()
        if action == 'imu-zero':
            self.imu.request_calibration()
            return
        if action == 'launch':
            if self.status()['online'] or self.process and self.process.poll() is None:
                return
            logs = HERE / '.control-logs'
            logs.mkdir(exist_ok=True)
            with (logs/'gazebo.log').open('a') as output:
                self.process = subprocess.Popen(
                    ['gz','sim','-r','--gui-config',str(HERE/'car_gui.config'),str(HERE/'freenove_world.sdf')],
                    cwd=HERE, stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
                    env=plugin_environment())
            return
        if action not in {'play', 'pause', 'reset'}:
            raise ValueError('Unknown simulation action')
        requested_at=time.monotonic()
        message = WorldControl()
        message.pause = action != 'play'
        if action == 'reset':
            message.reset.all = True
            self.imu.request_calibration()
        ok, response = self.node.request('/world/freenove_lab/control', message,
                                         WorldControl, Boolean, 5000)
        if not ok or not response.data:
            # Service discovery can briefly retain the previous server after
            # a world reload. Pause/play/reset are safe to retry once.
            time.sleep(.2)
            ok,response=self.node.request('/world/freenove_lab/control',message,WorldControl,Boolean,5000)
            if not ok or not response.data:
                # Reset can rebuild the service before its reply reaches this
                # node. A fresh world-statistics confirmation is authoritative.
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    with self.lock:
                        confirmed=self.last_stats>=requested_at and self.paused==message.pause and (action!='reset' or self.sim_time_s<.05)
                    if confirmed:return
                    time.sleep(.01)
                raise ValueError('Gazebo did not respond or confirm the requested state; open the simulation first')
        if action=='reset':
            # Gazebo acknowledges queuing the request before applying it. An
            # immediate Play can replace that pending control message. Wait for
            # a fresh paused, rewound world clock before enabling another action.
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                with self.lock:
                    confirmed=self.last_stats>=requested_at and self.paused and self.sim_time_s<.05
                if confirmed:return
                time.sleep(.01)
            raise ValueError('Gazebo accepted reset but has not confirmed the paused, reset world')

    def close(self):
        self.control.stop()
        self.closed.set()
        self.worker.join(timeout=1)
        self.publisher.publish(Twist())
        message=Double_V();message.data.extend([0,0]);self.duty_publisher.publish(message)
        time.sleep(.1)
