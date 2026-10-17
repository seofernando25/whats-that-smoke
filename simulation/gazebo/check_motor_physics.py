"""Isolated live Gazebo regression: requires the control panel Python environment."""
import os,sys,time,math,threading,subprocess,json,signal
from pathlib import Path
p=Path(__file__).resolve().parent;sys.path.insert(0,str(p))
from build_motor_plugin import plugin_environment
import uuid
os.environ['GZ_PARTITION']='ugv-tt-check-'+uuid.uuid4().hex
env=plugin_environment();env['GZ_PARTITION']=os.environ['GZ_PARTITION']
from gz.transport import Node
from gz.msgs.twist_pb2 import Twist
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.double_v_pb2 import Double_V
n=Node();pub=n.advertise('/model/freenove/cmd_vel',Twist)
duty_pub=n.advertise('/model/freenove/wheel_duty',Double_V)
pose={}; samples=[]
def poses(m):
 for q in m.pose:
  if q.name=='freenove':
   pose.update(x=q.position.x,y=q.position.y,z=q.position.z,yaw=math.atan2(2*q.orientation.w*q.orientation.z,1-2*q.orientation.z*q.orientation.z))
def motors(m): samples.append(list(m.data))
n.subscribe(Pose_V,'/world/freenove_lab/dynamic_pose/info',poses)
n.subscribe(Double_V,'/model/freenove/motor_state',motors)
(p/'.control-logs').mkdir(exist_ok=True)
log=(p/'.control-logs/physics-check.log').open('w')
proc=subprocess.Popen(['gz','sim','-s','-r','-v','3',str(p/'freenove_world.sdf')],cwd=p,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def drive(v,w,secs):
 end=time.monotonic()+secs
 while time.monotonic()<end:
  m=Twist();m.linear.x=v;m.angular.z=w;pub.publish(m);time.sleep(.05)
try:
 deadline=time.monotonic()+15
 while (not pose or not samples) and time.monotonic()<deadline: time.sleep(.1)
 assert pose and samples,'Motor system or physics did not initialize'
 drive(0,0,.4); before=dict(pose); drive(.10,0,2); straight=dict(pose)
 drive(0,0,.8); stopped=dict(pose);drive(0,0,.5); after=dict(pose)
 drive(0,.7,3); pivot=dict(pose); print("Pivot wheel telemetry:",samples[-1],flush=True); drive(0,0,.5)
 yaw=math.atan2(math.sin(pivot['yaw']-after['yaw']),math.cos(pivot['yaw']-after['yaw']))
 torque=max(abs(s[i]) for s in samples for i in (2,6,10,14))
 result=dict(before=before,straight=straight,stopped=stopped,after=after,pivot=pivot,pivot_delta_yaw=yaw,max_abs_torque_nm=torque,motor_samples=len(samples))
 print(json.dumps(result,indent=2),flush=True)
 assert straight['x']-before['x']>.08,'Forward command did not move car'
 assert math.hypot(after['x']-stopped['x'],after['y']-stopped['y'])<.01,'Car did not brake'
 assert yaw>.1,'Positive pivot command did not turn car'
 from motor_physics import load_profile
 assert torque<=load_profile()['stall_nm']+1e-8,'Torque exceeded reference stall bound'
 assert abs(pivot['z'])<.01,'Car lost stable wheel contact'
 # Verify direct real-car PWM and loss of command heartbeat independently.
 pwm_before=dict(pose);end=time.monotonic()+1.0
 while time.monotonic()<end:
  m=Double_V();m.data.extend([1800/4095]*2);duty_pub.publish(m);time.sleep(.05)
 time.sleep(.9);watchdog_stop=dict(pose);time.sleep(.35);watchdog_after=dict(pose)
 assert math.hypot(watchdog_stop['x']-pwm_before['x'],watchdog_stop['y']-pwm_before['y'])>.1,'PWM drive did not move car'
 assert math.hypot(watchdog_after['x']-watchdog_stop['x'],watchdog_after['y']-watchdog_stop['y'])<.005,'PWM watchdog did not brake'
 assert all(abs(samples[-1][i])<1e-9 for i in (3,7,11,15)),'PWM duty did not expire'
 assert max(abs(s[i]) for s in samples for i in (2,6,10,14))<=load_profile()['stall_nm']+1e-8
 print('PASS: velocity + PWM drive, dynamic braking, rubber pivot, stable support, finite torque, 400ms PWM deadman',flush=True)
finally:
 pub.publish(Twist());os.killpg(proc.pid,signal.SIGINT)
 try: proc.wait(timeout=5)
 except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM)
