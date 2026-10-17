"""Evaluate open-loop PWM Gazebo response in an isolated transport partition."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as E

ROOT=Path(__file__).resolve().parents[2];GAZEBO=ROOT/'simulation/gazebo'
sys.path.insert(0,str(GAZEBO))
from build_motor_plugin import plugin_environment
from commands import wheel_duties


def run(args):
    os.environ['GZ_PARTITION']='ugv-pwm-fit-'+uuid.uuid4().hex
    from gz.transport import Node
    from gz.msgs.double_v_pb2 import Double_V
    from gz.msgs.pose_v_pb2 import Pose_V
    from gz.msgs.world_control_pb2 import WorldControl
    from gz.msgs.boolean_pb2 import Boolean
    env=plugin_environment();env['GZ_PARTITION']=os.environ['GZ_PARTITION']
    root=E.parse(GAZEBO/'freenove_world.sdf')
    plugin=root.find('.//plugin[@name="ugv::TTMotorSystem"]')
    if not args.profile:
        for key in ('empirical_pwm_map','pwm_rise_tau'):
            mapping=plugin.find(key)
            if mapping is not None:plugin.remove(mapping)
    for key,value in [('pwm_left_voltage_gain',args.left),('pwm_right_voltage_gain',args.right)]:
        el=plugin.find(key)
        if el is None:el=E.SubElement(plugin,key)
        el.text=str(value)
    if args.brake_damping is not None:
        el=plugin.find('brake_damping')
        if el is None:el=E.SubElement(plugin,'brake_damping')
        el.text=str(args.brake_damping)
    if args.mu is not None:
        for tag in ('mu','mu2'):
            for el in root.findall('.//surface/friction/ode/'+tag):el.text=str(args.mu)
    if args.slip is not None:
        for el in root.findall('.//surface/friction/ode/slip1'):el.text=str(args.slip)
    for link in root.findall('.//link'):
        for visual in link.findall('visual'):link.remove(visual)
    node=Node();pub=node.advertise('/model/freenove/wheel_duty',Double_V)
    poses=[];motors=[]
    def receive(m):
        for p in m.pose:
            if p.name=='freenove':
                q=p.orientation
                poses.append(dict(t=m.header.stamp.sec+m.header.stamp.nsec*1e-9,
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
                    x=p.position.x,y=p.position.y,z=p.position.z))
    node.subscribe(Pose_V,'/world/freenove_lab/dynamic_pose/info',receive)
    node.subscribe(Double_V,'/model/freenove/motor_state',lambda m:motors.append(list(m.data)))
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    tmp=tempfile.TemporaryDirectory(prefix='ugv-fit-');world=Path(tmp.name)/'world.sdf';root.write(world)
    log=out.with_suffix('.log').open('w')
    proc=subprocess.Popen(['gz','sim','-s','-r',str(world)],env=env,cwd=GAZEBO,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def drive(keys,pwm,duration):
        d=wheel_duties(keys,pwm)
        message=Double_V();message.data.extend([-d['front-left']/4095,-d['front-right']/4095])
        until=time.monotonic()+duration
        while time.monotonic()<until:pub.publish(message);time.sleep(.04)
    results=[]
    try:
        until=time.monotonic()+20
        while not poses and time.monotonic()<until:time.sleep(.1)
        if not poses:raise RuntimeError('Gazebo did not initialize')
        time.sleep(1.0)
        for keys in args.keys.split(','):
            if args.reset_between:
                request=WorldControl();request.pause=True;request.reset.all=True
                ok,response=node.request('/world/freenove_lab/control',request,WorldControl,Boolean,5000)
                if not ok or not response.data:
                    time.sleep(.5)
                    ok,response=node.request('/world/freenove_lab/control',request,WorldControl,Boolean,5000)
                    if not ok or not response.data:raise RuntimeError(f'Reset failed: {ok}, {response}')
                request=WorldControl();request.pause=False
                node.request('/world/freenove_lab/control',request,WorldControl,Boolean,5000)
                time.sleep(.3)
            drive('',args.pwm,.7)
            begin=len(poses);drive(keys,args.pwm,args.duration);end=len(poses)
            motor_state_at_end=list(motors[-1])
            drive('',args.pwm,1.0)
            trajectory=poses[begin:];active=poses[begin:end]
            rates=[]
            for a,b in zip(active,active[1:]):
                dt=b['t']-a['t']
                if dt>0:rates.append((b['t'],math.degrees(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))/dt))
            tail=[r for t,r in rates if t>active[0]['t']+.65*(active[-1]['t']-active[0]['t'])]
            body_forward=[];body_side=[]
            for a,b in zip(active,active[1:]):
                dt=b['t']-a['t']
                if dt>0 and b['t']>active[0]['t']+.65*(active[-1]['t']-active[0]['t']):
                    vx=(b['x']-a['x'])/dt;vy=(b['y']-a['y'])/dt
                    body_forward.append(vx*math.cos(a['yaw'])+vy*math.sin(a['yaw']))
                    body_side.append(-vx*math.sin(a['yaw'])+vy*math.cos(a['yaw']))
            after=poses[end:]
            stop_path=[active[-1],*after]
            distance=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(stop_path,stop_path[1:]))
            stop_time=None
            for j in range(len(stop_path)-5):
                window=stop_path[j:j+6]
                speeds=[math.hypot(b['x']-a['x'],b['y']-a['y'])/(b['t']-a['t']) for a,b in zip(window,window[1:]) if b['t']>a['t']]
                if speeds and max(speeds)<.02:
                    stop_time=window[0]['t']-active[-1]['t'];break
            v=statistics.median(body_forward);side=statistics.median(body_side);w=math.radians(statistics.median(tail))
            metrics=dict(body_forward_mps=v,body_lateral_mps=side,
                instantaneous_center_body_xy_m=[-side/w,v/w] if abs(w)>.1 else None,
                stopping_path_m=distance,stop_time_to_002_mps_s=stop_time,
                yaw_coast_deg=math.degrees(math.atan2(math.sin(stop_path[-1]['yaw']-stop_path[0]['yaw']),math.cos(stop_path[-1]['yaw']-stop_path[0]['yaw']))))
            item=dict(motor_state_at_end=motor_state_at_end,metrics=metrics,keys=keys,pwm=args.pwm,yaw_dps=statistics.median(tail),trajectory=trajectory,
                      sim_duration_s=active[-1]['t']-active[0]['t'])
            results.append(item);print(keys,round(item['yaw_dps'],2),metrics,flush=True)
        report=dict(left_gain=args.left,right_gain=args.right,slip=args.slip,
                    max_abs_torque_nm=max(abs(s[i]) for s in motors for i in (2,6,10,14)),results=results)
        out.write_text(json.dumps(report,indent=2))
    finally:
        drive('',args.pwm,.1);os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM)
        tmp.cleanup();log.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--left',type=float,default=1.475);p.add_argument('--right',type=float,default=1.475)
    p.add_argument('--reset-between',action='store_true');p.add_argument('--brake-damping',type=float);p.add_argument('--mu',type=float)
    p.add_argument('--profile',action='store_true');p.add_argument('--slip',type=float);p.add_argument('--pwm',type=int,default=1800)
    p.add_argument('--keys',default='Q,E,WA,WD,SD,SA');p.add_argument('--duration',type=float,default=2)
    p.add_argument('--output',required=True);run(p.parse_args())
