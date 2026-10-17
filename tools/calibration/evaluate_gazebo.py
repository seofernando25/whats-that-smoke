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
        for keys in args.keys.split(','):
            drive('',args.pwm,.7)
            begin=len(poses);drive(keys,args.pwm,args.duration);end=len(poses)
            drive('',args.pwm,1.0)
            trajectory=poses[begin:];active=poses[begin:end]
            rates=[]
            for a,b in zip(active,active[1:]):
                dt=b['t']-a['t']
                if dt>0:rates.append((b['t'],math.degrees(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))/dt))
            tail=[r for t,r in rates if t>active[0]['t']+.65*(active[-1]['t']-active[0]['t'])]
            item=dict(keys=keys,pwm=args.pwm,yaw_dps=statistics.median(tail),trajectory=trajectory,
                      sim_duration_s=active[-1]['t']-active[0]['t'])
            results.append(item);print(keys,round(item['yaw_dps'],2),flush=True)
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
    p.add_argument('--profile',action='store_true');p.add_argument('--slip',type=float);p.add_argument('--pwm',type=int,default=1800)
    p.add_argument('--keys',default='Q,E,WA,WD,SD,SA');p.add_argument('--duration',type=float,default=2)
    p.add_argument('--output',required=True);run(p.parse_args())
