"""Drone-relative observation, conservative footprint planning and docking feedback.

The observer never commands motors. Unverified scale, mounting, envelope or gear
geometry prevents execution even when an illustrative route can be computed.
"""
from __future__ import annotations
import heapq, json, math, time, threading, os
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any
import numpy as np

_repository_config = Path(__file__).resolve().parents[2] / 'config' / 'parking.json'
CONFIG_PATH = Path(os.environ['WTS_PARKING_CONFIG']) if os.environ.get('WTS_PARKING_CONFIG') else (
    _repository_config if _repository_config.exists() else Path(__file__).with_name('default_parking.json'))


def angle_error(target, current):
    return (target - current + math.pi) % (2 * math.pi) - math.pi


def yaw_rotation(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def transform(rotation=None, translation=None):
    matrix = np.eye(4)
    if rotation is not None: matrix[:3, :3] = rotation
    if translation is not None: matrix[:3, 3] = translation
    return matrix


def inverse(matrix):
    return transform(matrix[:3, :3].T, -matrix[:3, :3].T @ matrix[:3, 3])


def valid_transform(value):
    try:
        matrix=np.asarray(value,dtype=float)
        return (matrix.shape==(4,4) and np.isfinite(matrix).all()
                and np.allclose(matrix[3],[0,0,0,1])
                and np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=.001)
                and abs(np.linalg.det(matrix[:3,:3])-1)<.001)
    except (TypeError,ValueError): return False


@dataclass(frozen=True)
class Pose2:
    x: float
    z: float
    yaw: float = 0.0  # radians; heading measured from +z toward +x


@dataclass(frozen=True)
class Capsule:
    ax: float
    az: float
    bx: float
    bz: float
    radius: float


def point_segment_distance(p, a, b):
    d = b - a
    den = float(d @ d)
    t = max(0., min(1., float((p - a) @ d) / den)) if den else 0.
    return float(np.linalg.norm(p - a - t * d))


def segment_distance(a, b, c, d):
    def cross(u, v): return float(u[0] * v[1] - u[1] * v[0])
    ab, cd = b - a, d - c
    den = cross(ab, cd)
    if abs(den) > 1e-10:
        t, u = cross(c - a, cd) / den, cross(c - a, ab) / den
        if 0 <= t <= 1 and 0 <= u <= 1: return 0.
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def footprint_corners(pose, width, length, inflation=0.):
    w, l = width / 2 + inflation, length / 2 + inflation
    local = np.array([[-w, -l], [w, -l], [w, l], [-w, l]])
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return local @ np.array([[c, -s], [s, c]]) + [pose.x, pose.z]


def collides(pose, width, length, obstacles, inflation=0.):
    corners = footprint_corners(pose, width, length, inflation)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    for obstacle in obstacles:
        a, b = np.array([obstacle.ax, obstacle.az]), np.array([obstacle.bx, obstacle.bz])
        for point in (a, b):
            dx, dz = point - [pose.x, pose.z]
            if abs(c * dx - s * dz) <= width / 2 + inflation and abs(s * dx + c * dz) <= length / 2 + inflation:
                return True
        for i in range(4):
            if segment_distance(a, b, corners[i], corners[(i + 1) % 4]) <= obstacle.radius:
                return True
    return False


def swept_clear(start, end, width, length, obstacles, inflation=0.):
    delta = angle_error(end.yaw, start.yaw)
    # Sample displacement of the furthest corner, not just the centre point.
    distance = math.hypot(end.x-start.x, end.z-start.z) + abs(delta)*math.hypot(width,length)/2
    steps = max(1, math.ceil(distance / .005))
    for i in range(steps + 1):
        t = i / steps
        pose = Pose2(start.x+t*(end.x-start.x), start.z+t*(end.z-start.z), start.yaw+t*delta)
        if collides(pose, width, length, obstacles, inflation+.0025): return False
    return True


def gear_obstacles(gear, vehicle_height=0.):
    gap = gear['clear_width_m'] - 2 * (gear.get('inward_slope') or 0.) * vehicle_height
    radius, length = gear['bar_radius_m'], gear['bar_length_m']
    return [Capsule(sign*(gap/2+radius), -length/2, sign*(gap/2+radius), length/2, radius)
            for sign in (-1, 1)]


def plan_route(start, goal, vehicle, obstacles, inflation=.015, max_nodes=18000):
    """Hybrid lattice search: forward/reverse and yaw, with swept OBB checks."""
    width, length = vehicle['width_m'], vehicle['length_m']
    if not width or not length: return []
    if collides(start,width,length,obstacles,inflation) or collides(goal,width,length,obstacles,inflation): return []
    resolution, yaw_step = .025, math.radians(10)
    def key(p): return (round(p.x/resolution), round(p.z/resolution), round(p.yaw/yaw_step)%36)
    def heuristic(p): return math.hypot(p.x-goal.x,p.z-goal.z) + .08*abs(angle_error(goal.yaw,p.yaw))
    initial = key(start)
    # Immutable search nodes keep reconstructed edges identical to the edges
    # whose swept footprint was checked, even when a lattice cell improves.
    nodes=[(start,None,0.)];costs={initial:0.}
    queue=[(heuristic(start),0)]
    for _ in range(max_nodes):
        if not queue: return []
        _,node=heapq.heappop(queue);p,parent,cost=nodes[node]
        if cost>costs.get(key(p),math.inf)+1e-10: continue
        if math.hypot(p.x-goal.x,p.z-goal.z)<.020 and abs(angle_error(goal.yaw,p.yaw))<yaw_step:
            aligned=Pose2(p.x,p.z,goal.yaw)
            if not swept_clear(p,aligned,width,length,obstacles,inflation): continue
            result=[aligned]
            while node is not None:
                pose,node,_=nodes[node];result.append(pose)
            return result[::-1]
        moves=[]
        for distance in (-resolution,resolution):
            moves.append((Pose2(p.x+math.sin(p.yaw)*distance,p.z+math.cos(p.yaw)*distance,p.yaw),abs(distance)*(1.25 if distance<0 else 1)))
        for direction in (-1,1):
            moves.append((Pose2(p.x,p.z,p.yaw+direction*yaw_step),.024))
        for candidate,step_cost in moves:
            if math.hypot(candidate.x-start.x,candidate.z-start.z)>2.0: continue
            if not swept_clear(p,candidate,width,length,obstacles,inflation): continue
            k=key(candidate);candidate_cost=cost+step_cost
            if candidate_cost>=costs.get(k,math.inf): continue
            costs[k]=candidate_cost;nodes.append((candidate,node,candidate_cost))
            heapq.heappush(queue,(candidate_cost+heuristic(candidate),len(nodes)-1))
    return []


def readiness(config, pose_age, sigma):
    reasons=[]; vehicle=config['vehicle']; gear=config['gear']
    def positive(v): return isinstance(v,(int,float)) and math.isfinite(v) and v>0
    model=config.get('motion_model',{})
    if (not model.get('confirmed') or not all(positive(model.get(k)) for k in ('forward_mps','reverse_mps','left_dps','right_dps','max_forward_mps','max_reverse_mps','max_left_dps','max_right_dps','stopping_distance_m','angular_stop_deg'))
        or any(model.get('max_'+k,0)<model.get(k,math.inf) for k in ('forward_mps','reverse_mps','left_dps','right_dps'))): reasons.append('motion response and stopping bounds unverified')
    try:
        k=np.asarray(config['camera_matrix'],float);d=np.asarray(config['distortion'],float)
        if k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0 or not np.isfinite(d).all() or not positive(config.get('tag_size_m')):
            reasons.append('invalid camera/tag calibration')
    except (TypeError,ValueError,KeyError): reasons.append('invalid camera/tag calibration')
    if not config.get('localization_error_confirmed') or not positive(config.get('position_error_bound_m')):
        reasons.append('localization error bound unverified')
    for field,label in (('tag_size_confirmed','tag size'),('camera_intrinsics_confirmed','camera intrinsics'),('camera_mount_confirmed','camera-to-car mounting')):
        if not config.get(field): reasons.append(f'{label} unverified')
    if not valid_transform(config.get('camera_to_car')): reasons.append('camera-to-car transform missing')
    if not valid_transform(config.get('drone_to_anchor')): reasons.append('drone-to-tag transform missing')
    if not all(positive(vehicle.get(k)) for k in ('width_m','length_m','height_m')): reasons.append('full car envelope unknown')
    if (not gear.get('confirmed') or not all(positive(gear.get(k)) for k in ('clear_width_m','bar_length_m','bar_radius_m','height_clearance_m'))
            or not isinstance(gear.get('inward_slope'),(int,float)) or not math.isfinite(gear['inward_slope']) or gear['inward_slope']<0):
        reasons.append('landing-gear clearance unverified')
    if not config.get('parking_goal') or not all(isinstance(config['parking_goal'].get(k),(int,float)) and math.isfinite(config['parking_goal'][k]) for k in ('x_m','z_m','yaw_deg')): reasons.append('drone centre/parking pose uncalibrated')
    if pose_age is None or not math.isfinite(pose_age) or pose_age<0 or pose_age>.25: reasons.append('fresh localized pose unavailable')
    if sigma is None or not math.isfinite(sigma) or sigma<0 or sigma>.015: reasons.append('position uncertainty too large for exact parking')
    if positive(vehicle.get('width_m')) and positive(vehicle.get('height_m')):
        gap=gear['clear_width_m']-2*(gear.get('inward_slope') or 0)*vehicle['height_m']
        if vehicle['width_m']+2*(config.get('clearance_margin_m',.015)+(sigma or .0))>=gap:
            reasons.append('insufficient lateral clearance including uncertainty')
        if gear.get('height_clearance_m') is not None and vehicle['height_m']+config.get('clearance_margin_m',.015)>=gear['height_clearance_m']:
            reasons.append('insufficient overhead clearance')
    return reasons


class DockingFeedback:
    """Bounded visual/IMU feedback for an already verified collision-free route."""
    def __init__(self, route, motion_model, vehicle, obstacles, inflation=.015):
        self.route=route;self.motion_model=motion_model;self.index=0;self.stable=0
        self.vehicle=vehicle;self.obstacles=obstacles;self.inflation=inflation

    def step(self, pose, blockers=()):
        if blockers or not self.route: return {'action':'stop','reason':'; '.join(blockers) or 'no route'}
        while self.index<len(self.route)-1 and math.hypot(pose.x-self.route[self.index].x,pose.z-self.route[self.index].z)<.02 and abs(angle_error(self.route[self.index].yaw,pose.yaw))<math.radians(3):
            self.index+=1
        goal=self.route[self.index];distance=math.hypot(goal.x-pose.x,goal.z-pose.z)
        direction=1.
        if distance>.02:
            bearing=math.atan2(goal.x-pose.x,goal.z-pose.z)
            direction=1. if math.cos(angle_error(bearing,goal.yaw))>=0 else -1.
            target_yaw=bearing if direction>0 else bearing+math.pi
        else:
            target_yaw=goal.yaw
        error=angle_error(target_yaw,pose.yaw)
        if abs(error)>math.radians(3):
            self.stable=0
            # Positive heading error is clockwise (manual E, negative turn).
            rate=self.motion_model['right_dps' if error>0 else 'left_dps']
            duration=max(.02,min(.10,abs(math.degrees(error))/rate*.7))
            bound=self.motion_model['max_right_dps' if error>0 else 'max_left_dps']
            predicted=Pose2(pose.x,pose.z,pose.yaw+math.copysign(math.radians(bound*duration+self.motion_model['angular_stop_deg']),error))
            if not swept_clear(pose,predicted,self.vehicle['width_m'],self.vehicle['length_m'],self.obstacles,self.inflation):
                return {'action':'stop','reason':'pivot footprint would intersect landing gear'}
            return {'action':'pivot','forward':0.,'turn':-math.copysign(1.,error),'duration':duration}
        if distance>.02:
            self.stable=0
            speed=self.motion_model['forward_mps' if direction>0 else 'reverse_mps']
            duration=max(.02,min(.12,distance/speed*.6))
            bound=self.motion_model['max_forward_mps' if direction>0 else 'max_reverse_mps']
            travel=bound*duration+self.motion_model['stopping_distance_m']
            predicted=Pose2(pose.x+math.sin(pose.yaw)*direction*travel,
                            pose.z+math.cos(pose.yaw)*direction*travel,pose.yaw)
            if not swept_clear(pose,predicted,self.vehicle['width_m'],self.vehicle['length_m'],self.obstacles,self.inflation):
                return {'action':'stop','reason':'advance footprint would intersect landing gear'}
            return {'action':'advance','forward':direction,'turn':0.,'duration':duration}
        self.stable+=1
        return {'action':'stop','reason':'parked' if self.index==len(self.route)-1 and self.stable>=3 else 'settling'}


class ParkingEstimator:
    def __init__(self, config_path=CONFIG_PATH):
        self.config=json.loads(Path(config_path).read_text())
        self.lock=threading.RLock()
        from .odometry import SceneOdometry
        self.odometry=SceneOdometry(self.config['camera_matrix'],self.config['distortion'])
        self.last_anchor=0.;self.last_anchor_sigma=None;self.pose_source='none'
        self.tags={};self.observations={};self.camera_pose=None;self.camera_reference=None
        self.imu_origin=None;self.last_localized=0.;self.sigma=None;self.status='waiting for drone tags'
        self.pan_tilt=None;self.updated=0.;self.rejected=0
        self.coobservations={}
        self.relocalization=[];self.map_anchor=None

    def update(self, detections, state, captured_at, now, gray=None):
        with self.lock:
            self._update(detections,state,captured_at,now,gray)

    def _bridge(self, gray, captured_at, now, state):
        if gray is None: return
        pose=self.odometry.update(gray)
        if pose is not None and state.heading_calibrated and self.imu_origin is not None:
            import cv2
            car_camera=np.asarray(self.config['camera_to_car']) if valid_transform(self.config.get('camera_to_car')) else np.eye(4)
            axis=self.camera_reference[:3,:3]@car_camera[:3,:3].T@np.array([0.,1.,0.])
            delta=math.radians((state.heading_deg-self.imu_origin+180)%360-180)
            expected=cv2.Rodrigues(axis*delta)[0]@self.camera_reference[:3,:3]
            difference=cv2.Rodrigues(expected.T@pose[:3,:3])[0]
            if np.linalg.norm(difference)>math.radians(8):
                self.status='scene rotation disagrees with IMU';return
        if pose is not None and self.last_anchor and captured_at-self.last_anchor<=1.5:
            self.camera_pose=pose;self.last_localized=captured_at;self.pose_source='scene features'
            self.sigma=self.last_anchor_sigma+.01*(captured_at-self.last_anchor)
            self.status='short static-scene visual bridge'

    def _update(self, detections, state, captured_at, now, gray=None):
        import cv2
        self.updated=now
        pan_tilt=(state.pan_us,state.tilt_us)
        if self.pan_tilt is not None and pan_tilt!=self.pan_tilt:
            self.status='camera moved; localization requires recalibration';self.camera_pose=None
            self.camera_reference=None;self.imu_origin=None
            self.tags.clear();self.coobservations.clear();self.last_anchor=0.
            from .odometry import SceneOdometry
            self.odometry=SceneOdometry(self.config['camera_matrix'],self.config['distortion'])
        self.pan_tilt=pan_tilt
        allowed=set(self.config['drone_tag_ids']); measurements={}
        for item in detections:
            if item.tag_id not in allowed or item.tracked or item.confidence<.99 or not item.translation_vector or item.rotation_vector is None:
                continue
            if item.reprojection_error_px is None or item.reprojection_error_px>2.0:
                self.rejected+=1;continue
            if self.config.get('camera_intrinsics_confirmed'):
                half=self.config['tag_size_m']/2
                object_points=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
                pixels=np.asarray(item.corners,np.float32)
                try:
                    k=np.asarray(self.config['camera_matrix'],float);dist=np.asarray(self.config['distortion'],float)
                    ok,rvec,tvec=cv2.solvePnP(object_points,pixels,k,dist,flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    projected=cv2.projectPoints(object_points,rvec,tvec,k,dist)[0].reshape(4,2)
                    error=np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1)))
                    if not ok or tvec[2,0]<=0 or not np.isfinite(tvec).all() or error>2:
                        self.rejected+=1;continue
                except (cv2.error,ValueError): self.rejected+=1;continue
                rotation=cv2.Rodrigues(rvec)[0];translation=tvec.reshape(3)
            else:
                rotation=cv2.Rodrigues(np.asarray(item.rotation_vector,float))[0]
                translation=np.asarray(item.translation_vector)*self.config['tag_size_m']/.05
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                self.rejected+=1;continue
            measurements[item.tag_id]=transform(rotation,translation)
            self.observations[item.tag_id]=self.observations.get(item.tag_id,0)+1
        if not measurements:
            self.status='holding pose; no fresh drone-tag measurement'
            self._bridge(gray,captured_at,now,state);return
        if not self.tags:
            anchor=min(measurements)
            self.map_anchor=anchor
            reference=inverse(measurements[anchor])
            # Camera-aligned provisional horizontal frame; true floor/body
            # mounting calibration remains an explicit prerequisite for docking.
            basis=reference[:3,:3]@np.diag([1.,-1.,-1.])
            self.tags[anchor]=transform(basis.T)
            self.camera_reference=self.tags[anchor]@reference
            self.imu_origin=state.heading_deg if state.heading_calibrated else None
        known=[tag for tag in measurements if tag in self.tags]
        if not known:
            self.status='unconnected tag; requires co-visibility with mapped anchor'
            self._bridge(gray,captured_at,now,state);return
        candidates=[]
        for tag in known:
            candidate=self.tags[tag]@inverse(measurements[tag])
            if self.imu_origin is not None and state.heading_calibrated:
                delta=math.radians((state.heading_deg-self.imu_origin+180)%360-180)
                # IMU heading is clockwise about car-down, which is map -Y
                # for the provisional image-aligned frame. Use calibrated
                # mounting when available rather than assuming level camera.
                car_camera=np.asarray(self.config['camera_to_car']) if valid_transform(self.config.get('camera_to_car')) else np.eye(4)
                axis=self.camera_reference[:3,:3]@car_camera[:3,:3].T@np.array([0.,1.,0.])
                rotation=cv2.Rodrigues(axis*delta)[0]@self.camera_reference[:3,:3]
                candidate=transform(rotation,self.tags[tag][:3,3]-rotation@measurements[tag][:3,3])
            candidates.append(candidate)
        position=np.median(np.array([p[:3,3] for p in candidates]),axis=0)
        if self.camera_pose is not None and np.linalg.norm(position-self.camera_pose[:3,3])>.25:
            self.rejected+=1
            if captured_at-self.last_localized<=.45:
                self.relocalization.clear();self.status='pose jump rejected';return
            # A deliberately changed outside viewpoint must reacquire without
            # requiring the operator to reset. Three consistent fresh anchor
            # poses permit reacquisition; execution was already stopped stale.
            self.relocalization.append(position.copy());del self.relocalization[:-3]
            if len(self.relocalization)<3 or np.max(np.linalg.norm(np.array(self.relocalization)-np.median(self.relocalization,axis=0),axis=1))>.03:
                self.status='reacquiring mapped drone tags';return
            from .odometry import SceneOdometry
            self.odometry=SceneOdometry(self.config['camera_matrix'],self.config['distortion'])
        self.relocalization.clear()
        self.camera_pose=transform(candidates[0][:3,:3],position)
        self.last_localized=captured_at
        distance=min(float(np.linalg.norm(t[:3,3])) for t in measurements.values())
        self.sigma=.005+distance*.015 if self.config['tag_size_confirmed'] and self.config['camera_intrinsics_confirmed'] else .08+distance*.10
        if self.config.get('localization_error_confirmed') and self.config.get('position_error_bound_m'):
            self.sigma=max(self.sigma,self.config['position_error_bound_m'])
        self.last_anchor=captured_at;self.last_anchor_sigma=self.sigma;self.pose_source='drone tags + IMU'
        if gray is not None: self.odometry.update(gray,self.camera_pose)
        for tag,measurement in measurements.items():
            estimate=self.camera_pose@measurement
            if tag not in self.tags:
                samples=self.coobservations.setdefault(tag,[])
                samples.append(estimate)
                del samples[:-8]
                if len(samples)>=3:
                    points=np.array([sample[:3,3] for sample in samples])
                    median=np.median(points,axis=0)
                    if np.max(np.linalg.norm(points-median,axis=1))<.05:
                        self.tags[tag]=transform(samples[-1][:3,:3],median)
            elif tag in self.tags and tag!=self.map_anchor and len(known)>1:
                # Limit map drift; reject inconsistent rigid-object geometry.
                if np.linalg.norm(self.tags[tag][:3,3]-estimate[:3,3])<.08:
                    self.tags[tag][:3,3]=.9*self.tags[tag][:3,3]+.1*estimate[:3,3]
        self.status='nominal tag/IMU localization' if self.sigma>.015 else 'localized'

    def snapshot(self, now=None):
        with self.lock: return self._snapshot(now)

    def _snapshot(self, now=None):
        now=time.monotonic() if now is None else now
        age=now-self.last_localized if self.last_localized else None
        pose=None
        if self.camera_pose is not None:
            p=self.camera_pose[:3,3];r=self.camera_pose[:3,:3]
            pose={'x_m':round(float(p[0]),3),'z_m':round(float(p[2]),3),
                  'yaw_deg':round(math.degrees(math.atan2(r[0,2],r[2,2])),1),'reference':'camera in provisional tag frame'}
        tags=[{'id':tag,'x_m':round(float(matrix[0,3]),3),'z_m':round(float(matrix[2,3]),3),'observations':self.observations.get(tag,0)} for tag,matrix in self.tags.items()]
        body_pose=None
        anchor=self.config.get('drone_anchor_id')
        if (self.camera_pose is not None and anchor in self.tags
                and valid_transform(self.config.get('camera_to_car'))
                and valid_transform(self.config.get('drone_to_anchor'))):
            drone_camera=np.array(self.config['drone_to_anchor'])@inverse(self.tags[anchor])@self.camera_pose
            drone_car=drone_camera@inverse(np.array(self.config['camera_to_car']))
            p,r=drone_car[:3,3],drone_car[:3,:3]
            body_pose={'x_m':float(p[0]),'z_m':float(p[2]),'yaw_deg':math.degrees(math.atan2(r[0,2],r[2,2]))}
        blockers=readiness(self.config,age if body_pose is not None else None,self.sigma)
        if self.pan_tilt and list(self.pan_tilt)!=self.config.get('camera_mount_servo',[1500,1500]):
            blockers.append('camera pan/tilt differs from calibrated mounting')
        if body_pose is None: blockers.append('car centre not localized in drone frame')
        return {'status':self.status,'odometry':self.odometry.snapshot(),'pose_source':self.pose_source,'camera_pose':pose,'pose_age_ms':round(age*1000) if age is not None else None,
                'position_sigma_m':round(self.sigma,3) if self.sigma is not None else None,
                'tags':tags,'seen_tag_ids':sorted(self.observations),'entry_ready':not blockers,'blockers':blockers,
                'gear':self.config['gear'],'vehicle':self.config['vehicle'],'motion_model':self.config['motion_model'],
                'rejected_poses':self.rejected,'distance_sensor':'not verified; no obstacle-range authority',
                'car_pose':body_pose,'car_pose_proxy':pose,'map_reference':'first mapped drone tag; not yet drone centre'}

    def plan_snapshot(self):
        state=self.snapshot()
        if state['blockers']: return {'ok':False,'blockers':state['blockers'],'path':[]}
        p=state['car_pose']; goal=self.config['parking_goal']
        start=Pose2(p['x_m'],p['z_m'],math.radians(p['yaw_deg']))
        end=Pose2(goal['x_m'],goal['z_m'],math.radians(goal['yaw_deg']))
        obstacles=gear_obstacles(self.config['gear'],self.config['vehicle']['height_m'])
        inflation=self.config['clearance_margin_m']+self.sigma
        route=plan_route(start,end,self.config['vehicle'],obstacles,inflation)
        return {'ok':bool(route),'blockers':[] if route else ['no collision-free route'],
                'path':[asdict(pose) for pose in route]}
