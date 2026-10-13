"""Ground-skid rectangle pose used to observe an empirically demonstrated gap.

Coordinates: x right, y down, z along the skids into the drone. Points must
identify the same four end-cap centres, ordered left near/right near/right
far/left far. Approximate dimensions remain labelled approximate; this
observer does not confer metric calibration or command the wheels.
"""
from dataclasses import dataclass
import math
import cv2
import numpy as np


@dataclass
class GroundPose:
    camera_xyz: np.ndarray
    camera_to_ground: np.ndarray
    yaw_deg: float
    reprojection_px: float
    image_points: np.ndarray


class SkidPoseObserver:
    def __init__(self, camera_matrix, clear_width=.254, bar_length=.3048, radius=.03):
        self.k=np.asarray(camera_matrix,float)
        w=clear_width/2+radius;l=bar_length/2
        self.objects=np.array([[-w,0,-l],[w,0,-l],[w,0,l],[-w,0,l]],float)
        self.previous=None;self.points=None;self.pose=None

    def solve(self, image_points, previous_yaw=None):
        pixels=np.asarray(image_points,np.float64).reshape(4,2)
        if not np.isfinite(pixels).all() or cv2.contourArea(pixels.astype(np.float32))<500:return None
        try:
            result=cv2.solvePnPGeneric(self.objects,pixels,self.k,np.zeros(5),flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:return None
        candidates=[]
        for rvec,tvec in zip(result[1],result[2]):
            rotation=cv2.Rodrigues(rvec)[0];position=-rotation.T@tvec.reshape(3)
            projected=cv2.projectPoints(self.objects,rvec,tvec,self.k,np.zeros(5))[0].reshape(4,2)
            error=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
            front=rotation.T@np.array([0,0,1.]);yaw=math.degrees(math.atan2(front[0],front[2]))
            # Reject mirrored/below-plane solutions. Height is relative to
            # cap centres, so a low camera may be only a few mm above them.
            if not np.isfinite(position).all() or position[1]>=0 or not -.5<position[1]<0 or rotation[1,1]<.7 or error>8:continue
            if previous_yaw is not None and abs((yaw-previous_yaw+180)%360-180)>15:continue
            candidates.append(GroundPose(position,rotation.T,yaw,error,pixels))
        return min(candidates,key=lambda p:p.reprojection_px) if candidates else None

    def seed(self, gray, points):
        pose=self.solve(points)
        if pose is None:raise ValueError('skid rectangle is not geometrically consistent')
        self.previous=gray.copy();self.points=np.asarray(points,np.float32).reshape(4,2);self.pose=pose
        return pose

    def update(self, gray, heading_delta=None):
        if self.previous is None:return None
        new=[]
        for centre in self.points:
            mask=np.zeros_like(self.previous)
            x,y=np.round(centre).astype(int)
            cv2.rectangle(mask,(max(0,x-22),max(0,y-22)),(min(mask.shape[1]-1,x+22),min(mask.shape[0]-1,y+22)),255,-1)
            features=cv2.goodFeaturesToTrack(self.previous,30,.01,3,mask=mask)
            if features is None or len(features)<3:return None
            moved,good,_=cv2.calcOpticalFlowPyrLK(self.previous,gray,features,None,winSize=(31,31),maxLevel=3)
            if moved is None:return None
            back,back_ok,_=cv2.calcOpticalFlowPyrLK(gray,self.previous,moved,None,winSize=(31,31),maxLevel=3)
            if back is None:return None
            good=good.reshape(-1).astype(bool)&back_ok.reshape(-1).astype(bool)
            good&=np.linalg.norm((back-features).reshape(-1,2),axis=1)<1
            if good.sum()<3:return None
            shifts=(moved-features).reshape(-1,2)[good]
            median=np.median(shifts,axis=0)
            if np.median(np.linalg.norm(shifts-median,axis=1))>1.5:return None
            new.append(centre+median)
        expected=self.pose.yaw_deg+(heading_delta or 0)
        pose=self.solve(new,expected)
        if pose is None:return None
        if heading_delta is not None and abs((pose.yaw_deg-expected+180)%360-180)>8:return None
        self.points=np.array(new,np.float32);self.previous=gray.copy();self.pose=pose
        return pose

    def sensitivity(self, points, samples=80, pixel_noise=.5):
        rng=np.random.default_rng(934);positions=[];yaws=[]
        initial=self.solve(points)
        if initial is None:return None
        for _ in range(samples):
            pose=self.solve(np.asarray(points)+rng.normal(0,pixel_noise,(4,2)),initial.yaw_deg)
            if pose is not None:positions.append(pose.camera_xyz);yaws.append(pose.yaw_deg)
        if len(positions)<samples*.8:return None
        return {'position_std_m':np.std(positions,axis=0).tolist(),'yaw_std_deg':float(np.std(yaws)),
                'pixel_noise_px':pixel_noise,'includes_scale_and_mounting_error':False}
