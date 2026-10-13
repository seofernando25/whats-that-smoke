"""Short visual-odometry bridge, metrically anchored by fresh fiducial poses.

No accelerometer double integration or assumed motor displacement is used.
Failed feature geometry returns no pose; it never reports guessed free space.
"""
from __future__ import annotations
import math
import cv2
import numpy as np


def invert(matrix):
    out=np.eye(4);out[:3,:3]=matrix[:3,:3].T;out[:3,3]=-out[:3,:3]@matrix[:3,3]
    return out


class SceneOdometry:
    def __init__(self, camera_matrix, distortion=None):
        self.k=np.array(camera_matrix,float).copy();self.k[:2]/=2
        self.dist=np.array(distortion if distortion is not None else np.zeros(5),float)
        self.key_image=None;self.key_pose=None;self.key_points=None
        self.cloud=None;self.points=None;self.previous=None;self.last_pose=None
        self.inliers=0;self.reprojection_px=None;self.status='needs translated tag observations'

    @staticmethod
    def features(gray):
        mask=np.zeros_like(gray);mask[int(gray.shape[0]*.20):int(gray.shape[0]*.95)]=255
        return cv2.goodFeaturesToTrack(gray,300,.015,9,mask=mask)

    @staticmethod
    def flow(before,after,points):
        if points is None:return None,None
        new,ok,_=cv2.calcOpticalFlowPyrLK(before,after,points,None,winSize=(31,31),maxLevel=3)
        if new is None:return None,None
        back,back_ok,_=cv2.calcOpticalFlowPyrLK(after,before,new,None,winSize=(31,31),maxLevel=3)
        if back is None:return None,None
        good=ok.reshape(-1).astype(bool)&back_ok.reshape(-1).astype(bool)
        good&=np.linalg.norm(back.reshape(-1,2)-points.reshape(-1,2),axis=1)<1.5
        q=new.reshape(-1,2);good&=(q[:,0]>3)&(q[:,0]<after.shape[1]-3)&(q[:,1]>3)&(q[:,1]<after.shape[0]-3)
        return new,good

    def update(self, gray, measured_pose=None):
        try:
            return self._update(gray,measured_pose)
        except (cv2.error, ValueError, FloatingPointError):
            self.cloud=None;self.points=None;self.previous=None
            self.status='feature geometry rejected'
            return measured_pose

    def _update(self, gray, measured_pose=None):
        gray=cv2.resize(gray,None,fx=.5,fy=.5)
        result=None;self.inliers=0
        if self.cloud is not None and self.previous is not None and len(self.cloud)>=20:
            new,good=self.flow(self.previous,gray,self.points)
            if new is not None and good.sum()>=20:
                self.cloud=self.cloud[good];self.points=new[good]
                if measured_pose is None:
                    ok,rvec,tvec,inliers=cv2.solvePnPRansac(self.cloud,self.points.reshape(-1,2),self.k,self.dist,
                                                         iterationsCount=100,reprojectionError=2.,confidence=.995,
                                                         flags=cv2.SOLVEPNP_SQPNP)
                    if ok and inliers is not None and len(inliers)>=20 and len(inliers)>=.6*len(self.cloud):
                        inds=inliers.reshape(-1);pixels=self.points[inds].reshape(-1,2)
                        span=np.ptp(pixels,axis=0)
                        if span[0]>gray.shape[1]*.25 and span[1]>gray.shape[0]*.15:
                            rvec,tvec=cv2.solvePnPRefineLM(self.cloud[inds],pixels,self.k,self.dist,rvec,tvec)
                            rotation,_=cv2.Rodrigues(rvec)
                            camera=np.eye(4);camera[:3,:3]=rotation;camera[:3,3]=tvec.reshape(3)
                            result=invert(camera)
                            projected,_=cv2.projectPoints(self.cloud[inds],rvec,tvec,self.k,self.dist)
                            error=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-pixels)**2,axis=1))))
                            if error>1.5 or (self.last_pose is not None and np.linalg.norm(result[:3,3]-self.last_pose[:3,3])>.25):
                                result=None
                            else:
                                self.inliers=len(inds);self.reprojection_px=error
                                self.cloud=self.cloud[inds];self.points=self.points[inds]
            else:
                self.cloud=None;self.points=None
        if measured_pose is not None:
            result=measured_pose
            if self.key_image is None:
                self.key_image=gray.copy();self.key_pose=measured_pose.copy();self.key_points=self.features(gray)
            elif np.linalg.norm(measured_pose[:3,3]-self.key_pose[:3,3])>.06:
                moved,good=self.flow(self.key_image,gray,self.key_points)
                if moved is not None and good.sum()>=25:
                    a=self.key_points[good].reshape(-1,2);b=moved[good].reshape(-1,2)
                    # Triangulate undistorted normalized rays. Distorted pixels
                    # remain available for optical flow and PnP reprojection.
                    ua=cv2.undistortPoints(a.reshape(-1,1,2),self.k,self.dist).reshape(-1,2)
                    ub=cv2.undistortPoints(b.reshape(-1,1,2),self.k,self.dist).reshape(-1,2)
                    p1=invert(self.key_pose)[:3];p2=invert(measured_pose)[:3]
                    homogeneous=cv2.triangulatePoints(p1,p2,ua.T,ub.T)
                    nonzero=np.abs(homogeneous[3])>1e-8
                    world=(homogeneous[:3]/np.where(nonzero,homogeneous[3],1)).T
                    cam1=world@invert(self.key_pose)[:3,:3].T+invert(self.key_pose)[:3,3]
                    cam2=world@invert(measured_pose)[:3,:3].T+invert(measured_pose)[:3,3]
                    ray1=world-self.key_pose[:3,3];ray2=world-measured_pose[:3,3]
                    ray1/=np.maximum(np.linalg.norm(ray1,axis=1,keepdims=True),1e-9)
                    ray2/=np.maximum(np.linalg.norm(ray2,axis=1,keepdims=True),1e-9)
                    parallax=np.arccos(np.clip(np.sum(ray1*ray2,axis=1),-1,1))
                    with np.errstate(divide='ignore',invalid='ignore'):
                        e1=np.linalg.norm((cam1[:,:2]/cam1[:,2:]-ua)*[self.k[0,0],self.k[1,1]],axis=1)
                        e2=np.linalg.norm((cam2[:,:2]/cam2[:,2:]-ub)*[self.k[0,0],self.k[1,1]],axis=1)
                    valid=nonzero&np.isfinite(world).all(axis=1)&(cam1[:,2]>.1)&(cam2[:,2]>.1)&(cam1[:,2]<8)&(cam2[:,2]<8)
                    valid&=(parallax>math.radians(.8))&(e1<1.5)&(e2<1.5)
                    if valid.sum()>=20:
                        self.cloud=world[valid].astype(np.float32);self.points=b[valid].astype(np.float32).reshape(-1,1,2)
                self.key_image=gray.copy();self.key_pose=measured_pose.copy();self.key_points=self.features(gray)
        if result is not None:self.last_pose=result.copy()
        self.previous=gray.copy()
        self.status='fiducial anchored' if measured_pose is not None else 'visual pose bridge' if result is not None else 'insufficient static scene geometry'
        return result

    def snapshot(self):
        return {'status':self.status,'mapped_features':len(self.cloud) if self.cloud is not None else 0,
                'inliers':self.inliers,'reprojection_error_px':self.reprojection_px}
