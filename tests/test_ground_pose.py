import math
import cv2
import numpy as np
from ugv.ground_pose import SkidPoseObserver


def test_ground_rectangle_recovers_camera_pose_and_rejects_degenerate_points():
    k=np.array([[968.,0,640],[0,968,360],[0,0,1]])
    observer=SkidPoseObserver(k)
    # Camera 100mm above cap centres, 0.8m outside the opening.
    rotation=cv2.Rodrigues(np.array([.08,.3,0.]))[0]
    centre=np.array([-.2,-.1,-.8]);translation=-rotation@centre
    pixels=cv2.projectPoints(observer.objects,cv2.Rodrigues(rotation)[0],translation,k,np.zeros(5))[0].reshape(4,2)
    pose=observer.solve(pixels)
    assert pose is not None and np.linalg.norm(pose.camera_xyz-centre)<1e-5
    assert observer.solve(np.zeros((4,2))) is None
    noise=observer.sensitivity(pixels)
    assert noise is not None and noise['includes_scale_and_mounting_error'] is False


def test_caps_with_no_trackable_texture_cannot_supply_new_pose():
    observer=SkidPoseObserver([[968.,0,640],[0,968,360],[0,0,1]])
    points=[[708,528],[987,528],[752,519],[520,516]]
    observer.seed(np.zeros((720,1280),np.uint8),points)
    assert observer.update(np.zeros((720,1280),np.uint8)) is None
