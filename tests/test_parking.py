import asyncio, copy, json, math, time
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
import pytest
from whats_that_smoke.parking import (Pose2, Capsule, collides, swept_clear, plan_route,
    readiness, ParkingEstimator, DockingFeedback, transform, inverse, gear_obstacles, CONFIG_PATH)
from whats_that_smoke.odometry import SceneOdometry
from whats_that_smoke.aruco import Detection
from whats_that_smoke.web import RobotController, RobotState


def test_collision_checks_full_rectangle_and_rotating_corners():
    obstacle=[Capsule(.11,.13,.11,.13,.005)]
    a=Pose2(0,0,0);b=Pose2(0,0,math.pi/2)
    assert not collides(a,.15,.3,obstacle)
    assert not collides(b,.15,.3,obstacle)
    assert not swept_clear(a,b,.15,.3,obstacle)
    assert collides(a,.15,.3,[Capsule(0,0,0,0,.005)])


def test_route_and_every_sweep_fit_between_gear():
    vehicle={'width_m':.16,'length_m':.24}
    gear={'clear_width_m':.254,'bar_radius_m':.03,'bar_length_m':.3048,'inward_slope':0}
    obstacles=gear_obstacles(gear)
    route=plan_route(Pose2(0,-.5),Pose2(0,0),vehicle,obstacles,.015)
    assert route and math.hypot(route[-1].x,route[-1].z)<.02
    assert all(swept_clear(a,b,.16,.24,obstacles,.015) for a,b in zip(route,route[1:]))
    assert not plan_route(Pose2(0,-.5),Pose2(0,0),{'width_m':.24,'length_m':.3},obstacles,.015)


def verified_config():
    c=json.loads(CONFIG_PATH.read_text())
    c.update(tag_size_confirmed=True,camera_intrinsics_confirmed=True,camera_mount_confirmed=True,
             camera_to_car=np.eye(4).tolist(),drone_to_anchor=np.eye(4).tolist(),
             parking_goal={'x_m':0,'z_m':0,'yaw_deg':0},localization_error_confirmed=True,position_error_bound_m=.005)
    c['motion_model']['confirmed']=True
    c['vehicle'].update(width_m=.16,length_m=.24,height_m=.1)
    c['gear'].update(confirmed=True,height_clearance_m=.3,inward_slope=0)
    return c


def test_nominal_geometry_and_missing_pose_cannot_enable_entry():
    c=json.loads(CONFIG_PATH.read_text())
    reasons=readiness(c,None,.2)
    assert 'full car envelope unknown' in reasons
    assert 'localization error bound unverified' in reasons
    assert readiness(verified_config(),.1,.01)==[]
    c=verified_config();c['vehicle']['width_m']=.24
    assert any('lateral' in r for r in readiness(c,.1,.01))
    c=verified_config();c['vehicle']['length_m']=float('nan')
    assert 'full car envelope unknown' in readiness(c,.1,.01)
    c=verified_config();c['camera_to_car'][0][0]=2
    assert 'camera-to-car transform missing' in readiness(c,.1,.01)
    assert 'fresh localized pose unavailable' in readiness(verified_config(),.5,.01)


def detection(tag_id, matrix):
    rvec=cv2.Rodrigues(matrix[:3,:3])[0].reshape(3).tolist()
    return Detection(tag_id,[[100.,100.]]*4,100,100,50,1.,rotation_vector=rvec,
                     translation_vector=matrix[:3,3].tolist(),reprojection_error_px=.2)


def test_imu_clockwise_axis_matches_camera_frame_without_fake_translation():
    estimator=ParkingEstimator();state=RobotState(heading_calibrated=True,heading_deg=0)
    measurement=transform(np.diag([1.,-1.,-1.]),[0,0,1.5])
    estimator.update([detection(9,measurement)],state,1.,1.)
    initial=estimator.camera_pose.copy();tag=estimator.tags[9].copy()
    # Left Q decreases clockwise heading. Car down is map -Y.
    angle=math.radians(-10)
    rotated=initial.copy();rotated[:3,:3]=cv2.Rodrigues(np.array([0.,-1.,0.])*angle)[0]@initial[:3,:3]
    state.heading_deg=350
    estimator.update([detection(9,inverse(rotated)@tag),detection(17,measurement)],state,1.1,1.1)
    assert np.linalg.norm(estimator.camera_pose[:3,3]-initial[:3,3])<1e-6
    assert np.allclose(estimator.camera_pose[:3,:3],rotated[:3,:3])
    assert 17 not in estimator.observations


def test_tag_loss_bridge_expires_and_pan_changes_invalidate_map():
    e=ParkingEstimator();state=RobotState(heading_calibrated=True,heading_deg=0)
    e.update([detection(9,transform(np.diag([1.,-1.,-1.]),[0,0,1]))],state,10,10)
    pose=e.camera_pose.copy()
    with patch.object(e.odometry,'update',return_value=pose):
        e.update([],state,11,11,np.zeros((720,1280),np.uint8))
        assert e.last_localized==11
        e.update([],state,12,12,np.zeros((720,1280),np.uint8))
        assert e.last_localized==11
    state.pan_us=1550;e.update([],state,12.1,12.1)
    assert not e.tags and e.camera_pose is None and e.last_anchor==0


def test_reverse_feedback_and_blocked_pivot_stop():
    c=verified_config();v=c['vehicle']
    feedback=DockingFeedback([Pose2(0,-.1)],c['motion_model'],v,[])
    result=feedback.step(Pose2(0,0))
    assert result['action']=='advance' and result['forward']==-1
    assert .02<=result['duration']<=.12
    assert feedback.step(Pose2(0,0),['stale'])['action']=='stop'
    feedback=DockingFeedback([Pose2(0,0,math.pi/2)],c['motion_model'],v,[Capsule(.11,.1,.11,.1,.01)])
    assert feedback.step(Pose2(0,0))['action']=='stop'


def test_blank_and_pure_rotation_do_not_invent_metric_depth():
    odometry=SceneOdometry([[968,0,640],[0,968,360],[0,0,1]])
    gray=np.zeros((720,1280),np.uint8)
    pose=np.eye(4);assert np.allclose(odometry.update(gray,pose),pose)
    pose[:3,:3]=cv2.Rodrigues(np.array([0.,.2,0.]))[0]
    odometry.update(gray,pose)
    assert odometry.cloud is None and odometry.update(gray) is None


def test_visual_pose_from_known_static_landmarks():
    odometry=SceneOdometry([[968,0,640],[0,968,360],[0,0,1]])
    rng=np.random.default_rng(17)
    points=rng.uniform([-1,-.6,2],[1,.6,4],(150,3)).astype(np.float32)
    moved=np.eye(4);moved[0,3]=.04
    pixels=cv2.projectPoints(points,np.zeros(3),np.array([-.04,0,0]),odometry.k,odometry.dist)[0].astype(np.float32)
    odometry.cloud=points;odometry.points=pixels.copy();odometry.previous=np.zeros((360,640),np.uint8);odometry.last_pose=np.eye(4)
    with patch.object(odometry,'flow',return_value=(pixels,np.ones(len(points),bool))):
        estimated=odometry.update(np.zeros((720,1280),np.uint8))
    assert estimated is not None and np.linalg.norm(estimated[:3,3]-moved[:3,3])<1e-4
    assert odometry.inliers>=20


def test_survey_deadline_and_parking_camera_interlock():
    async def scenario():
        c=RobotController();c.state.owner='test';c.state.armed=True;c._write=lambda wheels:None
        from whats_that_smoke import web
        with patch.object(web,'aruco',SimpleNamespace(follow=False)),patch.object(web.camera,'metrics',return_value={'camera_frame_age_ms':10}),patch.object(c,'_imu_ready',return_value=True):
            await c.survey_step('test',0,1,1800,.12)
            c._guard_check(c.drive_deadline)
            assert c.state.stopped
            with pytest.raises(ValueError):await c.survey_step('test',1,1,1800,.1)
            c.parking_active=True
            with pytest.raises(PermissionError):await c.camera_move('test','pan',50)
            with pytest.raises(PermissionError):await c.survey_step('test',1,0,1800,.1)
    asyncio.run(scenario())


def test_new_viewpoint_relocalizes_after_three_consistent_anchor_frames():
    e=ParkingEstimator();state=RobotState(heading_calibrated=True,heading_deg=0)
    measurement=transform(np.diag([1.,-1.,-1.]),[0,0,1.5])
    e.update([detection(9,measurement)],state,1.,1.)
    pose=e.camera_pose.copy();pose[0,3]+=.4
    shifted=detection(9,inverse(pose)@e.tags[9])
    e.update([shifted],state,1.1,1.1)
    assert e.last_localized==1.
    e.update([shifted],state,2.,2.)
    e.update([shifted],state,2.1,2.1)
    assert e.last_localized==1.
    e.update([shifted],state,2.2,2.2)
    assert e.last_localized==2.2 and np.allclose(e.camera_pose,pose)


def test_parking_guard_brakes_on_pose_expiry_independent_of_event_loop():
    c=RobotController();c.state.owner='test';c.state.armed=True;c.state.stopped=False
    c.parking_active=True;c.parking.last_localized=10.;c.parking_status='pivot'
    c._write=lambda wheels:None
    with patch.object(c,'_imu_ready',return_value=True):c._guard_check(10.31)
    assert c.state.stopped and not c.parking_active
    assert c.state.reason=='parking-sensors-stale'


def test_calibrated_parking_pose_uses_configured_intrinsics_and_tag_size():
    e=ParkingEstimator();e.config['camera_intrinsics_confirmed']=True;e.config['tag_size_m']=.08
    k=np.asarray(e.config['camera_matrix'],float);k[0,0]=1100;k[1,1]=1100;e.config['camera_matrix']=k.tolist()
    half=.04;objects=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],np.float32)
    rvec=np.array([math.pi,0.,0.]);tvec=np.array([.1,0.,1.2])
    pixels=cv2.projectPoints(objects,rvec,tvec,k,np.zeros(5))[0].reshape(4,2)
    item=detection(9,transform(np.diag([1.,-1.,-1.]),[.1,0,1.2]));item.corners=pixels.tolist()
    e.update([item],RobotState(heading_calibrated=True),1.,1.)
    assert np.isclose(np.linalg.norm(e.camera_pose[:3,3]),np.linalg.norm(tvec),atol=.01)


def test_stop_cancels_pending_planning_before_it_can_start_motors():
    from whats_that_smoke import web
    from unittest.mock import AsyncMock
    async def scenario():
        c=RobotController();c.state.owner='test';c.state.armed=True
        c.stop=AsyncMock();c.drive=AsyncMock();started=asyncio.Event();release=asyncio.Event()
        async def delayed_plan(*args):
            started.set();await release.wait()
            return {'ok':True,'path':[{'x':0,'z':0,'yaw':0}]}
        with patch.object(c,'_imu_ready',return_value=True),patch.object(c.parking,'snapshot',return_value={'blockers':[]}),patch.object(web,'aruco',SimpleNamespace(disable_follow=AsyncMock())),patch.object(web.asyncio,'to_thread',side_effect=delayed_plan):
            task=asyncio.create_task(c.start_parking('test'))
            await started.wait();await c.cancel_parking('client-stop');release.set()
            with pytest.raises(PermissionError):await task
        assert not c.parking_active and c.parking_task is None and c.drive.await_count==0
    asyncio.run(scenario())
