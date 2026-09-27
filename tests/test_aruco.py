import cv2
import numpy as np

from whats_that_smoke.aruco import ArucoFollower
from unittest.mock import patch, AsyncMock
from types import SimpleNamespace
import asyncio
import time


def test_rejected_perspective_quad_fallback_decodes_id_2():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 2, 96, borderBits=1)
    source = np.array([[0, 0], [95, 0], [95, 95], [0, 95]], dtype=np.float32)
    corners = np.array([[42, 26], [188, 14], [197, 169], [26, 177]], dtype=np.float32)
    warped = cv2.warpPerspective(
        marker,
        cv2.getPerspectiveTransform(source, corners),
        (240, 210),
        flags=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=255,
    )
    warped = cv2.GaussianBlur(warped, (3, 3), 0.6)

    found = ArucoFollower.decode_rejected(warped, [corners.reshape(1, 4, 2)], dictionary, 0.8)

    assert len(found) == 1
    tag_id, normalized_corners, bit_errors = found[0]
    assert tag_id == 2
    assert bit_errors == 0
    assert np.allclose(normalized_corners, corners)


def test_resampling_all_ids_rotations_and_blank_rejection():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    corners = np.array([[0, 0], [95, 0], [95, 95], [0, 95]], np.float32)
    for tag_id in range(50):
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 96)
        for turn in range(4):
            image = cv2.GaussianBlur(np.rot90(marker, turn).copy(), (3, 3), 0.6)
            found = ArucoFollower.decode_rejected(image, [corners], dictionary, 0.8)
            assert len(found) == 1 and found[0][0] == tag_id and found[0][2] == 0
    for value in (0, 127, 255):
        assert not ArucoFollower.decode_rejected(np.full((96, 96), value, np.uint8), [corners], dictionary, 0.8)


def sample_frame():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    image = np.full((720, 1280), 255, np.uint8)
    image[200:392, 400:592] = cv2.aruco.generateImageMarker(dictionary, 2, 192)
    return cv2.imencode(".jpg", image)[1].tobytes()


def test_oblique_blurred_cells_need_threshold_resampling_not_more_bit_errors():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    references = np.array([
        [np.rot90(cv2.aruco.generateImageMarker(dictionary, tag, 96).reshape(6, 16, 6, 16)
                  [:, 4:12, :, 4:12].mean(axis=(1, 3))[1:5, 1:5] > 127, rotation)
         for rotation in range(4)] for tag in range(50)
    ])
    # Cell measurements from the missed live candidate, without retaining a
    # photograph. Its three gray black cells cross the global Otsu threshold.
    means = np.array([[92,100,110,110,133,130], [118,202,122,215,222,126],
                      [159,166,192,179,165,124], [114,199,211,103,104,122],
                      [111,226,220,92,89,95], [110,137,122,94,96,102]], float)
    assert np.count_nonzero((means[1:5, 1:5] >= 161) != references[2, 2]) == 3
    assert ArucoFollower.decode_cells(means, 161, references, 1) == (2, 2, 0)
    # Compressing the contrast must not turn the same vague pattern into a
    # fresh ID. Nor should a damaged white border pass.
    assert ArucoFollower.decode_cells(150 + (means - 161) * .1, 150, references, 1) is None
    damaged = means.copy()
    damaged[0] = 250
    assert ArucoFollower.decode_cells(damaged, 161, references, 1) is None


def test_fresh_decode_skips_trackers_and_missed_decode_uses_flow():
    follower = ArucoFollower(None, None)
    frame = sample_frame()
    with patch.object(follower, "create_tracker", side_effect=AssertionError("unnecessary correlation")):
        with patch.object(cv2, "calcOpticalFlowPyrLK", side_effect=AssertionError("unnecessary flow")):
            for _ in range(2):
                assert follower.track_all(frame)[0].source == "decode"
        with patch.object(follower, "detect_gray", return_value=[]):
            assert follower.track_all(frame)[0].source == "flow"
            follower.tracks[2].decoded_at -= 2
            assert follower.track_all(frame) == []


def test_correlation_is_lazy_and_preserves_full_resolution_coordinates():
    follower = ArucoFollower(None, None)
    frame = sample_frame()
    original = follower.track_all(frame)[0]
    assert follower.tracks[2].tracker is None
    with patch.object(follower, "detect_gray", return_value=[]), patch.object(cv2, "calcOpticalFlowPyrLK", return_value=(None, None, None)):
        result = follower.track_all(frame)[0]
        assert follower.tracks[2].tracker is not None
        assert result.source == "correlation"
        assert abs(result.center_x - original.center_x) < 12
        assert abs(result.center_y - original.center_y) < 12


def test_stale_camera_clears_overlay_and_stops_follow():
    async def scenario(follow):
        state = SimpleNamespace(aruco_status="tracking", aruco_visible=True)
        controller = SimpleNamespace(state=state, stop=AsyncMock(), broadcast=AsyncMock())
        camera = SimpleNamespace(latest_sample=lambda: (b"old", 1, time.monotonic() - 2))
        follower = ArucoFollower(camera, controller)
        follower.enabled = True
        follower.follow = follow
        task = asyncio.create_task(follower.run())
        await asyncio.sleep(0.03)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        assert state.aruco_visible is False
        assert state.aruco_status == "aruco-frame-stale"
        assert controller.stop.call_count == int(follow)
    asyncio.run(scenario(False))
    asyncio.run(scenario(True))
