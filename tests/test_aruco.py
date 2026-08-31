from __future__ import annotations

import cv2
import numpy as np

from whats_that_smoke.aruco import ArucoFollower


def encoded_scene(destination: np.ndarray) -> bytes:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, 3, 240, borderBits=1)
    source = np.float32([[0, 0], [239, 0], [239, 239], [0, 239]])
    transform = cv2.getPerspectiveTransform(source, destination.astype(np.float32))
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    warped = cv2.warpPerspective(marker, transform, (640, 480), borderValue=255)
    mask = cv2.warpPerspective(np.full_like(marker, 255), transform, (640, 480), borderValue=0)
    canvas[mask > 0] = warped[mask > 0]
    ok, jpeg = cv2.imencode(".jpg", canvas)
    assert ok
    return jpeg.tobytes()


def test_detects_frontal_tag() -> None:
    result = ArucoFollower.detect(encoded_scene(np.array([[220, 120], [420, 120], [420, 320], [220, 320]])))
    assert result is not None
    assert result.tag_id == 3
    assert abs(result.center_x - 320) < 2


def test_detects_oblique_tag() -> None:
    # Strong perspective compression: representative of viewing a tag under a raised plane.
    result = ArucoFollower.detect(encoded_scene(np.array([[245, 150], [405, 185], [390, 285], [230, 330]])))
    assert result is not None
    assert result.tag_id == 3
    assert result.distance_m > 0


def test_detects_small_distant_tag() -> None:
    # Six payload/border cells across: ~2 px/cell is near the useful 640p floor.
    result = ArucoFollower.detect(encoded_scene(np.array([[300, 200], [312, 200], [312, 212], [300, 212]])))
    assert result is not None
    assert result.tag_id == 3


def test_detects_multiple_ids_in_one_frame() -> None:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    canvas[80:180, 80:180] = cv2.aruco.generateImageMarker(dictionary, 3, 100)
    canvas[240:380, 400:540] = cv2.aruco.generateImageMarker(dictionary, 17, 140)
    ok, jpeg = cv2.imencode(".jpg", canvas)
    assert ok
    results = ArucoFollower.detect_all(jpeg.tobytes())
    assert {result.tag_id for result in results} == {3, 17}
    assert results[0].tag_id == 17
