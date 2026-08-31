from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .web import CameraStream, RobotController

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TAG_SIZE_M = 0.050
TARGET_DISTANCE_M = 0.30
FOCAL_PX = 628.0  # OV5647 nominal 54 deg horizontal FOV at 640 px; calibrate for precision.
ALLOWED_IDS = frozenset(range(5))


@dataclass
class Detection:
    tag_id: int
    corners: list[list[float]]
    center_x: float
    center_y: float
    size_px: float
    distance_m: float


class ArucoFollower:
    def __init__(self, camera: "CameraStream", controller: "RobotController") -> None:
        self.camera = camera
        self.controller = controller
        self.enabled = False
        self.follow = False
        self.owner: str | None = None
        self.task: asyncio.Task[None] | None = None
        self.last_sequence = -1
        self.last_detection_at = 0.0

    def start(self) -> None:
        self.task = asyncio.create_task(self.run(), name="aruco-follower")

    async def close(self) -> None:
        self.enabled = False
        await self.disable_follow("aruco-shutdown")
        if self.task:
            self.task.cancel()
            try:
                await self.task
            except asyncio.CancelledError:
                pass

    async def configure(self, client_id: str, enabled: bool, follow: bool | None = None) -> None:
        if follow:
            if self.controller.state.owner != client_id or not self.controller.state.armed:
                raise PermissionError("arm controls before enabling ArUco follow")
            self.owner = client_id
            self.enabled = True
            self.follow = True
        elif follow is False:
            await self.disable_follow("aruco-follow-disabled")
        self.enabled = bool(enabled) or self.follow
        if not self.enabled:
            self.clear_detection("aruco-disabled")
        self.sync_state()
        await self.controller.broadcast()

    async def disable_follow(self, reason: str) -> None:
        was_following = self.follow
        self.follow = False
        self.owner = None
        self.sync_state()
        if was_following:
            await self.controller.stop(reason, release=False)

    def sync_state(self) -> None:
        state = self.controller.state
        state.aruco_enabled = self.enabled
        state.aruco_follow = self.follow

    def clear_detection(self, status: str) -> None:
        state = self.controller.state
        state.aruco_visible = False
        state.aruco_id = None
        state.aruco_distance_m = None
        state.aruco_error_x = None
        state.aruco_corners = []
        state.aruco_status = status

    @staticmethod
    def detect(jpeg: bytes) -> Detection | None:
        import cv2
        import numpy as np

        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            return None
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        parameters.minMarkerPerimeterRate = 0.02
        if hasattr(parameters, "useAruco3Detection"):
            parameters.useAruco3Detection = True
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(image)
        if ids is None:
            return None
        candidates: list[Detection] = []
        for raw_corners, raw_id in zip(corners, ids.flatten(), strict=True):
            tag_id = int(raw_id)
            if tag_id not in ALLOWED_IDS:
                continue
            points = raw_corners.reshape(4, 2)
            edges = [math.dist(points[i], points[(i + 1) % 4]) for i in range(4)]
            size_px = sum(edges) / 4
            if size_px < 12:
                continue
            center = points.mean(axis=0)
            camera_matrix = np.array([[FOCAL_PX, 0, FRAME_WIDTH / 2], [0, FOCAL_PX, FRAME_HEIGHT / 2], [0, 0, 1]], dtype=np.float64)
            half = TAG_SIZE_M / 2
            object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
            solved, _, translation = cv2.solvePnP(
                object_points, points.astype(np.float32), camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            distance_m = float(np.linalg.norm(translation)) if solved and translation[2, 0] > 0 else TAG_SIZE_M * FOCAL_PX / size_px
            candidates.append(
                Detection(
                    tag_id=tag_id,
                    corners=[[float(x), float(y)] for x, y in points],
                    center_x=float(center[0]),
                    center_y=float(center[1]),
                    size_px=size_px,
                    distance_m=distance_m,
                )
            )
        return max(candidates, key=lambda item: item.size_px, default=None)

    async def run(self) -> None:
        while True:
            await asyncio.sleep(0.10)
            if not self.enabled:
                continue
            frame, sequence = self.camera.latest()
            if not frame or sequence == self.last_sequence:
                if self.follow and time.monotonic() - self.last_detection_at > 0.45:
                    self.clear_detection("aruco-frame-stale")
                    await self.controller.stop("aruco-frame-stale", release=False)
                continue
            self.last_sequence = sequence
            detection = await asyncio.to_thread(self.detect, frame)
            if detection is None:
                self.clear_detection("aruco-searching")
                if self.follow:
                    await self.controller.stop("aruco-tag-lost", release=False)
                else:
                    await self.controller.broadcast()
                continue
            self.last_detection_at = time.monotonic()
            state = self.controller.state
            error_x = (detection.center_x - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
            state.aruco_visible = True
            state.aruco_id = detection.tag_id
            state.aruco_distance_m = round(detection.distance_m, 3)
            state.aruco_error_x = round(error_x, 3)
            state.aruco_corners = detection.corners
            state.aruco_status = "target-reached" if detection.distance_m <= TARGET_DISTANCE_M else "tracking"
            self.sync_state()
            if not self.follow:
                await self.controller.broadcast()
                continue
            if self.controller.state.owner != self.owner or not self.controller.state.armed:
                await self.disable_follow("aruco-control-lost")
                continue
            turn = max(-0.55, min(0.55, -1.15 * error_x))
            distance_error = detection.distance_m - TARGET_DISTANCE_M
            forward = max(0.0, min(0.48, distance_error * 0.9))
            if abs(error_x) > 0.32:
                forward = 0.0
            if distance_error <= 0.035 and abs(error_x) <= 0.08:
                await self.controller.stop("aruco-target-reached", release=False)
            else:
                await self.controller.drive(self.owner, forward, turn, 1300, autonomous=True)


def state_defaults() -> dict[str, Any]:
    return {
        "aruco_enabled": False,
        "aruco_follow": False,
        "aruco_visible": False,
        "aruco_id": None,
        "aruco_distance_m": None,
        "aruco_error_x": None,
        "aruco_corners": [],
        "aruco_status": "aruco-disabled",
    }
