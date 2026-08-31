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
ALLOWED_IDS = frozenset(range(50))


@dataclass
class Detection:
    tag_id: int
    corners: list[list[float]]
    center_x: float
    center_y: float
    size_px: float
    distance_m: float
    tracked: bool = False
    source: str = "decode"
    age_s: float = 0.0
    confidence: float = 1.0


@dataclass
class TrackMemory:
    detection: Detection
    decoded_at: float
    visual_at: float
    updated_at: float
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_size: float = 0.0
    tracker: Any | None = None
    bbox: tuple[float, float, float, float] | None = None


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
        self.previous_gray: Any | None = None
        self.tracks: dict[int, TrackMemory] = {}
        self.target_id: int | None = None
        self.filter_state: tuple[float, float, float, float, float] | None = None

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
        state.aruco_markers = []
        state.aruco_status = status
        self.previous_gray = None
        self.tracks.clear()
        self.target_id = None
        self.filter_state = None

    def filter_target(self, detection: Detection, now: float) -> tuple[float, float]:
        """Responsive constant-velocity alpha-beta filter for control state."""
        if self.filter_state is None:
            self.filter_state = (detection.center_x, detection.distance_m, 0.0, 0.0, now)
            return detection.center_x, detection.distance_m
        x, z, velocity_x, velocity_z, previous_at = self.filter_state
        dt = now - previous_at
        if dt <= 0 or dt > 0.5:
            self.filter_state = (detection.center_x, detection.distance_m, 0.0, 0.0, now)
            return detection.center_x, detection.distance_m
        predicted_x = x + velocity_x * dt
        predicted_z = z + velocity_z * dt
        residual_x = detection.center_x - predicted_x
        residual_z = detection.distance_m - predicted_z
        alpha, beta = 0.70, 0.12
        x = predicted_x + alpha * residual_x
        z = predicted_z + alpha * residual_z
        velocity_x += beta * residual_x / dt
        velocity_z += beta * residual_z / dt
        self.filter_state = (x, z, velocity_x, velocity_z, now)
        return x, z

    @staticmethod
    def from_points(
        tag_id: int, points: Any, tracked: bool = False, source: str = "decode", age_s: float = 0.0, confidence: float = 1.0
    ) -> Detection | None:
        import cv2
        import numpy as np

        edges = [math.dist(points[i], points[(i + 1) % 4]) for i in range(4)]
        size_px = sum(edges) / 4
        area = abs(float(cv2.contourArea(points.astype(np.float32))))
        if size_px < 7 or area < 35 or not cv2.isContourConvex(points.astype(np.float32)):
            return None
        center = points.mean(axis=0)
        camera_matrix = np.array([[FOCAL_PX, 0, FRAME_WIDTH / 2], [0, FOCAL_PX, FRAME_HEIGHT / 2], [0, 0, 1]], dtype=np.float64)
        half = TAG_SIZE_M / 2
        object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
        solved, _, translation = cv2.solvePnP(
            object_points, points.astype(np.float32), camera_matrix, np.zeros(5), flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        distance_m = float(np.linalg.norm(translation)) if solved and translation[2, 0] > 0 else TAG_SIZE_M * FOCAL_PX / size_px
        return Detection(
            tag_id=tag_id,
            corners=[[float(x), float(y)] for x, y in points],
            center_x=float(center[0]),
            center_y=float(center[1]),
            size_px=size_px,
            distance_m=distance_m,
            tracked=tracked,
            source=source,
            age_s=age_s,
            confidence=confidence,
        )

    @staticmethod
    def detect_all(jpeg: bytes) -> list[Detection]:
        import cv2
        import numpy as np

        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            return []
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Keep the normal threshold sweep: it finds the live small tags in a
        # few milliseconds. Very permissive sweeps cost hundreds of ms/frame.
        parameters.minCornerDistanceRate = 0.01
        parameters.minDistanceToBorder = 1
        parameters.errorCorrectionRate = 0.8
        if hasattr(parameters, "useAruco3Detection"):
            parameters.useAruco3Detection = False
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(image)
        if ids is None:
            return []
        candidates: list[Detection] = []
        for raw_corners, raw_id in zip(corners, ids.flatten(), strict=True):
            tag_id = int(raw_id)
            if tag_id not in ALLOWED_IDS:
                continue
            points = raw_corners.reshape(4, 2)
            detection = ArucoFollower.from_points(tag_id, points)
            if detection:
                candidates.append(detection)
        return sorted(candidates, key=lambda item: item.size_px, reverse=True)

    @staticmethod
    def detect(jpeg: bytes) -> Detection | None:
        detections = ArucoFollower.detect_all(jpeg)
        return detections[0] if detections else None

    @staticmethod
    def expanded_bbox(detection: Detection, width: int, height: int) -> tuple[float, float, float, float]:
        xs = [point[0] for point in detection.corners]
        ys = [point[1] for point in detection.corners]
        box_width = max(24.0, (max(xs) - min(xs)) * 1.8)
        box_height = max(24.0, (max(ys) - min(ys)) * 1.8)
        center_x, center_y = detection.center_x, detection.center_y
        left = max(0.0, min(width - box_width, center_x - box_width / 2))
        top = max(0.0, min(height - box_height, center_y - box_height / 2))
        return left, top, min(box_width, width - left), min(box_height, height - top)

    @staticmethod
    def create_tracker(image: Any, bbox: tuple[float, float, float, float]) -> Any | None:
        import cv2

        try:
            tracker = cv2.legacy.TrackerMOSSE_create()
            tracker.init(image, bbox)
            return tracker
        except (AttributeError, cv2.error):
            return None

    def update_motion(self, memory: TrackMemory, detection: Detection, now: float) -> None:
        dt = now - memory.updated_at
        if dt > 0.005 and detection.source != "predict":
            observed_x = (detection.center_x - memory.detection.center_x) / dt
            observed_y = (detection.center_y - memory.detection.center_y) / dt
            observed_size = (detection.size_px - memory.detection.size_px) / dt
            blend = 0.45
            memory.velocity_x = max(-1800.0, min(1800.0, (1 - blend) * memory.velocity_x + blend * observed_x))
            memory.velocity_y = max(-1800.0, min(1800.0, (1 - blend) * memory.velocity_y + blend * observed_y))
            memory.velocity_size = max(-900.0, min(900.0, (1 - blend) * memory.velocity_size + blend * observed_size))
        memory.detection = detection
        memory.updated_at = now
        if detection.source != "predict":
            memory.visual_at = now

    def transform_from_bbox(
        self, memory: TrackMemory, bbox: tuple[float, float, float, float], source: str, now: float
    ) -> Detection | None:
        import numpy as np

        if memory.bbox is None:
            return None
        old_x, old_y, old_w, old_h = memory.bbox
        new_x, new_y, new_w, new_h = bbox
        if min(new_w, new_h) < 12 or not 0.45 <= new_w / old_w <= 2.2 or not 0.45 <= new_h / old_h <= 2.2:
            return None
        old_center = np.array([old_x + old_w / 2, old_y + old_h / 2], dtype=np.float32)
        new_center = np.array([new_x + new_w / 2, new_y + new_h / 2], dtype=np.float32)
        points = np.array(memory.detection.corners, dtype=np.float32)
        points = (points - old_center) * np.array([new_w / old_w, new_h / old_h], dtype=np.float32) + new_center
        age = now - memory.decoded_at
        confidence = 0.72 * math.exp(-age / 0.75)
        return self.from_points(memory.detection.tag_id, points, True, source, age, confidence)

    def track_all(self, jpeg: bytes) -> list[Detection]:
        """Fuse ArUco, KLT, MOSSE correlation, and motion prediction."""
        import cv2
        import numpy as np

        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        now = time.monotonic()
        measured = self.detect_all(jpeg)
        by_id = {item.tag_id: item for item in measured}

        flow_candidates: dict[int, Detection] = {}

        if self.previous_gray is not None and self.tracks:
            old_ids = list(self.tracks)
            old_points = np.array(
                [[point for point in self.tracks[tag_id].detection.corners] for tag_id in old_ids], dtype=np.float32
            ).reshape(-1, 1, 2)
            initial_points = old_points.copy()
            for index, tag_id in enumerate(old_ids):
                memory = self.tracks[tag_id]
                dt = now - memory.updated_at
                initial_points[index * 4:(index + 1) * 4, 0, 0] += memory.velocity_x * dt
                initial_points[index * 4:(index + 1) * 4, 0, 1] += memory.velocity_y * dt
            new_points, forward_ok, _ = cv2.calcOpticalFlowPyrLK(
                self.previous_gray, gray, old_points, initial_points, winSize=(35, 35), maxLevel=4,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
            )
            if new_points is not None:
                back_points, backward_ok, _ = cv2.calcOpticalFlowPyrLK(
                    gray, self.previous_gray, new_points, None, winSize=(35, 35), maxLevel=4,
                    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
                )
                for index, tag_id in enumerate(old_ids):
                    memory = self.tracks[tag_id]
                    if tag_id in by_id or now - memory.decoded_at > 0.65:
                        continue
                    sl = slice(index * 4, index * 4 + 4)
                    valid = forward_ok[sl].all() and backward_ok is not None and backward_ok[sl].all()
                    fb_error = float(np.max(np.linalg.norm(old_points[sl] - back_points[sl], axis=2))) if back_points is not None else 999
                    if valid and fb_error <= 4.0:
                        age = now - memory.decoded_at
                        tracked = self.from_points(
                            tag_id, new_points[sl].reshape(4, 2), True, "flow", age,
                            0.88 * math.exp(-age / 0.65),
                        )
                        if tracked:
                            flow_candidates[tag_id] = tracked

        next_tracks: dict[int, TrackMemory] = {}
        for tag_id, detection in by_id.items():
            memory = self.tracks.get(tag_id)
            bbox = self.expanded_bbox(detection, image.shape[1], image.shape[0])
            if memory is None:
                memory = TrackMemory(detection, now, now, now)
            else:
                self.update_motion(memory, detection, now)
                memory.decoded_at = now
            memory.tracker = self.create_tracker(image, bbox)
            memory.bbox = bbox
            next_tracks[tag_id] = memory

        for tag_id, memory in self.tracks.items():
            if tag_id in next_tracks:
                continue
            correlation: Detection | None = None
            correlation_bbox: tuple[float, float, float, float] | None = None
            if memory.tracker is not None and now - memory.decoded_at <= 0.85:
                ok, raw_bbox = memory.tracker.update(image)
                if ok:
                    correlation_bbox = tuple(float(value) for value in raw_bbox)
                    correlation = self.transform_from_bbox(memory, correlation_bbox, "correlation", now)
            selected = flow_candidates.get(tag_id) or correlation
            if selected is None and now - memory.decoded_at <= 1.0:
                dt = now - memory.updated_at
                old_points = np.array(memory.detection.corners, dtype=np.float32)
                center = np.array([memory.detection.center_x, memory.detection.center_y], dtype=np.float32)
                predicted_size = max(7.0, memory.detection.size_px + memory.velocity_size * dt)
                scale = predicted_size / memory.detection.size_px
                shift = np.array([memory.velocity_x * dt, memory.velocity_y * dt], dtype=np.float32)
                points = (old_points - center) * scale + center + shift
                age = now - memory.decoded_at
                selected = self.from_points(
                    tag_id, points, True, "predict", age, 0.42 * math.exp(-age / 0.45)
                )
            if selected is not None and selected.confidence < 0.28:
                selected = None
            if selected is None:
                continue
            self.update_motion(memory, selected, now)
            if correlation_bbox is not None:
                memory.bbox = correlation_bbox
            next_tracks[tag_id] = memory
            by_id[tag_id] = selected

        self.tracks = next_tracks
        self.previous_gray = gray
        return sorted(by_id.values(), key=lambda item: item.size_px, reverse=True)

    async def run(self) -> None:
        while True:
            await asyncio.sleep(0.030)
            if not self.enabled:
                continue
            frame, sequence = self.camera.latest()
            if not frame or sequence == self.last_sequence:
                if self.follow and time.monotonic() - self.last_detection_at > 0.45:
                    self.clear_detection("aruco-frame-stale")
                    await self.controller.stop("aruco-frame-stale", release=False)
                continue
            self.last_sequence = sequence
            detections = await asyncio.to_thread(self.track_all, frame)
            if not detections:
                # A tiny/distant marker may miss an individual compressed
                # frame. Preserve the previous overlay/command briefly rather
                # than visually flickering or braking at every isolated miss.
                if time.monotonic() - self.last_detection_at <= 0.20:
                    continue
                self.clear_detection("aruco-searching")
                if self.follow:
                    await self.controller.stop("aruco-tag-lost", release=False)
                else:
                    await self.controller.broadcast()
                continue
            available = {item.tag_id: item for item in detections}
            detection = available.get(self.target_id) if self.target_id is not None else None
            if detection is None:
                detection = detections[0]
                self.target_id = detection.tag_id
                self.filter_state = None
            if any(not item.tracked for item in detections):
                self.last_detection_at = time.monotonic()
            state = self.controller.state
            filtered_x, filtered_distance = self.filter_target(detection, time.monotonic())
            error_x = (filtered_x - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
            state.aruco_visible = True
            state.aruco_id = detection.tag_id
            state.aruco_distance_m = round(filtered_distance, 3)
            state.aruco_error_x = round(error_x, 3)
            state.aruco_corners = detection.corners
            state.aruco_markers = [
                {"id": item.tag_id, "distance_m": round(item.distance_m, 3), "corners": item.corners,
                 "target": item.tag_id == detection.tag_id, "tracked": item.tracked,
                 "source": item.source, "age_ms": round(item.age_s * 1000), "confidence": round(item.confidence, 2)}
                for item in detections
            ]
            state.aruco_status = (
                "target-reached" if filtered_distance <= TARGET_DISTANCE_M else
                "tracking" if detection.source == "decode" else f"tracking-{detection.source}"
            )
            self.sync_state()
            if not self.follow:
                await self.controller.broadcast()
                continue
            if self.controller.state.owner != self.owner or not self.controller.state.armed:
                await self.disable_follow("aruco-control-lost")
                continue
            if detection.age_s > 0.35:
                await self.controller.stop("aruco-decode-stale", release=False)
                continue
            turn = max(-0.55, min(0.55, -1.15 * error_x))
            distance_error = filtered_distance - TARGET_DISTANCE_M
            forward = max(0.0, min(0.48, distance_error * 0.9))
            if abs(error_x) > 0.32:
                forward = 0.0
            if distance_error <= 0.035 and abs(error_x) <= 0.08:
                await self.controller.stop("aruco-target-reached", release=False)
            else:
                await self.controller.drive(self.owner, forward, 0.0, turn, 1300, autonomous=True)


def state_defaults() -> dict[str, Any]:
    return {
        "aruco_enabled": False,
        "aruco_follow": False,
        "aruco_visible": False,
        "aruco_id": None,
        "aruco_distance_m": None,
        "aruco_error_x": None,
        "aruco_corners": [],
        "aruco_markers": [],
        "aruco_status": "aruco-disabled",
    }
