from __future__ import annotations

import asyncio
import math
import time
from collections import deque
from dataclasses import dataclass
from functools import lru_cache
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from .web import CameraStream, RobotController

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TAG_SIZE_M = 0.050
TARGET_DISTANCE_M = 0.30
FOCAL_PX = 968.0  # UC-788/B0165 nominal 67° HFOV at 1280 px; calibrate for precision.
ALLOWED_IDS = frozenset(range(50))
_ARUCO_CODEBOOK: tuple[Any, Any] | None = None


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
        self.completed_at = deque(maxlen=90)
        self.processing_ms = 0.0
        self.decode_ms = 0.0
        self.source_at = 0.0
        self.skipped_frames = 0

    def metrics(self) -> dict:
        now = time.monotonic()
        recent = [t for t in self.completed_at if now - t < 3]
        fps = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and now - recent[-1] < 0.5 else 0.0
        return {"aruco_fps": round(fps, 1), "aruco_processing_ms": round(self.processing_ms, 1),
                "aruco_decode_ms": round(self.decode_ms, 1), "aruco_skipped_frames": self.skipped_frames,
                "aruco_result_age_ms": round((now - self.source_at) * 1000) if self.source_at else None}

    def start(self) -> None:
        import cv2
        # Pi4 live-frame benchmark: two workers reduced median and p95 time
        # versus four while leaving CPU available for capture and controls.
        cv2.setNumThreads(2)
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
            await self.controller.stop("aruco-follow-start", release=False)
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
    @lru_cache(maxsize=1)
    def detector():
        import cv2
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Keep the normal threshold sweep: it finds the live small tags in a
        # few milliseconds. Very permissive sweeps cost hundreds of ms/frame.
        parameters.minCornerDistanceRate = 0.01
        parameters.minDistanceToBorder = 1
        parameters.errorCorrectionRate = 0.8
        # Verified on the steeply foreshortened live ID 2: 4 px/cell rejects
        # its bits; 8 px/cell decodes it at essentially the same total cost.
        parameters.perspectiveRemovePixelPerCell = 8
        if hasattr(parameters, "useAruco3Detection"):
            parameters.useAruco3Detection = False
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return dictionary, parameters, detector

    @staticmethod
    def detect_all(jpeg: bytes) -> list[Detection]:
        import cv2
        import numpy as np
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        return [] if image is None else ArucoFollower.detect_gray(image)

    @staticmethod
    def detect_gray(image: Any) -> list[Detection]:
        dictionary, parameters, detector = ArucoFollower.detector()
        corners, ids, rejected = detector.detectMarkers(image)
        candidates: list[Detection] = []
        found_ids: set[int] = set()
        if ids is not None:
            for raw_corners, raw_id in zip(corners, ids.flatten(), strict=True):
                tag_id = int(raw_id)
                if tag_id not in ALLOWED_IDS:
                    continue
                points = raw_corners.reshape(4, 2)
                detection = ArucoFollower.from_points(tag_id, points)
                if detection:
                    candidates.append(detection)
                    found_ids.add(tag_id)

        # OpenCV can find a marker-shaped quad yet reject its bits under blur,
        # glare, or steep perspective. Re-sample only those quads after a
        # perspective warp; strict black-border + dictionary-distance checks
        # keep this cheap fallback from turning arbitrary rectangles into IDs.
        for tag_id, points, distance in ArucoFollower.decode_rejected(
            image, rejected, dictionary, parameters.errorCorrectionRate
        ):
            if tag_id in found_ids:
                continue
            detection = ArucoFollower.from_points(
                tag_id, points, source="decode-fallback", confidence=max(0.75, 1.0 - 0.15 * distance)
            )
            if detection:
                candidates.append(detection)
                found_ids.add(tag_id)
        return sorted(candidates, key=lambda item: item.size_px, reverse=True)

    @staticmethod
    def decode_rejected(image: Any, rejected: Any, dictionary: Any, error_correction_rate: float):
        """Decode high-confidence dictionary matches among rejected quads."""
        import cv2
        import numpy as np

        global _ARUCO_CODEBOOK
        if _ARUCO_CODEBOOK is None:
            codebook = []
            cell_size = 16
            for tag_id in ALLOWED_IDS:
                marker = cv2.aruco.generateImageMarker(
                    dictionary, tag_id, cell_size * 6, borderBits=1
                )
                code = np.empty((4, 4), dtype=np.uint8)
                for row in range(4):
                    for col in range(4):
                        patch = marker[
                            (row + 1) * cell_size + 4:(row + 1) * cell_size + 12,
                            (col + 1) * cell_size + 4:(col + 1) * cell_size + 12,
                        ]
                        code[row, col] = int(float(patch.mean()) >= 127.5)
                codebook.append((tag_id, code))
            _ARUCO_CODEBOOK = (
                np.array([tag_id for tag_id, _ in codebook]),
                np.array([[np.rot90(code, turn) for turn in range(4)] for _, code in codebook]),
            )

        decoded = []
        cell_size = 16
        destination = np.array(
            [[0, 0], [cell_size * 6 - 1, 0], [cell_size * 6 - 1, cell_size * 6 - 1], [0, cell_size * 6 - 1]],
            dtype=np.float32,
        )
        max_errors = min(dictionary.maxCorrectionBits, round(dictionary.maxCorrectionBits * error_correction_rate))
        for raw in rejected:
            points = raw.reshape(4, 2).astype(np.float32)
            if not cv2.isContourConvex(points) or abs(float(cv2.contourArea(points))) < 100:
                continue
            if min(math.dist(points[i], points[(i + 1) % 4]) for i in range(4)) < 12:
                continue
            warped = cv2.warpPerspective(
                image, cv2.getPerspectiveTransform(points, destination), (cell_size * 6, cell_size * 6)
            )
            threshold, _ = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            if not 15 < threshold < 240:
                continue
            means = warped.reshape(6, cell_size, 6, cell_size)[:, 4:12, :, 4:12].mean(axis=(1, 3))
            tag_ids, references = _ARUCO_CODEBOOK
            match = ArucoFollower.decode_cells(means, threshold, references, max_errors)
            if match is None and float(np.ptp(means)) >= 60 and min(
                math.dist(points[i], points[(i + 1) % 4]) for i in range(4)
            ) >= 24:
                # OpenCV normally refines corners after identifying the ID.
                # For shallow views the raw contour corners can shift the
                # projected cell centers enough to prevent that first decode.
                refined = points.copy()
                cv2.cornerSubPix(image, refined, (5, 5), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .03))
                if (np.max(np.linalg.norm(refined - points, axis=1)) <= 8
                        and cv2.isContourConvex(refined)):
                    retry = cv2.warpPerspective(image, cv2.getPerspectiveTransform(refined, destination), (96, 96))
                    retry_threshold, _ = cv2.threshold(retry, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                    # Smaller cell-center windows avoid blurred cell edges.
                    retry_means = retry.reshape(6, 16, 6, 16)[:, 6:10, :, 6:10].mean(axis=(1, 3))
                    match = ArucoFollower.decode_cells(retry_means, retry_threshold, references, 0)
                    if match is not None:
                        points = refined
            if match is None:
                continue
            best, rotation, distance = match
            tag_id = int(tag_ids[best])
            # Put the corners in marker-canonical order for solvePnP.
            canonical_points = np.roll(points, rotation, axis=0)
            decoded.append((tag_id, canonical_points, distance))
        return decoded

    @staticmethod
    def decode_cells(means, threshold, references, max_errors):
        """Bounded alternate thresholds require exact, unambiguous codes.

        Foreshortening mixes adjacent white pixels into black cells. Otsu on
        the entire warped patch can then under-estimate the cell threshold.
        Retain the original error allowance only at the original threshold;
        alternate readings must have a measurable black/white intensity gap.
        """
        import numpy as np
        border = np.concatenate((means[0], means[-1], means[1:-1, 0], means[1:-1, -1]))
        inner = means[1:5, 1:5]
        bits = inner >= threshold
        distances = np.count_nonzero(bits != references, axis=(2, 3))
        best_per_id = distances.min(axis=1)
        order = np.argsort(best_per_id)
        best = int(order[0])
        distance = int(best_per_id[best])
        if ((border < threshold).sum() >= 18 and distance <= max_errors
                and (len(order) < 2 or best_per_id[order[1]] > distance + 1)):
            return best, int(distances[best].argmin()), distance

        low, high = np.percentile(means, (10, 90))
        span = high - low
        if span < 40:
            return None
        alternatives = threshold + span * np.array([-.24, -.16, -.08, .08, .16, .24])
        readings = inner[None, :, :] >= alternatives[:, None, None]
        errors = np.count_nonzero(readings[:, None, None, :, :] != references[None, :, :, :, :], axis=(3, 4))
        matches = set()
        for index, candidate_threshold in enumerate(alternatives):
            exact = np.argwhere(errors[index] == 0)
            if len(exact) != 1 or (border < candidate_threshold).sum() < 18:
                continue
            white = readings[index]
            if not white.any() or white.all():
                continue
            if inner[white].min() - inner[~white].max() < max(6.0, .08 * span):
                continue
            matches.add(tuple(int(v) for v in exact[0]))
        if len(matches) == 1:
            best, rotation = matches.pop()
            return best, rotation, 0
        return None

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

        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            return []
        gray = image
        now = time.monotonic()
        measured = self.detect_gray(gray)
        self.decode_ms = (time.monotonic() - now) * 1000
        by_id = {item.tag_id: item for item in measured}

        flow_candidates: dict[int, Detection] = {}

        old_ids = [tag_id for tag_id, memory in self.tracks.items()
                   if tag_id not in by_id and now - memory.decoded_at <= 0.65]
        if self.previous_gray is not None and old_ids:
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
            # Successful decoding already provides the measurement. Initialize
            # correlation lazily, on the previous frame, only if flow fails.
            memory.tracker = None
            memory.bbox = bbox
            next_tracks[tag_id] = memory

        for tag_id, memory in self.tracks.items():
            if tag_id in next_tracks:
                continue
            correlation: Detection | None = None
            correlation_bbox: tuple[float, float, float, float] | None = None
            if tag_id not in flow_candidates and self.previous_gray is not None and now - memory.decoded_at <= 0.85:
                # Bound MOSSE's FFT cost even for large/nearby marker boxes.
                scale = 0.25
                if memory.tracker is None and memory.bbox is not None:
                    memory.tracker = self.create_tracker(
                        cv2.resize(self.previous_gray, None, fx=scale, fy=scale),
                        tuple(v * scale for v in memory.bbox),
                    )
                ok, raw_bbox = memory.tracker.update(cv2.resize(gray, None, fx=scale, fy=scale)) if memory.tracker is not None else (False, None)
                if ok:
                    correlation_bbox = tuple(float(value) / scale for value in raw_bbox)
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
            else:
                memory.tracker = None
                memory.bbox = self.expanded_bbox(selected, image.shape[1], image.shape[0])
            next_tracks[tag_id] = memory
            by_id[tag_id] = selected

        self.tracks = next_tracks
        self.previous_gray = gray
        return sorted(by_id.values(), key=lambda item: item.size_px, reverse=True)

    async def run(self) -> None:
        while True:
            await asyncio.sleep(0.005 if self.enabled else 0.030)
            if not self.enabled:
                continue
            frame, sequence, captured_at = self.camera.latest_sample()
            if not captured_at or time.monotonic() - captured_at > 0.45:
                if self.controller.state.aruco_status != "aruco-frame-stale":
                    self.clear_detection("aruco-frame-stale")
                    if self.follow:
                        await self.controller.stop("aruco-frame-stale", release=False)
                    await self.controller.broadcast()
                continue
            if not frame or sequence == self.last_sequence:
                if self.follow and time.monotonic() - self.last_detection_at > 0.45:
                    self.clear_detection("aruco-frame-stale")
                    await self.controller.stop("aruco-frame-stale", release=False)
                continue
            if self.last_sequence >= 0:
                self.skipped_frames += max(0, sequence - self.last_sequence - 1)
            self.last_sequence = sequence
            started = time.monotonic()
            detections = await asyncio.to_thread(self.track_all, frame)
            completed = time.monotonic()
            self.processing_ms = (completed - started) * 1000
            self.completed_at.append(completed)
            self.source_at = captured_at
            if completed - captured_at > 0.45:
                self.clear_detection("aruco-result-stale")
                if self.follow:
                    await self.controller.stop("aruco-result-stale", release=False)
                await self.controller.broadcast()
                continue
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
