"""Read-only live-frame benchmark; run with the dashboard's Python environment."""
import cProfile
import io
import json
import pstats
import statistics
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

from whats_that_smoke.aruco import ArucoFollower


def main():
    frames = []
    started = time.monotonic()
    with urllib.request.urlopen("http://127.0.0.1:8765/stream.mjpg", timeout=10) as response:
        buffer = b""
        while time.monotonic() - started < 3:
            buffer += response.read1(65536)
            while (begin := buffer.find(b"\xff\xd8")) >= 0:
                end = buffer.find(b"\xff\xd9", begin + 2)
                if end < 0:
                    break
                frames.append(buffer[begin:end + 2])
                buffer = buffer[end + 2:]
    elapsed = time.monotonic() - started
    print(json.dumps({"capture_seconds": elapsed, "frames": len(frames), "unique": len(set(frames)), "received_fps": len(frames) / elapsed}), flush=True)
    if not frames:
        return
    Path("/tmp/aruco-benchmark.jpg").write_bytes(frames[-1])
    gray = cv2.imdecode(np.frombuffer(frames[-1], np.uint8), cv2.IMREAD_GRAYSCALE)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    print("OpenCV", cv2.__version__, "threads", cv2.getNumThreads(), flush=True)
    for cell in (4, 8, 16):
        for margin in (0.13, 0.25):
            parameters = cv2.aruco.DetectorParameters()
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            parameters.minCornerDistanceRate = 0.01
            parameters.minDistanceToBorder = 1
            parameters.errorCorrectionRate = 0.8
            parameters.perspectiveRemovePixelPerCell = cell
            parameters.perspectiveRemoveIgnoredMarginPerCell = margin
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            timings = []
            for _ in range(12):
                t = time.perf_counter()
                _, ids, rejected = detector.detectMarkers(gray)
                timings.append((time.perf_counter() - t) * 1000)
            print(json.dumps({"cell": cell, "margin": margin, "ids": [] if ids is None else ids.flatten().tolist(), "rejected": len(rejected), "median_ms": statistics.median(timings[2:])}), flush=True)
    follower = ArucoFollower(None, None)
    profile = cProfile.Profile()
    profile.enable()
    timings = []
    for frame in frames[-20:]:
        t = time.perf_counter()
        result = follower.track_all(frame)
        timings.append((time.perf_counter() - t) * 1000)
    profile.disable()
    print("track_all", {"median_ms": statistics.median(timings), "max_ms": max(timings), "ids": [(d.tag_id, d.source) for d in result]}, flush=True)
    stream = io.StringIO()
    pstats.Stats(profile, stream=stream).sort_stats("cumtime").print_stats(18)
    print(stream.getvalue())


if __name__ == "__main__":
    main()
