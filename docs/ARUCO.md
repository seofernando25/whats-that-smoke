# ARUCO

```text
dict := DICT_4X4_50; accepted IDs={0..49}; detect=simultaneous
print := docs/aruco-tags-50mm.pdf; Actual Size/100%; black boundary=50mm
quiet-zone := >=5mm white; do-not laminate glossy
mount := distribute/angle tags under plane; exact edge-on(90deg)=>undetectable by geometry
```

## detect

```text
runtime := Pi4/OpenCV-contrib; source:=shared 1280x720 MONO MJPEG latest frame
capture := 30fps; shutter=8000µs; gain=1.5; denoise=off
rate := <=30Hz; overlay:=all; follow-target:=ID-lock then largest/closest
corners := subpixel refinement; cached detector; perspectiveRemovePixelPerCell=8; cv2 workers=2
decode-fallback := re-warp rejected quads to 6×6 cells; black-border≥18/20; DICT_4X4_50 Hamming≤1, unique best
fallback cost := rejected candidates only; all dictionary rotations cached; vectorized cells+Hamming
oblique rescue := if initial cell decode fails, threshold=Otsu±{.08,.16,.24}×cell-P90−P10; span>=40; exact code only; unique ID+rotation across accepted trials; black-border>=18/20; minWhite−maxBlack>=max(6,.08×span)
oblique corners := failed high-contrast candidate + min-edge>=24px => cornerSubPix5×5 radius, displacement<=8px, convex; 96×96 warp; central4×4 samples/cell; retry requires0 bit errors
oblique regression := live ID2 / shallow table angle; global threshold misclassified3 black cells; threshold-only rescue8/10 initial frames but intermittent live; corner+center refinement19/20 saved frames decoded incl10/10 latest; background(tag masked)0/20 detections; this sample≠general false-positive guarantee
decode := one grayscale JPEG decode/frame shared with trackers; latest-frame only; polling<=5ms
fallback label := exact/corrected code; percent≠probability; UI percentage removed
continuity := KLT[velocity-init,35px,L4,FB<=4px] > MOSSE[context=1.8x] > CV-predict
tracking cost := only undecoded IDs use flow; MOSSE initialized lazily from previous image only if flow fails; MOSSE scale=.25; coordinates restored to full resolution
visual TTL from last decode (absolute) := flow=.65s; correlation=.85s; prediction=1.0s
confidence floor∅; confidence<.28=>drop; fallback cannot self-refresh indefinitely
control TTL := fresh decoded ID<=.35s; visual optimism never broadens motion authority
association := decoded ID; no appearance model/Hungarian required for unique fiducials
state := target-ID alpha-beta constant-velocity[x,z]; α=.70 β=.12; reset(ID-change|dt>.5s)
pose := SOLVEPNP_IPPE_SQUARE; tag=0.050m
Kapprox := fx=fy=968px; cx=640; cy=360; distortion=0
distance precision => calibrate camera intrinsics later; control uses conservative deadband
```

## follow

```text
gate := browser owner + ARM + explicit FOLLOW ON
target := z/euclidean≈0.30m; tolerance=0.035m; center tolerance=0.08 frame-half
error_x := (tag_cx-640)/640
turn := clamp(-1.15*error_x, -0.55, 0.55)
forward := clamp(0.9*(distance-0.30), 0, 0.48)
abs(error_x)>0.32 => forward=0; rotate-only
speed_limit := 1300; reverse∅
```

## fail-safe

```text
tag∅|frame stale>450ms|disconnect|disarm|disable|shutdown => brake
watchdog=600ms independent hardware thread; loop stall=>brake+disarm
manual drive while FOLLOW ON => rejected
blind search∅; autonomous reverse∅; IDs outside DICT_4X4_50 ignored
frame age>450ms => overlay cleared even if FOLLOW OFF; stale JPEG resend∅
```

## performance · 2026-09-27

```text
scene := live ID2, steep perspective; OpenCV=5.0.0
native decoding := 4px/cell rejected; 8px/cell succeeds; fallback retained for harder frames
profile before := track_all median314ms; MOSSE init ~209ms/frame; JPEG decoded twice; redundant flow
profile after := track_all median50ms with old live service competing for CPU; native ID2
deployed sample := 15s/30 polls; camera median30fps; processed median28.3fps; compute median28.9ms/p95=50.3ms; result age median68ms/p95=103ms
source := native ID2 at20/30 polls; remaining incl resampled detection as viewpoint changed
telemetry := camera_fps,camera_frame_age_ms,camera_sequence,aruco_fps,aruco_processing_ms,aruco_decode_ms,aruco_result_age_ms,aruco_skipped_frames
FPS := rolling3s; aruco_fps=processed frames, not successful ID decodes; camera_fps=Pi JPEG arrival, not browser paint
age := Pi JPEG arrival→state observation; excludes sensor/encoder and browser/network display latency
reproduce := .venv/bin/python tools/profile_aruco.py on Pi; read-only, adds concurrent CPU load
verified := IDs0..49 × rotations0..3; blank rejection; fresh decode skips trackers; missed decode flow/correlation; absolute TTL; stale capture clears/brakes
```

## Curled floor tag correction · 2026-10-13

```text
problem := shallow curled paper makes a flat homography misread interior cells; live ID0 baseline0/50 visible polls
curved rescue := only after normal + refined decoding fail; shortest edge>=24px; warp384 with quiet-zone margin; connected black border with high contrast; smooth per-column top/bottom boundary
surface samples := central4×4 pixels/cell; monotone coordinate offsets bounded±.08 in .02 steps; vectorized81 geometries
identity := exact DICT_4X4_50 code; cell-pixel P20/P80 separation>=max(15,.12×contrast); black border>=19/20; >=3 agreeing geometries; any conflicting ID/rotation=>reject
edge gate := native/refined and fallback corners>=3px from image edge; do not accept clipped quiet zones
regression := retained ID0 marker-only crop, brightness±12; 50 IDs×4 rotations curved; 569 invalid random codes + blanks rejected; stale-camera/track-expiry tests retained
tracking TTL := unchanged; no longer visual hold; current identity comes from fresh code reads, not forced expected ID
pose := remains approximate planar pose; curling is corrected for identity extraction, not calibrated 3D shape
rollback := /home/advanced/wts-curved-tag-backup-WzCwZj/{aruco.py,test_aruco.py,ARUCO.md,STATE.md}; new fixture=tests/fixtures/curled-id0.png
```

## Capture continuity · 2026-10-13

```text
pipe := select(.25s) + os.read; startup allowance5s; no complete JPEG2s => restart capture
cleanup := SIGTERM + wait<=2s, then SIGKILL fallback; retry delay1s
camera regression := silent live pipe + non-JPEG byte stream + ignored SIGTERM; simulated processes only
verification := original visible ID0 stayed target-reached at268/268 polls over30s after curved rescue; camera median30fps; processed median11.7fps; median77.2ms compute; p95 result age180ms
edge correction := later rejected incidental clipped native candidate; native/fallback border regression passed
final := source matches tested candidate; service active, current capture30fps/fresh; human accepted goal complete and ended extra tests
scope := no claim of detection through physical occlusion or camera outage; stale frames still clear overlays; user-operated multi-tag/follow trials are separate from the stationary verification
rollback includes := /home/advanced/wts-curved-tag-backup-WzCwZj/web.py; new regression=tests/test_camera.py
```
