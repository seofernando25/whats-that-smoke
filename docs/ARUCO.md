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
