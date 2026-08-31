# ARUCO

```text
dict := DICT_4X4_50; accepted IDs={0..49}; detect=simultaneous
print := docs/aruco-tags-50mm.pdf; Actual Size/100%; black boundary=50mm
quiet-zone := >=5mm white; do-not laminate glossy
mount := distribute/angle tags under plane; exact edge-on(90deg)=>undetectable by geometry
```

## detect

```text
runtime := Pi4/OpenCV-contrib; source:=shared 640x480 MJPEG latest frame
capture := 30fps; exposure=sport; denoise=off; objective=short motion blur
rate := <=30Hz; overlay:=all; follow-target:=ID-lock then largest/closest
corners := subpixel refinement; standard detector (low-latency)
continuity := KLT[velocity-init,35px,L4,FB<=4px] > MOSSE[context=1.8x] > CV-predict
visual TTL := flow=.8s; correlation=1.0s; prediction=1.2s; dashed+source+confidence
control TTL := fresh decoded ID<=.35s; visual optimism never broadens motion authority
association := decoded ID; no appearance model/Hungarian required for unique fiducials
state := target-ID alpha-beta constant-velocity[x,z]; α=.70 β=.12; reset(ID-change|dt>.5s)
pose := SOLVEPNP_IPPE_SQUARE; tag=0.050m
Kapprox := fx=fy=628px; cx=320; cy=240; distortion=0
distance precision => calibrate camera intrinsics later; control uses conservative deadband
```

## follow

```text
gate := browser owner + ARM + explicit FOLLOW ON
target := z/euclidean≈0.30m; tolerance=0.035m; center tolerance=0.08 frame-half
error_x := (tag_cx-320)/320
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
```
