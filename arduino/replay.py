"""Replay a CSV recording: uv run python replay.py recordings/NAME.csv.
The first 3 seconds must be stationary for a valid evaluation.
"""
import argparse,csv,json
import numpy as np
from server import Tracker
p=argparse.ArgumentParser();p.add_argument('recording');args=p.parse_args()
with open(args.recording) as f:
    rows=np.array([[float(v) for v in r] for r in list(csv.reader(f))[1:]])
if len(rows)<300: raise SystemExit('Need at least 3 seconds of 100 Hz samples.')
base=rows[:300];a=base[:,1:4];g=base[:,4:7]
if np.max(a.std(0))>.025 or np.max(g.std(0))>1.5:
    raise SystemExit('First 3 seconds were not still; repeat the recording.')
t=Tracker();t.gravity=float(np.linalg.norm(a.mean(0)))
t.fusion.setBiasEstimate(np.deg2rad(g.mean(0)),.002)
positions=[];rest=0
for i,row in enumerate(rows):
    t.feed(row.tolist())
    if i==299:t.reset_position();t.calibrated=True
    if i>=300:
        positions.append(t.position.copy());rest+=t.latest['stationary']
print(json.dumps({'samples':len(rows),'estimated_rate_hz':t.latest['hz'],
 'final_position_m':t.position.tolist(),'max_distance_m':float(np.max(np.linalg.norm(positions,axis=1))) if positions else 0,
 'rest_fraction':rest/max(1,len(positions)),'timing_gaps':t.gaps},indent=2))
