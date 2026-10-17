"""Plot measured yaw versus replay and simulator-only paths; never infer real XY."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]

def main(output):
    ref=json.loads((ROOT/'simulation/gazebo/lap_reference.json').read_text())
    before=json.loads((ROOT/'calibration/runs/laps-before.json').read_text())
    after=json.loads((ROOT/'simulation/gazebo/.control-logs/lap-regression.json').read_text())
    real=[-p['yaw_angle_deg'] for p in ref['phases'] if p['keys']=='WD']
    fig,(ax,path)=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    colors=['#b55d35','#176f83']
    ax.plot(range(1,7),real,'o-',color='#30353a',label='Real gyro, bias corrected')
    for data,label,color in zip((before,after),('Previous Gazebo','Updated Gazebo'),colors):
        turns=[-p['yaw_angle_deg'] for p in data['phases'] if p['keys']=='WD']
        ax.plot(range(1,7),turns,'o--',color=color,label=label)
        points=data['trajectory'];path.plot([p['x'] for p in points],[p['y'] for p in points],color=color,label=label,lw=2)
    ax.set(title='Same recorded W+D hold durations',xlabel='Half-turn number',ylabel='Right turn (degrees)',xticks=range(1,7))
    ax.grid(alpha=.2);ax.legend(frameon=False,fontsize=9)
    path.set(title='Simulated paths only — real XY unmeasured',xlabel='World X (m)',ylabel='World Y (m)');path.set_aspect('equal');path.grid(alpha=.2);path.legend(frameon=False,fontsize=9)
    fig.suptitle('UGV · continuous W 600 ms → W+D half-turns',fontsize=16,fontweight='bold')
    output=Path(output);output.mkdir(parents=True,exist_ok=True);fig.savefig(output/'ugv-lap-comparison.png',dpi=160)
    plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);main(p.parse_args().output)
