"""Read-only capture of dashboard-driven motion; never arms or sends drive input."""
import argparse
import json
from pathlib import Path
import time
from urllib.request import urlopen


def capture(url, output, seconds):
    folder=Path(output);folder.mkdir(parents=True,exist_ok=False)
    def read(path):
        with urlopen(url+path,timeout=2) as response:return json.load(response)
    metadata=dict(read_only=True,initial_state=read('/api/state'),duration_s=seconds,
                  clock='Pi monotonic seconds',status='recording')
    (folder/'manifest.json').write_text(json.dumps(metadata,indent=2))
    sequence=0;count=0;drops=0;last=None;end=time.monotonic()+seconds
    try:
        with (folder/'samples.jsonl').open('w',buffering=1) as output:
            while time.monotonic()<end:
                message=read(f'/api/dynamics?after={sequence}')
                if message['latest_seq']<sequence:raise RuntimeError('Dashboard restarted')
                drops+=message['dropped']
                for sample in message['samples']:
                    output.write(json.dumps(sample,separators=(',',':'))+'\n');count+=1
                    command=(sample['forward'],sample['turn'],sample['pwm_limit'])
                    if command!=last:
                        print(round(sample['t'],3),command,flush=True);last=command
                sequence=message['latest_seq'];time.sleep(.08)
        metadata['status']='complete'
    finally:
        metadata.update(sample_count=count,dropped_samples=drops)
        (folder/'manifest.json').write_text(json.dumps(metadata,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--url',default='http://127.0.0.1:8765')
    p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=90)
    args=p.parse_args();capture(args.url.rstrip('/'),args.output,args.seconds)
