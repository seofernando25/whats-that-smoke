import sys,asyncio,json,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ble_transport import stream
async def test(name,drop=0,pause=None):
 stats={};rows=[]
 await stream(lambda r:rows.append((time.monotonic(),r)),lambda:None,stats,duration=20,inject_drop_every=drop,ack_pause=pause)
 intervals=[(b[1][0]-a[1][0])%2**32/1000 for a,b in zip(rows,rows[1:])]
 stats.update(test=name,max_sensor_interval_ms=max(intervals),sensor_rate_hz=1000/(sum(intervals)/len(intervals)),max_delivery_pause_s=max(b[0]-a[0] for a,b in zip(rows,rows[1:])))
 print(json.dumps(stats),flush=True);return stats
async def main():
 out=[]
 for name,drop,pause in [('normal',0,None),('drop_each_10th_notification',10,None),('pause_ack_1_second',0,(5,6))]:
  out.append(await test(name,drop,pause));Path('analysis/ble-stress.json').write_text(json.dumps(out,indent=2));await asyncio.sleep(1)
asyncio.run(main())
