import asyncio, json, os, time
from collections import deque
from vqf import VQF
import csv
from contextlib import asynccontextmanager
from pathlib import Path
import numpy as np
import struct
from ble_transport import stream
from scipy.spatial.transform import Rotation as R
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).parent
PORT = int(os.environ.get("ARDUINO_HTTP_PORT", "8766"))
class Tracker:
    def __init__(self):
        self.fusion=VQF(.01, restMinT=.5); self.filtered_acc=np.zeros(3); self.recording=None; self.record_writer=None; self.samples=0; self.elapsed=0.; self.raw_previous=None; self.uniform_time=None; self.raw_time=0.
        self.q = R.identity(); self.reference = R.identity()
        self.gravity = 1.; self.rest_bias = np.zeros(3); self.window = deque(maxlen=10)
        self.bias = np.zeros(3); self.velocity = np.zeros(3); self.position = np.zeros(3)
        self.prev = None; self.initialized = False; self.calibration = []; self.calibrating = False
        self.calibrated = False; self.still_time = 0.; self.stabilize = True
        self.message = 'Hold the board still and calibrate.'; self.latest = {}; self.last_data = 0
        self.connected = False; self.port = ''; self.gaps = 0; self.position_epoch = 0
    def reset_position(self):
        self.position[:] = 0; self.velocity[:] = 0; self.filtered_acc[:]=0; self.position_epoch += 1
    def command(self, action, value=None):
        if action == 'record':
            if self.recording:
                self.recording.close(); self.recording=None
            else:
                folder=ROOT/'recordings'; folder.mkdir(exist_ok=True)
                self.recording=open(folder/time.strftime('%Y%m%d-%H%M%S.csv'),'w')
                self.record_writer=csv.writer(self.recording); self.record_writer.writerow(['t_us','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps'])
        elif action == 'calibrate':
            self.calibration=[]; self.calibrating=True; self.message='Keep completely still for 3 seconds…'
        elif action == 'zero': self.reference=self.q; self.message=''
        elif action == 'position': self.reset_position(); self.message=''
        elif action == 'stabilize': self.stabilize=bool(value)
    def feed(self, row):
        if len(row)!=7 or not np.all(np.isfinite(row)): return
        row=np.asarray(row,dtype=float)
        if self.recording:
            self.record_writer.writerow(row)
            if self.samples%100==0: self.recording.flush()
        self.samples+=1
        if self.raw_previous is None:
            self.raw_previous=row; self.raw_time=float(row[0]); self.uniform_time=self.raw_time
            self.process(row.tolist()); return
        step=(row[0]-self.raw_previous[0])%2**32
        if step<=0: return
        self.elapsed+=step/1e6
        if step>50000:
            self.gaps+=1; self.velocity[:]=0; self.filtered_acc[:]=0
            # A lost BLE interval is not evidence that sensor bias changed.
            # Keep orientation/bias state; skip unknown displacement and velocity.
            if self.calibrating:
                self.calibration=[]
            self.raw_time+=step; self.uniform_time=self.raw_time
            self.prev=(row[0]-10000)%2**32
            was_calibrated=self.calibrated
            self.calibrated=False
            self.process(row.tolist())
            self.calibrated=was_calibrated
            self.latest['calibrated']=was_calibrated
        else:
            end=self.raw_time+step
            while self.uniform_time+10000<=end:
                self.uniform_time+=10000
                f=(self.uniform_time-self.raw_time)/step
                values=self.raw_previous[1:]*(1-f)+row[1:]*f
                self.process([self.uniform_time%2**32,*values])
            self.raw_time=end
        self.raw_previous=row
    def process(self, row):
        stamp=int(row[0])
        # Firmware streams a contiguous burst in the native BMI270 frame.
        a=np.array(row[1:4]); rawg=np.array(row[4:7])
        if not np.all(np.isfinite(row)) or np.linalg.norm(a)<0.1: return
        dt=((stamp-self.prev) % 2**32)/1e6 if self.prev is not None else .01
        self.prev=stamp
        if dt<=0 or dt>.1:
            self.gaps+=1; self.velocity[:]=0; dt=.01
        if not self.initialized:
            self.q=R.align_vectors([[0,0,1]], [a/np.linalg.norm(a)])[0]; self.initialized=True
        if self.calibrating:
            self.calibration.append((time.monotonic(),a.copy(),rawg.copy()))
            elapsed=self.calibration[-1][0]-self.calibration[0][0]
            if elapsed>=3:
                aa=np.array([v[1] for v in self.calibration]); gg=np.array([v[2] for v in self.calibration])
                if np.max(gg.std(axis=0))>1.5 or np.max(aa.std(axis=0))>.025 or not .9<np.linalg.norm(aa.mean(axis=0))<1.1:
                    self.message='Calibration rejected: board moved. Keep still and retry.'
                else:
                    self.bias=gg.mean(axis=0); av=aa.mean(axis=0)
                    self.gravity=float(np.linalg.norm(av)); self.rest_bias[:]=0; self.window.clear()
                    self.q=R.align_vectors([[0,0,1]], [av/np.linalg.norm(av)])[0]
                    self.fusion.setBiasEstimate(np.deg2rad(self.bias), .002)
                    self.reference=self.q; self.reset_position(); self.calibrated=True
                    self.message=''
                self.calibrating=False
        # VQF receives timestamp-resampled 100 Hz data; gaps suspend position.
        self.fusion.update(np.deg2rad(rawg), a*9.80665)
        quat=self.fusion.getQuat6D()
        self.q=R.from_quat([quat[1],quat[2],quat[3],quat[0]])
        self.bias=np.rad2deg(self.fusion.getBiasEstimate()[0])
        gyro=rawg-self.bias
        stationary=bool(self.fusion.getRestDetected())
        raw_linear=(self.q.apply(a)-[0,0,self.gravity])*9.80665
        # Bias is estimated only during VQF-confirmed rest.
        if stationary:
            self.rest_bias+=(raw_linear-self.rest_bias)*(1-np.exp(-dt/2))
        linear=raw_linear-self.rest_bias
        # Low-pass world acceleration, not body acceleration (which mixes gravity
        # during rotation). This is a position-only filter; orientation is VQF.
        self.filtered_acc+=(linear-self.filtered_acc)*(1-np.exp(-2*np.pi*8*dt))
        if self.calibrated and not self.calibrating:
            if self.stabilize and stationary:
                self.velocity[:]=0; self.filtered_acc[:]=0
            else:
                drive=self.filtered_acc.copy()
                drive[2]=0  # Horizontal world plane, after gravity removal.
                if self.stabilize:
                    drive=np.sign(drive)*np.maximum(np.abs(drive)-.06,0)
                    self.velocity*=np.exp(-1.2*dt)
                old=self.velocity.copy(); self.velocity+=drive*dt
                self.position+=(old+self.velocity)*.5*dt
        self.velocity[2]=0; self.position[2]=0
        rel=self.reference.inv()*self.q
        self.last_data=time.monotonic()
        self.latest=dict(q=rel.as_quat().tolist(),angles=rel.as_euler('xyz',degrees=True).tolist(),
            accel=a.tolist(),gyro=gyro.tolist(),linear=linear.tolist(),position=self.position.tolist(),velocity=self.velocity.tolist(),
            stationary=stationary,calibrated=self.calibrated,calibrating=self.calibrating,
            calibration_progress=min(1,(self.calibration[-1][0]-self.calibration[0][0])/3) if self.calibrating else 0,
            bias=self.bias.tolist(),hz=round(self.samples/max(self.elapsed,.01)),gaps=self.gaps)
    def snapshot(self):
        return dict(self.latest,connected=self.connected and time.monotonic()-self.last_data<2,
                    port=self.port,message=self.message,stabilize=self.stabilize,position_epoch=self.position_epoch,recording=self.recording is not None,filter="VQF",packet_loss=getattr(self,"transport_stats",{}).get("unrecoverable",0),transport=getattr(self,"transport_stats",{}))
tracker=Tracker()
async def bluetooth_loop():
    stats={}
    tracker.transport_stats=stats
    def connected():
        tracker.port='BLE'; tracker.connected=True; tracker.prev=None
        tracker.raw_previous=None; tracker.uniform_time=None; tracker.raw_time=0.
        tracker.fusion=VQF(.01,restMinT=.5); tracker.samples=0; tracker.elapsed=0.; tracker.rest_bias[:]=0
        tracker.initialized=False; tracker.calibrated=False; tracker.calibrating=False
        tracker.reference=R.identity(); tracker.latest={}; tracker.last_data=0
        tracker.message='Bluetooth connected. Hold still and calibrate.'; tracker.reset_position()
    while True:
        try:
            tracker.message='Finding Bluetooth board…'
            await stream(tracker.feed,connected,stats)
        except asyncio.CancelledError:raise
        except Exception as e:tracker.message=str(e)
        finally:tracker.connected=False
        await asyncio.sleep(1)
@asynccontextmanager
async def lifespan(app):
    task=asyncio.create_task(bluetooth_loop())
    yield
    task.cancel()
    if tracker.recording: tracker.recording.close()
    try: await task
    except asyncio.CancelledError: pass
app=FastAPI(lifespan=lifespan)
app.mount('/static',StaticFiles(directory=ROOT/'static'),name='static')
@app.get('/')
def index(): return FileResponse(ROOT/'static/index.html')
@app.get('/api/state')
def state(): return tracker.snapshot()
@app.websocket('/ws')
async def ws(socket: WebSocket):
    if socket.headers.get('origin') not in (f'http://127.0.0.1:{PORT}',f'http://localhost:{PORT}'):
        await socket.close(code=1008); return
    await socket.accept()
    async def receive():
        while True:
            data=await socket.receive_json(); tracker.command(data.get('action'),data.get('value'))
    receiver=asyncio.create_task(receive())
    try:
        while not receiver.done():
            await socket.send_json(tracker.snapshot()); await asyncio.sleep(1/30)
    except (WebSocketDisconnect,RuntimeError): pass
    finally:
        receiver.cancel()
        try: await receiver
        except (asyncio.CancelledError,WebSocketDisconnect): pass
if __name__=='__main__':
    import uvicorn
    uvicorn.run(app,host='127.0.0.1',port=PORT)
