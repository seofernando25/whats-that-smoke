import itertools
from unittest.mock import MagicMock, patch
from whats_that_smoke import web as m

def check_stall_recovery(byte_spam, force_kill=False):
    camera=m.CameraStream();processes=[];clock=itertools.count(0,.3)
    def spawn(*args,**kwargs):
        proc=MagicMock()
        if force_kill and not processes:
            proc.wait.side_effect=[m.subprocess.TimeoutExpired('rpicam-vid', 2), 0]
        processes.append(proc);return proc
    def selected(*args):
        return ([processes[-1].stdout] if byte_spam or len(processes)>1 else [],[],[])
    def read(*args):
        if len(processes)==1: return b'not a JPEG'
        camera.stop.set();return b'\xff\xd8sample\xff\xd9'
    with patch.object(m.subprocess,'Popen',side_effect=spawn),patch.object(m.select,'select',side_effect=selected),patch.object(m.os,'read',side_effect=read),patch.object(m.time,'monotonic',side_effect=lambda:next(clock)),patch.object(camera.stop,'wait',return_value=False):
        camera._run()
    assert len(processes)==2
    assert all(p.terminate.call_count==1 for p in processes)
    assert processes[0].kill.call_count == int(force_kill)
    assert processes[1].kill.call_count == 0
    assert camera.frame==b'\xff\xd8sample\xff\xd9' and camera.seq==1
    print('PASS camera recovers from', 'non-frame byte spam' if byte_spam else 'live process with silent pipe')

def test_camera_recovers_live_process_with_silent_pipe():
    check_stall_recovery(False)


def test_camera_recovers_process_sending_no_complete_frames():
    check_stall_recovery(True)


def test_camera_kills_process_that_ignores_termination():
    check_stall_recovery(False, force_kill=True)
