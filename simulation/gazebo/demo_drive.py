"""Drive the Gazebo-only car in a slow circle, then send a stop command."""
import argparse
import shutil
import signal
import subprocess
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=60)
args = parser.parse_args()
if not 0 < args.seconds <= 300:
    parser.error('--seconds must be between 0 and 300')
gz = shutil.which('gz')
if not gz:
    parser.error('gz is not installed or on PATH')

def publish(speed, turn):
    subprocess.run([gz, 'topic', '-t', '/model/freenove/cmd_vel',
                    '-m', 'gz.msgs.Twist', '-p',
                    f'linear: {{x: {speed}}}, angular: {{z: {turn}}}'],
                   check=True, timeout=5)

def interrupt(signum, frame):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, interrupt)
print(f'Simulation demo: slow circle for {args.seconds:g} seconds', flush=True)
try:
    publish(.06, .45)
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        time.sleep(min(.2, max(0, deadline-time.monotonic())))
except KeyboardInterrupt:
    pass
finally:
    publish(0, 0)
    print('Sent simulation STOP', flush=True)
