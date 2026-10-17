"""Local browser controls for the simulated Freenove car."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import threading
import webbrowser
from urllib.request import urlopen
from urllib.error import URLError

HERE = Path(__file__).resolve().parent

class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass

    def reply(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/api/status':
            return self.reply(dict(app='ugv-gazebo-controls', **self.server.bridge.status()))
        if self.path not in {'/', '/index.html', '/app.js', '/style.css'}:
            return self.reply({'error': 'Not found'}, 404)
        super().do_GET()

    def do_POST(self):
        allowed = f'http://127.0.0.1:{self.server.server_port}'
        if self.headers.get('Origin', allowed) != allowed:
            return self.reply({'error': 'Local panel requests only'}, 403)
        if self.headers.get_content_type() != 'application/json':
            return self.reply({'error': 'JSON required'}, 415)
        try:
            length = int(self.headers.get('Content-Length', 0))
            if not 0 < length <= 2048:
                raise ValueError('Invalid request size')
            message = json.loads(self.rfile.read(length))
            if self.path == '/api/drive':
                changed = self.server.bridge.control.drive(message['client'], message['seq'],
                                                          message['keys'], message['speed'], message.get('pwm'))
                if changed:self.server.bridge.publish_command()
                return self.reply({'accepted': changed})
            if self.path == '/api/action':
                self.server.bridge.action(message['action'])
                return self.reply({'ok': True})
            return self.reply({'error': 'Not found'}, 404)
        except (ValueError, KeyError, TypeError) as error:
            return self.reply({'error': str(error)}, 400)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    url = f'http://127.0.0.1:{args.port}'
    try:
        with urlopen(url+'/api/status', timeout=.3) as response:
            if json.load(response).get('app') == 'ugv-gazebo-controls':
                print(f'Control panel is already running: {url}', flush=True)
                if not args.no_browser:
                    webbrowser.open(url)
                return
    except (URLError, TimeoutError, ValueError):
        pass
    try:
        from gazebo_bridge import GazeboBridge
    except ImportError as error:
        parser.error(f'{error}. Run ./launch_control.command to use the Gazebo Python environment.')
    handler = partial(Handler, directory=str(HERE/'panel'))
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), handler)
    except OSError as error:
        parser.error(f'Could not open {url}: {error}')
    bridge = GazeboBridge()
    server.bridge = bridge
    signal.signal(signal.SIGTERM, lambda *_: threading.Thread(target=server.shutdown).start())
    print(f'UGV simulation controls: {url}', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        server.server_close()

if __name__ == '__main__':
    main()
