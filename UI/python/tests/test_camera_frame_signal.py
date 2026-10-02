"""A local 25 FPS MJPEG stream should reach the UI without timer polling losses."""
import sys
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))

from PySide6.QtCore import QByteArray, QBuffer, QIODevice, QObject, Qt
from PySide6.QtGui import QGuiApplication, QImage
from PySide6.QtTest import QTest

from main import EspCameraPresenter, HudBackend
from video_receiver import MjpegReceiver


app = QGuiApplication([])
image = QImage(160, 120, QImage.Format_RGB32)
image.fill(0x3478AA)
data = QByteArray()
buffer = QBuffer(data)
buffer.open(QIODevice.OpenModeFlag.WriteOnly)
assert image.save(buffer, 'JPEG')
jpeg = bytes(data)
stop_stream = threading.Event()


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while not stop_stream.is_set():
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')
                self.wfile.flush()
                time.sleep(0.04)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_args):
        pass


class Sink:
    def __init__(self):
        self.frames = 0

    def setVideoFrame(self, frame):
        assert frame.isValid()
        self.frames += 1


server = ThreadingHTTPServer(('127.0.0.1', 0), StreamHandler)
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()
receiver = MjpegReceiver(f'http://127.0.0.1:{server.server_port}/stream')
main_sink = Sink()
backend = HudBackend(esp_camera_enabled=True)
presenter = EspCameraPresenter(receiver, main_sink, Sink(), QObject(), backend)
receiver.notifier.frameReady.connect(presenter.present, Qt.ConnectionType.QueuedConnection)
receiver.start()
try:
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        QTest.qWait(20)
    stop_stream.set()
    QTest.qWait(100)
    assert receiver.frames >= 15, (receiver.frames, receiver.error)
    assert main_sink.frames >= receiver.frames - 3, (receiver.frames, main_sink.frames, receiver.dropped_for_display)
    assert backend._esp_camera_frames == main_sink.frames
finally:
    stop_stream.set()
    receiver.close()
    server.shutdown()
    server.server_close()

print(f'PASS: received={receiver.frames} displayed={main_sink.frames} skipped={receiver.dropped_for_display}')
