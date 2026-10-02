"""UDP worker with a single latest-frame mailbox; UI never queues every frame."""
import socket
import sys
from array import array
import threading
import time
from urllib.request import Request, urlopen
from camera_discovery import discover_camera_url
from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider
from video_protocol import Reassembler

class VideoReceiver:
    def __init__(self, host, port, board_ip=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
            self.sock.bind((host, port))
            self.sock.settimeout(0.1)
        except Exception:
            self.sock.close()
            raise
        self.board_ip = board_ip
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.packets = 0
        self.frames = 0
        self.thread = threading.Thread(target=self._run, daemon=True, name='camera-udp')
        self.thread.start()

    def _run(self):
        assembler = Reassembler()
        peer = None
        last_peer_packet = 0.0
        try:
            while not self.stop_event.is_set():
                try:
                    packet, sender = self.sock.recvfrom(2048)
                except socket.timeout:
                    assembler.expire()
                    continue
                now = time.monotonic()
                if self.board_ip and sender[0] != self.board_ip:
                    continue
                self.packets += 1
                if peer is not None and sender != peer:
                    if now-last_peer_packet < 1.2:
                        continue
                    assembler = Reassembler()
                    peer = None
                frame = assembler.feed(packet, now)
                if frame is not None:
                    self.frames += 1
                    peer = sender
                    last_peer_packet = now
                    with self.lock:
                        self.latest = (frame, now)
        except OSError as exc:
            if not self.stop_event.is_set():
                self.error = str(exc)

    def take_latest(self):
        with self.lock:
            result, self.latest = self.latest, None
        return result

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=0.5)
        self.sock.close()


class FrameNotifier(QObject):
    frameReady = Signal()


class MjpegReceiver:
    """Receive and decode ESP32-CAM frames off the Qt GUI thread."""

    def __init__(self, url):
        self.url = url
        self.resolved_url = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.notification_pending = False
        self.notifier = FrameNotifier()
        self.frames = 0
        self.dropped_for_display = 0
        self.invalid_frames = 0
        self.frame_size = None
        self.last_jpeg_bytes = 0
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True, name='esp32-cam')

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                url = discover_camera_url(stop_event=self.stop_event) if self.url == 'auto' else self.url
                if not url:
                    raise ConnectionError('ESP32-CAM not found via mDNS (_visioncam._tcp.local.)')
                self.resolved_url = url
                request = Request(url, headers={'User-Agent': 'MotionHUD/1.0'})
                with urlopen(request, timeout=4) as response:
                    if 'multipart/' not in response.headers.get('Content-Type', '').lower():
                        raise ValueError('Use the ESP32-CAM MJPEG URL on port 81: /stream')
                    self.error = None
                    buffer = bytearray()
                    while not self.stop_event.is_set():
                        chunk = response.read1(4096)
                        if not chunk:
                            raise ConnectionError('Camera stream ended')
                        buffer.extend(chunk)
                        while True:
                            start = buffer.find(b'\xff\xd8')
                            if start < 0:
                                trailing_ff = buffer.endswith(b'\xff')
                                buffer.clear()
                                if trailing_ff:
                                    buffer.append(0xff)
                                break
                            if start:
                                del buffer[:start]
                            end = buffer.find(b'\xff\xd9', 2)
                            if end < 0:
                                if len(buffer) > 1024 * 1024:
                                    buffer.clear()
                                break
                            jpeg = bytes(buffer[:end + 2])
                            del buffer[:end + 2]
                            try:
                                image = QImage.fromData(jpeg, 'JPEG')
                            except ValueError:
                                self.invalid_frames += 1
                                continue
                            if image.isNull():
                                self.invalid_frames += 1
                                continue
                            notify = False
                            with self.lock:
                                if self.latest is not None:
                                    self.dropped_for_display += 1
                                self.latest = (image, time.monotonic())
                                self.frames += 1
                                self.frame_size = (image.width(), image.height())
                                self.last_jpeg_bytes = len(jpeg)
                                if not self.notification_pending:
                                    self.notification_pending = True
                                    notify = True
                            if notify:
                                self.notifier.frameReady.emit()
            except Exception as exc:
                self.error = str(exc)
                self.stop_event.wait(1)

    def take_latest(self):
        with self.lock:
            result, self.latest = self.latest, None
            self.notification_pending = False
        return result

    def close(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=0.5)


class CameraProvider(QQuickImageProvider):
    def __init__(self):
        super().__init__(QQuickImageProvider.Image)
        self.lock = threading.Lock()
        self.image = QImage(320, 180, QImage.Format_RGB888)
        self.image.fill(0x07111c)

    def set_frame(self, frame):
        if frame.format == 3:
            words = array('H')
            words.frombytes(frame.pixels)
            if sys.byteorder == 'little':
                words.byteswap()
            image = QImage(words.tobytes(), frame.width, frame.height,
                           frame.width*2, QImage.Format_RGB16).copy()
        else:
            image = QImage(frame.pixels, frame.width, frame.height,
                           frame.width*3, QImage.Format_RGB888).copy()
        with self.lock:
            self.image = image

    def set_jpeg(self, jpeg):
        try:
            image = QImage.fromData(jpeg, 'JPEG')
        except ValueError:
            # A damaged frame should be skipped without stopping the Qt timer.
            return False
        if image.isNull():
            return False
        with self.lock:
            self.image = image
        return True

    def set_image(self, image):
        """Publish a JPEG already decoded by the camera receiver thread."""
        if image.isNull():
            return False
        with self.lock:
            self.image = image
        return True

    def requestImage(self, identifier, size, requestedSize):
        with self.lock:
            image = self.image.copy()
        size.setWidth(image.width())
        size.setHeight(image.height())
        return image
