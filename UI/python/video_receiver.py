"""UDP worker with a single latest-frame mailbox; UI never queues every frame."""
import socket
import sys
from array import array
import threading
import time
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

    def requestImage(self, identifier, size, requestedSize):
        with self.lock:
            image = self.image.copy()
        size.setWidth(image.width())
        size.setHeight(image.height())
        return image
