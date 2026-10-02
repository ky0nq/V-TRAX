import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path

from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot, QUrl
from PySide6.QtGui import QGuiApplication, QFont, QFontDatabase
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickWindow, QSGRendererInterface
from video_receiver import VideoReceiver, CameraProvider


class HudBackend(QObject):
    changed = Signal()

    def __init__(self):
        super().__init__()
        self._pressure, self._angle, self._connected = 85.0, 23.4, False
        self._camera_url = ""
        self._camera_connected = False
        self._cnn_fresh = False
        self._pressure_source = "demo"
        self._camera_frames = 0

    pressure = Property(float, lambda self: self._pressure, notify=changed)
    angle = Property(float, lambda self: self._angle, notify=changed)
    connected = Property(bool, lambda self: self._connected, notify=changed)
    status = Property(str, lambda self: "DANGER" if self._pressure >= 90 else "WARNING" if self._pressure >= 75 else "NORMAL", notify=changed)

    cameraUrl = Property(str, lambda self: self._camera_url, notify=changed)
    cameraConnected = Property(bool, lambda self: self._camera_connected, notify=changed)
    cnnFresh = Property(bool, lambda self: self._cnn_fresh, notify=changed)
    pressureSource = Property(str, lambda self: self._pressure_source, notify=changed)

    def setCameraFrame(self):
        self._camera_frames += 1
        self._camera_url = f"image://camera/frame/{self._camera_frames}"
        self.changed.emit()

    def setCameraConnected(self, value):
        if self._camera_connected != value:
            self._camera_connected = value
            self.changed.emit()

    def setMetadata(self, valid, source):
        source = str(source)
        if (self._cnn_fresh, self._pressure_source) != (valid, source):
            self._cnn_fresh, self._pressure_source = valid, source
            self.changed.emit()

    @Slot(float, float)
    def updateValues(self, pressure, angle):
        pressure, angle = float(pressure), float(angle)
        if not all(map(math.isfinite, (pressure, angle))):
            raise ValueError("Sensor values must be finite")
        self._pressure = max(0.0, min(100.0, pressure))
        self._angle = max(-90.0, min(90.0, angle))
        self.changed.emit()

    def setConnected(self, connected):
        if self._connected != connected:
            self._connected = connected
            self.changed.emit()


def main():
    parser = argparse.ArgumentParser(description="Night-drive HUD with demo and UDP input")
    parser.add_argument("--udp", action="store_true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7000)
    parser.add_argument("--snapshot", type=Path, help="Save a deterministic preview and exit")
    parser.add_argument("--video-port", type=int, default=7001)
    parser.add_argument("--board-ip", help="Accept UDP from this board IP only")
    parser.add_argument("--stats", action="store_true", help="Print video receive counters every 2 seconds")
    parser.add_argument("--snapshot-delay", type=int, default=3000, help="Snapshot delay in milliseconds")
    args = parser.parse_args()
    if args.udp and args.port == args.video_port:
        parser.error("Telemetry and video ports must differ")
    os.environ.setdefault("QT_QUICK_CONTROLS_STYLE", "Basic")
    os.environ.pop('QT_QUICK_BACKEND',None)
    QQuickWindow.setGraphicsApi(QSGRendererInterface.Direct3D11 if sys.platform=='win32' else QSGRendererInterface.OpenGL)
    app = QGuiApplication(sys.argv)
    for font_path in (Path(__file__).parent / "assets" / "fonts").glob("*.ttf"):
        QFontDatabase.addApplicationFont(str(font_path))
    app.setFont(QFont("Rajdhani", 12))
    # Offscreen Qt on Windows may not enumerate installed fonts automatically.
    if args.snapshot and sys.platform == "win32":
        fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for name in ("segoeui.ttf", "segoeuib.ttf", "consola.ttf", "consolab.ttf"):
            QFontDatabase.addApplicationFont(str(fonts / name))
    engine = QQmlApplicationEngine()
    backend = HudBackend()
    provider = CameraProvider()
    engine.addImageProvider("camera", provider)
    if args.udp:
        backend.updateValues(0, 0)
        backend.setMetadata(False, "unknown")
    engine.rootContext().setContextProperty("backend", backend)
    engine.rootContext().setContextProperty("sourceMode", "UDP" if args.udp else "DEMO")
    engine.load(QUrl.fromLocalFile(str(Path(__file__).with_name("Hud.qml"))))
    if not engine.rootObjects():
        return 1
    window = engine.rootObjects()[0]
    sock = None
    video = None
    if args.udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((args.host, args.port))
            video = VideoReceiver(args.host, args.video_port, args.board_ip)
        except OSError as exc:
            sock.close()
            print(f"UDP bind failed: {exc}", file=sys.stderr)
            return 1
        sock.setblocking(False)
    start = time.monotonic()
    last_packet = None
    last_video = None
    last_cnn_fresh = False
    next_stats = start + 2
    previous_frames = 0

    def tick():
        nonlocal last_packet, last_video, last_cnn_fresh, next_stats, previous_frames
        now = time.monotonic()
        if sock is not None:
            # Bound each batch so packet floods cannot starve the UI event loop.
            for _ in range(64):
                try:
                    payload, sender = sock.recvfrom(4096)
                    if args.board_ip and sender[0] != args.board_ip:
                        continue
                except BlockingIOError:
                    break
                try:
                    data = json.loads(payload)
                    if not isinstance(data, dict):
                        continue
                    backend.updateValues(data["pressure"], data["angle"])
                    last_packet = now
                    last_cnn_fresh = data.get("cnn_valid") is True
                    backend.setMetadata(last_cnn_fresh, data.get("pressure_source", "unknown"))
                except (ValueError, TypeError, KeyError, OverflowError):
                    continue
            data_live = last_packet is not None and now - last_packet < 1.2
            backend.setConnected(data_live)
            if not data_live:
                backend.setMetadata(False, backend.pressureSource)
            latest = video.take_latest()
            if latest:
                frame, last_video = latest
                provider.set_frame(frame)
                backend.setCameraFrame()
            backend.setCameraConnected(last_video is not None and now-last_video < 1.2)
            if args.stats and now >= next_stats:
                print(f"camera packets={video.packets} complete_frames={video.frames} "
                      f"fps~={(video.frames-previous_frames)/2:.1f} "
                      f"live={backend.cameraConnected} error={video.error or 'none'}", flush=True)
                previous_frames = video.frames
                next_stats = now + 2
        elif not window.property("paused") and not window.property("manualMode"):
            elapsed = now - start
            backend.updateValues(60 + 36 * math.sin(elapsed * 0.3), 85 * math.sin(elapsed * 0.55))

    timer = QTimer()
    timer.setInterval(33)
    timer.timeout.connect(tick)
    timer.start()
    if args.snapshot:
        def capture():
            try:
                args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                ok = window.grabWindow().save(str(args.snapshot))
                app.exit(0 if ok else 2)
            except Exception as exc:
                print(f"Snapshot failed: {exc}", file=sys.stderr)
                app.exit(2)
        QTimer.singleShot(args.snapshot_delay, capture)
    try:
        return app.exec()
    finally:
        timer.stop()
        if video is not None:
            video.close()
        if sock is not None:
            sock.close()


if __name__ == "__main__":
    sys.exit(main())

