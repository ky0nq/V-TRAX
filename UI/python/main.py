import argparse
import json
import math
import os
import socket
import sys
import time
from pathlib import Path

from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot, QUrl, QEvent, Qt
from PySide6.QtGui import QGuiApplication, QFont, QFontDatabase
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickWindow, QSGRendererInterface
from PySide6.QtMultimedia import QVideoFrame
from video_receiver import VideoReceiver, MjpegReceiver, CameraProvider
from capture_receiver import CaptureReceiver
from vehicle_serial import VehicleSerial, pressure_percent


class HudBackend(QObject):
    changed = Signal()
    cameraFrameChanged = Signal()
    espCameraFrameChanged = Signal()

    def __init__(self, vehicle_mode=False, esp_camera_enabled=False):
        super().__init__()
        self._vehicle_mode = vehicle_mode
        self._esp_camera_enabled = esp_camera_enabled
        self._pressure, self._angle, self._connected = 85.0, 23.4, False
        self._camera_url = ""
        self._camera_connected = False
        self._esp_camera_url = ""
        self._esp_camera_connected = False
        self._esp_camera_frames = 0
        self._cnn_fresh = False
        self._pressure_source = "demo"
        self._camera_frames = 0
        self._vehicle_status = "WAITING FOR ZYBO"
        self._emergency_stop = False
        self._accel_raw = 0
        self._brake_raw = 0
        self._accel_level = 0
        self._brake_level = 0

    pressure = Property(float, lambda self: self._pressure, notify=changed)
    angle = Property(float, lambda self: self._angle, notify=changed)
    connected = Property(bool, lambda self: self._connected, notify=changed)
    status = Property(str, lambda self: self._vehicle_status if self._vehicle_mode else "DANGER" if self._pressure >= 90 else "WARNING" if self._pressure >= 75 else "NORMAL", notify=changed)
    emergencyStop = Property(bool, lambda self: self._emergency_stop, notify=changed)
    accelRaw = Property(int, lambda self: self._accel_raw, notify=changed)
    brakeRaw = Property(int, lambda self: self._brake_raw, notify=changed)
    accelLevel = Property(int, lambda self: self._accel_level, notify=changed)
    brakeLevel = Property(int, lambda self: self._brake_level, notify=changed)

    cameraUrl = Property(str, lambda self: self._camera_url, notify=cameraFrameChanged)
    cameraConnected = Property(bool, lambda self: self._camera_connected, notify=changed)
    espCameraUrl = Property(str, lambda self: self._esp_camera_url, notify=espCameraFrameChanged)
    espCameraConnected = Property(bool, lambda self: self._esp_camera_connected, notify=changed)
    espCameraEnabled = Property(bool, lambda self: self._esp_camera_enabled, notify=changed)
    cnnFresh = Property(bool, lambda self: self._cnn_fresh, notify=changed)
    pressureSource = Property(str, lambda self: self._pressure_source, notify=changed)

    def setCameraFrame(self):
        self._camera_frames += 1
        self._camera_url = f"image://camera/frame/{self._camera_frames}"
        self.cameraFrameChanged.emit()

    def setCameraConnected(self, value):
        if self._camera_connected != value:
            self._camera_connected = value
            self.changed.emit()

    def setEspCameraFrame(self):
        self._esp_camera_frames += 1
        self._esp_camera_url = f"image://esp32camera/frame/{self._esp_camera_frames}"
        self.espCameraFrameChanged.emit()

    def setEspCameraConnected(self, value):
        if self._esp_camera_connected != value:
            self._esp_camera_connected = value
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

    def setVehicleState(self, status, local_estop=False):
        if status is None:
            state = ("WAITING FOR ZYBO", True, 0, 0, 0, 0)
        else:
            stopped = local_estop or status.estop or not status.sensor_ok or not status.steering_ok
            label = "E-STOP" if stopped else "BRAKING" if status.brake_level else "DRIVING"
            state = (label, stopped, status.accel_raw, status.brake_raw,
                     status.accel_level, status.brake_level)
        old = (self._vehicle_status, self._emergency_stop, self._accel_raw,
               self._brake_raw, self._accel_level, self._brake_level)
        if state != old:
            (self._vehicle_status, self._emergency_stop, self._accel_raw,
             self._brake_raw, self._accel_level, self._brake_level) = state
            self.changed.emit()


class VehicleKeys(QObject):
    def __init__(self, controller, app):
        super().__init__()
        self.controller = controller
        self.app = app

    def eventFilter(self, watched, event):
        if event.type() in (QEvent.Type.ApplicationDeactivate, QEvent.Type.WindowDeactivate):
            self.controller.held_since.clear()
            return False
        if event.type() not in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease):
            return False
        key = {Qt.Key.Key_Left: "left", Qt.Key.Key_Right: "right",
               Qt.Key.Key_Space: "space", Qt.Key.Key_R: "r",
               Qt.Key.Key_Q: "q"}.get(event.key())
        if key is None:
            return False
        if event.isAutoRepeat():
            return True
        if event.type() == QEvent.Type.KeyPress:
            if key == "q":
                self.app.quit()
            else:
                self.controller.key_press(key)
        else:
            self.controller.key_release(key)
        return True


class EspCameraPresenter(QObject):
    """Publish the newest decoded frame when Qt can process it, without a frame queue."""

    def __init__(self, receiver, main_sink, expanded_sink, window, backend):
        super().__init__(window)
        self.receiver = receiver
        self.main_sink = main_sink
        self.expanded_sink = expanded_sink
        self.window = window
        self.backend = backend
        self.last_frame_at = None

    @Slot()
    def present(self):
        latest = self.receiver.take_latest()
        if latest is None:
            return
        image, received_at = latest
        video_frame = QVideoFrame(image)
        if not video_frame.isValid():
            return
        self.main_sink.setVideoFrame(video_frame)
        if self.window.property("cameraExpanded") and self.window.property("expandedCamera") == "esp32":
            self.expanded_sink.setVideoFrame(video_frame)
        self.last_frame_at = received_at
        self.backend.setEspCameraFrame()
        self.backend.setEspCameraConnected(True)


def main():
    parser = argparse.ArgumentParser(description="Night-drive HUD with demo and UDP input")
    parser.add_argument("--capture-device", help="USB capture device name, index, or auto")
    parser.add_argument("--list-cameras", action="store_true")
    parser.add_argument("--udp", action="store_true")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7000)
    parser.add_argument("--snapshot", type=Path, help="Save a deterministic preview and exit")
    parser.add_argument("--video-port", type=int, default=7001)
    parser.add_argument("--esp32-cam-url", help="ESP32-CAM MJPEG URL or 'auto' for mDNS discovery")
    parser.add_argument("--board-ip", help="Accept UDP from this board IP only")
    parser.add_argument("--serial-port", help="Zybo vehicle UART port, e.g. COM5")
    parser.add_argument("--fsr-idle", type=int, default=7000, help="Accelerator RAW value treated as 0%%")
    parser.add_argument("--fsr-full", type=int, default=22000, help="Accelerator RAW value treated as 100%%")
    parser.add_argument("--stats", action="store_true", help="Print video receive counters every 2 seconds")
    parser.add_argument("--snapshot-delay", type=int, default=3000, help="Snapshot delay in milliseconds")
    args = parser.parse_args()
    if args.udp and args.capture_device:
        parser.error("Choose either --udp or --capture-device for Zybo video")
    if args.udp and args.port == args.video_port:
        parser.error("Telemetry and video ports must differ")
    if args.esp32_cam_url and args.esp32_cam_url != "auto" and not args.esp32_cam_url.startswith("http://"):
        parser.error("--esp32-cam-url must be 'auto' or start with http://")
    if args.fsr_full <= args.fsr_idle:
        parser.error("--fsr-full must exceed --fsr-idle")
    os.environ.setdefault("QT_QUICK_CONTROLS_STYLE", "Basic")
    os.environ.pop('QT_QUICK_BACKEND',None)
    QQuickWindow.setGraphicsApi(QSGRendererInterface.Direct3D11 if sys.platform=='win32' else QSGRendererInterface.OpenGL)
    app = QGuiApplication(sys.argv)
    if args.list_cameras:
        from PySide6.QtMultimedia import QMediaDevices
        for i, device in enumerate(QMediaDevices.videoInputs()):
            print(f"{i}: {device.description()}")
        return 0
    for font_path in (Path(__file__).parent / "assets" / "fonts").glob("*.ttf"):
        QFontDatabase.addApplicationFont(str(font_path))
    app.setFont(QFont("Rajdhani", 12))
    # Offscreen Qt on Windows may not enumerate installed fonts automatically.
    if args.snapshot and sys.platform == "win32":
        fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for name in ("segoeui.ttf", "segoeuib.ttf", "consola.ttf", "consolab.ttf"):
            QFontDatabase.addApplicationFont(str(fonts / name))
    engine = QQmlApplicationEngine()
    backend = HudBackend(vehicle_mode=bool(args.serial_port), esp_camera_enabled=bool(args.esp32_cam_url))
    controller = VehicleSerial(args.serial_port) if args.serial_port else None
    keys = VehicleKeys(controller, app) if controller else None
    if keys is not None:
        app.installEventFilter(keys)
    provider = CameraProvider()
    engine.addImageProvider("camera", provider)
    if args.udp or controller is not None:
        backend.updateValues(0, 0)
        backend.setMetadata(False, "unknown")
    engine.rootContext().setContextProperty("backend", backend)
    engine.rootContext().setContextProperty("sourceMode", "SERIAL" if controller else "UDP" if args.udp else "DEMO")
    engine.rootContext().setContextProperty("captureMode", bool(args.capture_device))
    engine.load(QUrl.fromLocalFile(str(Path(__file__).with_name("Hud.qml"))))
    if not engine.rootObjects():
        return 1
    window = engine.rootObjects()[0]
    main_video_output = window.findChild(QObject, "mainEspCamera")
    expanded_video_output = window.findChild(QObject, "expandedEspCamera")
    if main_video_output is None or expanded_video_output is None:
        raise RuntimeError("ESP32-CAM video outputs missing from Hud.qml")
    esp_video_sink = main_video_output.property("videoSink")
    expanded_esp_video_sink = expanded_video_output.property("videoSink")
    if esp_video_sink is None or expanded_esp_video_sink is None:
        raise RuntimeError("Qt Multimedia video sinks are unavailable")
    capture_video = None
    if args.capture_device:
        capture_video = CaptureReceiver(args.capture_device,
            window.findChild(QObject, "mainCaptureCamera").property("videoSink"),
            window.findChild(QObject, "expandedCaptureCamera").property("videoSink"), backend, window)
    sock = None
    udp_video = None
    esp_video = None
    if args.udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((args.host, args.port))
            udp_video = VideoReceiver(args.host, args.video_port, args.board_ip)
        except OSError as exc:
            sock.close()
            print(f"UDP bind failed: {exc}", file=sys.stderr)
            return 1
        sock.setblocking(False)
    esp_presenter = None
    if args.esp32_cam_url:
        esp_video = MjpegReceiver(args.esp32_cam_url)
        esp_presenter = EspCameraPresenter(esp_video, esp_video_sink,
                                           expanded_esp_video_sink, window, backend)
        esp_video.notifier.frameReady.connect(esp_presenter.present, Qt.ConnectionType.QueuedConnection)
        esp_video.start()
    start = time.monotonic()
    last_packet = None
    last_udp_video = None
    last_cnn_fresh = False
    next_stats = start + 2
    previous_udp_frames = 0
    previous_esp_frames = 0
    previous_esp_displayed = 0
    previous_stats_at = start
    ui_ticks = 0
    previous_ui_ticks = 0
    last_serial_update_at = None
    serial_was_live = False

    def tick():
        nonlocal last_packet, last_udp_video, last_cnn_fresh, next_stats
        nonlocal previous_udp_frames, previous_esp_frames, previous_esp_displayed, previous_stats_at
        nonlocal last_serial_update_at, serial_was_live
        nonlocal ui_ticks, previous_ui_ticks
        now = time.monotonic()
        ui_ticks += 1
        if controller is not None:
            controller.poll(now)
            status = controller.fresh_status(now)
            live = status is not None
            backend.setConnected(live)
            backend.setVehicleState(status, controller.emergency_stop)
            if live and controller.status_at != last_serial_update_at:
                pressure = pressure_percent(status, args.fsr_idle, args.fsr_full)
                if controller.emergency_stop:
                    pressure = 0.0
                backend.updateValues(pressure, status.steering)
                backend.setMetadata(status.steering_ok, "sensor")
                last_serial_update_at = controller.status_at
            elif not live and serial_was_live:
                backend.updateValues(0, 0)
                backend.setMetadata(False, "unknown")
            elif live and controller.emergency_stop and backend.pressure != 0:
                backend.updateValues(0, backend.angle)
            serial_was_live = live
        if sock is not None:
            # Bound each batch so packet floods cannot starve the UI event loop.
            for _ in range(64):
                try:
                    payload, sender = sock.recvfrom(4096)
                    if args.board_ip and sender[0] != args.board_ip:
                        continue
                except BlockingIOError:
                    break
                if controller is not None:
                    continue
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
            if controller is None:
                data_live = last_packet is not None and now - last_packet < 1.2
                backend.setConnected(data_live)
                if not data_live:
                    backend.setMetadata(False, backend.pressureSource)
        elif controller is None and not window.property("paused") and not window.property("manualMode"):
            elapsed = now - start
            backend.updateValues(60 + 36 * math.sin(elapsed * 0.3), 85 * math.sin(elapsed * 0.55))

        if capture_video is not None:
            backend.setCameraConnected(capture_video.last_frame_at is not None
                                       and now-capture_video.last_frame_at < 1.2)
        if udp_video is not None:
            latest = udp_video.take_latest()
            if latest:
                frame, received_at = latest
                provider.set_frame(frame)
                last_udp_video = received_at
                backend.setCameraFrame()
            backend.setCameraConnected(last_udp_video is not None and now-last_udp_video < 1.2)
        if esp_video is not None:
            backend.setEspCameraConnected(esp_presenter.last_frame_at is not None
                                          and now-esp_presenter.last_frame_at < 1.2)
        if args.stats and now >= next_stats:
            stats_seconds = max(now - previous_stats_at, 0.001)
            if udp_video is not None:
                print(f"zybo camera packets={udp_video.packets} frames={udp_video.frames} "
                      f"fps~={(udp_video.frames-previous_udp_frames)/stats_seconds:.1f} "
                      f"live={backend.cameraConnected} error={udp_video.error or 'none'}", flush=True)
                previous_udp_frames = udp_video.frames
            if esp_video is not None:
                print(f"esp32 camera frames={esp_video.frames} "
                      f"size={esp_video.frame_size or 'waiting'} jpeg_bytes={esp_video.last_jpeg_bytes} "
                      f"receive_fps~={(esp_video.frames-previous_esp_frames)/stats_seconds:.1f} "
                      f"display_fps~={(backend._esp_camera_frames-previous_esp_displayed)/stats_seconds:.1f} "
                      f"ui_tick_fps~={(ui_ticks-previous_ui_ticks)/stats_seconds:.1f} "
                      f"display_skipped={esp_video.dropped_for_display} "
                      f"invalid={esp_video.invalid_frames} "
                      f"live={backend.espCameraConnected} "
                      f"url={esp_video.resolved_url or 'searching'} "
                      f"error={esp_video.error or 'none'}", flush=True)
                previous_esp_frames = esp_video.frames
                previous_esp_displayed = backend._esp_camera_frames
            previous_stats_at = now
            previous_ui_ticks = ui_ticks
            next_stats = now + 2

    timer = QTimer()
    timer.setInterval(20 if controller else 33)
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
        if capture_video is not None:
            capture_video.close()
        if udp_video is not None:
            udp_video.close()
        if esp_video is not None:
            esp_video.close()
        if sock is not None:
            sock.close()
        if controller is not None:
            controller.close()


if __name__ == "__main__":
    sys.exit(main())

