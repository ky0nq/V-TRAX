"""USB/HDMI capture input; forwards native frames without JPEG conversion."""
import time
from PySide6.QtCore import QObject, Slot
from PySide6.QtMultimedia import QCamera, QMediaDevices, QMediaCaptureSession, QVideoSink


class CaptureReceiver(QObject):
    def __init__(self, selector, main_sink, expanded_sink, backend, parent=None):
        super().__init__(parent)
        self.selector = selector
        self.main_sink = main_sink
        self.expanded_sink = expanded_sink
        self.backend = backend
        self.camera = None
        self.last_frame_at = None
        self.frames = 0
        self.devices = QMediaDevices(self)
        self.devices.videoInputsChanged.connect(self.refresh)
        self.session = QMediaCaptureSession(self)
        self.sink = QVideoSink(self)
        self.session.setVideoSink(self.sink)
        self.sink.videoFrameChanged.connect(self.present)
        self.refresh()

    @Slot()
    def refresh(self):
        devices = QMediaDevices.videoInputs()
        if self.selector.isdigit():
            matches = devices[int(self.selector):int(self.selector)+1]
        elif self.selector.lower() == 'auto':
            matches = [d for d in devices if any(s in d.description().lower()
                       for s in ('capture', 'usb video', 'hdmi', 'cam link'))]
        else:
            matches = [d for d in devices if self.selector.lower() in d.description().lower()]
        if len(matches) != 1:
            self.close()
            print('USB capture waiting: specify --capture-device NAME or INDEX; available:',
                  [(i, d.description()) for i, d in enumerate(devices)], flush=True)
            return
        device = matches[0]
        if self.camera and self.camera.cameraDevice().id() == device.id():
            return
        self.close()
        self.camera = QCamera(device, self)
        self.camera.errorOccurred.connect(lambda *args: print('USB capture error:', args, flush=True))
        formats = [f for f in device.videoFormats() if f.maxFrameRate() >= 30
                   and f.resolution().width() <= 1920]
        if formats:
            preferred = min(formats, key=lambda f: abs(f.resolution().width()-1280)
                            + abs(f.resolution().height()-720))
            self.camera.setCameraFormat(preferred)
        self.session.setCamera(self.camera)
        self.camera.start()
        print('USB capture selected:', device.description(), flush=True)

    @Slot(object)
    def present(self, frame):
        if not frame.isValid():
            return
        self.main_sink.setVideoFrame(frame)
        self.expanded_sink.setVideoFrame(frame)
        self.last_frame_at = time.monotonic()
        self.frames += 1
        self.backend.setCameraConnected(True)

    def close(self):
        if self.camera:
            self.camera.stop()
            self.session.setCamera(None)
            self.camera.deleteLater()
            self.camera = None
        self.last_frame_at = None
        self.backend.setCameraConnected(False)
