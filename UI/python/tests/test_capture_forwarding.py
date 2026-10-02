import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PySide6.QtGui import QGuiApplication, QImage
from PySide6.QtMultimedia import QVideoFrame, QVideoSink
from capture_receiver import CaptureReceiver

class Backend:
    connected = False
    def setCameraConnected(self, value): self.connected = value

app = QGuiApplication([])
a, b, backend = QVideoSink(), QVideoSink(), Backend()
receiver = CaptureReceiver("__no_capture_device_test__", a, b, backend)
image = QImage(320, 240, QImage.Format_RGBA8888)
image.fill("red")
receiver.present(QVideoFrame(image))
assert receiver.frames == 1 and backend.connected
for sink in (a, b):
    assert sink.videoFrame().toImage().pixelColor(100, 100).red() == 255
receiver.present(QVideoFrame())
assert receiver.frames == 1
receiver.close()
assert not backend.connected
print("PASS: frame forwarding to thumbnail and expanded view; invalid frame ignored")
