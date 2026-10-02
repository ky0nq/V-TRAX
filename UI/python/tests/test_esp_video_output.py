"""Check that ESP32-CAM frames remain visible through the Qt video sinks."""
import os
import sys
from pathlib import Path

os.environ['QT_QUICK_CONTROLS_STYLE'] = 'Basic'
BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))

from PySide6.QtCore import QObject, QUrl
from PySide6.QtGui import QGuiApplication, QImage
from PySide6.QtMultimedia import QVideoFrame
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickWindow, QSGRendererInterface
from PySide6.QtTest import QTest

from main import HudBackend
from video_receiver import CameraProvider


QQuickWindow.setGraphicsApi(QSGRendererInterface.Direct3D11)
app = QGuiApplication([])
engine = QQmlApplicationEngine()
warnings = []
engine.warnings.connect(lambda items: warnings.extend(item.toString() for item in items))
engine.addImageProvider('camera', CameraProvider())
backend = HudBackend(esp_camera_enabled=True)
engine.rootContext().setContextProperty('backend', backend)
engine.rootContext().setContextProperty('sourceMode', 'DEMO')
engine.load(QUrl.fromLocalFile(str(BASE / 'Hud.qml')))
assert engine.rootObjects(), warnings
window = engine.rootObjects()[0]


def sink(name):
    output = window.findChild(QObject, name)
    assert output is not None, name
    result = output.property('videoSink')
    assert result is not None, name
    return result


main_sink = sink('mainEspCamera')
expanded_sink = sink('expandedEspCamera')


def frame(color):
    image = QImage(320, 240, QImage.Format_RGB32)
    image.fill(color)
    result = QVideoFrame(image)
    assert result.isValid()
    return result


def sample(x, y):
    shot = window.grabWindow()
    ratio = shot.width() / window.width()
    return shot.pixelColor(int(x * ratio), int(y * ratio))


backend.setEspCameraConnected(True)
main_sink.setVideoFrame(frame(0xFF0000))
QTest.qWait(250)
color = sample(700, 280)
assert color.red() > 220 and color.green() < 30, (color.red(), color.green(), warnings)

window.setProperty('cameraExpanded', True)
window.setProperty('expandedCamera', 'esp32')
green_frame = frame(0x00FF00)
main_sink.setVideoFrame(green_frame)
expanded_sink.setVideoFrame(green_frame)
QTest.qWait(250)
color = sample(900, 400)
assert color.green() > 220 and color.red() < 30, (color.red(), color.green(), warnings)

assert not warnings, warnings
window.close()
print('PASS: ESP32-CAM main and expanded video outputs show frames without URL reload')
