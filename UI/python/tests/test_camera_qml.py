"""Visual integration: real QML camera thumbnail and enlarged view refresh."""
import sys
import os
os.environ['QT_QUICK_CONTROLS_STYLE']='Basic'
from pathlib import Path
BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE))
from PySide6.QtCore import QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickWindow,QSGRendererInterface
from PySide6.QtTest import QTest
from main import HudBackend
from video_receiver import CameraProvider
from video_protocol import Frame

QQuickWindow.setGraphicsApi(QSGRendererInterface.Direct3D11)
app=QGuiApplication([]);engine=QQmlApplicationEngine();warnings=[]
engine.warnings.connect(lambda items:warnings.extend(i.toString() for i in items))
backend=HudBackend();provider=CameraProvider()
engine.addImageProvider('camera',provider)
engine.rootContext().setContextProperty('backend',backend)
engine.rootContext().setContextProperty('sourceMode','UDP')
engine.load(QUrl.fromLocalFile(str(BASE/'Hud.qml')))
assert engine.rootObjects(),'QML did not load'
window=engine.rootObjects()[0]
def frame(pixel,sequence):
    provider.set_frame(Frame(1280,720,pixel*(1280*720),sequence,3))
    backend.setCameraFrame();backend.setCameraConnected(True);QTest.qWait(300)
def sample(x,y):
    shot=window.grabWindow();ratio=shot.width()/window.width()
    return shot.pixelColor(int(x*ratio),int(y*ratio))
frame(b'\xf8\x00',1)
c=sample(1139,500);assert c.red()>240 and c.green()<10,(c.red(),c.green())
window.setProperty('cameraExpanded',True);QTest.qWait(300)
c=sample(900,400);assert c.red()>240 and c.green()<10
frame(b'\x07\xe0',2)
c=sample(900,400);assert c.green()>240 and c.red()<10
window.setProperty('cameraExpanded',False);QTest.qWait(200)
c=sample(1139,500);assert c.green()>240 and c.red()<10
backend.setCameraConnected(False);QTest.qWait(100)
assert not backend.cameraConnected
assert not warnings,warnings
window.close()
print('PASS: thumbnail, enlarged view, frame refresh, signal state, no QML warnings')
