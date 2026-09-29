"""Headless Qt integration checks: python tests/test_ui.py"""
import os
os.environ['QT_QPA_PLATFORM']='windows'
os.environ.pop('QT_QUICK_BACKEND',None)
os.environ.setdefault('QT_QUICK_CONTROLS_STYLE','Basic')
import sys
from pathlib import Path
BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE))
from main import HudBackend
from PySide6.QtCore import QObject,QPoint,Qt,QUrl
from PySide6.QtGui import QGuiApplication,QFont,QFontDatabase
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickWindow,QSGRendererInterface
from PySide6.QtTest import QTest
QQuickWindow.setGraphicsApi(QSGRendererInterface.Direct3D11)
app=QGuiApplication([])
for p in (BASE/'assets/fonts').glob('*.ttf'): QFontDatabase.addApplicationFont(str(p))
app.setFont(QFont('Rajdhani',12))
b=HudBackend()
e=QQmlApplicationEngine(); warnings=[]
e.warnings.connect(lambda items:warnings.extend(i.toString() for i in items))
e.rootContext().setContextProperty('backend',b)
e.rootContext().setContextProperty('sourceMode','DEMO')
e.load(QUrl.fromLocalFile(str(BASE/'Hud.qml')))
assert e.rootObjects(),'QML load failed'
w=e.rootObjects()[0]; QTest.qWait(300)
road=w.findChild(QObject,'drivingScene')
assert road is not None
for angle in (-90,0,90):
    b.updateValues(68,angle);QTest.qWait(220)
    assert abs(road.property('steering')-angle/90)<.001
    assert not w.grabWindow().isNull()
b.updateValues(140,-150); assert b.pressure==100 and b.angle==-90
for bad in (float('nan'),float('inf')):
    try: b.updateValues(50,bad)
    except ValueError: pass
    else: raise AssertionError('nonfinite accepted')
w.setProperty('paused',True);QTest.qWait(100)
p=road.property('phase');QTest.qWait(140);assert road.property('phase')==p
w.setProperty('paused',False);QTest.qWait(140);assert road.property('phase')!=p
# Sensor load controls measured lane travel, zero settles to a complete stop.
b.updateValues(0,0);QTest.qWait(650)
assert road.property('speedRatio') == 0
p=road.property('phase');QTest.qWait(200);assert road.property('phase')==p
samples=[]
for pressure in (25,75):
    b.updateValues(pressure,0);QTest.qWait(600)
    assert abs(road.property('speedRatio')-pressure/100)<.001
    p=road.property('travel');QTest.qWait(240)
    samples.append(road.property('travel')-p)
assert 2.3 < samples[1]/samples[0] < 3.7, samples
assert road.property('maxCyclesPerSecond') >= 12
print('Measured travel at 25% / 75%:', samples)
# Settings accept manual input without freezing road animation.
QTest.mouseClick(w,Qt.LeftButton,Qt.NoModifier,QPoint(1728,43));QTest.qWait(100)
assert w.property('controls')
QTest.mouseClick(w,Qt.LeftButton,Qt.NoModifier,QPoint(1700,375));QTest.qWait(120)
assert w.property('manualMode') and not w.property('paused') and b.angle>70
QTest.mouseClick(w,Qt.LeftButton,Qt.NoModifier,QPoint(1630,485));QTest.qWait(80)
assert not w.property('controls')
QTest.mouseClick(w,Qt.LeftButton,Qt.NoModifier,QPoint(1125,510));QTest.qWait(80)
assert w.property('cameraExpanded')
QTest.mouseClick(w,Qt.LeftButton,Qt.NoModifier,QPoint(100,300));QTest.qWait(80)
assert not w.property('cameraExpanded')
e.rootContext().setContextProperty('sourceMode','UDP');b.setConnected(False);QTest.qWait(220)
assert not w.property('live') and not road.property('live') and road.property('steering')==0
assert road.property('speedRatio')==0
p=road.property('phase');QTest.qWait(140);assert road.property('phase')==p
b.setConnected(True);QTest.qWait(100);assert w.property('live')
w.resize(1080,480);QTest.qWait(150);assert not w.grabWindow().isNull()
assert not warnings,warnings
w.close();print('PASS: steering direction/endpoints, clamps, nonfinite, animation pause, settings/slider, camera, no signal, resize')


