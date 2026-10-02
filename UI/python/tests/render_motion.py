"""Render the motion preview from the live QML scene."""

import os
import sys
import math
from io import BytesIO
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "windows")
os.environ.setdefault("QT_QUICK_CONTROLS_STYLE", "Basic")
os.environ.pop("QT_QUICK_BACKEND", None)

from PIL import Image, ImageDraw, ImageFont
from PySide6.QtCore import QBuffer, QByteArray, QIODevice, QUrl
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuick import QQuickWindow, QSGRendererInterface
from PySide6.QtTest import QTest

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE))
from main import HudBackend

QQuickWindow.setGraphicsApi(QSGRendererInterface.Direct3D11)
app = QGuiApplication([])
backend = HudBackend()
engine = QQmlApplicationEngine()
engine.rootContext().setContextProperty("backend", backend)
engine.rootContext().setContextProperty("sourceMode", "DEMO")
engine.load(QUrl.fromLocalFile(str(BASE / "Hud.qml")))
assert engine.rootObjects(), "HUD did not load"
window = engine.rootObjects()[0]
frames = BASE / "motion_frames"
frames.mkdir(exist_ok=True)
QTest.qWait(300)

for index in range(24):
    angle = 90 * math.sin(2 * math.pi * index / 24)
    backend.updateValues(80, angle)
    QTest.qWait(130)
    assert window.grabWindow().save(str(frames / f"{index:03}.png"))

views = []
for angle in (-90, 0, 90):
    backend.updateValues(0, angle)
    QTest.qWait(650)
    encoded = QByteArray()
    buffer = QBuffer(encoded)
    buffer.open(QIODevice.WriteOnly)
    assert window.grabWindow().save(buffer, "PNG")
    views.append(Image.open(BytesIO(bytes(encoded))).convert("RGB").crop((552, 125, 1248, 631)))
angle_preview = Image.new("RGB", (696 * 3, 506))
font = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 24)
for index, view in enumerate(views):
    angle_preview.paste(view, (696 * index, 0))
    ImageDraw.Draw(angle_preview).text((696 * index + 570, 17), f"{(-90, 0, 90)[index]:+d}°", font=font, fill="#d9f4ff")
angle_preview.save(BASE / "angle_preview.png")

window.close()
print(f"Rendered 24 motion frames and 3 steering views in {BASE}")
