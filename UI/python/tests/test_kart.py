from pathlib import Path
base=Path(__file__).resolve().parents[1]
source=(base/'tests/test_ui.py').read_text()
source=source[:source.index('for angle in')]
exec(compile(source,str(base/'tests/test_ui.py'),'exec'))
w.setProperty('manualMode',True)
b.updateValues(70,0);QTest.qWait(700)
assert w.findChild(QObject,'landscape3D') is not None
road.setProperty('heading',0)
x0=road.property('worldX');z0=road.property('worldZ')
QTest.qWait(300)
assert road.property('worldZ')<z0 and abs(road.property('worldX')-x0)<.001
b.updateValues(70,60);QTest.qWait(500)
assert road.property('heading')>0 and road.property('worldX')>x0
car=w.findChild(QObject,'carPose')
assert car is not None and car.property('steeringYaw')<0
assert car.property('position').z()<road.property('worldZ')
w.setProperty('paused',True);QTest.qWait(60)
state=tuple(road.property(k) for k in ('worldX','worldZ','heading','travel'))
QTest.qWait(200)
assert state==tuple(road.property(k) for k in ('worldX','worldZ','heading','travel'))
w.setProperty('paused',False);b.updateValues(0,0);QTest.qWait(600)
state=tuple(road.property(k) for k in ('worldX','worldZ','heading','travel'))
QTest.qWait(200)
assert state==tuple(road.property(k) for k in ('worldX','worldZ','heading','travel'))
assert not warnings,warnings
b.updateValues(70,0)
frames=[]
for i in range(24):
 QTest.qWait(80)
 path=base/'motion_frames'/f'{i:03}.png';path.parent.mkdir(exist_ok=True)
 assert w.grabWindow().save(str(path))
assert w.grabWindow().save(str(base/'preview.png'))
print('PASS: 3D scene, forward movement, steering path, pause freezes pose, pressure zero stops without rewind')
w.close()
