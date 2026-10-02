import QtQuick
import QtQuick.Window
import QtQuick.Controls

Window {
    id: root
    width: 1800; height: 800; minimumWidth: 1080; minimumHeight: 480
    visible: true; color: '#020509'; title: 'Vision Drive'
    property bool paused: false
    property bool manualMode: true
    property bool live: sourceMode==='DEMO' || backend.connected
    property bool controls: false
    property bool cameraExpanded: false
    property int sessionSeconds: 0
    property color accent: '#69d4ff'
    function adjust(p,a) { if(sourceMode==='DEMO'){manualMode=true;backend.updateValues(p,a)} }
    Timer { interval: 1000; repeat: true; running: root.live && !root.paused; onTriggered: root.sessionSeconds++ }
    Shortcut { sequence: 'Space'; enabled: sourceMode==='DEMO'; onActivated: root.paused=!root.paused }
    Shortcut { sequence: 'Left'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure,backend.angle-5) }
    Shortcut { sequence: 'Right'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure,backend.angle+5) }
    Shortcut { sequence: 'Up'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure+5,backend.angle) }
    Shortcut { sequence: 'Down'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure-5,backend.angle) }
    Shortcut { sequence: 'Escape'; onActivated: {root.controls=false;root.cameraExpanded=false} }
    Item {
        id: stage
        width: 1800; height: 800; anchors.centerIn: parent
        scale: Math.min(root.width/width,root.height/height)
        Rectangle { x: 45; y: 29; width: 4; height: 30; color: root.accent }
        Text { x: 64; y: 23; text: 'simulator'; color: '#f1f7ff'; font { family: 'Rajdhani'; pixelSize: 28; bold: true; letterSpacing: 1 } }
        Text { x: 1280; y: 33; text: sourceMode==='DEMO'?(root.paused?'●  DEMO PAUSED':root.manualMode?'●  MANUAL':'●  AUTO DEMO'):root.live?'●  UDP LIVE':'○  NO SIGNAL'; color: root.live?root.accent:'#ffa27b'; font { pixelSize: 14; letterSpacing: 2 } }
        Text { x: 1473; y: 33; text: 'Driving time  '+Math.floor(root.sessionSeconds/60).toString().padStart(2,'0')+':'+(root.sessionSeconds%60).toString().padStart(2,'0'); color: '#a8bed2'; font { pixelSize: 14; letterSpacing: 1 } }
        Button {
            objectName: 'settingsButton'; x: 1705; y: 24; width: 50; height: 38
            onClicked: root.controls=!root.controls
            background: Rectangle { color: parent.hovered?'#182e40':'transparent'; radius: 5; border.color: '#304758' }
            contentItem: Text { text: '≡'; color: '#c9eafa'; font.pixelSize: 27; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            ToolTip.visible: hovered; ToolTip.text: 'Settings / test input'
        }
        Rectangle { x: 45; y: 89; width: 1710; height: 1; color: '#284354' }
        SportGauge { x: 22; y: 144; scale: 1.1; transformOrigin: Item.TopLeft; value: backend.pressure; caption: sourceMode==='UDP' && backend.pressureSource!=='sensor' ? 'PRESSURE / '+backend.pressureSource.toUpperCase() : 'PRESSURE SENSOR'; unit: '% / NORMALIZED'; live: root.live }
        SportGauge { x: 1228; y: 144; scale: 1.1; transformOrigin: Item.TopLeft; value: backend.angle; minimum: -90; maximum: 90; signedValue: true; caption: sourceMode==='UDP' && !backend.cnnFresh ? 'STEERING / STALE' : 'STEERING ANGLE'; unit: 'DEGREES'; live: root.live }
        Rectangle {
            x: 551; y: 125; width: 698; height: 506; radius: 12; color: '#040a12'; border.color: '#345269'
            DrivingScene { id: drive; objectName: 'drivingScene'; x: 2; y: 2; width: parent.width-4; height: parent.height-4; clip: true; angle: backend.angle; pressure: backend.pressure; live: root.live; moving: !root.paused }
            Rectangle {
                x: 499; y: 306; width: 178; height: 180; radius: 6; color: '#07111c'; border.color: '#819baa'
                Image { x: 6; y: 6; width: 166; height: 146; source: sourceMode==='DEMO' ? 'assets/camera_sample.png' : backend.cameraUrl; cache: false; fillMode: Image.PreserveAspectFit }
                Text { anchors.horizontalCenter: parent.horizontalCenter; y: 158; text: sourceMode==='DEMO' ? 'CAMERA / SAMPLE  +' : backend.cameraConnected ? 'CAMERA / LIVE  +' : 'CAMERA / NO SIGNAL'; color: '#bad3e2'; font { pixelSize: 11; letterSpacing: 1 } }
                MouseArea { objectName: 'cameraButton'; anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: root.cameraExpanded=true }
            }
        }
        Rectangle { x: 45; y: 704; width: 1710; height: 1; color: '#284354' }
        Rectangle {
            x: 614; y: 736; width: 176; height: 38; radius: 19; color: '#091924'; border.color: root.accent
            Text { anchors.centerIn: parent; text: root.live?backend.status:'NO SIGNAL'; color: root.accent; font { pixelSize: 17; italic: true; bold: true; letterSpacing: 2 } }
        }
        Rectangle { x: 852; y: 732; width: 1; height: 48; color: '#35536b' }
        Canvas {
            x: 914; y: 733; width: 42; height: 42
            property real direction: !root.live?2:Math.abs(backend.angle)<2?0:backend.angle<0?-1:1
            onDirectionChanged: requestPaint()
            onPaint: {
                var c=getContext('2d');c.reset();c.strokeStyle='#69d4ff';c.lineWidth=5;c.lineJoin='round';c.lineCap='round'
                c.beginPath()
                if(direction===2){c.moveTo(10,21);c.lineTo(32,21)}
                else if(direction===0){c.moveTo(21,36);c.lineTo(21,7);c.moveTo(11,17);c.lineTo(21,7);c.lineTo(31,17)}
                else {c.save();if(direction<0){c.translate(42,0);c.scale(-1,1)}c.moveTo(10,36);c.lineTo(10,23);c.quadraticCurveTo(10,13,22,13);c.lineTo(34,13);c.moveTo(26,5);c.lineTo(34,13);c.lineTo(26,21)}
                c.stroke()
            }
        }
        Text { x: 985; y: 743; text: !root.live?'WAITING FOR DATA':Math.abs(backend.angle)<2?'STRAIGHT':backend.angle<0?'TURN LEFT':'TURN RIGHT'; color: '#deecf9'; font { pixelSize: 18; italic: true; bold: true; letterSpacing: 4 } }
        Rectangle {
            visible: root.cameraExpanded; anchors.fill: parent; color: '#df010409'; z: 20
            MouseArea { anchors.fill: parent; onClicked: root.cameraExpanded=false }
            Rectangle { anchors.centerIn: parent; width: 536; height: 592; radius: 8; color: '#0a141f'; border.color: '#57778f'
                Image { x: 12; y: 12; width: 512; height: 512; source: sourceMode==='DEMO' ? 'assets/camera_sample.png' : backend.cameraUrl; cache: false; fillMode: Image.PreserveAspectFit }
                Text { x: 20; y: 545; text: sourceMode==='DEMO' ? 'CAMERA / SAMPLE IMAGE' : backend.cameraConnected ? 'CAMERA / LIVE' : 'CAMERA / NO SIGNAL'; color: '#c0dfef'; font { pixelSize: 17; letterSpacing: 2 } }
                Text { anchors.right: parent.right; anchors.rightMargin: 20; y: 547; text: 'CLICK TO CLOSE'; color: '#7996ad'; font.pixelSize: 13 }
            }
        }
        Rectangle {
            visible: root.controls; anchors.fill: parent; color: '#bb010409'; z: 30
            MouseArea { anchors.fill: parent; onClicked: root.controls=false }
            Rectangle {
                x: 1193; y: 104; width: 555; height: 475; radius: 10; color: '#0a141f'; border.color: '#43677f'
                MouseArea { anchors.fill: parent }
                Text { x: 28; y: 25; text: 'SESSION / INPUT'; color: '#deedfa'; font { pixelSize: 25; bold: true; letterSpacing: 2 } }
                Text { x: 28; y: 76; text: sourceMode==='DEMO'?'Arrow keys: angle / pressure · Space: pause':'UDP input active — manual controls disabled'; color: '#8aaac1'; font.pixelSize: 15 }
                Text { x: 28; y: 120; text: 'PRESSURE  '+backend.pressure.toFixed(0)+' %'; color: '#b5d6eb'; font { pixelSize: 19; letterSpacing: 1 } }
                Slider { objectName: 'pressureSlider'; x: 22; y: 149; width: 510; enabled: sourceMode==='DEMO'; from: 0; to: 100; value: backend.pressure; onMoved: root.adjust(value,backend.angle) }
                Text { x: 28; y: 215; text: 'STEERING  '+backend.angle.toFixed(1)+'°'; color: '#b5d6eb'; font { pixelSize: 19; letterSpacing: 1 } }
                Slider { objectName: 'angleSlider'; x: 22; y: 248; width: 510; enabled: sourceMode==='DEMO'; from: -90; to: 90; value: backend.angle; onMoved: root.adjust(backend.pressure,value) }
                Text { x: 28; y: 298; text: '−90°                               0°                               +90°'; color: '#7697ad'; font.pixelSize: 16 }
                Button { objectName: 'pauseButton'; x: 28; y: 358; width: 158; height: 42; text: root.paused?'RESUME':'PAUSE'; enabled: sourceMode==='DEMO'; onClicked: root.paused=!root.paused }
                Button { objectName: 'autoDemoButton'; x: 199; y: 358; width: 158; height: 42; text: root.manualMode?'AUTO DEMO':'AUTO ACTIVE'; enabled: sourceMode==='DEMO'; onClicked: { root.manualMode=false; root.paused=false } }
                Button { objectName: 'closeSettings'; x: 370; y: 358; width: 157; height: 42; text: 'CLOSE'; onClicked: root.controls=false }
                Text { x: 28; y: 426; text: 'Open plain / pressure: speed / angle: 180° panorama.'; color: '#728da2'; font.pixelSize: 14 }
            }
        }
    }
}


