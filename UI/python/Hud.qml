import QtQuick
import QtQuick.Window
import QtQuick.Controls
import QtMultimedia

Window {
    id: root
    width: 1800; height: 840; minimumWidth: 1080; minimumHeight: 504
    visible: true; color: '#020509'; title: 'Vision Drive'
    property bool paused: false
    property bool manualMode: true
    property bool live: sourceMode==='DEMO' || backend.connected
    property bool captureCrop: false
    property bool controls: false
    property bool cameraExpanded: false
    property string expandedCamera: 'zybo'
    property int sessionSeconds: 0
    property color accent: '#69d4ff'
    function adjust(p,a) { if(sourceMode==='DEMO'){manualMode=true;backend.updateValues(p,a)} }
    Timer { interval: 1000; repeat: true; running: root.live && !root.paused; onTriggered: root.sessionSeconds++ }
    Shortcut { sequence: 'Space'; enabled: sourceMode==='DEMO'; onActivated: root.paused=!root.paused }
    Shortcut { sequence: 'Left'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure,backend.angle-5) }
    Shortcut { sequence: 'Right'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure,backend.angle+5) }
    Shortcut { sequence: 'Up'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure+5,backend.angle) }
    Shortcut { sequence: 'Down'; enabled: sourceMode==='DEMO'; onActivated: root.adjust(backend.pressure-5,backend.angle) }
    Shortcut { sequence: 'I'; enabled: sourceMode==='DEMO'; onActivated: backend.toggleDemoDrive() }
    Shortcut { sequence: 'B'; enabled: sourceMode==='DEMO'; onActivated: backend.setDemoBrake(backend.brakePercent>0?0:50) }
    Shortcut { sequence: 'R'; enabled: sourceMode==='DEMO'; onActivated: backend.setDemoReverse(!backend.reverse) }
    Shortcut { sequence: 'Escape'; onActivated: {root.controls=false;root.cameraExpanded=false} }
    Item {
        id: stage
        width: 1800; height: 840; anchors.centerIn: parent
        scale: Math.min(root.width/width,root.height/height)
        Rectangle { x: 45; y: 29; width: 4; height: 30; color: root.accent }
        Text { x: 64; y: 23; text: 'simulator'; color: '#f1f7ff'; font { family: 'Rajdhani'; pixelSize: 28; bold: true; letterSpacing: 1 } }
        Text { x: 1280; y: 33; text: sourceMode==='DEMO'?(root.paused?'●  DEMO PAUSED':root.manualMode?'●  MANUAL':'●  AUTO DEMO'):(sourceMode==='CAPTURE'?backend.cameraConnected:backend.connected)?'●  CONNECTED':'○  CONNECTING'; color: (sourceMode==='CAPTURE'?backend.cameraConnected:root.live)?root.accent:'#ffa27b'; font { pixelSize: 14; letterSpacing: 2 } }
        Text { x: 1473; y: 33; text: 'Driving time  '+Math.floor(root.sessionSeconds/60).toString().padStart(2,'0')+':'+(root.sessionSeconds%60).toString().padStart(2,'0'); color: '#a8bed2'; font { pixelSize: 14; letterSpacing: 1 } }
        Button {
            objectName: 'settingsButton'; x: 1705; y: 24; width: 50; height: 38
            onClicked: root.controls=!root.controls
            background: Rectangle { color: parent.hovered?'#182e40':'transparent'; radius: 5; border.color: '#304758' }
            contentItem: Text { text: '≡'; color: '#c9eafa'; font.pixelSize: 27; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            ToolTip.visible: hovered; ToolTip.text: 'Settings / test input'
        }
        Rectangle { x: 45; y: 89; width: 1710; height: 1; color: '#284354' }
        SportGauge { x: 22; y: 144; scale: 1.1; transformOrigin: Item.TopLeft; value: backend.motionSpeed; caption: 'DRIVE OUTPUT'; unit: '%'; live: root.live }
        SportGauge { x: 1228; y: 144; scale: 1.1; transformOrigin: Item.TopLeft; value: backend.angle; minimum: -90; maximum: 90; signedValue: true; caption: sourceMode!=='DEMO' && !backend.cnnFresh ? 'STEERING / STALE' : 'STEERING ANGLE'; unit: 'DEGREES'; live: root.live }
        Rectangle {
            x: 551; y: 106; width: 698; height: 350; radius: 8; color: '#040a12'; border.color: '#345269'; clip: true
            DrivingScene { id: drive; objectName: 'drivingScene'; x: 2; y: 2; width: parent.width-4; height: parent.height-4; clip: true; angle: backend.angle; pressure: backend.motionSpeed; reverse: backend.reverse; live: root.live; moving: !root.paused && !backend.emergencyStop }
        }
        Rectangle {
            x: 551; y: 468; width: 698; height: 202; radius: 8; color: '#060d15'; border.color: '#182d3c'
            Rectangle {x:260; y:28; width:1; height:146; color:'#1d303d'}
            Rectangle {x:494; y:28; width:1; height:146; color:'#1d303d'}
        }
        Rectangle {
            objectName: 'espCameraDock'; visible: backend.espCameraEnabled
            x: 563; y: 480; width: 178*4/3; height: 178; radius: 3
            color: '#06111b'; clip: true
            VideoOutput { objectName: 'mainEspCamera'; anchors.fill: parent; fillMode: VideoOutput.PreserveAspectFit }
            MouseArea { objectName: 'espCameraButton'; anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: {root.expandedCamera='esp32';root.cameraExpanded=true} }
        }
        Rectangle {
            objectName: 'steeringCameraDock'; x: 1059; y: 480; width: 178; height: 178; radius: 3
            color: '#06111b'; clip: true
            VideoOutput { objectName: 'mainCaptureCamera'; x:root.captureCrop?-440*parent.width/256:0; y:root.captureCrop?-142*parent.height/256:0; width:root.captureCrop?1280*parent.width/256:parent.width; height:root.captureCrop?720*parent.height/256:parent.height; visible: typeof captureMode !== 'undefined' && captureMode; fillMode: VideoOutput.PreserveAspectFit }
            Image { visible: !(typeof captureMode !== 'undefined' && captureMode); anchors.fill: parent; source: sourceMode==='DEMO' ? 'assets/camera_sample.png' : backend.cameraUrl; cache: false; fillMode: Image.PreserveAspectFit }
            MouseArea { objectName: 'cameraButton'; anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: {root.expandedCamera='zybo';root.cameraExpanded=true} }
        }
        Item {
            objectName: 'directionDock'; x: 825; y: 509; width: 210; height: 120
            readonly property int direction: Math.abs(backend.angle)<2 ? 0 : backend.angle<0 ? -1 : 1
            
            
            Canvas {
                x: 85; y: 27; width: 40; height: 48; opacity: root.live ? 1 : 0.35
                rotation: parent.direction * 90
                Behavior on rotation { NumberAnimation {duration: 140; easing.type: Easing.OutCubic} }
                onPaint: {
                    var c=getContext('2d'); c.clearRect(0,0,width,height);
                    c.strokeStyle='#edf7ff'; c.lineWidth=4; c.lineCap='round'; c.lineJoin='miter';
                    c.beginPath(); c.moveTo(20,43); c.lineTo(20,7); c.stroke();
                    c.beginPath(); c.moveTo(8,19); c.lineTo(20,7); c.lineTo(32,19); c.stroke();
                }
            }
            Text { anchors.horizontalCenter: parent.horizontalCenter; y: 86;
                text: !root.live ? 'WAITING' : parent.direction===0 ? 'STRAIGHT' : parent.direction<0 ? 'TURN LEFT' : 'TURN RIGHT'
                color: root.live ? '#69d4ff' : '#687c90'; font {family:'Rajdhani'; pixelSize:14; bold:true; letterSpacing:2} }
        }
        Rectangle { x: 45; y: 732; width: 1710; height: 1; color: '#284354' }
        Row {
            x:551; y:682; spacing:24
            Repeater {
                model:[{label:'ACCEL',value:backend.accelLevel*20,tint:'#69d4ff'},{label:'BRAKE',value:backend.brakeLevel*20,tint:'#ffb56b'}]
                delegate: Item {
                    required property var modelData
                    width:337; height:38
                    Text {text:modelData.label; color:'#96afc4'; font {pixelSize:12; bold:true; letterSpacing:2}}
                    Text {anchors.right:parent.right; text:backend.commandKnown ? (modelData.value/20).toFixed(0)+' / 5' : '— / 5'; color:modelData.tint; font {pixelSize:13; bold:true}}
                    Rectangle {y:23; width:337; height:5; radius:2.5; color:'#152432'
                        Rectangle {width:parent.width*modelData.value/100; height:5; radius:2.5; color:modelData.tint; Behavior on width {NumberAnimation {duration:100}}}
                    }
                }
            }
        }
        Row {
            x:551; y:744; spacing:12
            Repeater {
                model:[
                    {key:'DRIVE PERMISSION', note:'', label:!root.live || !backend.driveKnown?'WAITING':backend.driveEnabled?'READY':'LOCKED', mark:'●', known:root.live && backend.driveKnown, active:backend.driveEnabled, tint:backend.driveEnabled?'#68e4cd':'#ff898c'},
                    {key:'DIRECTION', note:'', label:!root.live || !backend.reverseKnown?'WAITING':backend.reverse?'REVERSE':'FORWARD', mark:backend.reverse?'R':'D', known:root.live && backend.reverseKnown, active:true, tint:backend.reverse?'#b7a2ff':'#69d4ff'},
                    {key:'BRAKE', note:'', label:'BRAKE', mark:'Ⅱ', known:root.live && backend.brakeKnown, active:backend.brakeActive, tint:'#ffc078'}
                ]
                delegate: Rectangle {
                    required property var modelData
                    objectName:'state_'+modelData.key
                    property color tint:modelData.known?modelData.tint:'#687c90'
                    property bool lit:modelData.known && (modelData.active || modelData.label==='LOCKED')
                    width:225; height:72; radius:7
                    color:lit?Qt.rgba(tint.r,tint.g,tint.b,0.10):'#080f17'
                    border.color:lit?tint:'#233648'; border.width:lit?1.5:1
                    Rectangle {x:12; y:19; width:38; height:38; radius:9; color:lit?tint:'#162332'
                        Text {anchors.centerIn:parent; text:parent.parent.modelData.known?modelData.mark:'—'; color:parent.parent.lit?'#061017':'#7990a7'; font {family:'Rajdhani'; pixelSize:25; bold:true}}
                    }
                    Text {x:61; y:8; text:modelData.title || ''; color:'#7e9bb3'; font {pixelSize:10; bold:true; letterSpacing:1}}
                    Text {x:61; y:23; text:modelData.label; color:lit?tint:'#8296aa'; font {family:'Rajdhani'; pixelSize:modelData.label==='SENSOR OFFLINE'?17:23; bold:true; letterSpacing:1}}
                    Text {x:61; y:53; text:modelData.note; color:'#7690a7'; font {pixelSize:9; letterSpacing:1}}
                }
            }
        }
        Rectangle {
            visible: root.cameraExpanded; anchors.fill: parent; color: '#df010409'; z: 20
            MouseArea { anchors.fill: parent; onClicked: root.cameraExpanded=false }
            Rectangle { anchors.centerIn: parent; width: 536; height: 592; radius: 8; color: '#0a141f'; border.color: '#57778f'
                VideoOutput { objectName: 'expandedEspCamera'; x: 12; y: 12; width: 512; height: 512; visible: root.expandedCamera==='esp32'; fillMode: VideoOutput.PreserveAspectFit }
                VideoOutput { objectName: 'expandedCaptureCamera'; x: 12; y: 12; width: 512; height: 512; visible: root.expandedCamera!=='esp32' && typeof captureMode !== 'undefined' && captureMode; fillMode: VideoOutput.PreserveAspectFit }
                Image { x: 12; y: 12; width: 512; height: 512; visible: root.expandedCamera!=='esp32' && !(typeof captureMode !== 'undefined' && captureMode); source: !root.cameraExpanded || root.expandedCamera==='esp32' ? '' : sourceMode==='DEMO' ? 'assets/camera_sample.png' : backend.cameraUrl; cache: false; fillMode: Image.PreserveAspectFit }
                Text { anchors.right: parent.right; anchors.rightMargin: 20; y: 547; text: 'CLICK TO CLOSE'; color: '#7996ad'; font.pixelSize: 13 }
            }
        }
        Rectangle {
            visible: root.controls; anchors.fill: parent; color: '#bb010409'; z: 30
            MouseArea { anchors.fill: parent; onClicked: root.controls=false }
            Rectangle {
                x: 1193; y: 104; width: 555; height: 575; radius: 10; color: '#0a141f'; border.color: '#43677f'
                MouseArea { anchors.fill: parent }
                Text { x: 28; y: 25; text: 'SESSION / INPUT'; color: '#deedfa'; font { pixelSize: 25; bold: true; letterSpacing: 2 } }
                Text { x: 28; y: 76; text: sourceMode==='DEMO'?'I ignition · R reverse · B brake · arrows: pedals / steering':'HDMI capture · COM4 telemetry'; color: '#8aaac1'; font.pixelSize: 15 }
                Text { x: 28; y: 120; text: 'PRESSURE  '+backend.pressure.toFixed(0)+' %'; color: '#b5d6eb'; font { pixelSize: 19; letterSpacing: 1 } }
                Slider { objectName: 'pressureSlider'; x: 22; y: 149; width: 510; enabled: sourceMode==='DEMO'; from: 0; to: 100; value: backend.pressure; onMoved: root.adjust(value,backend.angle) }
                Text { x: 28; y: 215; text: 'STEERING  '+backend.angle.toFixed(1)+'°'; color: '#b5d6eb'; font { pixelSize: 19; letterSpacing: 1 } }
                Slider { objectName: 'angleSlider'; x: 22; y: 248; width: 510; enabled: sourceMode==='DEMO'; from: -90; to: 90; value: backend.angle; onMoved: root.adjust(backend.pressure,value) }
                Text { x: 28; y: 298; text: '−90°                               0°                               +90°'; color: '#7697ad'; font.pixelSize: 16 }
                Button { objectName: 'pauseButton'; x: 28; y: 358; width: 158; height: 42; text: sourceMode==='DEMO'?(root.paused?'RESUME':'PAUSE'):(backend.canSendBoardCommand?'BOARD DEMO (T)':'T: BOARD TERMINAL'); visible: sourceMode==='DEMO'; enabled: sourceMode==='DEMO'; onClicked: {if(sourceMode==='DEMO') root.paused=!root.paused; else backend.requestBoardDemo()} }
                Button { objectName: 'autoDemoButton'; x: 199; y: 358; width: 158; height: 42; text: sourceMode==='DEMO'?(root.manualMode?'AUTO DEMO':'AUTO ACTIVE'):(root.captureCrop?'HDMI CROP':'HDMI FULL'); onClicked: { if(sourceMode==='DEMO') {root.manualMode=false;root.paused=false} else root.captureCrop=!root.captureCrop } }
                Button { objectName: 'closeSettings'; x: 370; y: 358; width: 157; height: 42; text: 'CLOSE'; onClicked: root.controls=false }
                Text {x:28; y:425; text:'BRAKE  '+backend.brakePercent.toFixed(0)+' %'; color:'#ffc078'; font.pixelSize:17}
                Slider {x:22; y:451; width:510; enabled:sourceMode==='DEMO'; from:0; to:100; value:backend.brakePercent; onMoved:backend.setDemoBrake(value)}
                Text { x: 28; y: 525; text: sourceMode==='SERIAL' ? 'FSR RAW  ACC '+backend.accelRaw+' / BRAKE '+backend.brakeRaw+'     LEVEL '+backend.accelLevel+' / '+backend.brakeLevel : 'Open plain / pressure: speed / angle: 180° panorama.'; color: '#728da2'; font.pixelSize: 14 }
            }
        }

    }
}

