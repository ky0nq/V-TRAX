import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
Item {
 id: road
 property real angle: 0
 property real pressure: 0
 property real accelerationPerSecond: 2.8
 property real maxCyclesPerSecond: 14
 property bool live: true
 property bool moving: true
 readonly property real targetSpeed: live ? Math.max(0,Math.min(100,pressure))/100 : 0
 property real speedRatio: 0
 readonly property real visualSpeed: live && moving ? speedRatio : 0
 property real travel: 0
 property real phase: 0
 property real lateral: worldX
 readonly property int lap: 1
 readonly property bool offTrack: false
 property real steering: live ? Math.max(-1,Math.min(1,angle/90)) : 0
 property real worldX: 0
 property real worldZ: 0
 property real heading: 0
 readonly property int cellX: Math.floor(worldX/5)
 readonly property int cellZ: Math.floor(worldZ/5)
 function hash(x,z){ var n=Math.sin(x*127.1+z*311.7)*43758.5453;return n-Math.floor(n) }
 onLiveChanged: if(!live) speedRatio=0
 Timer {
  interval: 16; repeat: true
  running: road.live && road.moving && (road.targetSpeed>0 || road.speedRatio>0)
  property double lastTick: 0
  onRunningChanged: lastTick=Date.now()
  onTriggered: {
   var now=Date.now(),dt=Math.max(0,Math.min(.1,(now-lastTick)/1000));lastTick=now
   var delta=road.targetSpeed-road.speedRatio,step=road.accelerationPerSecond*dt
   road.speedRatio=Math.abs(delta)<=step?road.targetSpeed:road.speedRatio+(delta>0?step:-step)
   var distance=road.speedRatio*road.maxCyclesPerSecond*dt
   road.heading=road.steering*7
   var yaw=road.heading*Math.PI/180
   road.worldX=Math.max(-2.2,Math.min(2.2,road.worldX+Math.sin(yaw)*distance))
   road.worldZ-=Math.cos(yaw)*distance
   road.travel+=distance;road.phase=road.travel%1
  }
 }
 // Supplied artwork is used only for the distant sky/mountains.
 // All nearby objects below it have persistent world positions.
 Image {
  source: 'assets/speed_idle.png'
  sourceClipRect: Qt.rect(0,0,1448,430)
  width: parent.width*1.65;height: parent.height*.53
  x: (parent.width-width)/2-Math.sin(road.heading*Math.PI/180)*parent.width*.3
 }
 View3D {
  id: view;objectName: 'landscape3D';anchors.fill: parent
  environment: SceneEnvironment {
   backgroundMode: SceneEnvironment.Transparent
   antialiasingMode: SceneEnvironment.MSAA;antialiasingQuality: SceneEnvironment.Medium
   fog: Fog { enabled: true; color: '#d9a36b'; depthEnabled: true;depthNear: 18;depthFar: 55 }
  }
  DirectionalLight { eulerRotation: Qt.vector3d(-18,-15,0);color: '#ffe1ad';ambientColor: '#948778';brightness: 1.1 }
  PrincipledMaterial {
   id: sand;roughness: 1
   baseColorMap: Texture { source: 'assets/sand.svg';scaleU: 2000;scaleV: 2000;tilingModeHorizontal: Texture.Repeat;tilingModeVertical: Texture.Repeat;generateMipmaps: true;mipFilter: Texture.Linear }
  }
  Model {
   source: '#Cube';position: Qt.vector3d(0,-.3,0);scale: Qt.vector3d(200,.006,200);materials: sand
  }

  PrincipledMaterial {
   id: duneSand;roughness: 1;cullMode: Material.NoCulling
   baseColorMap: Texture { source: 'assets/sand.svg';tilingModeHorizontal: Texture.Repeat;tilingModeVertical: Texture.Repeat;generateMipmaps: true;mipFilter: Texture.Linear }
  }
  Repeater3D {
   model: 3
   Model {
    required property int index
    property int tile: index%3-1+Math.floor(road.worldZ/160)
    z: tile*160
    geometry: DuneGeometry {}
    materials: duneSand
   }
  }

  Node {
   id: carPose; objectName: 'carPose'
   property real steeringYaw: road.moving ? -road.steering*15 : 0
   Behavior on steeringYaw { NumberAnimation { duration: 140 } }
   x: road.worldX+Math.sin(road.heading*Math.PI/180)*8
   z: road.worldZ-Math.cos(road.heading*Math.PI/180)*8
   y: .05+Math.sin(road.travel*2)*road.visualSpeed*.02
   eulerRotation: Qt.vector3d(0,-road.heading+steeringYaw,road.steering*1.5)
   DesertCar { steer: -road.steering; distance: road.travel }
  }
  PerspectiveCamera {
   id: camera;objectName: 'driverCamera'
   position: Qt.vector3d(road.worldX,3.6+Math.sin(road.travel*1.7)*road.visualSpeed*.018,road.worldZ)
   eulerRotation: Qt.vector3d(-5,-road.heading,0)
   clipNear: .15;clipFar: 180;fieldOfView: 65+road.visualSpeed*4
  }
  camera: camera
 }
 Rectangle {
  x: 14;y: 14;width: 210;height: 42;radius: 7;color: '#b320232a'
  Text { anchors.centerIn: parent;text: 'DESERT DRIVE  /  '+Math.floor(road.travel)+' units';color: '#ffe1ad';font { pixelSize: 15;letterSpacing: 1 } }
 }
 Rectangle {
  anchors.fill: parent;visible: !road.live;color: '#99030a10'
  Text { anchors.centerIn: parent;text: 'NO SIGNAL';color: '#d7e9f5';font.pixelSize: 24 }
 }
}
