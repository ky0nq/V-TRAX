import QtQuick
import QtQuick3D

Item {
    id: road
    property real angle: 0
    property real pressure: 0
    property bool reverse: false
    property real accelerationPerSecond: 2.8
    property real maxCyclesPerSecond: 50
    property bool live: true
    property bool moving: true
    readonly property real targetSpeed: live ? Math.max(0, Math.min(100, pressure)) / 100 : 0
    readonly property real speedRatio: targetSpeed
    readonly property real visualSpeed: live && moving ? speedRatio : 0
    property real travel: 0
    property real phase: 0
    property real worldX: 0
    property real worldZ: 0
    property real heading: 0
    readonly property real lateral: worldX
    readonly property int lap: 1
    readonly property bool offTrack: false
    readonly property real steering: live ? Math.max(-1, Math.min(1, angle / 90)) : 0
    property real visualSteering: steering
    Behavior on visualSteering { NumberAnimation { duration: 190; easing.type: Easing.OutCubic } }


    Timer {
        interval: 16
        repeat: true
        running: road.live && road.moving && (road.targetSpeed > 0 || road.speedRatio > 0)
        property double lastTick: 0
        onRunningChanged: lastTick = Date.now()
        onTriggered: {
            var now = Date.now()
            var dt = Math.max(0, Math.min(.1, (now - lastTick) / 1000))
            lastTick = now
            var distance = road.speedRatio * road.maxCyclesPerSecond * dt * (road.reverse ? -1 : 1)
            road.heading = road.visualSteering * 7
            road.worldX = Math.max(-80, Math.min(80, road.worldX + Math.sin(road.heading * Math.PI / 180) * distance))
            road.worldZ -= Math.cos(road.heading * Math.PI / 180) * distance
            road.travel += distance
            road.phase = road.travel % 1
        }
    }

    Rectangle { anchors.fill: parent; color: "#23497d" }

    // Keep only the panorama's sky and mountains; its painted ground stays hidden.
    Item {
        width: road.width
        height: road.height * .60
        clip: true
        Image {
            id: panorama
            objectName: "nightPanorama"
            source: "assets/night_panorama_180.png"
            height: road.height
            width: height * sourceSize.width / Math.max(1, sourceSize.height)
            x: -(width - road.width) * (road.visualSteering + 1) / 2
            y: 0
            smooth: true
            mipmap: true
        }
    }

    // One continuous 3D plain covers the foreground up to its natural horizon.
    View3D {
        id: groundScene
        objectName: "landscape3D"
        anchors.fill: parent
        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Transparent
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            fog: Fog {
                enabled: true
                color: "#23497d"
                depthEnabled: true
                depthNear: 90
                depthFar: 360
            }
        }

        PrincipledMaterial {
            id: plain
            lighting: PrincipledMaterial.NoLighting
            baseColor: "#b0c4e3"
            baseColorMap: Texture {
                source: "assets/night_plain.png"
                tilingModeHorizontal: Texture.Repeat
                tilingModeVertical: Texture.Repeat
                scaleU: 12
                scaleV: 8
                generateMipmaps: true
                mipFilter: Texture.Linear
            }
        }
        PrincipledMaterial {
            id: shadow
            lighting: PrincipledMaterial.NoLighting
            alphaMode: PrincipledMaterial.Blend
            depthDrawMode: Material.NeverDepthDraw
            baseColorMap: Texture { source: "assets/shadow.svg" }
        }
        Repeater3D {
            model: 8
            Model {
                required property int index
                source: "#Rectangle"
                x: 0
                y: 0
                z: (Math.floor(road.worldZ / 100) + 1 - index) * 100 - 50
                eulerRotation.x: -90
                scale: Qt.vector3d(5, 1, 1)
                materials: plain
            }
        }
        Model {
            source: "#Rectangle"
            position: Qt.vector3d(carPose.x, .04, carPose.z)
            eulerRotation.x: -90
            scale: Qt.vector3d(.046, .032, 1)
            materials: shadow
        }
        Node {
            id: carPose
            objectName: "carPose"
            property real steeringYaw: -road.visualSteering * 10
            x: road.worldX + road.visualSteering * .4
            y: 1.22
            z: road.worldZ - 8
            eulerRotation: Qt.vector3d(-5, steeringYaw, 0)
        }
        PerspectiveCamera {
            id: camera
            objectName: "driverCamera"
            position: Qt.vector3d(road.worldX, 2.8, road.worldZ)
            eulerRotation.x: 8
            eulerRotation.y: -road.heading
            clipNear: .15
            clipFar: 700
            fieldOfView: 68
        }
        camera: camera
    }

    Image {
        source: "assets/shadow.svg"
        width: carImage.width * .95
        height: carImage.height * .34
        x: carImage.x + (carImage.width - width) / 2
        y: carImage.y + carImage.height * .80
        opacity: .50
        smooth: true
    }

    Item {
        id: carImage
        objectName: "carImage"
        width: 212
        height: width * 2 / 3
        x: (road.width - width) / 2 + visualSteer * 18
        y: Math.min(road.height * .65, road.height - height - 14)
        property real visualSteer: road.visualSteering
        property int poseIndex: 0
        onVisualSteerChanged: {
            var amount = Math.abs(visualSteer)
            if (poseIndex === 0 && amount > .32)
                poseIndex = 1
            else if (poseIndex === 1 && amount > .72)
                poseIndex = 2
            else if (poseIndex === 1 && amount < .22)
                poseIndex = 0
            else if (poseIndex === 2 && amount < .60)
                poseIndex = 1
        }
        Image {
            anchors.fill: parent
            source: "assets/race_gt_rear.png"
            visible: carImage.poseIndex === 0
            smooth: true
            mipmap: true
        }
        Image {
            anchors.fill: parent
            source: "assets/race_gt_midturn.png"
            mirror: carImage.visualSteer < 0
            visible: carImage.poseIndex === 1
            smooth: true
            mipmap: true
        }
        Image {
            anchors.fill: parent
            source: "assets/race_gt_turn.png"
            mirror: carImage.visualSteer < 0
            visible: carImage.poseIndex === 2
            smooth: true
            mipmap: true
        }
    }

}
