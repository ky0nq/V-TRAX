## USB HDMI capture mode (2026-10-01)

The default `run_car_ui.cmd` now receives Zybo HDMI video through a USB capture device. Sensor values and vehicle control continue through UART. ESP32-CAM remains a separate Wi-Fi stream.

- List devices: `.venv\Scripts\python.exe main.py --list-cameras`
- Run: `run_car_ui.cmd COM4 auto "USB Video"` (replace COM4 and device name)
- Without ESP32-CAM: `run_car_ui.cmd COM4 OFF "USB Video"`
- Video-only check: `run_capture_test.cmd "USB Video"`
- If no device is specified, auto selects one uniquely named capture/HDMI/USB Video device; otherwise it waits. Numeric device indices are also accepted.
- Close other applications using the capture device before testing.
- The board must output the loading screen on HDMI for it to appear here.

Legacy UDP code remains available through `run_car_ui_udp.cmd`; `run_board.cmd` also remains a legacy UDP launcher. No Vitis firmware was changed. A backup of changed original files is in `D:\SoC\vision_drive_before_usb_capture_20261001`.

Actual HDMI capture could not be checked during implementation: only `LG AIO MNT` was enumerated. UI startup without a capture device and synthetic frame forwarding were tested.

# Vision Drive - 야간 주행 장면 + 소형 ESP32-CAM

이 버전은 `motion_hud_night_car_serial`의 차량 제어와 FSR 수신을 유지하면서 가운데 야간 3D 풍경과 가상 자동차를 다시 표시합니다. ESP32-CAM 실시간 영상은 가운데 화면 왼쪽 위의 작은 FRONT CAM 창에, Zybo UDP 카메라는 오른쪽 아래에 표시합니다. 두 영상은 동시에 수신됩니다. 이전 가운데 영상 전용 UI는 `D:\SoC\vision_drive_before_scene_restore_20261001`에 백업했습니다.

## 차량과 UI를 함께 실행

처음 한 번 `setup.cmd`를 다시 실행하여 `zeroconf`를 설치한 뒤 `run_car_ui.cmd COM4`를 실행하세요. `COM4`는 실제 **Zybo UART 포트**로 바꿉니다. UI가 Zybo에 20ms마다 `K,<각도>,<비상정지>`를 보내므로 별도의 `car_control_fsr.py`는 실행하지 않습니다. PC와 보드의 기존 Ethernet 설정(`192.168.10.1` / `192.168.10.2`)은 UDP 카메라용으로 계속 사용합니다.

ESP32-CAM 영상도 함께 보려면 PC와 ESP32-CAM을 같은 Wi-Fi에 연결하고 [ESP32_CAM_MDNS_PATCH.md](ESP32_CAM_MDNS_PATCH.md)의 세 부분을 Arduino CameraWebServer 스케치에 추가하여 다시 업로드하세요. 이후 `run_car_ui.cmd COM4`는 카메라 IP를 자동 탐색하고, 영상 연결이 끊기면 다시 탐색합니다. 작은 FRONT CAM과 오른쪽 아래 Zybo UDP 카메라는 각각 연결 상태를 표시하며, 어느 쪽이 끊겨도 다른 쪽 수신은 계속됩니다. 카메라만 먼저 시험하려면 `run_camera_test.cmd`를 실행합니다. mDNS가 차단된 네트워크라면 기존처럼 `run_car_ui.cmd COM4 http://카메라IP:81/stream` 또는 `run_camera_test.cmd http://카메라IP:81/stream`을 사용하세요. ESP32-CAM을 쓰지 않고 계기판만 시험하려면 `run_car_ui.cmd COM4 OFF`입니다.

좌·우 방향키를 누르면 최대 ±90°까지 조향합니다. Space는 비상정지, R은 해제, Q는 종료입니다. UI는 Zybo 시리얼 로그의 `ACC_RAW`를 기본 7000~22000 범위에서 0~100%로 환산하고, 브레이크·비상정지·센서 단절 시 화면의 주행을 멈춥니다. 실제 모터의 0~5 LEVEL 제어와 안전 판정은 Zybo 펌웨어가 담당합니다. 보정이 필요하면 `main.py --serial-port COM4 --udp --board-ip 192.168.10.2 --fsr-idle 7000 --fsr-full 22000`에서 범위를 조정하세요.

계기판과 센서 제어는 이전 통합본을 유지합니다. 가운데 3D 주행 장면은 압력과 조향 각도에 따라 움직입니다.

## 영상이 끊겨 보일 때

`run_car_ui.cmd COM4`의 콘솔에는 2초마다 ESP32-CAM의 `size`, `receive_fps`, `display_fps`, `ui_tick_fps`, `display_skipped`가 출력됩니다. `receive_fps`부터 낮다면 PC에 도착하는 카메라 영상 자체가 느린 것이므로 [ESP32_CAM_FPS_FIX.md](ESP32_CAM_FPS_FIX.md)의 설정을 적용하세요. 손을 영상에 넣을 때만 전송이 흔들리면 별도 썸네일 우선 스케치 `D:\SoC\ESP32_CAM_SMOOTH_THUMBNAIL`을 시험할 수 있습니다. `display_fps`만 낮다면 PC UI 갱신이 병목입니다. 이 버전은 JPEG 해석을 UI 스레드 밖에서 수행하고, 프레임 도착 신호로 Qt VideoOutput에 최신 영상만 전달합니다.

기존 스케치가 `FRAMESIZE_VGA`, `CAMERA_FB_IN_DRAM`, `fb_count = 1`이면 ESP32-CAM 측 프레임 생산 속도가 먼저 제한될 수 있습니다. 우선 `FRAMESIZE_QVGA`로 바꿔 차이를 확인하세요. 보드에서 PSRAM이 실제로 감지될 때에만 PSRAM 버퍼와 복수 프레임 버퍼 설정을 검토하세요. UI 측 수정만으로 카메라가 보내지 않는 프레임을 만들어낼 수는 없습니다.

## 입력 구조

| 구분 | 포트 | 형식 | 용도 |
|---|---:|---|---|
| Telemetry | UDP 7000 | JSON | pressure, angle 수신 |
| Camera | UDP 7001 | HUDV v1 / RGB565 또는 RGB888 | 실시간 카메라 프레임 수신 |

Telemetry JSON 예시:

```json
{"pressure":70,"angle":-25,"cnn_valid":true,"pressure_source":"sensor"}
```

필수 필드는 `pressure`, `angle`이다.

- pressure: 0~100 범위로 clamp
- angle: -90~90 범위로 clamp
- 수신 timeout: 1.2초
- timeout 발생 시 UI를 `NO SIGNAL` 상태로 전환
- UDP 모드에서는 수동 슬라이더와 방향키 입력 비활성화

## 보드 연결

기존 `UI.zip` 기준 네트워크 구성을 그대로 사용한다.

- PC Ethernet: `192.168.10.1 / 255.255.255.0`
- Zybo: `192.168.10.2`
- Telemetry: UDP 7000
- Camera: UDP 7001

Windows 방화벽에서 UDP 7000, 7001 수신을 허용해야 한다.

## 실행

### 1. 최초 설정

`setup.cmd` 실행.

### 2. 보드 UDP 모드

`run_board.cmd` 실행.

동일한 명령:

```bat
.venv\Scripts\python.exe main.py --udp --host 0.0.0.0 --port 7000 --video-port 7001 --board-ip 192.168.10.2 --stats
```

### 3. 보드 없이 UI 확인

`run_demo.cmd` 실행.

Demo 모드에서는 기존 수동 입력과 AUTO DEMO를 사용할 수 있다.

## UDP 단독 확인

HUD를 종료한 뒤 Telemetry만 확인한다.

```bat
.venv\Scripts\python.exe check_udp.py --port 7000 --seconds 10
```

PC 내부 loopback으로 Telemetry/Camera 송수신을 확인한다.

```bat
.venv\Scripts\python.exe main.py --udp --host 0.0.0.0 --port 7000 --video-port 7001
```

다른 터미널:

```bat
.venv\Scripts\python.exe test_sender.py --host 127.0.0.1
```

## 수정 범위

- `main.py`: UDP Telemetry + Camera 수신 추가
- `Hud.qml`: UDP LIVE / NO SIGNAL 상태 표시 추가
- `Hud.qml`: UDP 모드에서 실시간 Camera Provider 사용
- `Hud.qml`: UDP 모드 수동 입력 차단
- `video_protocol.py`: HUDV 프레임 재조립
- `video_receiver.py`: UDP Camera worker 및 Qt Image Provider
- `run_board.cmd`, `run_demo.cmd`, `setup.cmd`: 실행 스크립트 추가
- `check_udp.py`, `test_sender.py`: UDP 확인 도구 추가

`DrivingScene.qml`과 가상 자동차 자산을 가운데 주행 화면에서 다시 사용합니다.
