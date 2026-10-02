# Night panorama driving HUD - UDP version

`motion_hud_night_manual`의 야간 파노라마/차량/주행 UI를 유지하면서 `UI.zip`의 UDP 수신 구조를 이식한 버전이다.

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

`DrivingScene.qml`, 차량 이미지, 야간 파노라마 및 주행 애니메이션 구조는 기존 manual 버전을 유지한다.
