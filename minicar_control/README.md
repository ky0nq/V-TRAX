# Minicar Control (ICD v0.2)

무선 미니카 제어 파이프라인. FSR 페달과 PC 키보드 조향 입력을 Zybo가 해석해
`CommandPacket`으로 인코딩하고, ESP32 두 대를 거쳐 L298N + TT 모터 4개를 구동한다.

```
Sensor → Zybo → ESP32 #1 (무선 브릿지) → ESP32 #2 (차량 제어) → L298N → TT Motor ×4
        IF-01   IF-02        IF-03           IF-04/05             IF-06
```

계층 분리가 이 설계의 핵심이다. Zybo가 만든 8-byte `CommandPacket`은 UART와
ESP-NOW를 **재가공 없이 그대로 통과**하고, ESP32 #2의 Vehicle Controller는 통신
방식을 전혀 모른 채 `DriverCommand` 구조체만 입력으로 받는다. 따라서 통신 계층을
교체해도 차량 제어 로직은 그대로 유지된다.

전체 아키텍처 다이어그램은 [`docs/architecture.html`](docs/architecture.html) 참고.

## 인터페이스

| ID | 구간 | 사양 |
|---|---|---|
| IF-01 | FSR → Zybo | I2C1 (MIO12/13) · ADS1115 @0x48 · A0=Accel, A1=Brake |
| IF-02 | Zybo → ESP32 #1 | UART 115200 8-N-1 · 100 Hz · Zybo UART0 MIO14/15 → ESP32 GPIO16 |
| IF-03 | ESP32 #1 → #2 | ESP-NOW · 채널 1 고정 · 발신 MAC 필터링 · 100 Hz |
| IF-04 | 내부 | `CommandPacket` → `DriverCommand` |
| IF-05 | ESP32 #2 → L298N | GPIO + PWM 5 kHz / 8 bit |
| IF-06 | L298N → Motor | H-Bridge |

### CommandPacket (8 byte, packed)

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| | `0xAA` | `0x55` | SEQ | STEER | ACCEL | BRAKE | FLAGS | CRC8 |

- `STEER` −90~+90, 10° 단위 19단계
- `ACCEL` / `BRAKE` 0~5 — 속도가 아니라 **요청 강도**. `BRAKE > 0`이면 항상 `ACCEL`보다 우선
- `FLAGS` bit0 = Emergency Stop
- `CRC8` poly `0x07`, init `0x00`, 대상 Byte 0~6

### 안전 동작

| 조건 | 동작 |
|---|---|
| 정상 패킷 300 ms 미수신 | Failsafe → `hardStop()` |
| `FLAGS` bit0 = 1 | Emergency Stop → `hardStop()` (최우선) |
| PC 명령 500 ms 미수신 | Zybo가 E-Stop 플래그를 latch |
| ADS1115 연속 5회 실패 | E-Stop 송신 + I2C 컨트롤러 리셋 |

트리거 조건은 다르지만 Failsafe와 E-Stop 모두 `hardStop()`으로 수렴해 모터를 즉시 0으로 만든다.

## 구성

```
minicar_control/
├─ zybo/
│  ├─ src/main.c          센서 해석 + 패킷 인코딩 (100 Hz 제어 루프)
│  └─ hw/
│     ├─ bd/design_1/     Vivado 블록 디자인 (PS only)
│     └─ xsa/             Vitis용 하드웨어 핸드오프
├─ firmware/
│  ├─ ESP32_1_WirelessBridge/   UART → ESP-NOW 중계 (검증만, 재가공 없음)
│  └─ ESP32_2_VehicleControl/   패킷 복원 + 차량 제어 (50 Hz) + L298N 구동
├─ host/car_control_fsr.py      PC 조향/E-Stop GUI (Tkinter)
└─ docs/architecture.html       ICD v0.2 다이어그램
```

## 빌드 / 실행

### Zybo (Vitis)

`zybo/hw/xsa/design_1_wrapper.xsa`로 플랫폼을 만들고 `zybo/src/main.c`를 애플리케이션에
추가한다. BSP의 `stdout`/`stdin`은 **`ps7_uart_1`** 이어야 한다 — `ps7_uart_0`으로 두면
`xil_printf` 디버그 출력이 ESP32로 가는 바이너리 패킷 스트림을 오염시킨다.

### ESP32 ×2

Arduino IDE (ESP32 core 3.x — `ledcAttach` API 사용). 각 보드에 해당 스케치를 굽고,
서로의 MAC 주소를 상대 스케치 상단 상수에 맞춘다.

| | MAC | 역할 |
|---|---|---|
| ESP32 #1 | `B0:3F:D3:75:17:50` | 브릿지 |
| ESP32 #2 | `B0:3F:D3:64:04:14` | 차량 |

### PC

```bash
pip install pyserial
python host/car_control_fsr.py
```

`SERIAL_PORT`를 Zybo USB-UART의 COM 포트로 맞춘다. 해당 포트를 **다른 시리얼 모니터가
잡고 있으면 안 된다** (Vitis Serial Terminal, Arduino IDE Serial Monitor 등 —
`PermissionError(13)`의 원인).

조작: `←`/`→` 조향, `SPACE` E-Stop, `R` 해제, `Q` 종료. 가감속은 FSR 페달.

## L298N 배선

| | IN1 | IN2 | ENA | IN3 | IN4 | ENB |
|---|---|---|---|---|---|---|
| GPIO | 25 | 26 | 27 | 32 | 14 | 13 |

좌측이 `IN1/IN2/ENA`, 우측이 `IN3/IN4/ENB`. 우측은 장착 방향이 반대라 코드에서 논리를 반전한다.

## 남은 TBD

- FSR 전압 범위와 ADC Resolution
- `main.c`의 Accel/Brake 임계값 — **현재 값은 임시**, 실측 raw 기준으로 보정 필요
- Steering 보정, 최종 모터 전원, 튜닝 파라미터
- `vehicleSpeed`는 0~100 가상 상태값이며 실제 km/h가 아니다 (엔코더 추가 전)
