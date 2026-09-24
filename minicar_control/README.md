# Minicar Control (ICD v0.3)

무선 미니카 제어 파이프라인. FSR 페달과 PC 키보드 조향 입력이 4개 노드를 거쳐
L298N + TT 모터 4개를 구동한다.

```
FSR ×2 → ADS1115 → ESP32 #3 ──ESP-NOW──┐
                                        ├─→ ESP32 #1 (Hub) ──UART──→ Zybo
PC 키보드 ──UART1──────────────────────┘                              │
                                                                      │ UART
                                        ESP32 #2 ←──ESP-NOW── ESP32 #1 ←┘
                                            │
                                        L298N → TT Motor ×4
```

## 설계의 핵심

파이프라인에는 **SOF가 다른 8바이트 패킷 두 종류**가 돈다.

| 패킷 | SOF | 방향 |
|---|---|---|
| `SensorPacket` | `A5 5A` | ESP32 #3 → 허브 → Zybo (페달 원시값 상행) |
| `CommandPacket` | `AA 55` | Zybo → 허브 → ESP32 #2 (주행 명령 하행) |

**ESP32 #1은 양방향 허브**다. 두 패킷 모두 검증만 하고 payload는 건드리지 않으며,
두 채널은 서로의 내용을 보지 않는다. Zybo↔허브 구간은 **하나의 UART 페어를 양방향으로**
공유하는데, SOF가 다르므로 두 스트림이 섞이지 않는다.

**판단 지점은 Zybo 한 곳뿐이다.** ESP32 #3은 임계값을 모른 채 ADS1115 원시 counts를
그대로 올리고, Level 0~5 변환은 Zybo에서만 일어난다. ESP32 #2의 Vehicle Controller는
통신 방식을 전혀 모른 채 `DriverCommand` 구조체만 입력으로 받는다.

전체 다이어그램은 [`docs/architecture.html`](docs/architecture.html) 참고.

## 인터페이스

| ID | 구간 | 사양 |
|---|---|---|
| IF-01 | FSR → ADS1115 | 아날로그 분압 (회로 TBD) |
| IF-02 | ADS1115 → ESP32 #3 | I2C 100 kHz · SDA=GPIO21, SCL=GPIO22 · Addr 0x48 |
| IF-03 | ESP32 #3 → #1 | ESP-NOW ch.1 · `SensorPacket` · 50 Hz |
| IF-04 | ESP32 #1 → Zybo | UART 115200 8-N-1 · GPIO17 → MIO14 (JF9) |
| IF-05 | Zybo → ESP32 #1 | UART 115200 8-N-1 · MIO15 (JF10) → GPIO16 |
| IF-06 | ESP32 #1 → #2 | ESP-NOW ch.1 · `CommandPacket` · 50 Hz |
| IF-07 | 내부 | `CommandPacket` → `DriverCommand` |
| IF-08 | ESP32 #2 → L298N | GPIO + PWM 5 kHz / 8 bit |
| IF-09 | L298N → Motor | H-Bridge |

### SensorPacket (8 byte, packed)

| Byte | 0 | 1 | 2 | 3–4 | 5–6 | 7 |
|---|---|---|---|---|---|---|
| | `0xA5` | `0x5A` | SEQ | ACCEL_RAW `int16` | BRAKE_RAW `int16` | CRC8 |

ADS1115 원시 counts를 그대로 전송한다 (PGA ±4.096 V → 1 LSB = 125 µV).

### CommandPacket (8 byte, packed)

| Byte | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| | `0xAA` | `0x55` | SEQ | STEER | ACCEL | BRAKE | FLAGS | CRC8 |

- `STEER` −90~+90, 10° 단위 19단계
- `ACCEL` / `BRAKE` 0~5 — 속도가 아니라 **요청 강도**. `BRAKE > 0`이면 항상 `ACCEL`보다 우선
- `FLAGS` bit0 = Emergency Stop

두 패킷 모두 CRC-8 poly `0x07`, init `0x00`, 대상 Byte 0~6.

## 안전 동작

| 조건 | 동작 |
|---|---|
| 센서 패킷 200 ms 미수신 | Zybo가 E-Stop 플래그 송신 |
| PC 명령 500 ms 미수신 | Zybo가 Steering 0 + E-Stop 래치 |
| 명령 패킷 300 ms 미수신 | ESP32 #2 Failsafe → `hardStop()` |
| `FLAGS` bit0 = 1 | Emergency Stop → `hardStop()` (최우선) |
| ADS1115 읽기 실패 | ESP32 #3이 송신 생략 → 위 센서 타임아웃으로 수렴 |

**세 타임아웃 모두 벽시계 기준**이다. 루프 횟수로 세면 CPU 부하나 노드 간 클럭 드리프트에
따라 안전 동작 시점이 달라지는데, 센서 노드가 자기 클럭으로 50 Hz를 내보내는 구조에서는
이 결합이 실제 위험이 된다.

### 센서 값 검증

CRC는 바이트가 링크를 통과했음만 증명할 뿐 값이 타당함을 보장하지 않는다. 그래서 두 단계를 둔다.

1. **범위** — `-1000 ≤ raw ≤ 26400`. 분압이 3.3 V에서 나오므로 26400 counts를 넘는 값은
   물리적으로 불가능하다. ESP32 #1과 Zybo 양쪽에서 검사한다.
2. **변화율** (Zybo) — 20 ms 샘플 간 `|Δraw| ≤ 15000`. 사람이 낼 수 있는 가장 빠른 페달
   조작도 full scale까지 ~80 ms (샘플당 ~6600 counts)가 걸린다. FSR이 단선되어 분압이 한
   샘플 만에 레일로 튀는 경우를 잡기 위한 것이다.

거부된 샘플은 최신값을 갱신하지 않는다. 계속되면 센서 타임아웃으로 넘어가 E-Stop이 걸린다.

## 구성

```
minicar_control/
├─ zybo/
│  ├─ src/main.c          센서 해석 + PC 입력 병합 + 명령 생성 (50 Hz)
│  └─ hw/
│     ├─ bd/design_1/     Vivado 블록 디자인 (PS only)
│     └─ xsa/             Vitis용 하드웨어 핸드오프
├─ firmware/
│  ├─ ESP32_1_WirelessBridge/   양방향 허브 (ESP-NOW ↔ UART)
│  ├─ ESP32_2_VehicleControl/   차량 제어 (50 Hz) + L298N 구동
│  └─ ESP32_3_FSR/              ADS1115 읽기 + SensorPacket 송신 (50 Hz)
├─ host/car_control_fsr.py      PC 조향·E-Stop GUI (Tkinter)
└─ docs/architecture.html       ICD v0.3 다이어그램
```

## 노드 / MAC

| 노드 | MAC | 비고 |
|---|---|---|
| ESP32 #1 Hub | `B0:3F:D3:75:17:50` | UART2 RX=GPIO16 / TX=GPIO17 |
| ESP32 #2 Vehicle | `B0:3F:D3:64:04:14` | L298N 구동 |
| ESP32 #3 Sensor | `38:3E:51:CB:14:40` | I2C SDA=GPIO21 / SCL=GPIO22 |

MAC 상수가 세 스케치에 흩어져 있다. 보드를 교체하면 `ESP32_1`의 `VEHICLE_MAC`/`SENSOR_MAC`,
`ESP32_2`의 `BRIDGE_MAC`, `ESP32_3`의 `BRIDGE_MAC`을 함께 고쳐야 한다.

## 빌드 / 실행

### Zybo (Vitis)

`zybo/hw/xsa/design_1_wrapper.xsa`로 플랫폼을 만들고 `zybo/src/main.c`를 애플리케이션에
추가한다. BSP의 `stdout`/`stdin`은 **`ps7_uart_1`** 이어야 한다 — `ps7_uart_0`으로 두면
`xil_printf` 디버그 출력이 ESP32 #1로 가는 바이너리 패킷 스트림을 오염시킨다.

UART0 = MIO14(RX)/MIO15(TX) = Pmod **JF9/JF10**, UART1 = MIO48/49 = PC USB-UART.

### ESP32 ×3

Arduino IDE (ESP32 core 3.x — `ledcAttach` API 사용). 각 보드에 해당 스케치를 굽는다.
세 노드 모두 ESP-NOW 채널 1을 명시적으로 고정한다.

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

- FSR 분압 회로와 전압 범위
- `main.c`의 Accel/Brake 임계값 — **현재 값은 임시**, 실측 raw 기준으로 보정 필요
- Steering 보정, 최종 모터 전원, 튜닝 파라미터
- `vehicleSpeed`는 0~100 가상 상태값이며 실제 km/h가 아니다 (엔코더 추가 전)
- Vivado PS에 I2C1(MIO12/13)이 여전히 활성화되어 있다. 센서가 ESP32 #3으로 옮겨간 뒤로
  Zybo는 I2C를 쓰지 않으므로 MIO12/13은 현재 유휴 상태다.

## 변경 이력

### v0.3
센서 취득을 **Zybo의 I2C1에서 ESP32 #3으로 분리**. `SensorPacket` 신설, Zybo↔ESP32 #1
UART가 양방향화, 인터페이스가 IF-01~06 → **IF-01~09**로 재편. 명령 송신률은 100 Hz →
**50 Hz** (센서가 50 Hz로 도착하므로 그보다 빠른 명령 생성은 같은 샘플의 반복에 불과하다).
센서 링크 200 ms 타임아웃과 센서 값 범위·변화율 검증 추가.

### v0.2
`hardStop()`이 입력 `DriverCommand`를 덮어쓰던 계층 위반 수정, I2C 버스 복구 추가,
타임아웃을 루프 횟수에서 벽시계 기준으로 전환.
