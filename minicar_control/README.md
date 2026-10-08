# Minicar Control (ICD v0.4)

무선 미니카 제어 파이프라인. FSR 페달과 PC 키보드 조향 입력이 4개 노드를 거쳐 차량을 구동한다.

차량 측 컨트롤러는 **세 가지**가 있다. 셋 다 같은 `CommandPacket`을 받으므로, 링크
반대편에 무엇이 붙든 나머지 노드는 바뀌지 않는다.

```
FSR ×2 → ADS1115 → ESP32 #3 ──ESP-NOW──┐
                                        ├─→ ESP32 #1 (Hub) ──UART──→ Zybo
PC 키보드 ──UART1──────────────────────┘                              │
                                                                      │ UART
                                        ESP32 #2 ←──ESP-NOW── ESP32 #1 ←┘
                                            │
                                         차량 구동
```

| 스케치 | 차량 구동 | 가속 페달의 의미 |
|---|---|---|
| `ESP32_2_VehicleControl` | L298N + TT 모터 ×4 | 가속도 |
| `ESP32_2_NewVehicleControl` | ACEBOTT QD001 V2 내장 드라이버 (9600 시리얼) | 가속도 |
| **`ESP32_2_FinalVehicleControl`** | **MDD10A 직접 PWM — 현재 사용** | **목표 속도** |

세 번째 스케치에서 **가속 페달의 의미가 바뀌었다.** 앞의 두 스케치는 레벨이 가감속률을
정했지만, `Final`에서는 레벨이 **목표 속도**를 고르고 `vehicleSpeed`가 그 목표로 부드럽게
접근한다. 제동은 여전히 감속률이다.

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

## 조향 소스 전환

조향은 현재 PC 키보드가 담당하지만, 최종 시스템에서는 CNN 가속기가 대신한다.
교체 지점은 `main.c` 상단의 스위치 하나다.

```c
#define STEERING_SOURCE_CNN  0   // 0 = PC 키보드, 1 = CNN RESULT 레지스터
```

`0`인 동안에도 전체 파이프라인이 그대로 동작하므로, CNN IP가 블록 디자인에 올라가기 전까지
키보드로 검증과 FSR 임계값 보정을 계속할 수 있다. 어느 모드든 **수동 정지는 PC가 유지**한다.

## 인터페이스

| ID | 구간 | 사양 |
|---|---|---|
| IF-01 | FSR → ADS1115 | 아날로그 분압 (회로 TBD) |
| IF-02 | ADS1115 → ESP32 #3 | I2C 100 kHz · SDA=GPIO21, SCL=GPIO22 · Addr 0x48 |
| IF-03 | ESP32 #3 → #1 | ESP-NOW ch.1 · `SensorPacket` · 50 Hz |
| IF-04 | ESP32 #1 → Zybo | UART 115200 8-N-1 · GPIO17 → MIO14 (JF9) |
| IF-05 | Zybo → ESP32 #1 | UART 115200 8-N-1 · 100 Hz · MIO15 (JF10) → GPIO16 |
| IF-06 | ESP32 #1 → #2 | ESP-NOW ch.1 · `CommandPacket` · 100 Hz |
| IF-07 | 내부 | `CommandPacket` → `DriverCommand` |
| IF-08 | ESP32 #2 → 모터 드라이버 | L298N: GPIO + PWM 5 kHz / 8 bit · ACEBOTT: 9600 baud 시리얼 · MDD10A: PWM + DIR ×2채널 |
| IF-09 | 드라이버 → Motor | L298N: H-Bridge · ACEBOTT: QD001 V2 내장 드라이버 · MDD10A: 좌/우 병렬 2채널 |

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
│  ├─ src/main.c          센서 해석 + PC 입력 병합 + 명령 생성 (100 Hz)
│  └─ hw/
│     ├─ bd/design_1/     Vivado 블록 디자인 (PS only)
│     └─ xsa/             Vitis용 하드웨어 핸드오프
├─ firmware/
│  ├─ ESP32_1_WirelessBridge/   양방향 허브 (ESP-NOW ↔ UART)
│  ├─ ESP32_2_VehicleControl/      차량 제어 (50 Hz) + L298N 구동
│  ├─ ESP32_2_NewVehicleControl/   차량 제어 (40 Hz) + ACEBOTT QD001 V2 구동
│  ├─ ESP32_2_FinalVehicleControl/ 차량 제어 (40 Hz) + MDD10A 직접 PWM 구동
│  └─ ESP32_3_FSR/                 ADS1115 읽기 + SensorPacket 송신 (50 Hz)
└─ host/car_control_fsr.py      PC 조향·E-Stop GUI (Tkinter)
```

## 노드 / MAC

| 노드 | MAC | 비고 |
|---|---|---|
| ESP32 #1 Hub | `B0:3F:D3:75:17:50` | UART2 RX=GPIO16 / TX=GPIO17 |
| ESP32 #2 Vehicle | `B0:3F:D3:64:04:14` | 차량 구동 |
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

Arduino IDE (ESP32 core 3.x). 각 보드에 해당 스케치를 굽는다. 세 노드 모두 ESP-NOW
채널 1을 명시적으로 고정한다.

`ESP32_2_VehicleControl`은 `ledcAttach`로 L298N을 직접 PWM 구동하고,
`ESP32_2_NewVehicleControl`은 `ACB_SmartCar_V2` 라이브러리가 필요하다.
`ESP32_2_FinalVehicleControl`은 **추가 라이브러리 없이** `analogWrite` / `digitalWrite`로
MDD10A를 직접 구동한다.

차량 보드를 바꾸면 **ESP32 #1의 `VEHICLE_MAC`도 새 보드의 MAC으로 고쳐야 한다.**
허브가 그 주소로만 송신한다.

### PC

```bash
pip install pyserial
python host/car_control_fsr.py
```

`SERIAL_PORT`를 Zybo USB-UART의 COM 포트로 맞춘다. 해당 포트를 **다른 시리얼 모니터가
잡고 있으면 안 된다** (Vitis Serial Terminal, Arduino IDE Serial Monitor 등 —
`PermissionError(13)`의 원인).

조작: `←`/`→` 조향, `SPACE` E-Stop, `R` 해제, `Q` 종료. 가감속은 FSR 페달.

## 차량 구동

### L298N  (`ESP32_2_VehicleControl`)

| | IN1 | IN2 | ENA | IN3 | IN4 | ENB |
|---|---|---|---|---|---|---|
| GPIO | 25 | 26 | 27 | 32 | 14 | 13 |

좌측이 `IN1/IN2/ENA`, 우측이 `IN3/IN4/ENB`. 우측은 장착 방향이 반대라 코드에서 논리를 반전한다.
PWM 5 kHz / 8 bit, 가상 속도 0~100.

### ACEBOTT QD001 V2  (`ESP32_2_NewVehicleControl`)

PWM 핀을 직접 물리지 않는다. `ACB_SmartCar_V2` 라이브러리가 **9600 baud 시리얼**로
차량 자체 드라이버에 모터 명령을 보낸다.

| 항목 | 값 |
|---|---|
| 모터 번호 | 1 = 앞좌 · 2 = 뒤좌 · 3 = 앞우 · 4 = 뒤우 |
| 명령 범위 | −255 ~ +255 |
| 최소 구동 | **125** — 110에서는 일부 바퀴만 돌고 불안정, 120부터 움직임 |
| 조향 차등 | **0.30** — 양쪽 모두 전진, 최대 조향에서 바깥 ×1.30 / 안쪽 ×0.70 |
| 제어 주기 | **25 ms (40 Hz)** — 9600 baud로 모터 명령 4개를 보낼 여유 |

L298N판과 달리 **조향이 바퀴를 역회전시키지 않는다.** 양쪽이 계속 전진하면서 속도 차이만
생기므로 제자리 선회가 아니라 호를 그리며 돈다. 움직이는 바퀴는 항상 125 이상으로 명령되어
불안정 구간(1~124)을 피한다.

같은 명령이 반복되면 시리얼 전송을 생략한다. 그렇게 하지 않으면 값이 그대로여도 매 주기
4개 프레임이 나간다.

### MDD10A  (`ESP32_2_FinalVehicleControl`) — 현재 사용

시리얼 드라이버를 버리고 **PWM + DIR 2채널**을 직접 물린다. 바퀴 4개가 좌/우 두 채널에
병렬로 묶여 있다.

| | PWM | DIR | 전진 시 DIR |
|---|---|---|---|
| CH1 좌측 (앞좌 + 뒤좌) | GPIO 16 | GPIO 17 | `LOW` |
| CH2 우측 (앞우 + 뒤우) | GPIO 18 | GPIO 19 | `HIGH` |

좌우 DIR 극성이 반대인 것은 모터 장착 방향이 반대이기 때문이다. 실차에서 확인한 값이다.

| 항목 | 값 |
|---|---|
| 하드웨어 PWM 상한 | **230** — 무부하 실측 약 5.8~6.3 V. 255가 아니라 230으로 둔다 |
| 내부 명령 범위 | −255 ~ +255 (음수 = 후진) |
| 최소 구동 명령 | **60** — 실 PWM 약 54. 저속 조향·감속 여유를 위해 A1 목표(67)보다 낮다 |
| 조향 차등 | **0.30** — 바깥 ×1.30 / 안쪽 ×0.70, 양쪽 모두 전진 |
| 제어 주기 | **25 ms (40 Hz)** — 시리얼 병목은 없어졌지만 거동을 바꾸지 않으려고 유지 |

`commandToPwm()`이 내부 명령 0~255를 실제 PWM 0~230으로 마지막에 한 번 환산한다.
내부 명령값과 PWM값을 섞어 읽지 않도록 주의한다.

#### 가속 페달 = 목표 속도

앞의 두 스케치와 **여기가 다르다.** 레벨이 가속도가 아니라 목표 속도를 고르고,
`vehicleSpeed`가 그 목표로 램프를 타고 접근한다.

| 레벨 | 목표 (내부 명령) | 실 PWM 근사 |
|---|---|---|
| A1 | 67 | 60 |
| A2 | 100 | 90 |
| A3 | 139 | 125 |
| A4 | 188 | 170 |
| A5 | 255 | 230 |

접근 속도는 올라갈 때 `SPEED_RISE_RATE 50`, 내려갈 때 `SPEED_FALL_RATE 80` (단위 /초).
레벨을 계속 밟고 있어도 속도가 무한정 오르지 않고 그 레벨의 목표에서 멈춘다.

제동은 여전히 **감속률**이다.

| 레벨 | 감속률 | 255 → 0 |
|---|---|---|
| B1 | 25.5 | 10.0 초 |
| B2 | 45.9 | 5.6 초 |
| B3 | 76.5 | 3.3 초 |
| B4 | 255.0 | 1.0 초 |
| B5 | 318.75 | 0.8 초 |

가속·제동이 모두 0이면 `COAST_RATE 10.2`로 타력 주행한다 (255 → 0 약 25초).
이것은 `SPEED_FALL_RATE`와 별개다 — 후자는 더 낮은 레벨을 밟았을 때 쓴다.
