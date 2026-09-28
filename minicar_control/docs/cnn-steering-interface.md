# CNN Steering Interface

CNN 가속기가 조향 값을 Zybo 애플리케이션에 넘기는 방법을 정의한다.
**CNN팀이 구현할 것은 레지스터 3개뿐이고, 읽는 코드는 Zybo 쪽에서 작성한다.**

대상 파일: [`zybo/src/main.c`](../zybo/src/main.c)

---

## 1. 무엇을 넘기는가

조향 값 하나. **−90 ~ +90, 10° 단위 19단계.**

```
-90  -80  -70  -60  -50  -40  -30  -20  -10
  0
+10  +20  +30  +40  +50  +60  +70  +80  +90
```

| 값 | 의미 |
|---|---|
| `-90` | 최대 좌회전 |
| `-10` | 약한 좌회전 |
| `0` | 직진 |
| `+10` | 약한 우회전 |
| `+90` | 최대 우회전 |

가속·제동은 CNN이 관여하지 않는다. FSR 페달(ESP32 #3)이 결정한다.

### 19단계는 패킷 해상도이지, 분류기 클래스 수가 아니다

위 19개 값은 `CommandPacket.steering`이 실어 나를 수 있는 값의 집합이다.
**CNN이 반드시 19-class 분류기여야 한다는 뜻은 아니다.**

예를 들어 5-class 분류기를 쓰고 다음처럼 매핑해도 프로토콜은 그대로 만족한다.

| 클래스 | 보낼 값 |
|---|---|
| Hard Left | `-90` |
| Left | `-40` |
| Center | `0` |
| Right | `+40` |
| Hard Right | `+90` |

나중에 클래스를 늘려도 프로토콜은 바뀌지 않는다. **단, 어떤 매핑을 쓰든 값은 10의 배수여야 한다** —
`±45` 같은 값은 아래 §4의 검증 3곳에서 전부 거부된다.

### 부드럽게 만들 필요 없다

차량 쪽(ESP32 #2)이 `STEERING_RATE = 180 deg/sec`로 목표값을 따라가는 smoothing을 이미 한다.
CNN은 프레임마다 판단 결과를 그대로 내면 된다.

---

## 2. 어떻게 넘기는가 — AXI4-Lite 레지스터

Zybo의 50Hz 제어 루프가 매 주기 레지스터를 읽는다(pull). **CNN팀이 Zybo 코드를 호출할 필요는 없다.**

| Offset | 이름 | 방향 | 내용 |
|---|---|---|---|
| `0x00` | `CONTROL` | W | bit0 = START, bit1 = IRQ clear, bit2 = IRQ enable |
| `0x04` | `STATUS` | R | bit0 = BUSY, bit1 = START_READY, **bit2 = DONE**, bit3 = START_PENDING |
| `0x08` | `RESULT` | R | 조향 결과 (8 bit, zero-extend) |

Zybo가 하는 일은 이게 전부다.

```c
status = Xil_In32(CNN_BASEADDR + 0x04);
if (status & 0x04) {                        // DONE = bit2
    value = (int32_t)Xil_In32(CNN_BASEADDR + 0x08);
    // 범위 / 10배수 검증 후 사용
}
```

> 주소와 비트 배치는 CNN팀의 `AXI4_Lite_interconnect_v1_0_S00_AXI.v`와 대조해 확인했다.
> STATUS는 `{ start_pending, done_status, start_ready, busy }` 순서로 조립된다.

### CNN팀이 알려줘야 할 것

1. **IP를 블록 디자인에 인스턴스**하고 `S_AXI_LITE`를 `ps7_0_axi_periph`에 연결
   → `xparameters.h`에 베이스 주소 매크로가 생긴다
2. 그 **매크로 이름**을 알려줄 것. 현재 `main.c`는 아래 이름을 가정하고 있다
   ```c
   #define CNN_BASEADDR  XPAR_CNN_ACCELERATOR_0_S_AXI_LITE_BASEADDR
   ```
   다르면 이 한 줄만 고치면 된다
3. 위 오프셋/비트 배치가 다르면 알려줄 것 (`main.c` 상단 `CNN_REG_*` 매크로)

> 참고: `ps7_0_axi_periph`의 마스터 포트 M00~M07이 **현재 전부 사용 중**이다.
> CNN을 붙이려면 9번째 포트를 늘려야 한다. 상세는
> [Zybo 영상 DMA 실측 구조](https://claude.ai/artifact/UwCAk6hnu7R1rZud6mPXVK) 참고.

### push 방식을 쓰고 싶다면

CNN Done 인터럽트에서 Zybo 함수를 호출하는 방식도 가능하지만, **타임스탬프의 원자성**을 지켜야 한다.
freshness 판정에 쓰는 `XTime`은 64비트라 ARM에서 원자적으로 읽히지 않는다. ISR이 쓰는 도중 제어 루프가
읽으면 찢어진 값이 나와 §3의 타임아웃 판정이 깨진다. push로 갈 거면 32비트 ms 타임스탬프로 바꿔야 하므로,
**pull 방식을 권장한다.**

---

## 3. Freshness — 반드시 지켜야 할 것

`RESULT` 레지스터는 마지막 값을 **무기한 유지한다.** 이게 위험하다.

CNN이 멈춰도 레지스터에는 마지막 조향 값이 남아 있고, 센서 링크도 정상, 차량 명령 링크도 정상이라
**어떤 failsafe도 걸리지 않은 채 차가 그 각도로 계속 달린다.**

그래서 Zybo는 **200ms(`CNN_TIMEOUT_MS`)** 안에 새 결과가 갱신되지 않으면 조향을 0으로 만들고
E-Stop을 건다.

```
CNN 결과 200ms 이상 갱신 없음
        ↓
steering = 0 + FLAG_EMERGENCY_STOP
        ↓
차량 정지
```

**CNN팀이 지킬 것:** 새 추론 결과가 나올 때마다 `STATUS.DONE`을 갱신할 것.
결과가 이전과 같은 값이어도 마찬가지다 — DONE이 "값이 바뀌었다"가 아니라 **"살아있다"**는 신호다.

200ms는 50Hz 기준 10프레임에 해당한다. 추론이 이보다 느리면 미리 알려주면 값을 조정한다.

---

## 4. 검증은 3곳에서 걸린다

범위를 벗어나거나 10의 배수가 아닌 값은 아래 세 군데에서 각각 거부된다.

| 위치 | 함수 |
|---|---|
| Zybo | `readCNNSteering()` — 여기서 먼저 걸러 패킷에 못 들어간다 |
| ESP32 #1 | `validateCommandPacket()` |
| ESP32 #2 | `validatePacket()` |

Zybo에서 거부된 값은 **"업데이트 없음"으로 처리**된다. 마지막 정상값으로 덮어쓰지 않고 그대로 두기
때문에, 이상값이 계속되면 §3의 200ms 타임아웃으로 넘어가 정지한다. 조향이 굳은 채로 남지 않는다.

---

## 5. 전환 방법

`main.c` 상단의 스위치 하나다.

```c
#define STEERING_SOURCE_CNN  0   // 0 = PC 키보드, 1 = CNN
```

**`0`인 동안에도 전체 파이프라인이 그대로 동작한다.** CNN IP가 블록 디자인에 올라가기 전에는
`xparameters.h`에 베이스 주소가 없어 빌드가 깨지므로, 레지스터 접근 코드는 전부 `#if` 안에 있다.

순서:

1. CNN IP를 BD에 인스턴스, AXI4-Lite 연결, 비트스트림 생성
2. `xparameters.h`의 베이스 주소 매크로 이름 확인 → `CNN_BASEADDR` 수정
3. 필요하면 `CNN_REG_STATUS` / `CNN_REG_RESULT` / `CNN_STATUS_DONE_MASK` 수정
4. `STEERING_SOURCE_CNN`을 `1`로 변경
5. 빌드 후 디버그 출력에서 `STEER_SRC=CNN:OK` 확인

---

## 6. 안전 구조 (참고)

조향 담당이 바뀌어도 아래 구조는 그대로다. **네 개의 독립적인 정지 경로**가 있다.

| # | 트리거 | 감지 위치 | 시간 |
|---|---|---|---|
| 1 | 사람이 누르는 수동 정지 | PC → Zybo | 즉시 |
| 2 | 페달 센서 무선 두절 | Zybo | 200 ms |
| 3 | **조향 소스 정지 (CNN 포함)** | Zybo | 200 ms |
| 4 | 차량 명령 두절 | **ESP32 #2 자체** | 300 ms |

1~3은 Zybo가 `FLAG_EMERGENCY_STOP`을 세워 보내고, **4는 Zybo와 무관하게 차량이 스스로 판단한다.**
Zybo가 꺼지든 ESP32 #1이 꺼지든 무선이 끊기든, 차량은 300ms 뒤 자력으로 멈춘다.

### E-Stop은 브레이크 레벨 5가 아니다

`FLAG_EMERGENCY_STOP`을 받으면 ESP32 #2는 `accel`/`brake` 값을 **보기도 전에** `hardStop()`으로
모터를 즉시 0으로 만든다. 그래서 Zybo는 E-Stop 시 `accel=0, brake=0`을 보낸다 —
`brake=5`(정상 제동 램프, `-80 unit/s`)와는 개념이 다르다.

### 수동 정지는 남겨둘 것을 권한다

CNN이 **정상 동작하면서 판단만 틀린** 경우(예: 벽으로 향함) 위 2·3·4번은 아무것도 걸리지 않는다.
브레이크 페달이 수동 정지 역할을 하긴 하지만, 레벨 5도 속도 100에서 약 1.25초가 걸리는 램프이고
센서 무선이 살아있어야 동작한다. 즉시 정지 수단은 별도로 유지하는 것이 안전하다.

현재는 PC(UART1, `SPACE`)가 그 역할을 하며, `STEERING_SOURCE_CNN`을 `1`로 바꿔도 **수동 정지 경로는
그대로 남는다.** 최종 시연에서 PC를 완전히 떼려면, 그 전에 대체 수단(ESP32 #3에 물리 버튼을 달아
`SensorPacket`에 플래그 비트를 추가하는 등)을 먼저 마련해야 한다.

---

## 7. 요약 — CNN팀이 할 일

1. 추론 결과를 **−90~+90, 10의 배수**로 매핑해 `RESULT` 레지스터에 쓴다
2. 결과를 쓸 때마다 **`STATUS.DONE`을 갱신**한다 (값이 같아도)
3. CNN IP를 BD에 올리고 **AXI4-Lite 베이스 주소 매크로 이름**을 알려준다
4. 오프셋/비트 배치가 위와 다르면 알려준다

Zybo 쪽은 이후 스위치 하나만 바꾸면 된다.
