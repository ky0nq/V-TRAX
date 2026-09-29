/*
 *  filter_sw.h
 *
 *  CPU(PS)로 프레임버퍼를 직접 처리하는 소프트웨어 필터
 *  (custom VDMA 버전).
 *  하드웨어(블록디자인/비트스트림) 변경은 전혀 없습니다.
 *
 *  ---------------------------------------------------------------------
 *  S2MM_SR[10:8]이 가장 최근에 완료된 캡처 버퍼를 제공하고, MM2S의
 *  SA/LIVE 설정은 다음 프레임 경계에서 반영됩니다.
 *
 *  ---------------------------------------------------------------------
 *  메모리 안의 바이트 순서  ― 먼저 확인할 것
 *
 *  이 파이프라인의 AXI4-Stream 은 R-B-G 배치입니다.
 *
 *      AXI_BayerToRGB.vhd:419
 *          m_axis_video_tdata <= "00" & Red & Blue & Green;
 *
 *  AXI 는 tdata 의 최하위 바이트가 낮은 주소로 감으로, 메모리에는
 *
 *      byte 0 = tdata[7:0]   = G
 *      byte 1 = tdata[15:8]  = B
 *      byte 2 = tdata[23:16] = R
 *
 *  즉 픽셀당 [G][B][R] 입니다. 흔히 예상하는 RGB 도 BGR 도 아닙니다.
 *
 *  이게 버그는 아닙니다. 그래서 채널 분리 필터를 먼저 넣어뒀습니다.
 *    FILT_CH_R 을 걸었을 때 화면이 빨간 계열로만 남으면 맞는 것이고,
 *    초록이나 파랑으로 나오면 filter_sw.c 위쪽의 OFF_R / OFF_G / OFF_B
 *    세 줄만 고치면 나머지 필터가 전부 따라서 맞습니다.
 *
 *  ---------------------------------------------------------------------
 *  반드시 지켜야 하는 것 : 캐시
 *
 *  custom VDMA는 HP 포트로 DDR을 직접 읽고 씁니다.
 *  CPU 캐시를 거치지 않습니다.
 *
 *      읽기 전 : Xil_DCacheInvalidateRange()  - 캐시의 낡은 사본을 버린다
 *      쓴 다음 : Xil_DCacheFlushRange()       - 캐시에만 있는 결과를 내려보낸다
 */

#ifndef FILTER_SW_H
#define FILTER_SW_H

#include "xil_types.h"
#include "../mcdma_api/mcdma_api.h"

typedef enum {
    FILT_COPY = 0,
    FILT_CH_R,
    FILT_CH_G,
    FILT_CH_B,
    FILT_GRAY,
    FILT_BINARY,
    FILT_SOBEL,
    FILT_COUNT
} Filt_kind;

/*
 *  vdma_ctx   : main.c에서 설정하고 기동한 custom VDMA handle
 *  disp_base  : 표시 버퍼 2장을 놓을 시작 주소. 캡처 버퍼들과 겹치면 안 됨.
 *  w, h       : 해상도
 *
 *  반환 : 0 이면 성공
 */
int         filter_sw_init(VdmaHandle *vdma_ctx, UINTPTR disp_base,
                           u16 w, u16 h);

void        filter_sw_next(void);
Filt_kind   filter_sw_get(void);
const char *filter_sw_name(Filt_kind k);

void        filter_sw_apply(void);
void        filter_sw_live(void);
void        filter_sw_thresh(int delta);
void        filter_sw_dump(void);

#endif /* FILTER_SW_H */
