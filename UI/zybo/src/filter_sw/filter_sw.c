

/*
 *  filter_sw.c
 *
 *  소프트웨어 필터 (custom VDMA 버전). 구조는 filter_sw.h 를 먼저 읽으세요.
 */

#include <string.h>

#include "filter_sw.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xtime_l.h"

/*===========================================================================
 *  픽셀 안의 채널 위치
 *===========================================================================*/
#define OFF_G   0
#define OFF_B   1
#define OFF_R   2

#define BYTES_PER_PIXEL 3

/*===========================================================================
 *  상태
 *===========================================================================*/
static VdmaHandle *s_ctx;
static UINTPTR   s_disp[2];
static u16       s_w, s_h;
static u32       s_stride;
static u32       s_frame_bytes;
static int       s_ready;
static int       s_disp_idx;
static int       s_live;
static Filt_kind s_kind = FILT_GRAY;
static int       s_thresh = 128;

static const char *s_names[FILT_COUNT] = {
    "copy   (baseline, no processing)",
    "R only (byte order check)",
    "G only (byte order check)",
    "B only (byte order check)",
    "gray",
    "binary",
    "sobel 3x3",
};

/*===========================================================================
 *  헬퍼
 *===========================================================================*/

static u8 gray_of(const u8 *p)
{
    return (u8)(((u32)p[OFF_R] * 77u +
                 (u32)p[OFF_G] * 150u +
                 (u32)p[OFF_B] * 29u) >> 8);
}

static void put_gray(u8 *p, u8 v)
{
    p[OFF_R] = v;
    p[OFF_G] = v;
    p[OFF_B] = v;
}

/*
 *  가장 최근에 완성된 캡처 버퍼의 인덱스.
 *
 *  ★ VDMA의 PARKPTR 레지스터를 대신하는 부분 ★
 *  main.c 의 S2MM 완료 인터럽트 핸들러가 매 프레임마다
 *  s_ctx->newest_rx_idx 를 갱신해두므로, 여기서는 그냥 읽기만 하면 됨.
 */
static int newest_capture(void)
{
    return (int)s_ctx->newest_rx_idx;
}

/*
 *  MM2S(HDMI 출력)를 특정 주소 하나로 고정시킴.
 *
 *  ★ VDMA 버전의 commit_read_addrs()/point_display_at() 를 대체 ★
 *  custom VDMA의 SA와 LIVE 비트를 바꾸면 다음 프레임 경계에서 반영됨.
 */
static void point_display_at(UINTPTR addr)
{
    vdma_set_fixed_source(s_ctx, addr);
}

/*===========================================================================
 *  필터 본체 (VDMA 때와 동일, DMA와 무관한 순수 CPU 연산이라 안 바뀜)
 *===========================================================================*/

static void filt_copy(const u8 *src, u8 *dst)
{
    memcpy(dst, src, s_frame_bytes);
}

static void filt_channel(const u8 *src, u8 *dst, int keep)
{
    u32 n = (u32)s_w * s_h;
    u32 i;

    for (i = 0; i < n; i++) {
        const u8 *sp = src + i * BYTES_PER_PIXEL;
        u8       *dp = dst + i * BYTES_PER_PIXEL;

        dp[0] = 0;
        dp[1] = 0;
        dp[2] = 0;
        dp[keep] = sp[keep];
    }
}

static void filt_gray(const u8 *src, u8 *dst)
{
    u32 n = (u32)s_w * s_h;
    u32 i;

    for (i = 0; i < n; i++) {
        put_gray(dst + i * BYTES_PER_PIXEL, gray_of(src + i * BYTES_PER_PIXEL));
    }
}

static void filt_binary(const u8 *src, u8 *dst)
{
    u32 n = (u32)s_w * s_h;
    u32 i;

    for (i = 0; i < n; i++) {
        u8 g = gray_of(src + i * BYTES_PER_PIXEL);
        put_gray(dst + i * BYTES_PER_PIXEL, (g >= (u8)s_thresh) ? 255 : 0);
    }
}

static void filt_sobel(const u8 *src, u8 *dst)
{
    int x, y;

    memset(dst, 0, s_frame_bytes);

    for (y = 1; y < (int)s_h - 1; y++) {
        const u8 *r0 = src + (u32)(y - 1) * s_stride;
        const u8 *r1 = src + (u32) y      * s_stride;
        const u8 *r2 = src + (u32)(y + 1) * s_stride;
        u8       *dp = dst + (u32) y      * s_stride + BYTES_PER_PIXEL;

        for (x = 1; x < (int)s_w - 1; x++) {
            int xm = (x - 1) * BYTES_PER_PIXEL;
            int xc =  x      * BYTES_PER_PIXEL;
            int xp = (x + 1) * BYTES_PER_PIXEL;

            int p00 = gray_of(r0 + xm), p01 = gray_of(r0 + xc), p02 = gray_of(r0 + xp);
            int p10 = gray_of(r1 + xm),                         p12 = gray_of(r1 + xp);
            int p20 = gray_of(r2 + xm), p21 = gray_of(r2 + xc), p22 = gray_of(r2 + xp);

            int gx = (p02 + 2 * p12 + p22) - (p00 + 2 * p10 + p20);
            int gy = (p20 + 2 * p21 + p22) - (p00 + 2 * p01 + p02);
            int m;

            if (gx < 0) gx = -gx;
            if (gy < 0) gy = -gy;

            m = gx + gy;
            if (m > 255) m = 255;

            put_gray(dp, (u8)m);
            dp += BYTES_PER_PIXEL;
        }
    }
}

/*===========================================================================
 *  공개 함수
 *===========================================================================*/

int filter_sw_init(VdmaHandle *vdma_ctx, UINTPTR disp_base, u16 w, u16 h)
{
    if (vdma_ctx == NULL || vdma_ctx->num_buffers == 0 || w == 0 || h == 0) {
        return -1;
    }

    s_ctx         = vdma_ctx;
    s_w           = w;
    s_h           = h;
    s_stride      = (u32)w * BYTES_PER_PIXEL;
    s_frame_bytes = s_stride * h;
    s_disp[0]     = disp_base;
    s_disp[1]     = disp_base + s_frame_bytes;
    s_disp_idx    = 0;
    s_live        = 1;
    s_ready       = 1;

    memset((void *)s_disp[0], 0, s_frame_bytes);
    memset((void *)s_disp[1], 0, s_frame_bytes);
    Xil_DCacheFlushRange((INTPTR)s_disp[0], s_frame_bytes * 2);

    xil_printf("filter : ready. capture 0x%08X xN, display 0x%08X x2 "
               "(%d bytes each)\r\n",
               (unsigned)s_ctx->buffer_address[0], (unsigned)s_disp[0],
               (int)s_frame_bytes);
    return 0;
}

void filter_sw_next(void)
{
    s_kind = (Filt_kind)((s_kind + 1) % FILT_COUNT);
    xil_printf("filter : %s   (press 'f' to apply)\r\n", s_names[s_kind]);
}

Filt_kind filter_sw_get(void)
{
    return s_kind;
}

const char *filter_sw_name(Filt_kind k)
{
    return (k < FILT_COUNT) ? s_names[k] : "?";
}

void filter_sw_thresh(int delta)
{
    s_thresh += delta;
    if (s_thresh < 0)   s_thresh = 0;
    if (s_thresh > 255) s_thresh = 255;
    xil_printf("filter : binary threshold = %d\r\n", s_thresh);
}

void filter_sw_apply(void)
{
    int       cap;
    const u8 *src;
    u8       *dst;
    XTime     t0, t1;
    u32       us, ms;

    if (!s_ready) {
        xil_printf("filter : not initialised\r\n");
        return;
    }

    cap = newest_capture();
    src = (const u8 *)s_ctx->buffer_address[cap];
    dst = (u8 *)s_disp[s_disp_idx];

    Xil_DCacheInvalidateRange((INTPTR)src, s_frame_bytes);

    XTime_GetTime(&t0);

    switch (s_kind) {
    case FILT_COPY:   filt_copy(src, dst);                break;
    case FILT_CH_R:   filt_channel(src, dst, OFF_R);      break;
    case FILT_CH_G:   filt_channel(src, dst, OFF_G);      break;
    case FILT_CH_B:   filt_channel(src, dst, OFF_B);      break;
    case FILT_GRAY:   filt_gray(src, dst);                break;
    case FILT_BINARY: filt_binary(src, dst);              break;
    case FILT_SOBEL:  filt_sobel(src, dst);               break;
    default:          filt_copy(src, dst);                break;
    }

    XTime_GetTime(&t1);

    Xil_DCacheFlushRange((INTPTR)dst, s_frame_bytes);

    /* ★ 여기가 VDMA 버전이랑 제일 다름 — 즉시 반영이 아니라
     * "다음 재무장 때 이 주소로 바꿔라"는 플래그만 세움 */
    point_display_at(s_disp[s_disp_idx]);

    s_disp_idx ^= 1;
    s_live = 0;

    us = (u32)(((t1 - t0) * 1000000ULL) / COUNTS_PER_SECOND);
    ms = us / 1000u;

    xil_printf("filter : %s\r\n", s_names[s_kind]);
    xil_printf("  source capture buffer : %d\r\n", cap);
    xil_printf("  processing time       : %d.%03d ms\r\n",
               (int)ms, (int)(us % 1000u));
    if (us > 0) {
        xil_printf("  => %d fps if run continuously\r\n",
                   (int)(1000000u / us));
    }
    xil_printf("  ('b' to go back to live video)\r\n");
}

void filter_sw_live(void)
{
    if (!s_ready) {
        return;
    }
    if (s_live) {
        xil_printf("filter : already showing live video\r\n");
        return;
    }

    /* LIVE selects S2MM's most recently completed capture buffer. */
    vdma_set_live(s_ctx);

    s_live = 1;
    xil_printf("filter : back to live video\r\n");
}

void filter_sw_dump(void)
{
    unsigned int i;

    xil_printf("\r\n--- software filter -----------------------------\r\n");
    xil_printf("state      : %s\r\n", s_live ? "live video" : "frozen (filtered)");
    xil_printf("filter     : %s\r\n", s_names[s_kind]);
    xil_printf("threshold  : %d  (binary only)\r\n", s_thresh);
    xil_printf("resolution : %dx%d, stride %d, frame %d bytes\r\n",
               (int)s_w, (int)s_h, (int)s_stride, (int)s_frame_bytes);
    xil_printf("capture    :");
    for (i = 0; i < s_ctx->num_buffers; ++i)
        xil_printf(" %08X", (unsigned)s_ctx->buffer_address[i]);
    xil_printf("\r\n");
    xil_printf("display    : 0x%08X / 0x%08X\r\n",
               (unsigned)s_disp[0], (unsigned)s_disp[1]);
    xil_printf("byte order : G=%d B=%d R=%d  (see filter_sw.c if colours look wrong)\r\n",
               OFF_G, OFF_B, OFF_R);
    xil_printf("-------------------------------------------------\r\n");
}
