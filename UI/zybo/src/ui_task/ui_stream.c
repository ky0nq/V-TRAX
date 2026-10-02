#include "ui_stream.h"
#include "ui_wire.h"
#include <stdio.h>
#include <string.h>
#include "xparameters.h"
#include "xstatus.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xtime_l.h"
#include "../ui_driver/raw_udp.h"
#include "sleep.h"

#if UI_VIDEO_WIDTH > 1280 || UI_VIDEO_HEIGHT > 720 || UI_VIDEO_FPS == 0 || UI_CROP_WIDTH == 0 || UI_CROP_HEIGHT == 0
#error "Unsupported video dimensions or FPS"
#endif

#define HEADER_BYTES 28U
#define VIDEO_BYTES (UI_VIDEO_WIDTH * UI_VIDEO_HEIGHT * 2U)
#define CHUNKS ((VIDEO_BYTES + UI_PAYLOAD_BYTES - 1U) / UI_PAYLOAD_BYTES)
static int ready, phase; /* 0 idle, 1 copying, 2 transmitting */
static u8 frame[VIDEO_BYTES] __attribute__((aligned(32)));
static UINTPTR source;
static unsigned source_w, source_h, source_stride, source_count, row, chunk;
static u32 frame_id, session_id;
static XTime next_frame, frame_start, next_data;
static int pressure_value, angle_value, cnn_valid_value, pressure_test_value = 1;
static unsigned cnn_age_value;
static unsigned frames_sent, frames_dropped, packet_errors;

static XTime Now(void) { XTime t; XTime_GetTime(&t); return t; }
static XTime Ms(unsigned ms) { return (COUNTS_PER_SECOND / 1000U) * (XTime)ms; }
static int Send(unsigned port, const void *bytes, unsigned len) {
    if(RawUdp_Send((u16)port,bytes,(u16)len)!=XST_SUCCESS){++packet_errors;return 0;}
    return 1;
}
int UiStream_Init(void) {
    XTime now;
    if(RawUdp_Init()!=XST_SUCCESS)return XST_FAILURE;
    now=Now();session_id=(u32)now^(u32)(now>>32);
    next_frame=next_data=now;ready=1;
    xil_printf("[UI] %ux%u RGB565, target %u fps, UDP 7000/7001\r\n",
        UI_VIDEO_WIDTH,UI_VIDEO_HEIGHT,UI_VIDEO_FPS);
    return XST_SUCCESS;
}

static void UiStreamSetTelemetry(int p, int a, int valid, unsigned age, int test) {
    pressure_value = p<0 ? 0 : p>100 ? 100 : p;
    angle_value = a<-90 ? -90 : a>90 ? 90 : a;
    cnn_valid_value=valid; cnn_age_value=age; pressure_test_value=test;
}
static int UiStreamWantsFrame(void) {
    return ready && phase==0 && Now() >= next_frame;
}
static void UiStreamBeginFrame(UINTPTR addr, unsigned w, unsigned h, unsigned stride,
                        unsigned completed) {
    if (!UiStreamWantsFrame() || !addr || !w || !h || stride<w*3U ||
        UI_CROP_X+UI_CROP_WIDTH>w || UI_CROP_Y+UI_CROP_HEIGHT>h) return;
    source=addr; source_w=w; source_h=h; source_stride=stride;
    source_count=completed; row=0; chunk=0; phase=1;
    frame_start=Now(); next_frame=frame_start+COUNTS_PER_SECOND/UI_VIDEO_FPS;
}
static void UiStreamCopyFrame(unsigned completed) {
    unsigned stop, y, x;
    if (phase != 1) return;
    /* Three DMA buffers: source may start being overwritten after TWO further
     * completions. Discard the entire snapshot if that happens. The caller
     * must also validate the completion count AFTER this function returns. */
    if ((unsigned)(completed-source_count)>=2U) {
        phase=0; ++frames_dropped; return;
    }
    stop=row+UI_COPY_ROWS_PER_POLL;
    if(stop>UI_VIDEO_HEIGHT) stop=UI_VIDEO_HEIGHT;
    for(y=row;y<stop;++y) {
        unsigned sy=UI_CROP_Y+y*UI_CROP_HEIGHT/UI_VIDEO_HEIGHT;
        const volatile u8 *src=(const volatile u8 *)(source+(UINTPTR)sy*source_stride);
        u8 *dst=frame+y*UI_VIDEO_WIDTH*2U;
        Xil_DCacheInvalidateRange((INTPTR)(src+UI_CROP_X*3U), UI_CROP_WIDTH*3U);
        for(x=0;x<UI_VIDEO_WIDTH;++x) {
            unsigned sx=(UI_CROP_X+x*UI_CROP_WIDTH/UI_VIDEO_WIDTH)*3U;
            u16 pixel=UiRgb565(src[sx+UI_DDR_R_OFFSET],
                src[sx+UI_DDR_G_OFFSET],src[sx+UI_DDR_B_OFFSET]);
            UiPut16(dst,pixel); dst+=2;
        }
    }
    row=stop;
    /* Caller rereads DMA count before approving this snapshot for transmission. */
}

/* Called by main after every copy pass, including the final one. */
static void UiStreamValidateFrame(unsigned completed) {
    if(phase!=1) return;
    if((unsigned)(completed-source_count)>=2U) {phase=0;++frames_dropped;return;}
    if(row==UI_VIDEO_HEIGHT) {phase=2; ++frame_id;}
}

static void UiStreamPoll(void) {
    XTime now, budget_end;
    unsigned n;
    if(!ready) return;
    RawUdp_Poll();
    now=Now();
    if(now>=next_data) {
        char json[192];
        int len=snprintf(json,sizeof(json),
            "{\"pressure\":%d,\"angle\":%d,\"cnn_valid\":%s,\"cnn_age_ms\":%u,\"pressure_source\":\"%s\"}",
            pressure_value,angle_value,cnn_valid_value?"true":"false",cnn_age_value,
            pressure_test_value?"test":"sensor");
        if(len>0 && (unsigned)len<sizeof(json)) Send(UI_DATA_PORT,json,(unsigned)len);
        next_data=now+Ms(1000U/UI_TELEMETRY_HZ);
    }
    if(phase!=2) return;
    if(now-frame_start>Ms(UI_FRAME_DEADLINE_MS)) {phase=0;++frames_dropped;return;}
    budget_end=Now()+(COUNTS_PER_SECOND/1000000U)*UI_SEND_BUDGET_US;
    for(n=0;n<UI_PACKETS_PER_POLL && chunk<CHUNKS;++n) {
        u8 packet[HEADER_BYTES+UI_PAYLOAD_BYTES];
        unsigned offset=chunk*UI_PAYLOAD_BYTES, len=VIDEO_BYTES-offset;
        if(len>UI_PAYLOAD_BYTES) len=UI_PAYLOAD_BYTES;
        UiVideoHeader(packet,UI_VIDEO_WIDTH,UI_VIDEO_HEIGHT,chunk,CHUNKS,
            len,frame_id,VIDEO_BYTES,session_id);
        memcpy(packet+HEADER_BYTES,frame+offset,len);
        if (!Send(UI_VIDEO_PORT, packet, HEADER_BYTES + len))
            break;

        ++chunk;

        /* GEM -> PC�� �ʹ� �����ϰ� ���� �۽����� �ʵ��� pacing */
        usleep(200U);

        if (Now() >= budget_end)
            break;
    }
    if(chunk==CHUNKS) {phase=0; ++frames_sent;}
}
void UiStream_PrintStats(void) {
    xil_printf("UI stream: ready=%d frames=%u dropped=%u send_errors=%u phase=%d\r\n",
               ready,frames_sent,frames_dropped,packet_errors,phase);
}


void UiStream_Service(UINTPTR addr,u32 width,u32 height,int angle,int pressure,
    unsigned snapshot_count,const volatile unsigned *completed,int cnn_valid,unsigned cnn_age_ms) {
    if(!ready)return;
    UiStreamSetTelemetry(pressure,angle,cnn_valid,cnn_age_ms,UI_PRESSURE_IS_TEST);
    UiStreamPoll();
    if(addr && completed && UiStreamWantsFrame())
        UiStreamBeginFrame(addr,width,height,width*3U,snapshot_count);
    if(completed){
        UiStreamCopyFrame(*completed);
        UiStreamValidateFrame(*completed);
    }
}
