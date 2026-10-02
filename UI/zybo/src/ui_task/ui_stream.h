#ifndef UI_STREAM_H
#define UI_STREAM_H
#include "xil_types.h"
#include "ui_stream_config.h"
int UiStream_Init(void);
void UiStream_Service(UINTPTR frame_addr,u32 src_width,u32 src_height,
    int angle_deg,int pressure,unsigned snapshot_count,
    const volatile unsigned *completion_counter,int cnn_valid,unsigned cnn_age_ms);
void UiStream_PrintStats(void);
#endif
