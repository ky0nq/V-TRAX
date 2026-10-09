#ifndef CAPTURE_TASK_H
#define CAPTURE_TASK_H

#include "xil_types.h"
#include "xtime_l.h"

typedef enum {
    CAPTURE_MODE_TEST = 0,
    CAPTURE_MODE_DEMO
} CaptureMode;

CaptureMode CaptureGetMode(void);
void CaptureEnterDemoMode(void);

extern volatile unsigned int capture_download_active;
extern unsigned int capture_waiting;
extern UINTPTR capture_source_addr;
extern unsigned int cnn_waiting;
extern unsigned int cnn_result_valid;
extern unsigned int cnn_start_count;
extern unsigned int cnn_done_count;
extern unsigned int cnn_timeout_count;
extern s8 cnn_last_result;
extern XTime ui_cnn_done_time;

void CapturePrintPixels(void);
void CaptureDownloadPixels(void);
void CaptureStartFromLastFrame(void);
void CapturePoll(void);
void CnnPoll(void);
void CaptureTimerPoll(void);

#endif /* CAPTURE_TASK_H */
