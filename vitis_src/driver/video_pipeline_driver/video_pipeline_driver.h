#ifndef VIDEO_PIPELINE_DRIVER_H
#define VIDEO_PIPELINE_DRIVER_H

#include "xil_types.h"
#include "xparameters.h"
#include "xtime_l.h"
#include "../dma_driver/dma_driver.h"   /* VdmaHandle, vdma_configure/start/... */

#define FRAME_WIDTH         1280
#define FRAME_HEIGHT        720
#define BYTES_PER_PIXEL     3
#define FRAME_SIZE_BYTES    (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)
#define NUM_FRAME_BUFFERS   3   /* number of VDMA frame buffers in DDR */

#define VDMA_S2MM_INTR_ID XPAR_FABRIC_DMA_0_S2MM_IRQ_INTR
#define VDMA_MM2S_INTR_ID XPAR_FABRIC_DMA_0_MM2S_IRQ_INTR
#define JA1_GPIO_INTR_ID XPAR_FABRIC_APB_GPIO_0_O_IRQ_0_INTR
#define JA2_GPIO_INTR_ID XPAR_FABRIC_APB_GPIO_0_O_IRQ_1_INTR
#define JA3_GPIO_INTR_ID XPAR_FABRIC_APB_GPIO_0_O_IRQ_2_INTR
#define CNN_INTR_ID XPAR_FABRIC_IP_CNN_0_O_IRQ_INTR
#define CAPTURE_INTR_ID XPAR_FABRIC_CAPTURE_AXI_LITE_0_O_IRQ_INTR
extern volatile unsigned int capture_irq_pending;
extern volatile u32 timer_irq_count;
extern volatile unsigned int capture_irq_count;
extern volatile unsigned int capture_irq_frame_count;

extern VdmaHandle   vdma;
extern volatile u32 ja1_irq_count;
extern volatile u32 ja2_irq_count;
extern volatile u32 ja3_irq_count;
extern volatile unsigned int cnn_irq_pending;
extern volatile unsigned int cnn_irq_count;
extern volatile u32 cnn_irq_last_status;
extern volatile u32 cnn_irq_last_result;
extern volatile XTime cnn_irq_done_time;

extern volatile unsigned int s2mm_done_count;
extern volatile unsigned int s2mm_valid_count;
extern volatile unsigned int mm2s_done_count;
extern volatile unsigned int s2mm_error_count;
extern volatile unsigned int mm2s_error_count;
extern volatile u32 s2mm_last_error;
extern volatile u32 mm2s_last_error;
extern volatile u32 s2mm_last_status;
extern volatile u32 mm2s_last_status;
extern volatile unsigned int s2mm_last_buffer_idx;
extern volatile unsigned int mm2s_last_buffer_idx;

extern const UINTPTR frame_buffer_addresses[NUM_FRAME_BUFFERS];

int SetupVdmaInterrupts(void);
void VdmaLogPoll(void);
void PrintVdmaDiagnostics(void);
void Ja1GpioPrintPoll(void);
void Ja2GpioPrintPoll(void);
void Ja3GpioPrintPoll(void);

#endif /* VIDEO_PIPELINE_DRIVER_H */
