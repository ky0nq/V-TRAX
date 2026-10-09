#ifndef CAPTURE_DRIVER_H
#define CAPTURE_DRIVER_H

#include "xil_types.h"
#include "xparameters.h"

/* CAPTURE_AXI_Lite_0: generated xparameters.h and AXI-Lite RTL. */
#define CAPTURE_BASEADDR    XPAR_CAPTURE_AXI_LITE_0_S00_AXI_BASEADDR
#define CAPTURE_CTRL_OFFSET 0x00U
#define CAPTURE_CTRL_IRQ_CLEAR 0x02U
#define CAPTURE_READ_CPU_ENABLE 0x01U
#define CAPTURE_IRQ_ENABLE 0x02U
#define CAPTURE_SRC_OFFSET  0x04U
#define CAPTURE_X_OFFSET    0x08U
#define CAPTURE_Y_OFFSET    0x0CU
#define CAPTURE_STS_OFFSET  0x10U
#define CAPTURE_PIXEL_INDEX_OFFSET 0x14U
#define CAPTURE_PIXEL_DATA_OFFSET  0x18U
#define CAPTURE_READ_SELECT_OFFSET 0x1CU
#define CAPTURE_READBACK_ID  0x43505200U
#define CAPTURE_STS_DONE    0x01U
#define CAPTURE_STS_BUSY    0x02U
#define CAPTURE_CROP_SIZE   256U
#define CAPTURE_RESULT_SIZE 64U
/* Lower center: cover the steering wheel at the bottom of the camera view. */
#define CAPTURE_X      444U
#define CAPTURE_CROP_X      (CAPTURE_X) - (CAPTURE_X%8)
#define CAPTURE_CROP_Y      142U
u32 CaptureStatus(void);
void CaptureIrqClear(void);
void CaptureSelectReadPort(unsigned int cpu_read);
u32 CaptureReadPixel(unsigned int x, unsigned int y);

#endif /* CAPTURE_DRIVER_H */
