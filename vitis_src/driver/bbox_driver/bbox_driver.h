#ifndef BBOX_DRIVER_H
#define BBOX_DRIVER_H

#include "xstatus.h"
#include "xparameters.h"

/* BBOX_0 overlays the fixed 256x256 capture ROI on the HDMI stream. */
#define BBOX_BASEADDR       XPAR_BBOX_0_S00_AXI_BASEADDR
#define BBOX_ENABLE         1U
#define BBOX_COLOR_RED      0x00FF0000U

int BboxInitialize(void);
void BboxToggle(void);
void BboxDisable(void);

#endif /* BBOX_DRIVER_H */
