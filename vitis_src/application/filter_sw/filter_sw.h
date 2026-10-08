/*
 *  filter_sw.h
 *
 *  Software filter. The CPU (PS) works on the frame buffer directly.
 *  (custom VDMA version)
 *  No hardware change is needed (no block design or bitstream change).
 *
 *  ---------------------------------------------------------------------
 *  S2MM_SR[10:8] tells which capture buffer was finished most recently.
 *  A change to the MM2S SA/LIVE setting takes effect at the next frame.
 *
 *  ---------------------------------------------------------------------
 *  BYTE ORDER IN MEMORY  - check this first
 *
 *  The AXI4-Stream in this pipeline is in R-B-G order.
 *
 *      AXI_BayerToRGB.vhd:419
 *          m_axis_video_tdata <= "00" & Red & Blue & Green;
 *
 *  AXI puts the lowest byte of tdata at the lowest address, so in memory:
 *
 *      byte 0 = tdata[7:0]   = G
 *      byte 1 = tdata[15:8]  = B
 *      byte 2 = tdata[23:16] = R
 *
 *  So each pixel is [G][B][R]. It is not RGB, and it is not BGR.
 *
 *  This is not a bug. That is why the channel filters were added first.
 *    Turn on FILT_CH_R. If the screen shows only red, the order is right.
 *    If it shows green or blue, fix only the three lines OFF_R / OFF_G /
 *    OFF_B at the top of filter_sw.c, and all other filters will be right.
 *
 *  ---------------------------------------------------------------------
 *  YOU MUST DO THIS : CACHE
 *
 *  The custom VDMA reads and writes DDR directly through the HP port.
 *  It does not go through the CPU cache.
 *
 *      Before reading : Xil_DCacheInvalidateRange()  - drop old data in the cache
 *      After writing  : Xil_DCacheFlushRange()       - push cache data out to DDR
 */

#ifndef FILTER_SW_H
#define FILTER_SW_H

#include "xil_types.h"
#include "../../driver/dma_driver/dma_driver.h"

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
 *  vdma_ctx   : custom VDMA handle, set up and started in main.c
 *  disp_base  : start address for the 2 display buffers.
 *               Must not overlap the capture buffers.
 *  w, h       : resolution
 *
 *  Returns 0 on success.
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
