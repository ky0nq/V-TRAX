/*
 *  display_ctrl.h
 *
 *  HDMI output control : pixel clock (axi_dynclk) + video timing (v_tc)
 *
 *  ---------------------------------------------------------------------
 *  WHY SOFTWARE MUST MAKE THE CLOCK IN THIS DESIGN
 *
 *  In the block design, the pixel clock is not fixed anywhere.
 *
 *      axi_dynclk_0/REF_CLK_I      <- FCLK_CLK0 (100 MHz)
 *      axi_dynclk_0/PXL_CLK_O      -> v_tc_0/clk
 *                                     v_axi4s_vid_out_0/vid_io_out_clk
 *                                     rgb2dvi_0/PixelClk
 *      axi_dynclk_0/PXL_CLK_5X_O   -> rgb2dvi_0/SerialClk
 *      axi_dynclk_0/LOCKED_O       -> rgb2dvi_0/aRst_n
 *
 *  axi_dynclk is an IP that reprograms an MMCM through DRP. Right after reset
 *  it gives out no clock, and LOCKED_O = 0, so rgb2dvi is also held in
 *  reset. So the whole HDMI output is stopped until DisplayStart() in this
 *  file is called.
 *
 *  -> If software does not make the clock, you get no picture at all.
 *     (The monitor shows "No signal". Not even noise.)
 *
 *  This is the difference from the old rgb2dvi version. There, rgb2dvi got
 *  the pixel clock from a fixed Clocking Wizard and made the 5x serial
 *  clock inside the IP, so software did not need to do anything.
 *
 *  ---------------------------------------------------------------------
 *  DRIVER CHOICE : WE USE ddynclk FROM THE BSP
 *
 *  The PZ7020 project copied Digilent's old driver
 *  (ClkFindParams / ClkFindReg / ClkWriteReg / ClkStart) into src/dynclk/.
 *  The BSP of this platform already has dynclk_v1_1 (ddynclk), so we use
 *  that. It has one more good point :
 *
 *      DDynClk_CfgInitialize() reads the reference clock frequency from a
 *      read-only register (0x20) in the IP. This works because the generic
 *      kRefClkFreqHz in axi_dynclk.vhd is placed in slv_reg8.
 *      So we do not need to hard-code 100 MHz. If REF_CLK_I is changed in
 *      the block design later, the software still works.
 *
 *  ---------------------------------------------------------------------
 *  YOU CAN CHANGE THE RESOLUTION
 *
 *  Software makes the clock, so you can pass another VideoMode from
 *  lcd_modes.h to DisplaySetMode() and call DisplayStart() again. The
 *  output really changes to that resolution. In the fixed-clock version
 *  only the VTC changed, the clock did not follow, and the monitor lost
 *  sync. In this design it works correctly.
 */

#ifndef DISPLAY_CTRL_H_
#define DISPLAY_CTRL_H_

#include "xil_types.h"
#include "xvtc.h"
#include "ddynclk.h"        /* BSP : dynclk_v1_1 */
#include "lcd_modes.h"

#define DISPLAY_NUM_FRAMES 1

typedef enum {
    DISPLAY_STOPPED = 0,
    DISPLAY_RUNNING = 1
} DisplayState;

typedef struct {
    XVtc         vtc;        /* VTC driver instance                         */
    DDynClk      dynClk;     /* axi_dynclk driver instance                  */
    VideoMode    vMode;      /* current video mode                          */
    u32          pxlFreqHz;  /* pixel clock we actually asked for (Hz)      */
    DisplayState state;      /* is the timing generator running?            */
} DisplayCtrl;

/*
 *  vtcId     : XPAR_VTC_0_DEVICE_ID
 *  dynClkId  : XPAR_DYNCLK_0_DEVICE_ID
 */
int DisplayInitialize(DisplayCtrl *dispPtr, u16 vtcId, u16 dynClkId);
int DisplaySetMode(DisplayCtrl *dispPtr, const VideoMode *newMode);
int DisplayStart(DisplayCtrl *dispPtr);
int DisplayStop(DisplayCtrl *dispPtr);

#endif /* DISPLAY_CTRL_H_ */
