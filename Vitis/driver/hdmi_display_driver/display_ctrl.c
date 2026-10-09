/*
 *  display_ctrl.c
 *
 *  Makes the pixel clock (axi_dynclk) and the video timing (v_tc).
 *  Read the comments in display_ctrl.h first to see what this does and why.
 */

#include <string.h>

#include "display_ctrl.h"
#include "xdebug.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "sleep.h"

/*
 *  Stops the video output.
 *
 *  Turning off only the generator is enough. The VTC cannot stop in the
 *  middle of a frame, so it finishes the current line and then stops the
 *  timing signals. Then nobody reads the video data, so the VDMA read
 *  channel stops by itself. That is why there is no VDMA call here.
 *
 *  We do NOT turn off the clock on purpose. If axi_dynclk is turned off,
 *  LOCKED_O goes to 0, rgb2dvi goes into reset, and the monitor loses the
 *  signal completely. While changing modes, the monitor would lose the
 *  input and find it again, and the screen would be black for a few
 *  seconds. We want to avoid that. DisplayStart() sets up the clock again.
 */
int DisplayStop(DisplayCtrl *dispPtr)
{
    if (dispPtr->state == DISPLAY_STOPPED) {
        return XST_SUCCESS;
    }

    XVtc_DisableGenerator(&dispPtr->vtc);
    dispPtr->state = DISPLAY_STOPPED;

    return XST_SUCCESS;
}

/*
 *  Makes the pixel clock, sets up the VTC with the current VideoMode,
 *  and starts it.
 *
 *  The timing math below changes the VideoMode values into the porch and
 *  sync widths that the VTC wants. In VideoMode, hps/hpe/hmax are counted
 *  from the start of the line (running totals).
 *  If one value is off by 1, the picture moves by one pixel, which you
 *  cannot see. If it is very wrong, the picture tears or the monitor
 *  rejects the signal.
 */
int DisplayStart(DisplayCtrl *dispPtr)
{
    int               Status;
    u32               dynStatus;
    u32               vtcControl;
    u32               vtcEvents;
    XVtc_Timing       vtcTiming;
    XVtc_SourceSelect SourceSelect;

    if (dispPtr->state == DISPLAY_RUNNING) {
        return XST_SUCCESS;
    }

    /*-------------------------------------------------------------------
     *  1. Pixel clock
     *
     *  VideoMode.freq is a double in MHz, so we change it to an integer
     *  in Hz. (720p is 74.25 MHz -> 74250000)
     *
     *  DDynClk_SetRate() does disable -> DRP setup -> enable inside, and
     *  then checks the STATUS register again and again until the MMCM locks.
     *
     *  * That check has no timeout (this is how the BSP driver is written).
     *    If the program stops after the message below is printed, the MMCM
     *    did not lock. The cause is usually one of these two:
     *       - the requested frequency cannot be made from this reference clock
     *       - no clock is really coming into REF_CLK_I
     *-------------------------------------------------------------------*/
    dispPtr->pxlFreqHz = (u32)(dispPtr->vMode.freq * 1000000.0);

    /* xil_printf is a small version of printf. We only use %d, which it surely supports. */
    xil_printf("display : requesting pixel clock %d Hz (ref %d Hz) ...\r\n",
               (int)dispPtr->pxlFreqHz,
               (int)dispPtr->dynClk.RefClkFreqHz);

    Status = DDynClk_SetRate(&dispPtr->dynClk, dispPtr->pxlFreqHz);
    if (Status != XST_SUCCESS) {
        xil_printf("display : DDynClk_SetRate failed\r\n");
        return XST_FAILURE;
    }
    xil_printf("display : pixel clock locked\r\n");

    /*-------------------------------------------------------------------
     *  2. Video timing
     *-------------------------------------------------------------------*/
    vtcTiming.HActiveVideo  = dispPtr->vMode.width;
    vtcTiming.HFrontPorch   = dispPtr->vMode.hps  - dispPtr->vMode.width;
    vtcTiming.HSyncWidth    = dispPtr->vMode.hpe  - dispPtr->vMode.hps;
    vtcTiming.HBackPorch    = dispPtr->vMode.hmax - dispPtr->vMode.hpe + 1;
    vtcTiming.HSyncPolarity = dispPtr->vMode.hpol;

    vtcTiming.VActiveVideo  = dispPtr->vMode.height;
    vtcTiming.V0FrontPorch  = dispPtr->vMode.vps  - dispPtr->vMode.height;
    vtcTiming.V0SyncWidth   = dispPtr->vMode.vpe  - dispPtr->vMode.vps;
    vtcTiming.V0BackPorch   = dispPtr->vMode.vmax - dispPtr->vMode.vpe + 1;
    vtcTiming.V1FrontPorch  = dispPtr->vMode.vps  - dispPtr->vMode.height;
    vtcTiming.V1SyncWidth   = dispPtr->vMode.vpe  - dispPtr->vMode.vps;
    vtcTiming.V1BackPorch   = dispPtr->vMode.vmax - dispPtr->vMode.vpe + 1;
    vtcTiming.VSyncPolarity = dispPtr->vMode.vpol;

    vtcTiming.Interlaced    = 0;

    /* Take every field from the generator registers, not from the detector.
     * The v_tc in this design has enable_detection = false, so it has no
     * detector at all. All 17 values below must be 1. */
    memset((void *)&SourceSelect, 0, sizeof(SourceSelect));
    SourceSelect.VBlankPolSrc       = 1;
    SourceSelect.VSyncPolSrc        = 1;
    SourceSelect.HBlankPolSrc       = 1;
    SourceSelect.HSyncPolSrc        = 1;
    SourceSelect.ActiveVideoPolSrc  = 1;
    SourceSelect.ActiveChromaPolSrc = 1;
    SourceSelect.VChromaSrc         = 1;
    SourceSelect.VActiveSrc         = 1;
    SourceSelect.VBackPorchSrc      = 1;
    SourceSelect.VSyncSrc           = 1;
    SourceSelect.VFrontPorchSrc     = 1;
    SourceSelect.VTotalSrc          = 1;
    SourceSelect.HActiveSrc         = 1;
    SourceSelect.HBackPorchSrc      = 1;
    SourceSelect.HSyncSrc           = 1;
    SourceSelect.HFrontPorchSrc     = 1;
    SourceSelect.HTotalSrc          = 1;

    XVtc_SelfTest(&(dispPtr->vtc));

    XVtc_RegUpdateEnable(&(dispPtr->vtc));
    XVtc_SetGeneratorTiming(&(dispPtr->vtc), &vtcTiming);
    XVtc_SetSource(&(dispPtr->vtc), &SourceSelect);

    /* GE selects the generator; SW enables the VTC core itself. The BSP's
     * XVtc_EnableGenerator() sets only GE, so both calls are required. */
    XVtc_EnableGenerator(&dispPtr->vtc);
    XVtc_Enable(&dispPtr->vtc);

    dynStatus = DDynClk_ReadReg(dispPtr->dynClk.Config.BaseAddress,
                                 DDYNCLK_STATUS);
    vtcControl = XVtc_ReadReg(dispPtr->vtc.Config.BaseAddress,
                              XVTC_CTL_OFFSET);
    xil_printf("display : dynclk STATUS=%08X VTC CTL=%08X HSIZE=%08X VSIZE=%08X\r\n",
               (unsigned)dynStatus, (unsigned)vtcControl,
               (unsigned)XVtc_ReadReg(dispPtr->vtc.Config.BaseAddress,
                                       XVTC_GHSIZE_OFFSET),
               (unsigned)XVtc_ReadReg(dispPtr->vtc.Config.BaseAddress,
                                       XVTC_GVSIZE_OFFSET));
    if (dynStatus == 0 ||
        (vtcControl & (XVTC_CTL_SW_MASK | XVTC_CTL_GE_MASK)) !=
            (XVTC_CTL_SW_MASK | XVTC_CTL_GE_MASK)) {
        xil_printf("display : clock lock or VTC enable readback failed\r\n");
        return XST_FAILURE;
    }

    /* The control readback proves configuration only. Generator AV/VBLANK
     * events show whether video timing actually advances on PXL_CLK_O. */
    XVtc_IntrClear(&dispPtr->vtc, XVTC_IXR_G_ALL_MASK);
    usleep(50000);
    vtcEvents = XVtc_ReadReg(dispPtr->vtc.Config.BaseAddress,
                             XVTC_ISR_OFFSET) & XVTC_IXR_G_ALL_MASK;
    dynStatus = DDynClk_ReadReg(dispPtr->dynClk.Config.BaseAddress,
                                 DDYNCLK_STATUS);
    xil_printf("display : 50ms dynclk STATUS=%08X VTC events=%08X (AV=%d VBLANK=%d)\r\n",
               (unsigned)dynStatus, (unsigned)vtcEvents,
               !!(vtcEvents & XVTC_IXR_G_AV_MASK),
               !!(vtcEvents & XVTC_IXR_G_VBLANK_MASK));

    dispPtr->state = DISPLAY_RUNNING;

    return XST_SUCCESS;
}

/*
 *  Driver init.
 *
 *  Note : this does NOT make the clock. The clock is made in
 *  DisplayStart(). If we turned on the clock here, the HDMI output would
 *  start before the VTC timing is ready, and the monitor would catch a
 *  garbage signal for a moment.
 */
int DisplayInitialize(DisplayCtrl *dispPtr, u16 vtcId, u16 dynClkId)
{
    int             Status;
    XVtc_Config    *vtcConfig;
    DDynClk_Config *dynClkConfig;

    dispPtr->state     = DISPLAY_STOPPED;
    dispPtr->vMode     = VMODE_1280x720;
    dispPtr->pxlFreqHz = 0;

    /*-------------------------------------------------------------------
     *  axi_dynclk
     *
     *  CfgInitialize fails in practically only one case :
     *  the IP's 0x20 register (reference clock frequency) reads as 0.
     *  That means AXI-Lite itself is not connected. Check the address map
     *  and the s_axi_lite_aclk / aresetn wiring.
     *-------------------------------------------------------------------*/
    dynClkConfig = DDynClk_LookupConfig(dynClkId);
    if (NULL == dynClkConfig) {
        xil_printf("display : dynclk LookupConfig failed (id %d)\r\n",
                   (int)dynClkId);
        return XST_FAILURE;
    }

    Status = DDynClk_CfgInitialize(&(dispPtr->dynClk), dynClkConfig,
                                   dynClkConfig->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("display : dynclk CfgInitialize failed\r\n");
        xil_printf("          Reference clock register read back as 0.\r\n"
                   "          Check axi_dynclk AXI-Lite wiring and address map.\r\n");
        return XST_FAILURE;
    }

    /*-------------------------------------------------------------------
     *  v_tc
     *-------------------------------------------------------------------*/
    vtcConfig = XVtc_LookupConfig(vtcId);
    if (NULL == vtcConfig) {
        xil_printf("display : VTC LookupConfig failed (id %d)\r\n", (int)vtcId);
        return XST_FAILURE;
    }

    Status = XVtc_CfgInitialize(&(dispPtr->vtc), vtcConfig,
                                vtcConfig->BaseAddress);
    if (Status != XST_SUCCESS) {
        xil_printf("display : VTC CfgInitialize failed\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/*
 *  Change the resolution.
 *
 *  The change really happens in DisplayStart(). In this design the clock
 *  changes too, so another VideoMode works correctly.
 */
int DisplaySetMode(DisplayCtrl *dispPtr, const VideoMode *newMode)
{
    int Status;

    if (dispPtr->state == DISPLAY_RUNNING) {
        Status = DisplayStop(dispPtr);
        if (Status != XST_SUCCESS) {
            xdbg_printf(XDBG_DEBUG_GENERAL,
                        "Cannot change mode, unable to stop display %d\r\n",
                        Status);
            return XST_FAILURE;
        }
    }

    dispPtr->vMode = *newMode;

    return XST_SUCCESS;
}
