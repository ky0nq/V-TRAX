/*
 *  main.c
 *
 *  Zybo Z7-20 + Pcam 5C (OV5640, MIPI CSI-2) -> DDR -> HDMI, 720p60.
 *
 *  This file now ONLY does two things, matching the layered SW stack:
 *    1. the one-time bring-up sequence (Application layer, "init"), and
 *    2. the super-loop that calls each *_task module once per pass.
 *
 *  Every register access (Capture/CNN/Button/BBOX/JA1-GPIO/video DMA) now
 *  lives in its own driver/ folder; every piece of "what to do
 *  periodically" logic now lives in its own *_task/ folder. main.c only
 *  wires them together.
 *
 *  Terminal : 115200 8N1.
 */

#include "sleep.h"
#include "xil_types.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xstatus.h"
#include "xtime_l.h"

#include "driver/cam_gpio/cam_gpio.h"
#include "driver/iic_sccb_cfg/iic_sccb_cfg.h"
#include "driver/ov5640/OV5640.h"
#include "driver/mipi_rx/mipi_rx.h"
#include "driver/gamma/gamma.h"
#include "driver/cam_ae/cam_ae.h"
#include "driver/dma_driver/dma_driver.h"          /* custom VDMA register driver */
#include "driver/hdmi_display_driver/display_ctrl.h"
#include "driver/gpio_driver/gpio_driver.h"                /* JA1 external-pin driver */
#include "driver/vehicle_driver/vehicle.h"

/* Driver layer */
#include "driver/video_pipeline_driver/video_pipeline_driver.h"
#include "driver/bbox_driver/bbox_driver.h"

/* Application layer (task modules) */
#include "application/capture_cnn_ctrl/capture_cnn_ctrl.h"
#include "application/button_input/button_input.h"
#include "application/display_mode/display_mode.h"
#include "application/camera_profile/camera_profile.h"
#include "application/console_menu/console_menu.h"
#include "application/vehicle_ctrl/vehicle_ctrl.h"
#include "application/vehicle_ctrl/reverse_mode.h"
#include "application/ui_telemetry/ui_telemetry.h"

#define VDMA_BASEADDR   XPAR_DMA_0_BASEADDR
#define DISP_VTC_ID     XPAR_VTC_0_DEVICE_ID
#define DISP_DYNCLK_ID  XPAR_DYNCLK_0_DEVICE_ID

#define BRAM_LOADING_ADDR       XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#define LOADING_SCREEN_SECONDS  5U

DisplayCtrl  dispCtrl;
VideoMode    vd_mode;

int main(void)
{
	xil_printf("START");
    u16 w, h;
    XTime diagnostics_start, diagnostics_now;
    XTime loading_start, loading_now;
    int diagnostics_done = 0;


    xil_printf("\r\n\r\n");
    xil_printf("=================================================\r\n");
    xil_printf(" Zybo Z7-20 + Pcam 5C (MIPI CSI-2) + HDMI  720p60\r\n");
    xil_printf(" custom VDMA + capture + BRAM loading screen\r\n");
    xil_printf("=================================================\r\n");
    xil_printf("[STAGE1] system init starting...\r\n");

    /*-------------------------------------------------------------------
     *  1. Turn on the camera power pin (EMIO GPIO).
     *-------------------------------------------------------------------*/
    if (cam_gpio_init() != CAM_GPIO_OK) {
        xil_printf("EMIO GPIO did not come up. Stopping.\r\n");
        xil_printf("  check : PS block has EMIO GPIO enabled with Width=1,\r\n"
                   "          GPIO_0 made external, XDC pin G20 with PULLUP,\r\n"
                   "          XSA re-exported and the platform updated.\r\n");
        return 1;
    }
	xil_printf("[OK] camera GPIO\r\n");

    /*-------------------------------------------------------------------
     *  2. Reset the MIPI receiver cores and print their versions.
     *-------------------------------------------------------------------*/
    mipi_rx_reset();
    mipi_rx_print_version();
	xil_printf("[OK] MIPI reset/version\r\n");

    /*-------------------------------------------------------------------
     *  3. Start the SCCB (I2C) bus to the camera.
     *-------------------------------------------------------------------*/
    if (OV5640_Init() != IIC_SCCB_OK) {
        xil_printf("SCCB bus did not come up. Stopping.\r\n");
        xil_printf("  check : I2C 0 enabled in the PS block and routed to\r\n"
                   "          EMIO, IIC_0 made external, XDC pins F20/F19,\r\n"
                   "          XSA re-exported and the platform updated.\r\n");
        return 1;
    }

    /*-------------------------------------------------------------------
     *  4. Turn the camera sensor off and on, then detect and set it up.
     *-------------------------------------------------------------------*/
    OV5640_PowerCycle();

    if (OV5640_InitSensor() != 0) {
        xil_printf("OV5640 detected failed!\r\n");
        xil_printf("  check : Pcam flat cable fully latched (press the\r\n"
                   "          connector down with two fingers), 5V external\r\n"
                   "          supply with JP6 on WALL, and the F20/F19 pins\r\n");
        return 1;
    }
    xil_printf("OV5640 detected successful!\r\n");
	xil_printf("[OK] OV5640 detect/init\r\n");

    if (iic_sccb_error_count()) {
        xil_printf("WARNING : %d SCCB error(s) during init sequence\r\n",
                   (int)iic_sccb_error_count());
        iic_sccb_clear_errors();
    }

    /*-------------------------------------------------------------------
     *  4-1. Configure the custom VDMA, then arm its two level IRQs.
     *-------------------------------------------------------------------*/
    vd_mode = VMODE_1280x720;

    if (vdma_configure(&vdma, VDMA_BASEADDR, FRAME_SIZE_BYTES,
                       frame_buffer_addresses, NUM_FRAME_BUFFERS,
                       BRAM_LOADING_ADDR) != XST_SUCCESS) {
        xil_printf("custom VDMA register setup failed. Stopping.\r\n");
        return 1;
    }
	xil_printf("[OK] custom VDMA registers/frame buffers configured\r\n");

	if (BboxInitialize() != XST_SUCCESS) {
		xil_printf("BBOX setup failed. Stopping.\r\n");
		return 1;
	}

    gpio_ja1_init_input();
    gpio_ja2_init_input();
    gpio_ja3_init_input();
    gpio_ja1_irq_clear();
    gpio_ja2_irq_clear();
    gpio_ja3_irq_clear();

    AdditionalFeatureInit();

    xil_printf(
        "[OK] JA2 additional feature initialized: mode=%s\r\n",
        AdditionalFeatureModeName()
    );

    if (SetupVdmaInterrupts() != XST_SUCCESS) {
        xil_printf("custom VDMA interrupt setup failed. Stopping.\r\n");
        return 1;
    }
	xil_printf("[OK] GIC and custom VDMA/CNN/Capture/Timer IRQ routing\r\n");

    xil_printf("[UI] HDMI capture + UART HUD4 telemetry\r\n");

    /*-------------------------------------------------------------------
     *  5. Start S2MM before the sensor, and MM2S on the BRAM image.
     *-------------------------------------------------------------------*/
    if (vdma_start(&vdma) != XST_SUCCESS) {
        xil_printf("custom VDMA start failed. Stopping.\r\n");
        return 1;
    }
	xil_printf("[OK] custom VDMA S2MM/MM2S started\r\n");

    xil_printf("[OK] loading-screen MM2S fixed source at %08X\r\n",
               (unsigned)BRAM_LOADING_ADDR);

    /*-------------------------------------------------------------------
     *  6. Enable MIPI, then start the camera stream (720p, AWB, AE).
     *-------------------------------------------------------------------*/
    mipi_rx_enable();
	xil_printf("[OK] MIPI enable\r\n");

    gamma_init();
    ButtonInit();
    OV5640_SetMode720p();
    OV5640_SetAWB(AWB_ADVANCED);
    cam_ae_set(AE_LEVEL_P2);
	xil_printf("[OK] sensor stream/AE setup\r\n");

    /*-------------------------------------------------------------------
     *  7. Start the HDMI output (pixel clock + video timing).
     *-------------------------------------------------------------------*/
    if (DisplayInitialize(&dispCtrl, DISP_VTC_ID, DISP_DYNCLK_ID)
            != XST_SUCCESS) {
        xil_printf("HDMI output init failed. Stopping.\r\n");
        return 1;
    }
	xil_printf("[OK] display drivers\r\n");
    DisplaySetMode(&dispCtrl, &vd_mode);
    if (DisplayStart(&dispCtrl) != XST_SUCCESS) {
        xil_printf("Pixel clock generation failed. Stopping.\r\n");
        return 1;
    }
	xil_printf("[OK] pixel clock/VTC\r\n");

    OV5640_GetImageInfo(&w, &h);
    xil_printf("camera  : %dx%d RAW10, 2 lane MIPI\r\n", w, h);
    xil_printf("display : %dx%d pixel clock %d Hz (from axi_dynclk)\r\n",
               vd_mode.width, vd_mode.height, (int)dispCtrl.pxlFreqHz);
    xil_printf("frame buffers at %08X, %08X, %08X\r\n",
               (unsigned)frame_buffer_addresses[0],
               (unsigned)frame_buffer_addresses[1],
               (unsigned)frame_buffer_addresses[2]);

    /*-------------------------------------------------------------------
     *   * 8. Vehicle stack:
     *    vehicle_driver.c + vehicle_task.c + vehicle.h
     *
     *  PS UART0 RX : sensor/ESP32 -> Zybo (MIO14 / JF9)
     *  PS UART0 TX : Zybo -> vehicle/ESP32 (MIO15 / JF10)
     *  PS UART1    : PC steering / stdout as defined by vehicle.c/BSP
     *-------------------------------------------------------------------*/
    if (vehicleInit() != XST_SUCCESS) {
        xil_printf("VEHICLE INIT FAILED\r\n");
        return 1;
    }
    vehicle_control_ready = 1;
    VehicleControlPoll();
    xil_printf("[OK] vehicle stack initialized\r\n");
    xil_printf("Sensor RX  : PS UART0 RX / MIO14 / JF9\r\n");
    xil_printf("Vehicle TX : PS UART0 TX / MIO15 / JF10\r\n");
    xil_printf("[STAGE1] system init complete.\r\n");
    xil_printf("VDMA counters: S2MM=%lu valid=%lu MM2S=%lu\r\n",
			(unsigned long)s2mm_done_count,
			(unsigned long)s2mm_valid_count,
			(unsigned long)mm2s_done_count);

    xil_printf("\r\n>>> [STAGE2] LOADING SCREEN ACTIVE (%u seconds) <<<\r\n",
               (unsigned)LOADING_SCREEN_SECONDS);
    XTime_GetTime(&loading_start);
    do {
        VehicleControlPoll();
		VdmaLogPoll();
        Ja1GpioPrintPoll();
        Ja2GpioPrintPoll();
        Ja3GpioPrintPoll();
        XTime_GetTime(&loading_now);
        if (loading_now - loading_start >=
            (XTime)LOADING_SCREEN_SECONDS * COUNTS_PER_SECOND)
            break;
        usleep(1000U);
    } while (1);
    xil_printf("[STAGE2] done. S2MM(cam->ddr)=%lu valid=%lu err=%lu "
               "MM2S(loading)=%lu SCCBerr=%lu\r\n",
               (unsigned long)s2mm_done_count,
			   (unsigned long)s2mm_valid_count,
               (unsigned long)s2mm_error_count,
               (unsigned long)mm2s_done_count,
               (unsigned long)iic_sccb_error_count());

    if (SwitchDisplayToLive() == XST_SUCCESS) {
        xil_printf(">>> [STAGE3] LIVE CAMERA STREAM ACTIVE <<<\r\n\r\n");
    } else {
        xil_printf(">>> [STAGE3] LIVE SWITCH FAILED; press 'd' for status <<<\r\n\r\n");
    }



    /*-------------------------------------------------------------------
     *  9. Super-loop
     *
     *  Each pass of this loop calls every task's Poll() function once.
     *-------------------------------------------------------------------*/
    XTime_GetTime(&diagnostics_start);
    for (;;) {
        // JA1 = ARM / STOP
        VehicleJa1Poll();
        // JA2 = NORMAL / REVERSE
        if (AdditionalFeaturePoll()) {
            xil_printf(
                "DRIVE MODE: %s\r\n",
                AdditionalFeatureModeName()
            );
        }
        // Make the real vehicle command
        VehicleControlPoll();

        VdmaLogPoll();

    #if ENABLE_CAMERA_CONSOLE_MENU
        menu_run();
    #endif

    #if STEERING_SOURCE_CNN
        CaptureTimerPoll();
    #endif

        CapturePoll();

        CnnPoll();

        ButtonPoll();
        Ja1GpioPrintPoll();
        Ja2GpioPrintPoll();
        Ja3GpioPrintPoll();

        RomPreviewPoll();

        UiApplicationService();

        VehicleJa1Poll();

        if (AdditionalFeaturePoll()) {
            xil_printf(
                "DRIVE MODE: %s\r\n",
                AdditionalFeatureModeName()
            );
        }

        VehicleControlPoll();

        if (!diagnostics_done) {
            XTime_GetTime(&diagnostics_now);

            if (diagnostics_now - diagnostics_start >=
                    2ULL * COUNTS_PER_SECOND) {

                diagnostics_done = 1;

                if (mm2s_done_count == 0) {
                    xil_printf(
                        "VDMA MM2S has no completions after 2 seconds\r\n"
                    );

                    PrintVdmaDiagnostics();
                }
            }
        }

        usleep(1000U);
    }
    /* not reached */
}
