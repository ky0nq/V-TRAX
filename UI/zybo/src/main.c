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

#include "cam_gpio/cam_gpio.h"
#include "iic_sccb_cfg/iic_sccb_cfg.h"
#include "ov5640/OV5640.h"
#include "mipi_rx/mipi_rx.h"
#include "gamma/gamma.h"
#include "cam_ae/cam_ae.h"
#include "dma_api/dma_api.h"          /* custom VDMA register driver */
#include "display_ctrl_hdmi/display_ctrl.h"
#include "gpio/gpio.h"                /* JA1 external-pin driver */
#include "vehicle/vehicle.h"

/* Driver layer */
#include "video_pipeline_driver/video_pipeline_driver.h"
#include "bbox_driver/bbox_driver.h"

/* Application layer (task modules) */
#include "capture_task/capture_task.h"
#include "button_task/button_task.h"
#include "display_task/display_task.h"
#include "camera_profile_task/camera_profile_task.h"
#include "console_menu/console_menu.h"
#include "vehicle_task/vehicle_task.h"
#include "vehicle/additionalfeature.h"
#include "ui_task/ui_task.h"

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
     *  1. �뜝�럥�맶�뜝�럥�쑅�뜝�럥�떀�뜝�럥�맶�뜝�럥�쐾�뜝�럥占쏙옙 �뜝�럩留� 占쎈쇀�뜝占� �뜝�럩�몷�뜝�럩�뀇�윜諭�瑗띄댚占� 占쎈쨨�뜝占� �뜝�럩留� �뜝�럩�몗 �뜝�럩�맽 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�뵓怨뚮굫占쎌맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋嶺뚮엪�삕
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
     *  2. MIPI �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈닱熬곻옙占쎌맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀧솾�꺂�뒧筌ㅻ㉡�삕占쎈��뜝�럩�쟼�뜝�럥�걶�뜝�럥占쏙┼�슢�뀖�뤃占썲뜝�럩援� 占쎌젂�뜝占� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅嶺뚯쉸占싸살맶�뜝�럥�쑋占쎈쨨�뜝占�  �뜝�럩留� �뜝�럩�몗 �뜝�럩�젢 �뜝�럩留� �뜝�럩�몗�뜝�럡愿뚦뜝�럥�렠�뜝�럩留� �뜝�럩�몗癲ル슔�걞占쎄뎃�뜝�럩援� 占쎈쨨饔끸뫀�맶�뜝�럥�쐾�뜝�럥�몘 �뜝�럩留뜹뜝�럥�맶   �뜝�럩留� �뜝�럩�맽�뜝�럥�뇢  �뜝�럩留� �뜝�럩�몗 �뛾占썲뜝占� �뜝�럩留� �뜝�럩�몝�썒占쏙옙援쏉옙�쐷�뜝�럩援� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅�뜝�럥占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쐾占쎈쨨占쎈섰占쎌맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩紐� �뜝�럥�뜦 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩�읆�뜝�럥�맶
     *-------------------------------------------------------------------*/
    mipi_rx_reset();
    mipi_rx_print_version();
	xil_printf("[OK] MIPI reset/version\r\n");

    /*-------------------------------------------------------------------
     *  3. SCCB �뜝�럥�맶�뜝�럥�쑅�뜝�럡�떐 �뜝�럥占썲뜝�럥裕앾쫲�냲�삕亦껋꼻�맊占쎈㎍ �뜝�럩�몗癲ル슪�삕
     *-------------------------------------------------------------------*/
    if (OV5640_Init() != IIC_SCCB_OK) {
        xil_printf("SCCB bus did not come up. Stopping.\r\n");
        xil_printf("  check : I2C 0 enabled in the PS block and routed to\r\n"
                   "          EMIO, IIC_0 made external, XDC pins F20/F19,\r\n"
                   "          XSA re-exported and the platform updated.\r\n");
        return 1;
    }

    /*-------------------------------------------------------------------
     *  4. �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅嶺뚮슢踰⑼옙�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅鶯ㅼ룊�삕 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�뵓怨뚮굫占쎌맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋嶺뚮엪�삕 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅 �뜝�럥�뵰占쎄뎡 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럥�룒筌욑옙 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲슖�떜媛�占쎈룜�뜝�럩援� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋嶺뚮엪�삕  �뜝�럩留� �뜝�럩紐섓쫲�냲�삕 �뜝�럥�꺐占썩뫀踰∽㎖占� �뜝�럡�렊 �뜝�럥�뒌�뜝�럥�맶   �뜝�럩留� �뜝�럩�몗輿삳뿫遊억옙�뗥뜝�럩留� �뜝�럩�몗 嶺뚮씧�뜉占쎌맶�뜝�럥�쑅�뜝�럥�젛 �뜝�럥占썲뜝�럡肄� 占쎈쨨�뜝占� �뜝�럩留� �뜝�럩�몗�뜝�럥�걤
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
     *  6. MIPI �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈닱熬곻옙占쎌맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀧솾�꺂�뒧筌ㅻ㉡�삕占쎈��뜝�럩�쟼�뜝�럥�걶�뜝�럥占쏙┼�슢�뀖�뤃占썲뜝�럩援� 占쎌젂�뜝占� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅嶺뚯쉸占싸살맶�뜝�럥�쑋占쎈쨨�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅嶺뚯쉸占싸살맶�뜝�럥�쑋占쎈쨨�뜝占� �뜝�럩留� �뜝�럩紐섓쫲�냲�삕�뜝�럡�뜦�뜝�럩援� 占쎈쨨�뜝占�, �뜝�럥�맶�뜝�럥�쑋�뜝�럥�렊�뜝�럥�럸 占쎈꽠�⑨퐢爾댐옙琉껃뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩�읆占쎈꽠�뜝占�  �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�꺐占썩뫀踰졾퐲占� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅占쎈쨨�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅嶺뚮슢踰⑼옙�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅鶯ㅼ룊�삕 �뜝�럩留� �뜝�럩�몗 濾곌풝源뤄옙�맶�뜝�럥�쑅�뜝�럥�럪 �뜝�럩留뜹뜝�럥�맶  �뜝�럩�뮋�뜝�럥堉붹뤆�룊�삕�뜝�럥�맶�뜝�럥堉℡뜝�럡���뜝�룞�삕 �뜝�럩留� �뜝�럩�몗 嶺뚮씭�뼮占쎌맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩紐� �뜝�럥�뜦 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩�읆�뜝�럥�맶
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
     *  7. HDMI  �뜝�럩留� �뜝�럩�몗 �뜝�럥竊� �뜝�럩留� �뜝�럩�몗癲ル쉵�궠爰썲뜝�럩留� �뜝�럩�몝占쎈뙑�뜝占�
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
     *  �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�늾占쎈쇊�⑤�異� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썹솾�꺂�뒧占쎈역�뜝�럩援� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쐾占쎈쇊�눧�봿�뀋�뜝�럡�땽�뜝�럥裕� 嶺뚮엪�삕 �뜝�럩留� 占쎈쇀�뜝占� �뜝�럡�렊�뜝�럥�맚嶺뚮Ĳ猷귨옙援� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럥�룒筌욑옙 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�댖怨ㅼ삕 8-1�뜝�럥�맶�뜝�럥�쑅�뜝�럡�떐 �뜝�럥占썲뜝�럡肄� 占쎈쨨�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럥由됧뜝�럥苡녑뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩留썲뜝�럥�맶�뜝�럥�쑅�뜝�럩�걤�뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑅 �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�럸 �뜝�럥�맶  �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅嶺뚮슢肉ο옙�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� 嶺뚮슪�삕 �뜝�럥�걨 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑅嶺뚳퐢�ワ옙�맶�뜝�럥�쑅勇싲·猿딆맶�뜝�럥�쑅嶺뚯빖�占쎌맶�뜝�럥�쑋占쎈쨨�뜝占�  �뜝�럩留� �뜝�럩紐섓┼�슢履뉛옙�뮲�뜝�럩援뀐쫲�냲�삕亦껋꼻�맊占쎈㎍ �뜝�럩�맽 �뜝�럥�� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�꺐�뇦猿됱뫊甕곕맮�삕筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐앭뜝�럥�맶�뜝�럥�쑋�썒�슢�섓옙�맶�뜝�럥�쑋�뜝�럥占쎈돍�삕占쎌맳�뛾占썼쥈�굥彛� �뜝�럡�렊占쎌뒙占쎈뼔揶쏉옙占쎌뒜  �뜝�럥�맶�뜝�럥�쑋占쎈쨨�뜝占� �뜝�럩留� �뜝�럩�몗 �뜝�럥夷��뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩�읆�뜝�럥�맶 .
     *  �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅�뜝�럥�삓�뜝�럥�맶�뜝�럥�쑅�뜝�룞�삕占쏙옙�겫�뼐�꼤嶺뚳옙筌롡뫀�맶�뜝�럥吏쀥뜝�럥彛� 占쎈쑏占쎈퉮占쎌맶�뜝�럥�쑋�뜝�럥占쏙옙�뜝�럥�맶�뜝�럥�쑋占쎈쨨�뜝占� �뜝�럩留� �뜝�럩�몗癲ル슪�삕  �뜝�럩�뀋�뜝�럡�땽�뜝�럥裕� �뜝�럥竊� �뜝�럩占쏙옙�녃域뱀꼶爾닷뜝�럡�렊 �뜝�럩留뜹뜝�럥�맶  �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맶�뜝�럥�쑅�뜝�럥�룇 �뜝�럩�뀋�뜝�럡�땽�뜝�럥裕� 占쎌뼲�삕�솾�꺂�뒧占쎈턂占쎈쨨�뜝占� �뜝�럡�렊 占쎈쨨�뜝占� �뜝�룞�삕 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀥뜝�럥�맚嶺뚮Ĳ猷귨옙援� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럥�꽔�뜝�럩援�(UART �뜝�럩�뀋�뜝�럡�땽�뜝�럥裕� �뜝�럥堉뚦뜝�럥�맶�뜝�럥吏쀥뜝�럩援� �뜝�럩留� �뜝�럩�몗�뜝�럥�맶 )�뜝�럩�뀋�뜝�럡�땽�뜝�럥裕� 占쎄덩占쎄턁占쎌맶�뜝�럥�쑋占쎈쨨�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩留� �뜝�럩紐� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩紐� �뜝�럥�뜦 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩�읆�뜝�럥�맶  �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅嶺뚯쉸占싸살맶�뜝�럥�쑋占쎈쨨�뜝占� �뜝�럥�맶�뜝�럥�쑋�뜝�럥�렊�뜝�럥�럸 �뜝�럥�맶�뜝�럥�뱺嶺뚯쉻�삕 �뜝�럥利� �뜝�럡�렊�뜝�럥�맶�뜝�럥�쑅 �뜝�럩逾쎾뜝�럩援� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅嶺뚯쉸占싸살맶�뜝�럥�쑋占쎈쨨�뜝占� VDMA
     *  �뜝�럥�맶�뜝�럥�쑅�뜝�럡�떐 �뜝�럥占썲뜝�럥裕앾쫲�냲�삕亦껋꼻�맊占쎈㎍ �뜝�럩�몝 �뵓怨ㅼ삕 �뜝�럩留� �뜝�럩紐섓쫲�냲�삕�뜝�럡�뜦�뜝�럩援� 占쎈쨨�뜝占� �뜝�럥�맶�뜝�럥�쑅�뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐쀧솾�꺂�뒩占쎈뤅 潁뺛꺈�삕筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐� �뜝�럥由э옙�쇊占쎈늉筌띾씛�삕占쎌맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩留� �뜝�럩紐� �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩紐� �뜝�럥�뜦 �뜝�럥�럪�뜝�럥�맶�뜝�럥�쑅�뜝�럩紐� �뜝�럩�읆�뜝�럥�맶 .
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
        // 실제 차량 command 생성
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
