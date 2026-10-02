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
#include "ui_task/ui_stream.h"

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

#if UI_NETWORK_ONLY
    if (UiStream_Init() != XST_SUCCESS) return 1;
    xil_printf("[UI] NETWORK-ONLY: UDP telemetry active; ping is not supported.\r\n");
    for (;;) {
        UiStream_Service(0U, 0U, 0U, 0, 0, 0U, 0, 0, 0U);
        usleep(1000U);
    }
#endif

    xil_printf("\r\n\r\n");
    xil_printf("=================================================\r\n");
    xil_printf(" Zybo Z7-20 + Pcam 5C (MIPI CSI-2) + HDMI  720p60\r\n");
    xil_printf(" custom VDMA + capture + BRAM loading screen\r\n");
    xil_printf("=================================================\r\n");
    xil_printf("[STAGE1] system init starting...\r\n");

    /*-------------------------------------------------------------------
     *  1. 占쎈쐻占쎈윥占쎈닖占쎈쐻占쎈윞占쎈�� 占쎌맶 �뙴占� 占쎌쑟占쎌녃域뱀꼶爾� �뤃占� 占쎌맶 占쎌쑅 占쎌쐾 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윪甕곌낑�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윪筌랃옙
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
     *  2. MIPI 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥�눧袁��쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗癲ル슢理먲옙�뮀占쎌젂占쎈ぜ占쎈�嶺뚮ㅏ援�占쎌굲 �젆占� 占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占�  占쎌맶 占쎌쑅 占쎌젛 占쎌맶 占쎌쑅占쎄괌占쎈펱占쎌맶 占쎌쑅嶺뚣끇�굫占쎌굲 �뤃轅⑤쐻占쎈윞占쎈쑆 占쎌맶占쎈쐻   占쎌맶 占쎌쐾占쎈쇊  占쎌맶 占쎌쑅 獄�占� 占쎌맶 占쎌쑋雅��굛�뜙占쎌굲 占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윞�뤃�뼹�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌몡 占쎈덩 占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌쟼占쎈쐻
     *-------------------------------------------------------------------*/
    mipi_rx_reset();
    mipi_rx_print_version();
	xil_printf("[OK] MIPI reset/version\r\n");

    /*-------------------------------------------------------------------
     *  3. SCCB 占쎈쐻占쎈윥占쎄틦 占쎈�占쎈뮝力놂옙沃섓퐢�맶 占쎌쑅嶺뚳옙
     *-------------------------------------------------------------------*/
    if (OV5640_Init() != IIC_SCCB_OK) {
        xil_printf("SCCB bus did not come up. Stopping.\r\n");
        xil_printf("  check : I2C 0 enabled in the PS block and routed to\r\n"
                   "          EMIO, IIC_0 made external, XDC pins F20/F19,\r\n"
                   "          XSA re-exported and the platform updated.\r\n");
        return 1;
    }

    /*-------------------------------------------------------------------
     *  4. 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌맽占쎈쐻占쎈윥筌뚮벩�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌맽占쎈쐻占쎈윥壤쏉옙 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윪甕곌낑�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윪筌랃옙 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌맽占쎈쐻占쎈윥 占쎈씮�굲 占쎈㎍占쎈쐻占쎈윥占쎌몝 占쎈폏吏� 占쎈㎍占쎈쐻占쎈윥占쎌맽嚥싲갭�돘占쎌굲 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윪筌랃옙  占쎌맶 占쎌몘力놂옙 占쎈탶�⑤벡梨� 占쎄뎡 占쎈튉占쎈쐻   占쎌맶 占쎌쑅櫻뗫봾�뀋占쎌맶 占쎌쑅 筌띻퍌�쐻占쎈윥占쎈젗 占쎈�占쎄콬 �뤃占� 占쎌맶 占쎌쑅占쎈き
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
    if (SetupVdmaInterrupts() != XST_SUCCESS) {
        xil_printf("custom VDMA interrupt setup failed. Stopping.\r\n");
        return 1;
    }
	xil_printf("[OK] GIC and custom VDMA IRQ routing\r\n");

    /* PHY negotiation may take seconds. Do it before 10 ms vehicle UART starts. */
    if (UiStream_Init() != XST_SUCCESS)
        xil_printf("[UI] Ethernet unavailable; camera/HDMI/CNN continue.\r\n");


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
     *  6. MIPI 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥�눧袁��쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗癲ル슢理먲옙�뮀占쎌젂占쎈ぜ占쎈�嶺뚮ㅏ援�占쎌굲 �젆占� 占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎌맶 占쎌몘力놂옙占쎄덩占쎌굲 �뤃占�, 占쎈쐻占쎈윪占쎈㎡占쎈쎗 �넭怨ｋ쳴�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌쟼�넭占�  占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈탶�⑤베彛� 占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌맽占쎈쐻占쎈윥筌뚮벩�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌맽占쎈쐻占쎈윥壤쏉옙 占쎌맶 占쎌쑅 椰꾠깷�쐻占쎈윥占쎈㎍ 占쎌맶占쎈쐻  占쎌뒙占쎈뼔揶쏉옙占쎈쐻占쎈뼢占쎄땀占쏙옙 占쎌맶 占쎌쑅 筌띾떱�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌몡 占쎈덩 占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌쟼占쎈쐻
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
     *  7. HDMI  占쎌맶 占쎌쑅 占쎈＞ 占쎌맶 占쎌쑅嶺뚋삳꺽占쎌맶 占쎌쑋�댖占�
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
     *  8. Vehicle stack: existing vehicle.c / vehicle.h
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
    xil_printf("[OK] vehicle.c initialized\r\n");
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
     *  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈눇�뙼怨대춱 占쎈㎍占쎈쐻占쎈윥占쎌맽癲ル슢�뿪占쎌굲 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윞�뙼臾딆녇占쎄틓占쎈뮦 筌랃옙 占쎌맶 �뙴占� 占쎄뎡占쎈쐪筌먦룂�굲 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝 占쎈폏吏� 占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윪鈺곤옙 8-1占쎈쐻占쎈윥占쎄틦 占쎈�占쎄콬 �뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝 占쎈릉占쎈쾳占쎈㎍占쎈쐻占쎈윥占쎌맽占쎈쐻占쎈윥占쎌끍占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윥 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쎗 占쎈쐻  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥筌뚮뿥�쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗 筌뚳옙 占쎈け 占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈쐻占쎈윥筌ｋ퀫�쐻占쎈윥鸚롤껊쐻占쎈윥筌앸ŀ�쐻占쎈윪�뤃占�  占쎌맶 占쎌몘嶺뚮쪇�뒻占쎌굲力놂옙沃섓퐢�맶 占쎌쐾 占쎈폀 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝占쎈탶野껉쑬踰됵옙留띰옙�쐻占쎈윥占쎌몝占쎈쐻占쎈윪雅뚮뀘�쐻占쎈윪占쎈�뉛옙�쐺獄�袁る쭟 占쎄뎡�슖�떜媛��슙  占쎈쐻占쎈윪�뤃占� 占쎌맶 占쎌쑅 占쎈쨦占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌쟼占쎈쐻 .
     *  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈옘占쎈쐻占쎈윥占쏙옙��遺얘턁筌�硫⑤쐻占쎈짗占쎈쭋 �뜮�빆�쐻占쎈윪占쎈��占쎈쐻占쎈윪�뤃占� 占쎌맶 占쎌쑅嶺뚳옙  占쎌녇占쎄틓占쎈뮝 占쎈＋ 占쎌��솻洹섎쳴占쎄뎡 占쎌맶占쎈쐻  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐻占쎈윥占쎈폆 占쎌녇占쎄틓占쎈뮛 �억옙癲ル슢�뵞�뤃占� 占쎄뎡 �뤃占� 占쏙옙 占쎈㎍占쎈쐻占쎈윥占쎌몗占쎈쐪筌먦룂�굲 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몝 占쎈섣占쎌굲(UART 占쎌녇占쎄틓占쎈뮛 占쎈뼌占쎈쐻占쎈짗占쎌굲 占쎌맶 占쎌쑅占쎈쐻 )占쎌녇占쎄틓占쎈뮛 �굢�굝�쐻占쎈윪�뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌맶 占쎌몧 占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌몡 占쎈덩 占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌쟼占쎈쐻  占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� 占쎈쐻占쎈윪占쎈㎡占쎈쎗 占쎈쐻占쎈쓠筌욑옙 占쎈즽 占쎄뎡占쎈쐻占쎈윥 占쎌뵛占쎌굲 占쎈㎍占쎈쐻占쎈윥筌욎�λ쐻占쎈윪�뤃占� VDMA
     *  占쎈쐻占쎈윥占쎄틦 占쎈�占쎈뮝力놂옙沃섓퐢�맶 占쎌쑋 甕곤옙 占쎌맶 占쎌몘力놂옙占쎄덩占쎌굲 �뤃占� 占쎈쐻占쎈윥占쎈㎍占쎈쐻占쎈윥占쎌몗癲ル슣�돵 縕ワ옙留띰옙�쐻占쎈윥占쎌몝 占쎈리�뙼�봿留띰옙�쐻占쎈윥占쎌몗 占쎌맶 占쎌몧 占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌몡 占쎈덩 占쎈㎍占쎈쐻占쎈윥占쎌몗 占쎌쟼占쎈쐻 .
     *-------------------------------------------------------------------*/
    XTime_GetTime(&diagnostics_start);
    for (;;) {

        VehicleControlPoll();

        VdmaLogPoll();

    #if ENABLE_CAMERA_CONSOLE_MENU
        menu_run();
    #endif

    #if STEERING_SOURCE_CNN
        CnnAutoCapturePoll();
    #endif

        CapturePoll();

        CnnPoll();

        ButtonPoll();
        Ja1GpioPrintPoll();
        Ja2GpioPrintPoll();
        Ja3GpioPrintPoll();

        RomPreviewPoll();

        UiApplicationService();

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
