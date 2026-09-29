
/*
 *  main.c
 *
 *  Zybo Z7-20 + Pcam 5C (OV5640, MIPI CSI-2) -> DDR -> HDMI, 720p60.
 *  The PL APB UART also sends vehicle state packets every 10 ms.
 *  珥덇린�솕 �쟾�슜 理쒖냼 踰꾩쟾 (custom VDMA 踰꾩쟾).
 *
 *  ---------------------------------------------------------------------
 *  �씠 �뙆�씪�씠 �븯�뒗 �씪
 *
 *  �쟾�썝�씠 �뱾�뼱�삩 �뮘 "移대찓�씪 �쁺�긽�씠 HDMI濡� �굹�삤�뒗 �긽�깭"源뚯� 珥덇린�솕�븯怨�,
 *  �씠�썑�뿉�뒗 UART 硫붾돱 泥섎━ �젙�룄留� �빀�땲�떎. �븘�꽣 �넗湲� 媛숈� 媛�踰쇱슫 嫄�
 *  �뿬湲곗꽌 �븯吏�留�, "�쁺�긽�씠 怨꾩냽 �쓲瑜닿쾶 �쑀吏��븯�뒗 �씪"�� �븘�옒 8-1踰�
 *  �씤�꽣�읇�듃 �빖�뱾�윭媛� �떞�떦�빀�땲�떎.
 *
 *  custom VDMA�뒗 S2MM �듃由ы뵆 踰꾪띁�� MM2S 理쒖떊 �봽�젅�엫 �꽑�깮�쓣 �븯�뱶�썾�뼱�뿉�꽌
 *  �옄�룞 泥섎━�빀�땲�떎. CPU�뒗 AXI-Lite �젅吏��뒪�꽣 珥덇린�솕, 紐⑤뱶 �쟾�솚, �긽�깭/IRQ
 *  �솗�씤留� �떞�떦�빀�땲�떎.
 *
 *  湲곕뒫�쓣 遺숈씪 �븣�뒗 �븘�옒 以� �븯�굹�뿉 �꽔寃� �맗�땲�떎.
 *      - 珥덇린�솕 �닚�꽌�뿉 �븳 以� 異붽�          (main �븞, 8踰� �븵)
 *      - 留� �봽�젅�엫 �셿猷뚮쭏�떎 �빐�빞 �븷 �씪       (S2MM/MM2S �씤�꽣�읇�듃 �빖�뱾�윭 �븞)
 *      - 二쇨린�쟻�쑝濡�(鍮꾨룞湲곕줈) �빐�빞 �븷 �씪     (9踰� super-loop �븞)
 *
 *  ---------------------------------------------------------------------
 *  THE ONE THING THAT IS GENUINELY DIFFERENT : THE BRING-UP ORDER
 *
 *  (�씠 遺�遺꾩� DMA 醫낅쪟�� �긽愿��뾾�씠 �룞�씪�빀�땲�떎 �� MIPI CSI-2 釉뚮쭅�뾽 �닚�꽌 臾몄젣)
 *
 *      由ъ뀑   : �냼鍮꾩옄 -> �깮�궛�옄     (VDMA, CSI-2, D-PHY, �꽱�꽌)
 *      �꽕�젙   : �쟾遺�. �븘吏� 硫덉떠 �엳�뒗 �긽�깭濡�
 *      湲곕룞   : �깮�궛�옄 -> �냼鍮꾩옄     (D-PHY, CSI-2, ... , �꽱�꽌媛� 留덉�留�)
 *
 *  �꽱�꽌瑜� 源⑥슦�뒗 OV5640_SetMode720p() 媛� 媛��옣 留덉�留됱엯�땲�떎. 洹� �떆�젏�뿉�뒗
 *  �븯瑜� �떒怨꾧� �쟾遺� �궡�븘�꽌 湲곕떎由ш퀬 �엳�뒿�땲�떎.
 *
 *  Terminal : 115200 8N1.
 */

#include "sleep.h"
#include "xil_types.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xil_io.h"

#include "xscugic.h"

#include "cam_gpio/cam_gpio.h"
#include "iic_sccb_cfg/iic_sccb_cfg.h"
#include "ov5640/OV5640.h"
#include "mipi_rx/mipi_rx.h"
#include "gamma/gamma.h"
#include "cam_ae/cam_ae.h"
#include "mcdma_api/mcdma_api.h"       /* custom VDMA register driver */
#include "display_ctrl_hdmi/display_ctrl.h"
#include "BBOX.h"

#include "xuartps_hw.h"
#include "xtime_l.h"
#include "apb_uart_driver.h"
#include "ui_stream.h"

/*===========================================================================
 *  Platform glue
 *===========================================================================*/
#define VDMA_BASEADDR   XPAR_DMA_0_BASEADDR
#define DISP_VTC_ID     XPAR_VTC_0_DEVICE_ID
#define DISP_DYNCLK_ID  XPAR_DYNCLK_0_DEVICE_ID

/* Use the AXI BRAM address from the regenerated platform BSP. */
#define BRAM_LOADING_ADDR       XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#define LOADING_SCREEN_SECONDS  5U
#define ROM_PREVIEW_SECONDS     3U

#define VEHICLE_TX_PERIOD_TICKS (COUNTS_PER_SECOND / 100U) /* 10 ms */
#define VEHICLE_FLAG_ESTOP      0x01U
#define VEHICLE_STEER_MAX       90
#define VEHICLE_LEVEL_MAX       5U
#define COLOR_PROFILE_COUNT     10U
#define COLOR_PROFILE_DEFAULT   2U
#define CAMERA_HISTORY_DEPTH    16U

/*
 * DDR �씪遺� �쁺�뿭 怨좎옣 �뿬遺�瑜� 遺꾨━�빐�꽌 �솗�씤�븷 �닔 �엳�룄濡� �꽭 �봽�젅�엫�쓣
 * 16 MiB 媛꾧꺽�쓽 �룆由� 二쇱냼�뿉 �몦�떎. 媛� 720p RGB888 �봽�젅�엫�� 0x2A3000
 * 諛붿씠�듃�씠誘�濡� �떎�쓬 踰꾪띁�� 寃뱀튂吏� �븡�뒗�떎.
 */
#define FRAME_WIDTH         1280
#define FRAME_HEIGHT        720
#define BYTES_PER_PIXEL     3
#define FRAME_SIZE_BYTES    (FRAME_WIDTH * FRAME_HEIGHT * BYTES_PER_PIXEL)
#define NUM_FRAME_BUFFERS   3   /* VDMA �븣�� �룞�씪�븯寃� �듃由ы뵆踰꾪띁 �쑀吏� */

#define FRAME_BUFFER_0_ADDR ((UINTPTR)0x02000000U)
#define FRAME_BUFFER_1_ADDR ((UINTPTR)0x03000000U)
#define FRAME_BUFFER_2_ADDR ((UINTPTR)0x04000000U)

static const UINTPTR frame_buffer_addresses[NUM_FRAME_BUFFERS] = {
	FRAME_BUFFER_0_ADDR,
	FRAME_BUFFER_1_ADDR,
	FRAME_BUFFER_2_ADDR
};

/* CAPTURE_AXI_Lite_0: generated xparameters.h and AXI-Lite RTL. */
#define CAPTURE_BASEADDR    XPAR_CAPTURE_AXI_LITE_0_S00_AXI_BASEADDR
#define CAPTURE_CTRL_OFFSET 0x00U
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
#define CAPTURE_CROP_X      000U
#define CAPTURE_CROP_Y      464U

/* BBOX_0 overlays the fixed 256x256 capture ROI on the HDMI stream. */
#define BBOX_BASEADDR       XPAR_BBOX_0_S00_AXI_BASEADDR
#define BBOX_ENABLE         1U
#define BBOX_COLOR_RED      0x00FF0000U

/* IP_CNN_0: consumes the 64x64 CAPTURE RAM and returns one signed INT8 angle. */
#define CNN_BASEADDR             XPAR_IP_CNN_0_BASEADDR
#define CNN_CTRL_OFFSET          0x00U
#define CNN_STATUS_OFFSET        0x04U
#define CNN_RESULT_OFFSET        0x08U
#define CNN_CTRL_START           0x01U
#define CNN_CTRL_IRQ_CLEAR       0x02U
#define CNN_CTRL_IRQ_ENABLE      0x04U
#define CNN_STATUS_BUSY          0x01U
#define CNN_STATUS_START_READY   0x02U
#define CNN_STATUS_DONE          0x04U
#define CNN_STATUS_START_PENDING 0x08U
#define CNN_TIMEOUT_SECONDS      5U

/* AXI GPIO channel 1 is connected to the four active-high board buttons. */
#define BUTTON_GPIO_BASEADDR     XPAR_AXI_GPIO_0_BASEADDR
#define BUTTON_GPIO_DATA_OFFSET  0x00U
#define BUTTON_MASK              0x0FU
#define BUTTON_CAPTURE_MASK      0x01U /* BTN0 */
#define BUTTON_CNN_RESULT_MASK   0x02U /* BTN1 */
#define BUTTON_DEBOUNCE_TICKS    (COUNTS_PER_SECOND / 50U) /* 20 ms */

#define VDMA_S2MM_INTR_ID XPAR_FABRIC_DMA_0_S2MM_IRQ_INTR
#define VDMA_MM2S_INTR_ID XPAR_FABRIC_DMA_0_MM2S_IRQ_INTR

VdmaHandle   vdma;
XScuGic      IntcInstance;
DisplayCtrl  dispCtrl;
VideoMode    vd_mode;
static volatile unsigned int s2mm_done_count = 0;
static volatile unsigned int s2mm_valid_count = 0;
static volatile unsigned int mm2s_done_count = 0;
static volatile unsigned int s2mm_error_count = 0;
static volatile unsigned int mm2s_error_count = 0;
static volatile u32 s2mm_last_error = 0;
static volatile u32 mm2s_last_error = 0;
static volatile u32 s2mm_last_status = 0;
static volatile u32 mm2s_last_status = 0;
static volatile unsigned int s2mm_last_buffer_idx = 0;
static volatile unsigned int mm2s_last_buffer_idx = 0;
static volatile unsigned int capture_download_active = 0;
static unsigned int capture_waiting = 0;
static unsigned int capture_source_reused = 0;
static unsigned int capture_result_valid = 0;
static unsigned int capture_start_frame_count = 0;
static UINTPTR capture_source_addr = 0;
static XTime capture_start_time;
static XTime capture_next_poll_time;
static unsigned int cnn_waiting = 0U;
static unsigned int cnn_result_valid = 0U;
static unsigned int cnn_start_count = 0U;
static unsigned int cnn_done_count = 0U;
static unsigned int cnn_timeout_count = 0U;
static s8 cnn_last_result = 0;
static XTime cnn_start_time;
static XTime cnn_next_poll_time;
static XTime ui_cnn_done_time;
static u32 button_raw_state = 0U;
static u32 button_stable_state = 0U;
static XTime button_raw_change_time;
static unsigned int rom_preview_active = 0U;
static XTime rom_preview_start_time;

/* The vehicle link is separate from the PS UART console and video DMA. */
static int vehicle_ready = 0;
static XTime vehicle_next_send_time;
static XTime vehicle_last_service_time;
static XTime vehicle_max_service_gap;
static unsigned int vehicle_service_count = 0U;
static unsigned int vehicle_sent_count = 0U;
static unsigned int vehicle_send_fail_count = 0U;
static u8 vehicle_sequence = 0U;
static int vehicle_steering = 0;
static u8 vehicle_accel = 0U;
static u8 vehicle_brake = 5U;
static u8 vehicle_flags = 0U;
static int vehicle_step_mode = 0; /* 0: direct target, 1: step per key */
static unsigned int color_profile = COLOR_PROFILE_DEFAULT;
typedef struct {
    Ae_level ae;
    Gamma_factor gamma;
    unsigned int profile;
} CameraSettings;
static CameraSettings camera_history[CAMERA_HISTORY_DEPTH];
static unsigned int camera_history_count = 0U;

static u8 VehicleCrc8(const u8 *data, unsigned int len)
{
    u8 crc = 0U;
    unsigned int i, bit;

    for (i = 0U; i < len; ++i) {
        crc ^= data[i];
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 0x80U) ? (u8)((crc << 1) ^ 0x07U) : (u8)(crc << 1);
    }
    return crc;
}

static void VehicleService(void)
{
    u8 packet[8];
    XTime now;

    if (!vehicle_ready)
        return;
    XTime_GetTime(&now);
    if (now < vehicle_next_send_time)
        return;

    if (vehicle_service_count != 0U &&
        now - vehicle_last_service_time > vehicle_max_service_gap)
        vehicle_max_service_gap = now - vehicle_last_service_time;
    vehicle_last_service_time = now;
    vehicle_service_count++;
    vehicle_next_send_time = now + VEHICLE_TX_PERIOD_TICKS;

    packet[0] = 0xAAU;
    packet[1] = 0x55U;
    packet[2] = vehicle_sequence++;
    packet[3] = (u8)(s8)vehicle_steering;
    packet[4] = vehicle_accel;
    packet[5] = vehicle_brake;
    packet[6] = vehicle_flags;
    packet[7] = VehicleCrc8(packet, 7U);

    if (apb_uart_send_buf(packet, sizeof(packet)) == XST_SUCCESS)
        vehicle_sent_count++;
    else
        vehicle_send_fail_count++;
}

static void VehiclePrintState(void)
{
    xil_printf("vehicle: steer=%d accel=%u brake=%u flags=%02X steps=%s tx=%lu fail=%lu\r\n",
               vehicle_steering, (unsigned)vehicle_accel,
               (unsigned)vehicle_brake, (unsigned)vehicle_flags,
               vehicle_step_mode ? "on" : "off",
               (unsigned long)vehicle_sent_count,
               (unsigned long)vehicle_send_fail_count);
}

/* Keep the actual AE/gamma values so undo also works after a jump to profile 0. */
static void CameraSaveSettings(void)
{
    unsigned int i;

    if (camera_history_count == CAMERA_HISTORY_DEPTH) {
        for (i = 1U; i < CAMERA_HISTORY_DEPTH; ++i)
            camera_history[i - 1U] = camera_history[i];
        --camera_history_count;
    }
    camera_history[camera_history_count].ae = cam_ae_get();
    camera_history[camera_history_count].gamma = gamma_get();
    camera_history[camera_history_count].profile = color_profile;
    ++camera_history_count;
}

static int CameraApplySettings(Ae_level ae, Gamma_factor gamma,
                               unsigned int profile)
{
    u32 errors_before;

    VehicleService();
    errors_before = iic_sccb_error_count();
    cam_ae_set(ae);
    if (iic_sccb_error_count() != errors_before) {
        xil_printf("camera profile %u: SCCB error changing AE target; press u to retry previous settings\r\n",
                   profile);
        VehicleService();
        return 0;
    }
    gamma_set(gamma);
    color_profile = profile;
    xil_printf("camera profile %u/9: AE %s, gamma %s (undo=%u)\r\n",
               profile, cam_ae_name(ae), gamma_name(gamma),
               camera_history_count);
    VehicleService();
    return 1;
}

/* The reference photo came from the same Pcam without colour postprocessing.
 * Both controls precede the DDR capture, so HDMI and 64x64 captures agree. */
static void CameraSetColorProfile(unsigned int index)
{
    static const struct {
        Ae_level ae;
        Gamma_factor gamma;
    } profiles[COLOR_PROFILE_COUNT] = {
        { AE_LEVEL_0,  GAMMA_1_1_8 }, /* previous startup settings */
        { AE_LEVEL_P1, GAMMA_1_1_8 },
        { AE_LEVEL_P2, GAMMA_1_1_8 }, /* startup settings */
        { AE_LEVEL_0,  GAMMA_1_1_5 },
        { AE_LEVEL_P1, GAMMA_1_1_5 },
        { AE_LEVEL_P2, GAMMA_1_1_5 },
        { AE_LEVEL_M1, GAMMA_1_1_5 },
        { AE_LEVEL_0,  GAMMA_1_1_2 },
        { AE_LEVEL_P1, GAMMA_1_1_2 },
        { AE_LEVEL_P2, GAMMA_1_1_2 }
    };
    if (index >= COLOR_PROFILE_COUNT)
        return;
    if (color_profile == index && cam_ae_get() == profiles[index].ae &&
        gamma_get() == profiles[index].gamma)
        return;
    CameraSaveSettings();
    CameraApplySettings(profiles[index].ae, profiles[index].gamma, index);
}

static void CameraNextColorProfile(void)
{
    CameraSetColorProfile((color_profile + 1U) % COLOR_PROFILE_COUNT);
}

static void CameraUndoColorProfile(void)
{
    CameraSettings previous;

    if (camera_history_count == 0U) {
        xil_printf("camera profile: no previous settings\r\n");
        return;
    }
    previous = camera_history[camera_history_count - 1U];
    --camera_history_count;
    if (!CameraApplySettings(previous.ae, previous.gamma, previous.profile))
        ++camera_history_count;
}

/*===========================================================================
 *  8-1. Custom VDMA interrupt handling
 *
 *  The hardware owns buffer rotation.  The ISR only snapshots status and
 *  acknowledges the level-sensitive W1C interrupt bits.
 *===========================================================================*/
static void S2MM_IntrHandler(void *CallBackRef)
{
	VdmaHandle *ctx = (VdmaHandle *)CallBackRef;
	u32 status = vdma_ack_s2mm_irq(ctx);

	s2mm_last_status = status;
	if (status & VDMA_SR_IOC_IRQ_MASK) {
		s2mm_last_buffer_idx = ctx->newest_rx_idx;
		++s2mm_done_count;
		if ((status & VDMA_SR_ERROR_MASK) == 0U)
			++s2mm_valid_count;
	}
	if (status & VDMA_SR_ERR_IRQ_MASK) {
		s2mm_last_error = Xil_In32(ctx->base_address +
			VDMA_S2MM_WRITE_ERR_OFFSET);
		++s2mm_error_count;
	}
}

static void MM2S_IntrHandler(void *CallBackRef)
{
	VdmaHandle *ctx = (VdmaHandle *)CallBackRef;
	u32 status = vdma_ack_mm2s_irq(ctx);

	mm2s_last_status = status;
	if (status & VDMA_SR_IOC_IRQ_MASK) {
		mm2s_last_buffer_idx = ctx->mm2s_fixed_source ?
			NUM_FRAME_BUFFERS : ctx->mm2s_cur_idx;
		++mm2s_done_count;
	}
	if (status & VDMA_SR_ERR_IRQ_MASK) {
		mm2s_last_error = Xil_In32(ctx->base_address +
			VDMA_MM2S_READ_ERR_OFFSET);
		++mm2s_error_count;
	}
}

/* UART output is deliberately deferred out of the interrupt handlers. */
static void VdmaLogPoll(void)
{
	static unsigned int s2mm_done_seen = 0U;
	static unsigned int mm2s_done_seen = 0U;
	static unsigned int s2mm_error_seen = 0U;
	static unsigned int mm2s_error_seen = 0U;
	unsigned int count;

	if (capture_download_active)
		return;

	count = s2mm_done_count;
	if (count != s2mm_done_seen) {
		if (count <= 3U || (count % 60U) == 0U)
			xil_printf("VDMA S2MM done=%lu valid=%lu buf=%lu\r\n",
				(unsigned long)count, (unsigned long)s2mm_valid_count,
				(unsigned long)s2mm_last_buffer_idx);
		s2mm_done_seen = count;
	}
	count = mm2s_done_count;
	if (count != mm2s_done_seen) {
		if (count <= 3U || (count % 60U) == 0U)
			xil_printf("VDMA MM2S done=%lu buf=%lu\r\n",
				(unsigned long)count,
				(unsigned long)mm2s_last_buffer_idx);
		mm2s_done_seen = count;
	}

	count = s2mm_error_count;
	if (count != s2mm_error_seen) {
		if (count <= 3U || (count % 60U) == 0U)
			xil_printf("VDMA S2MM ERROR count=%lu addr=%08X sr=%08X\r\n",
				(unsigned long)count, (unsigned)s2mm_last_error,
				(unsigned)s2mm_last_status);
		s2mm_error_seen = count;
	}
	count = mm2s_error_count;
	if (count != mm2s_error_seen) {
		if (count <= 3U || (count % 60U) == 0U)
			xil_printf("VDMA MM2S ERROR count=%lu addr=%08X sr=%08X\r\n",
				(unsigned long)count, (unsigned)mm2s_last_error,
				(unsigned)mm2s_last_status);
		mm2s_error_seen = count;
	}
}

static int SetupVdmaInterrupts(void)
{
	int Status;
	XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
	if (!IntcConfig) return XST_FAILURE;

	Status = XScuGic_CfgInitialize(&IntcInstance, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) return XST_FAILURE;

	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			&IntcInstance);

	Status = XScuGic_Connect(&IntcInstance, VDMA_S2MM_INTR_ID,
			(Xil_ExceptionHandler)S2MM_IntrHandler, &vdma);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, VDMA_MM2S_INTR_ID,
			(Xil_ExceptionHandler)MM2S_IntrHandler, &vdma);
	if (Status != XST_SUCCESS) return XST_FAILURE;

	XScuGic_SetPriorityTriggerType(&IntcInstance, VDMA_S2MM_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_SetPriorityTriggerType(&IntcInstance, VDMA_MM2S_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_Enable(&IntcInstance, VDMA_S2MM_INTR_ID);
	XScuGic_Enable(&IntcInstance, VDMA_MM2S_INTR_ID);
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

static int BboxInitialize(void)
{
	u32 enable;
	u32 x;
	u32 y;
	u32 color;

	/* Keep the overlay disabled until every shadow register is ready. */
	BBOX_mWriteReg(BBOX_BASEADDR, BBOX_S00_AXI_SLV_REG0_OFFSET, 0U);
	BBOX_mWriteReg(BBOX_BASEADDR, BBOX_S00_AXI_SLV_REG1_OFFSET,
		CAPTURE_CROP_X);
	BBOX_mWriteReg(BBOX_BASEADDR, BBOX_S00_AXI_SLV_REG2_OFFSET,
		CAPTURE_CROP_Y);
	BBOX_mWriteReg(BBOX_BASEADDR, BBOX_S00_AXI_SLV_REG3_OFFSET,
		BBOX_COLOR_RED);
	BBOX_mWriteReg(BBOX_BASEADDR, BBOX_S00_AXI_SLV_REG0_OFFSET,
		BBOX_ENABLE);

	/* Verify the AXI-Lite path before starting the video stream. */
	enable = BBOX_mReadReg(BBOX_BASEADDR,
		BBOX_S00_AXI_SLV_REG0_OFFSET) & 0x1U;
	x = BBOX_mReadReg(BBOX_BASEADDR,
		BBOX_S00_AXI_SLV_REG1_OFFSET) & 0x7FFU;
	y = BBOX_mReadReg(BBOX_BASEADDR,
		BBOX_S00_AXI_SLV_REG2_OFFSET) & 0x1FFU;
	color = BBOX_mReadReg(BBOX_BASEADDR,
		BBOX_S00_AXI_SLV_REG3_OFFSET) & 0x00FFFFFFU;

	if (enable != BBOX_ENABLE || x != CAPTURE_CROP_X ||
	    y != CAPTURE_CROP_Y || color != BBOX_COLOR_RED) {
		BBOX_mWriteReg(BBOX_BASEADDR,
			BBOX_S00_AXI_SLV_REG0_OFFSET, 0U);
		xil_printf("BBOX register verification failed: "
			"EN=%u X=%u Y=%u COLOR=%06X\r\n",
			(unsigned)enable, (unsigned)x, (unsigned)y,
			(unsigned)color);
		return XST_FAILURE;
	}

	xil_printf("[OK] BBOX enabled: %ux%u at (%u,%u), color=%06X\r\n",
		(unsigned)CAPTURE_CROP_SIZE, (unsigned)CAPTURE_CROP_SIZE,
		(unsigned)x, (unsigned)y, (unsigned)color);
	return XST_SUCCESS;
}

static void BboxToggle(void)
{
	u32 enabled;
	u32 requested;

	enabled = BBOX_mReadReg(BBOX_BASEADDR,
		BBOX_S00_AXI_SLV_REG0_OFFSET) & 0x1U;
	requested = enabled ^ 0x1U;
	BBOX_mWriteReg(BBOX_BASEADDR, BBOX_S00_AXI_SLV_REG0_OFFSET,
		requested);

	enabled = BBOX_mReadReg(BBOX_BASEADDR,
		BBOX_S00_AXI_SLV_REG0_OFFSET) & 0x1U;
	if (enabled != requested) {
		xil_printf("BBOX toggle failed: requested=%s readback=%s\r\n",
			requested != 0U ? "ON" : "OFF",
			enabled != 0U ? "ON" : "OFF");
		return;
	}

	xil_printf("BBOX %s\r\n", enabled != 0U ? "ON" : "OFF");
}

/*===========================================================================
 *  PS UART console menu; vehicle packets use the separate PL UART.
 *===========================================================================*/
static void menu_help(){
	xil_printf("\r\n--- keys ------------------------------\r\n");
	VehicleService();
	xil_printf("  d  : dump custom VDMA status and counters \r\n");
	VehicleService();
	xil_printf("  c  : test capture control on last DDR frame\r\n");
	VehicleService();
	xil_printf("  i  : run CNN again on the completed 64x64 capture\r\n");
	VehicleService();
	xil_printf("  BTN0: capture last DDR frame and run CNN\r\n");
	VehicleService();
	xil_printf("  BTN1: print the latest CNN accelerator result\r\n");
	VehicleService();
	xil_printf("  r  : read captured 64x64 RGB pixels\r\n");
	VehicleService();
	xil_printf("  x  : download full 64x64 RGB888 as HEX\r\n");
	VehicleService();
	xil_printf("  l  : show BRAM ROM image for 3 seconds\r\n");
	VehicleService();
	xil_printf("  j  : toggle BBOX overlay ON/OFF\r\n");
	VehicleService();
	xil_printf("  R/L/N : steer right/left/center (direct: +/-90, steps: +/-10)\r\n");
	VehicleService();
	xil_printf("  a/b : accel/brake (direct: 5, steps: +1; range 0..5)\r\n");
	VehicleService();
	xil_printf("  z/s : coast/stop, e : toggle emergency stop\r\n");
	VehicleService();
	xil_printf("  m : toggle input steps (initially off)\r\n");
	VehicleService();
	xil_printf("  y : next camera AE/gamma profile (0..9)\r\n");
	VehicleService();
	xil_printf("  Y : restore startup camera profile (2)\r\n");
	VehicleService();
	xil_printf("  u : undo camera AE/gamma change (up to 16)\r\n");
	VehicleService();
	xil_printf("  t : PL UART status\r\n");
	VehicleService();
	xil_printf("  ?  : menu help \r\n");
}

static u32 CaptureStatus(void)
{
	return Xil_In32(CAPTURE_BASEADDR + CAPTURE_STS_OFFSET);
}

static u32 CnnStatus(void)
{
	return Xil_In32(CNN_BASEADDR + CNN_STATUS_OFFSET);
}

static u32 CaptureReadPixel(unsigned int x, unsigned int y)
{
	u32 index = y * CAPTURE_RESULT_SIZE + x;
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_PIXEL_INDEX_OFFSET, index);
	return Xil_In32(CAPTURE_BASEADDR + CAPTURE_PIXEL_DATA_OFFSET) & 0x00FFFFFFU;
}

static void CapturePrintPixels(void)
{
	static const unsigned int sample_xy[][2] = {
		{0U, 0U}, {31U, 0U}, {63U, 0U},
		{0U, 31U}, {31U, 31U}, {63U, 31U},
		{0U, 63U}, {31U, 63U}, {63U, 63U}
	};
	u32 checksum = 2166136261U;
	unsigned int nonzero = 0U;
	unsigned int i, x, y;
	u32 rgb;

	if (cnn_waiting) {
		xil_printf("capture: CNN is reading the 64x64 RAM\r\n");
		return;
	}
	if (!capture_result_valid || capture_waiting ||
	    (CaptureStatus() & (CAPTURE_STS_DONE | CAPTURE_STS_BUSY)) != CAPTURE_STS_DONE) {
		xil_printf("capture: no completed result to read\r\n");
		return;
	}
	if ((Xil_In32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET) & ~1U) != CAPTURE_READBACK_ID) {
		xil_printf("capture: current bitstream has no CPU pixel readback\r\n");
		return;
	}

	/* The single RAM read port is temporarily assigned to the CPU. */
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET, 1U);
	for (y = 0U; y < CAPTURE_RESULT_SIZE; ++y) {
		for (x = 0U; x < CAPTURE_RESULT_SIZE; ++x) {
			rgb = CaptureReadPixel(x, y);
			if (rgb != 0U) ++nonzero;
			checksum = (checksum ^ rgb) * 16777619U;
			if ((x & 15U) == 15U) VehicleService();
		}
	}
	xil_printf("capture: RAM 64x64 nonzero=%u/4096 checksum=%08X\r\n",
		nonzero, (unsigned)checksum);
	for (i = 0U; i < sizeof(sample_xy) / sizeof(sample_xy[0]); ++i) {
		x = sample_xy[i][0];
		y = sample_xy[i][1];
		rgb = CaptureReadPixel(x, y);
		xil_printf("capture: pixel(%u,%u)=%06X R=%02X G=%02X B=%02X\r\n",
			x, y, (unsigned)rgb, (unsigned)((rgb >> 16) & 0xFFU),
			(unsigned)((rgb >> 8) & 0xFFU), (unsigned)(rgb & 0xFFU));
		VehicleService();
	}
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET, 0U);
}

static void CaptureDownloadPixels(void)
{
	static const char hex_digit[] = "0123456789ABCDEF";
	char row_hex[CAPTURE_RESULT_SIZE * 6U + 1U];
	u32 checksum = 2166136261U;
	unsigned int x, y;
	u32 rgb;

	if (cnn_waiting) {
		xil_printf("capture: CNN is reading the 64x64 RAM\r\n");
		return;
	}
	if (!capture_result_valid || capture_waiting ||
	    (CaptureStatus() & (CAPTURE_STS_DONE | CAPTURE_STS_BUSY)) !=
	        CAPTURE_STS_DONE) {
		xil_printf("capture: no completed result to download\r\n");
		return;
	}
	if ((Xil_In32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET) & ~1U) !=
	    CAPTURE_READBACK_ID) {
		xil_printf("capture: current bitstream has no CPU pixel readback\r\n");
		return;
	}

	/* Keep DMA interrupts running, but suppress their UART messages so that
	 * the machine-readable capture block is not interrupted. */
	capture_download_active = 1U;
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET, 1U);
	xil_printf("CAPTURE64_BEGIN width=64 height=64 format=RGB888 encoding=HEX "
		"crop_x=%u crop_y=%u cnn_valid=%u cnn_result=%d\r\n",
		(unsigned)CAPTURE_CROP_X, (unsigned)CAPTURE_CROP_Y,
		cnn_result_valid, (int)cnn_last_result);

	for (y = 0U; y < CAPTURE_RESULT_SIZE; ++y) {
		for (x = 0U; x < CAPTURE_RESULT_SIZE; ++x) {
			unsigned int p = x * 6U;

			rgb = CaptureReadPixel(x, y);
			checksum = (checksum ^ rgb) * 16777619U;
			row_hex[p + 0U] = hex_digit[(rgb >> 20) & 0x0FU];
			row_hex[p + 1U] = hex_digit[(rgb >> 16) & 0x0FU];
			row_hex[p + 2U] = hex_digit[(rgb >> 12) & 0x0FU];
			row_hex[p + 3U] = hex_digit[(rgb >> 8)  & 0x0FU];
			row_hex[p + 4U] = hex_digit[(rgb >> 4)  & 0x0FU];
			row_hex[p + 5U] = hex_digit[rgb & 0x0FU];
			if ((x & 15U) == 15U) VehicleService();
		}
		row_hex[CAPTURE_RESULT_SIZE * 6U] = '\0';
		xil_printf("ROW %u ", y);
		/* A full row blocks the console for about 33 ms at 115200 baud.
		 * Keep the same line format while servicing vehicle TX. */
		for (x = 0U; x < CAPTURE_RESULT_SIZE * 6U; x += 48U) {
			char saved = row_hex[x + 48U];
			row_hex[x + 48U] = '\0';
			xil_printf("%s", &row_hex[x]);
			row_hex[x + 48U] = saved;
			VehicleService();
		}
		xil_printf("\r\n");
		VehicleService();
	}

	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET, 0U);
	xil_printf("CAPTURE64_END checksum=%08X cnn_valid=%u cnn_result=%d\r\n",
		(unsigned)checksum, cnn_result_valid, (int)cnn_last_result);
	capture_download_active = 0U;
}

static int CnnStartFromCapture(void)
{
	u32 status;
	unsigned int retry;

	if (!capture_result_valid || capture_waiting) {
		xil_printf("CNN: no completed 64x64 capture\r\n");
		return XST_FAILURE;
	}
	if (cnn_waiting) {
		xil_printf("CNN: already waiting for completion\r\n");
		return XST_FAILURE;
	}

	status = CnnStatus();
	if (status & (CNN_STATUS_BUSY | CNN_STATUS_START_PENDING)) {
		xil_printf("CNN: hardware busy, status=%08X\r\n", (unsigned)status);
		return XST_FAILURE;
	}

	/* Give the CAPTURE RAM read port to IP_CNN_0. */
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET, 0U);

	/* DONE is sticky. Clear the previous result before issuing a new start. */
	Xil_Out32(CNN_BASEADDR + CNN_CTRL_OFFSET, CNN_CTRL_IRQ_CLEAR);
	for (retry = 0U; retry < 1000U; ++retry) {
		status = CnnStatus();
		if ((status & CNN_STATUS_DONE) == 0U)
			break;
	}
	if ((status & CNN_STATUS_DONE) != 0U) {
		xil_printf("CNN: could not clear DONE, status=%08X\r\n",
			(unsigned)status);
		return XST_FAILURE;
	}
	if ((status & CNN_STATUS_START_READY) == 0U) {
		xil_printf("CNN: START_READY is low, status=%08X\r\n",
			(unsigned)status);
		return XST_FAILURE;
	}

	/* Each AXI write with bit 0 set creates one start request. */
	Xil_Out32(CNN_BASEADDR + CNN_CTRL_OFFSET, CNN_CTRL_START);
	++cnn_start_count;
	cnn_result_valid = 0U;
	cnn_waiting = 1U;
	XTime_GetTime(&cnn_start_time);
	cnn_next_poll_time = cnn_start_time;
	xil_printf("CNN: start #%u status=%08X\r\n",
		cnn_start_count, (unsigned)CnnStatus());
	return XST_SUCCESS;
}

static void CnnPoll(void)
{
	u32 status;
	u32 raw;
	XTime now;

	if (!cnn_waiting)
		return;
	XTime_GetTime(&now);
	if (now < cnn_next_poll_time)
		return;
	cnn_next_poll_time = now + COUNTS_PER_SECOND / 100U;
	status = CnnStatus();

	if (status & CNN_STATUS_DONE) {
		raw = Xil_In32(CNN_BASEADDR + CNN_RESULT_OFFSET);
		cnn_last_result = (s8)(raw & 0xFFU);
		++cnn_done_count;
		cnn_result_valid = 1U;
		ui_cnn_done_time = now;
		cnn_waiting = 0U;
		Xil_Out32(CNN_BASEADDR + CNN_CTRL_OFFSET, CNN_CTRL_IRQ_CLEAR);
		xil_printf("CNN: done #%u status=%08X result_raw=%02X result=%d\r\n",
			cnn_done_count, (unsigned)status, (unsigned)(raw & 0xFFU),
			(int)cnn_last_result);
		return;
	}

	if (now - cnn_start_time >=
	    (XTime)CNN_TIMEOUT_SECONDS * COUNTS_PER_SECOND) {
		cnn_waiting = 0U;
		cnn_result_valid = 0U;
		++cnn_timeout_count;
		xil_printf("CNN: timeout #%u status=%08X\r\n",
			cnn_timeout_count, (unsigned)status);
	}
}

static void CaptureStartFromLastFrame(void)
{
	unsigned int valid_count;
	unsigned int index;
	UINTPTR source;
	u32 status;

	if (capture_waiting) {
		xil_printf("capture: already waiting for completion\r\n");
		return;
	}
	if (cnn_waiting ||
	    (CnnStatus() & (CNN_STATUS_BUSY | CNN_STATUS_START_PENDING))) {
		xil_printf("capture: CNN is still using the 64x64 RAM\r\n");
		return;
	}
	status = CaptureStatus();
	if (status & CAPTURE_STS_BUSY) {
		xil_printf("capture: hardware busy, status=%08X\r\n", (unsigned)status);
		return;
	}

	/* The ISR updates the valid buffer index before its valid-frame counter. */
	do {
		valid_count = s2mm_valid_count;
		index = vdma.newest_rx_idx;
	} while (valid_count != s2mm_valid_count);
	if (valid_count == 0U || index >= NUM_FRAME_BUFFERS) {
		xil_printf("capture: no completed DDR frame yet\r\n");
		return;
	}
	source = vdma.buffer_address[index];
	if ((source & 7U) != 0U) {
		xil_printf("capture: unaligned DDR address %08X\r\n", (unsigned)source);
		return;
	}

	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_SRC_OFFSET, (u32)source);
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_X_OFFSET, CAPTURE_CROP_X);
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_Y_OFFSET, CAPTURE_CROP_Y);
	/* CAPTURE_START is edge-sensitive; clear it before and after the pulse. */
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_CTRL_OFFSET, 0U);
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_CTRL_OFFSET, 1U);
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_CTRL_OFFSET, 0U);
	capture_source_addr = source;
	capture_start_frame_count = s2mm_done_count;
	capture_source_reused = 0U;
	capture_result_valid = 0U;
	capture_waiting = 1U;
	XTime_GetTime(&capture_start_time);
	capture_next_poll_time = capture_start_time;
	xil_printf("capture: start src=%08X buf=%u crop=(%u,%u) status=%08X\r\n",
		(unsigned)source, index, (unsigned)CAPTURE_CROP_X,
		(unsigned)CAPTURE_CROP_Y, (unsigned)CaptureStatus());
}

static void CapturePoll(void)
{
	u32 status;
	XTime now;

	if (!capture_waiting)
		return;
	XTime_GetTime(&now);
	if (now < capture_next_poll_time)
		return;
	capture_next_poll_time = now + COUNTS_PER_SECOND / 100U;
	status = CaptureStatus();
	/* After the other two buffers complete, S2MM can start reusing this one. */
	if (!capture_source_reused &&
	    s2mm_done_count - capture_start_frame_count >= NUM_FRAME_BUFFERS - 1U) {
		capture_source_reused = 1U;
		capture_result_valid = 0U;
		xil_printf("capture: DDR source was reused; result will be discarded\r\n");
	}
	if (status & CAPTURE_STS_DONE) {
		capture_result_valid = capture_source_reused ? 0U : 1U;
		capture_waiting = 0U;
		if (capture_result_valid) {
			xil_printf("capture: hardware done src=%08X status=%08X (r=summary, x=download)\r\n",
				(unsigned)capture_source_addr, (unsigned)status);
			if (CnnStartFromCapture() != XST_SUCCESS)
				xil_printf("capture: CNN did not start; r/x result remains available\r\n");
		} else {
			xil_printf("capture: hardware done after source reuse; result discarded\r\n");
		}
		return;
	}
	if (now - capture_start_time >= COUNTS_PER_SECOND) {
		capture_result_valid = 0U;
		xil_printf("capture: timeout status=%08X src=%08X\r\n",
			(unsigned)status, (unsigned)capture_source_addr);
		capture_waiting = 0U;
	}
}

static void ButtonInit(void)
{
	button_raw_state = Xil_In32(BUTTON_GPIO_BASEADDR +
		BUTTON_GPIO_DATA_OFFSET) & BUTTON_MASK;
	button_stable_state = button_raw_state;
	XTime_GetTime(&button_raw_change_time);
	xil_printf("buttons: BTN0=capture+CNN, BTN1=print CNN result (state=%X)\r\n",
		(unsigned)button_stable_state);
}

static void ButtonPrintCnnResult(void)
{
	u32 status = CnnStatus();
	u32 raw = Xil_In32(CNN_BASEADDR + CNN_RESULT_OFFSET);

	xil_printf("BTN1 CNN: status=%08X waiting=%u valid=%u raw=%02X "
		"result=%d starts=%u done=%u timeout=%u\r\n",
		(unsigned)status, cnn_waiting, cnn_result_valid,
		(unsigned)(raw & 0xFFU), (int)(s8)(raw & 0xFFU),
		cnn_start_count, cnn_done_count, cnn_timeout_count);
}

static void ButtonPoll(void)
{
	u32 current;
	u32 pressed;
	XTime now;

	current = Xil_In32(BUTTON_GPIO_BASEADDR +
		BUTTON_GPIO_DATA_OFFSET) & BUTTON_MASK;
	XTime_GetTime(&now);

	if (current != button_raw_state) {
		button_raw_state = current;
		button_raw_change_time = now;
		return;
	}
	if (now - button_raw_change_time < BUTTON_DEBOUNCE_TICKS ||
	    current == button_stable_state)
		return;

	pressed = (current ^ button_stable_state) & current;
	button_stable_state = current;

	if (pressed & BUTTON_CAPTURE_MASK) {
		xil_printf("BTN0: capture requested\r\n");
		CaptureStartFromLastFrame();
	}
	if (pressed & BUTTON_CNN_RESULT_MASK)
		ButtonPrintCnnResult();
}

static void PrintVdmaDiagnostics(void)
{
	VehicleService();
	xil_printf("VDMA counters: S2MM=%lu valid=%lu MM2S=%lu "
		"errors=%lu/%lu error_addr=%08X/%08X status=%08X/%08X\r\n",
		(unsigned long)s2mm_done_count, (unsigned long)s2mm_valid_count,
		(unsigned long)mm2s_done_count,
		(unsigned long)s2mm_error_count, (unsigned long)mm2s_error_count,
		(unsigned)s2mm_last_error, (unsigned)mm2s_last_error,
		(unsigned)s2mm_last_status, (unsigned)mm2s_last_status);
	xil_printf("capture: status=%08X waiting=%u src=%08X\r\n",
		(unsigned)CaptureStatus(), capture_waiting,
		(unsigned)capture_source_addr);
	xil_printf("CNN: status=%08X waiting=%u starts=%u done=%u timeout=%u "
		"result_valid=%u result=%d\r\n",
		(unsigned)CnnStatus(), cnn_waiting, cnn_start_count,
		cnn_done_count, cnn_timeout_count, cnn_result_valid,
		(int)cnn_last_result);
	VehicleService();
	xil_printf("vehicle: tx=%lu fail=%lu max service gap=%lu ms\r\n",
		(unsigned long)vehicle_sent_count,
		(unsigned long)vehicle_send_fail_count,
		(unsigned long)((vehicle_max_service_gap * 1000U) / COUNTS_PER_SECOND));
	vdma_dump_status(&vdma);
	VehicleService();
}

static int SwitchDisplayToLive(void)
{
	unsigned int switch_count;
	XTime start, now;

	if (s2mm_valid_count == 0U) {
		/* Do not leave the loading-screen override latched forever. Switch to
		 * DDR unconditionally; if the camera starts later, the same live ring
		 * will begin showing it without another mode change. */
		xil_printf("[STAGE3] WARNING: no valid completed camera frame in DDR; "
			"switching to the DDR ring anyway\r\n");
		vdma_dump_status(&vdma);
	}

	/* LIVE is sampled by the custom VDMA at the next frame boundary. */
	switch_count = mm2s_done_count;
	vdma_set_live(&vdma);

	XTime_GetTime(&start);
	do {
		if (mm2s_done_count != switch_count)
			return XST_SUCCESS;
		VehicleService();
		VdmaLogPoll();
		XTime_GetTime(&now);
		if (now - start >= 2ULL * COUNTS_PER_SECOND)
			break;
		usleep(1000U);
	} while (1);

	xil_printf("[STAGE3] FAIL: MM2S produced no live DDR frame "
		"before timeout (done=%lu -> %lu)\r\n",
		(unsigned long)switch_count, (unsigned long)mm2s_done_count);
	return XST_FAILURE;
}

static void RomPreviewStart(void)
{
	/* MM2S switches to this fixed source at the next frame boundary. */
	vdma_set_fixed_source(&vdma, BRAM_LOADING_ADDR);
	XTime_GetTime(&rom_preview_start_time);
	rom_preview_active = 1U;
	xil_printf("ROM display enabled for %u seconds at %08X\r\n",
		(unsigned)ROM_PREVIEW_SECONDS, (unsigned)BRAM_LOADING_ADDR);
}

static void RomPreviewPoll(void)
{
	XTime now;

	if (!rom_preview_active)
		return;
	XTime_GetTime(&now);
	if (now - rom_preview_start_time <
	    (XTime)ROM_PREVIEW_SECONDS * COUNTS_PER_SECOND)
		return;
	rom_preview_active = 0U;
	if (SwitchDisplayToLive() == XST_SUCCESS)
		xil_printf("ROM display ended; live camera restored\r\n");
	else
		xil_printf("ROM display ended; live camera restart failed\r\n");
}

static void menu_run()
{
	char c;
	int vehicle_changed = 0;

	if (!XUartPs_IsReceiveData(STDIN_BASEADDRESS)){
		return;
	}
	c = (char)XUartPs_RecvByte(STDIN_BASEADDRESS);

	switch(c){
		case	'd' : PrintVdmaDiagnostics(); UiStream_PrintStats(); break;
		case	'c' : CaptureStartFromLastFrame(); break;
		case	'i' : (void)CnnStartFromCapture(); break;
		case	'r' : CapturePrintPixels(); break;
		case	'x' : CaptureDownloadPixels(); break;
		case 'l': RomPreviewStart(); break;
		case 'j': BboxToggle(); break;
		case 'R':
			if (vehicle_step_mode) {
				if (vehicle_steering < VEHICLE_STEER_MAX)
					vehicle_steering += 10;
			} else {
				vehicle_steering = VEHICLE_STEER_MAX;
			}
			vehicle_changed = 1;
			break;
		case 'L':
			if (vehicle_step_mode) {
				if (vehicle_steering > -VEHICLE_STEER_MAX)
					vehicle_steering -= 10;
			} else {
				vehicle_steering = -VEHICLE_STEER_MAX;
			}
			vehicle_changed = 1;
			break;
		case 'N':
			vehicle_steering = 0;
			vehicle_changed = 1;
			break;
		case 'a':
			if ((vehicle_flags & VEHICLE_FLAG_ESTOP) == 0U) {
				if (vehicle_step_mode) {
					if (vehicle_accel < VEHICLE_LEVEL_MAX)
						++vehicle_accel;
				} else {
					vehicle_accel = VEHICLE_LEVEL_MAX;
				}
				vehicle_brake = 0U;
				vehicle_changed = 1;
			}
			break;
		case 'b':
			if (vehicle_step_mode) {
				if (vehicle_brake < VEHICLE_LEVEL_MAX)
					++vehicle_brake;
			} else {
				vehicle_brake = VEHICLE_LEVEL_MAX;
			}
			vehicle_accel = 0U;
			vehicle_changed = 1;
			break;
		case 'm':
			vehicle_step_mode = !vehicle_step_mode;
			vehicle_changed = 1;
			break;
		case 'y':
			CameraNextColorProfile();
			break;
		case 'Y':
			CameraSetColorProfile(COLOR_PROFILE_DEFAULT);
			break;
		case 'u':
			CameraUndoColorProfile();
			break;
		case 'z':
			if ((vehicle_flags & VEHICLE_FLAG_ESTOP) == 0U) {
				vehicle_accel = 0U;
				vehicle_brake = 0U;
				vehicle_changed = 1;
			}
			break;
		case 's':
			vehicle_accel = 0U;
			vehicle_brake = 5U;
			vehicle_changed = 1;
			break;
		case 'e':
			vehicle_flags ^= VEHICLE_FLAG_ESTOP;
			if ((vehicle_flags & VEHICLE_FLAG_ESTOP) != 0U) {
				vehicle_accel = 0U;
				vehicle_brake = 5U;
			}
			vehicle_changed = 1;
			break;
		case 't':
			xil_printf("PL UART base=%08X busy=%d\r\n",
				(unsigned)APB_UART_BASE, apb_uart_tx_busy());
			VehiclePrintState();
			xil_printf("camera color profile=%u/9 AE=%s gamma=%s undo=%u\r\n",
				color_profile, cam_ae_name(cam_ae_get()),
				gamma_name(gamma_get()), camera_history_count);
			break;
		case	'?' : menu_help();		break;
		default : break;
	}
	if (vehicle_changed) VehiclePrintState();
}

/* Take the newest completed custom-VDMA buffer into the UDP snapshot copier.
 * Keep all Ethernet work outside the S2MM interrupt handler. */
static void UiApplicationService(void)
{
	unsigned before, index, age;
	UINTPTR address = 0U;
	XTime now;
	XTime_GetTime(&now);
	age = cnn_done_count ?
		(unsigned)((now - ui_cnn_done_time) / (COUNTS_PER_SECOND / 1000U)) : 0U;
	do {
		before = s2mm_done_count;
		index = vdma.newest_rx_idx;
	} while (before != s2mm_done_count);
	if (s2mm_valid_count != 0U && before != 0U &&
	    index < NUM_FRAME_BUFFERS && index == s2mm_last_buffer_idx &&
	    (s2mm_last_status & VDMA_SR_ERROR_MASK) == 0U)
		address = vdma.buffer_address[index];
	UiStream_Service(address, FRAME_WIDTH, FRAME_HEIGHT,
		(int)cnn_last_result, UI_TEST_PRESSURE_PERCENT, before,
		&s2mm_done_count, cnn_result_valid && age < UI_CNN_FRESH_MS, age);
}

    int main(void)
    {
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
     *  1. 移대찓�씪 �쟾�썝
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
     *  2. MIPI �닔�떊湲곕�� 由ъ뀑�뿉 遺숈옟�븘 �몼�땲�떎
     *-------------------------------------------------------------------*/
    mipi_rx_reset();
    mipi_rx_print_version();
	xil_printf("[OK] MIPI reset/version\r\n");

    /*-------------------------------------------------------------------
     *  3. SCCB 踰꾩뒪
     *-------------------------------------------------------------------*/
    if (OV5640_Init() != IIC_SCCB_OK) {
        xil_printf("SCCB bus did not come up. Stopping.\r\n");
        xil_printf("  check : I2C 0 enabled in the PS block and routed to\r\n"
                   "          EMIO, IIC_0 made external, XDC pins F20/F19,\r\n"
                   "          XSA re-exported and the platform updated.\r\n");
        return 1;
    }

    /*-------------------------------------------------------------------
     *  4. �꽱�꽌 �쟾�썝 �궗�씠�겢 �썑 怨듯넻 珥덇린�솕
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
     *  6. MIPI �닔�떊湲곕�� ��怨�, 洹� �떎�쓬�뿉 �꽱�꽌瑜� 源⑥썎�땲�떎
     *-------------------------------------------------------------------*/
    mipi_rx_enable();
	xil_printf("[OK] MIPI enable\r\n");

    gamma_init();

    OV5640_SetMode720p();
    OV5640_SetAWB(AWB_ADVANCED);
    cam_ae_set(AE_LEVEL_P2);
	xil_printf("[OK] sensor stream/AE setup\r\n");

    /*-------------------------------------------------------------------
     *  7. HDMI 異쒕젰
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

    apb_uart_init();
    XTime_GetTime(&vehicle_next_send_time);
    vehicle_ready = 1;
    VehicleService(); /* Send the initial stopped state immediately. */
    xil_printf("PL UART vehicle TX at %08X, period 10 ms\r\n",
               (unsigned)APB_UART_BASE);
    xil_printf("[STAGE1] system init complete.\r\n");
    xil_printf("VDMA counters: S2MM=%lu valid=%lu MM2S=%lu\r\n",
			(unsigned long)s2mm_done_count,
			(unsigned long)s2mm_valid_count,
			(unsigned long)mm2s_done_count);

    xil_printf("\r\n>>> [STAGE2] LOADING SCREEN ACTIVE (%u seconds) <<<\r\n",
               (unsigned)LOADING_SCREEN_SECONDS);
    XTime_GetTime(&loading_start);
    do {
        VehicleService();
		VdmaLogPoll();
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

	ButtonInit();

    /*-------------------------------------------------------------------
     *  9. Super-loop
     *
     *  �쁺�긽 �옄泥대뒗 �씠�젣 8-1踰� �씤�꽣�읇�듃 �빖�뱾�윭媛� 怨꾩냽 �쓽�젮蹂대깄�땲�떎.
     *  �뿬湲곗꽌�뒗 吏㏐퀬 �븞 留됲엳�뒗 �씪(UART 硫붾돱)留� �빀�땲�떎 �� 洹쒖튃�� VDMA
     *  踰꾩쟾怨� �룞�씪�빀�땲�떎.
     *-------------------------------------------------------------------*/
    XTime_GetTime(&diagnostics_start);
    for (;;) {
        VehicleService();
		VdmaLogPoll();
        menu_run();
		VehicleService();
		CapturePoll();
		CnnPoll();
		ButtonPoll();
		RomPreviewPoll();
		UiApplicationService();
		VehicleService();
        if (!diagnostics_done) {
            XTime_GetTime(&diagnostics_now);
            if (diagnostics_now - diagnostics_start >=
                    2ULL * COUNTS_PER_SECOND) {
                diagnostics_done = 1;
                if (mm2s_done_count == 0) {
                    xil_printf("VDMA MM2S has no completions after 2 seconds\r\n");
                    PrintVdmaDiagnostics();
                }
            }
        }
        usleep(1000U);
    }

    /* not reached */
}

