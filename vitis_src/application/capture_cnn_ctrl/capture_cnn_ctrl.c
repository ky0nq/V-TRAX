#include "capture_cnn_ctrl.h"
#include "../../driver/capture_driver/capture_driver.h"
#include "../../driver/cnn_driver/cnn_driver.h"
#include "../../driver/video_pipeline_driver/video_pipeline_driver.h"
#include "../../driver/vehicle_driver/vehicle.h"
#include "../../driver/bbox_driver/bbox_driver.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "xil_exception.h"

#define CNN_TIMEOUT_SECONDS      5U

static CaptureMode capture_mode = CAPTURE_MODE_TEST;
static u32 capture_timer_seen = 0U;

CaptureMode CaptureGetMode(void)
{
    return capture_mode;
}

void CaptureEnterDemoMode(void)
{
    if (capture_mode == CAPTURE_MODE_DEMO)
        return;
    /* Start with the next IRQ, not ticks accumulated in TEST mode. */
    capture_timer_seen = timer_irq_count;
    capture_mode = CAPTURE_MODE_DEMO;

    BboxDisable();
}

volatile unsigned int capture_download_active = 0;

unsigned int capture_waiting = 0;
static unsigned int capture_source_reused = 0;
static unsigned int capture_result_valid = 0;
static unsigned int capture_start_frame_count = 0;
UINTPTR capture_source_addr = 0;
static XTime capture_start_time;
static XTime capture_next_poll_time;

unsigned int cnn_waiting = 0U;
unsigned int cnn_result_valid = 0U;
unsigned int cnn_start_count = 0U;
unsigned int cnn_done_count = 0U;
unsigned int cnn_timeout_count = 0U;
s8 cnn_last_result = 0;
static XTime cnn_start_time;
static XTime cnn_next_poll_time;

XTime ui_cnn_done_time;

void CapturePrintPixels(void)
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
	if ((Xil_In32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET) &
	     ~(CAPTURE_READ_CPU_ENABLE | CAPTURE_IRQ_ENABLE)) != CAPTURE_READBACK_ID) {
		xil_printf("capture: current bitstream has no CPU pixel readback\r\n");
		return;
	}

	/* The single RAM read port is temporarily assigned to the CPU. */
	CaptureSelectReadPort(1U);
	for (y = 0U; y < CAPTURE_RESULT_SIZE; ++y) {
		for (x = 0U; x < CAPTURE_RESULT_SIZE; ++x) {
			rgb = CaptureReadPixel(x, y);
			if (rgb != 0U) ++nonzero;
			checksum = (checksum ^ rgb) * 16777619U;
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
	}
	CaptureSelectReadPort(0U);
}

void CaptureDownloadPixels(void)
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
	if ((Xil_In32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET) &
	     ~(CAPTURE_READ_CPU_ENABLE | CAPTURE_IRQ_ENABLE)) !=
	    CAPTURE_READBACK_ID) {
		xil_printf("capture: current bitstream has no CPU pixel readback\r\n");
		return;
	}

	/* Keep DMA interrupts running, but suppress their UART messages so that
	 * the machine-readable capture block is not interrupted. */
	capture_download_active = 1U;
	CaptureSelectReadPort(1U);
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
		}
		row_hex[CAPTURE_RESULT_SIZE * 6U] = '\0';
		xil_printf("ROW %u ", y);
		/* A full row blocks the console for about 33 ms at 115200 baud.*/
		for (x = 0U; x < CAPTURE_RESULT_SIZE * 6U; x += 48U) {
			char saved = row_hex[x + 48U];
			row_hex[x + 48U] = '\0';
			xil_printf("%s", &row_hex[x]);
			row_hex[x + 48U] = saved;
		}
		xil_printf("\r\n");
	}

	CaptureSelectReadPort(0U);
	xil_printf("CAPTURE64_END checksum=%08X cnn_valid=%u cnn_result=%d\r\n",
		(unsigned)checksum, cnn_result_valid, (int)cnn_last_result);
	capture_download_active = 0U;
}

/* Only CapturePoll may start inference, once per valid Capture DONE IRQ. */
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
	CaptureSelectReadPort(0U);

	/* DONE is sticky. Clear the previous result before issuing a new start. */
	cnn_irq_pending = 0U;
	CnnWriteCommand(CNN_CTRL_IRQ_CLEAR);
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
	CnnWriteCommand(CNN_CTRL_START);
	++cnn_start_count;
	cnn_result_valid = 0U;
	cnn_waiting = 1U;
	XTime_GetTime(&cnn_start_time);
	cnn_next_poll_time = cnn_start_time;
	//xil_printf("CNN: start #%u status=%08X\r\n",
	//	cnn_start_count, (unsigned)CnnStatus());
	return XST_SUCCESS;
}

void CnnPoll(void)
{
	u32 status;
	u32 raw;
	XTime now;
	XTime completed_at;
	u32 saved_cpsr;
	unsigned int completed = 0U;

	if (!cnn_waiting)
		return;
	XTime_GetTime(&now);

	/* Snapshot the 64-bit timestamp with RESULT while IRQs are masked.
	 * Preserve the caller's IRQ state and keep this section short. */
	saved_cpsr = mfcpsr();
	Xil_ExceptionDisableMask(XIL_EXCEPTION_IRQ);
	if (cnn_irq_pending != 0U) {
		raw = cnn_irq_last_result;
		completed_at = cnn_irq_done_time;
		cnn_irq_pending = 0U;
		completed = 1U;
	} else if (now >= cnn_next_poll_time) {
		cnn_next_poll_time = now + COUNTS_PER_SECOND / 100U;
		status = CnnStatus();
		if ((status & CNN_STATUS_DONE) != 0U) {
			raw = CnnResult();
			/* No IRQ timestamp: use the request time conservatively instead
			 * of making a delayed result look newly completed. */
			completed_at = cnn_start_time;
			CnnWriteCommand(CNN_CTRL_IRQ_CLEAR);
			completed = 1U;
		}
	}
	if ((saved_cpsr & XIL_EXCEPTION_IRQ) == 0U)
		Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

	if (completed) {
		cnn_last_result = (s8)(raw & 0xFFU);
		++cnn_done_count;
		cnn_result_valid = 1U;
		ui_cnn_done_time = completed_at;
		cnn_waiting = 0U;

	#if STEERING_SOURCE_CNN
		vehicleSetCnnSteering((int8_t)cnn_last_result, completed_at);

	#endif

		//xil_printf("CNN: done #%u status=%08X result_raw=%02X result=%d\r\n",
		//	cnn_done_count, (unsigned)status, (unsigned)(raw & 0xFFU),
		//	(int)cnn_last_result);
		return;
	}

	if (now - cnn_start_time >=
	    (XTime)CNN_TIMEOUT_SECONDS * COUNTS_PER_SECOND) {
		status = CnnStatus();
		cnn_waiting = 0U;
		cnn_result_valid = 0U;
		++cnn_timeout_count;
		xil_printf("CNN: timeout #%u status=%08X\r\n",
			cnn_timeout_count, (unsigned)status);
	}
}

void CaptureStartFromLastFrame(void)
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
	CaptureIrqClear();
	capture_irq_pending = 0U;
	capture_source_addr = source;
	capture_start_frame_count = s2mm_done_count;
	capture_source_reused = 0U;
	capture_result_valid = 0U;
	capture_waiting = 1U;
	XTime_GetTime(&capture_start_time);
	capture_next_poll_time = capture_start_time;
	/* Publish request state before the hardware can complete. */
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_CTRL_OFFSET, 1U);
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_CTRL_OFFSET, 0U);
//	xil_printf("capture: start src=%08X buf=%u crop=(%u,%u) status=%08X\r\n",
//		(unsigned)source, index, (unsigned)CAPTURE_CROP_X,
//		(unsigned)CAPTURE_CROP_Y, (unsigned)CaptureStatus());
}

void CapturePoll(void)
{
	u32 status;
	unsigned int completed;
	unsigned int frame_count;
	XTime now;

	if (!capture_waiting)
		return;
	XTime_GetTime(&now);
	completed = capture_irq_pending;
	if (!completed && now < capture_next_poll_time)
		return;
	capture_next_poll_time = now + COUNTS_PER_SECOND / 100U;
	status = CaptureStatus();
	/* Use the ISR snapshot so foreground delays do not invalidate an
	 * already completed crop. Status reads below are for diagnostics only. */
	frame_count = completed ? capture_irq_frame_count : s2mm_done_count;
	/* After the other two buffers complete, S2MM can start reusing this one. */
	if (!capture_source_reused &&
	    frame_count - capture_start_frame_count >= NUM_FRAME_BUFFERS - 1U) {
		capture_source_reused = 1U;
		capture_result_valid = 0U;
		xil_printf("capture: DDR source was reused; result will be discarded\r\n");
	}
	if (completed) {
		capture_irq_pending = 0U;
		capture_result_valid = capture_source_reused ? 0U : 1U;
		capture_waiting = 0U;
		if (capture_result_valid) {
			//xil_printf("capture: hardware done src=%08X status=%08X (r=summary, x=download)\r\n",
			//	(unsigned)capture_source_addr, (unsigned)status);
			if (CnnStartFromCapture() != XST_SUCCESS) {
				xil_printf("capture: DONE received but CNN start failed\r\n");
			}
			//	xil_printf("capture: CNN did not start; r/x result remains available\r\n");
		} else {
			//xil_printf("capture: hardware done after source reuse; result discarded\r\n");
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

void CaptureTimerPoll(void)
{
#if STEERING_SOURCE_CNN

    u32 tick = timer_irq_count;
    u32 cnn_status;
    u32 capture_status;

    if (capture_mode != CAPTURE_MODE_DEMO) {
        capture_timer_seen = tick;
        return;
    }
    if (tick == capture_timer_seen)
        return;
    /* Consume the tick even when busy; never queue a burst of old requests. */
    capture_timer_seen = tick;

    /*
     * Skip this tick if a capture or CNN job is still running.
     */
    if (capture_waiting)
        return;

    if (cnn_waiting)
        return;

    cnn_status = CnnStatus();

    if (cnn_status &
        (CNN_STATUS_BUSY | CNN_STATUS_START_PENDING))
        return;

    capture_status = CaptureStatus();

    if (capture_status & CAPTURE_STS_BUSY)
        return;

    /*
     * Skip until at least one camera -> DDR frame has been finished.
     */
    if (s2mm_valid_count == 0U)
        return;

    /*
     * Everything is free: start the next capture + CNN run.
     */
    CaptureStartFromLastFrame();

#endif
}
