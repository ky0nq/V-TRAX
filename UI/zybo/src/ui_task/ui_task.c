#include "ui_task.h"
#include "../video_pipeline_driver/video_pipeline_driver.h"
#include "../capture_task/capture_task.h"
#include "../vehicle_task/vehicle_task.h"
#include "ui_stream.h"
#include "xtime_l.h"

/* Take the newest completed custom-VDMA buffer into the UDP snapshot copier.
 * Keep all Ethernet work outside the S2MM interrupt handler. */
void UiApplicationService(void)
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
	UiStream_Service(
	    address,
	    FRAME_WIDTH,
	    FRAME_HEIGHT,
	    (int)cnn_last_result,
	    ui_pressure_percent,
	    before,
	    &s2mm_done_count,
	    cnn_result_valid && age < UI_CNN_FRESH_MS,
	    age
	);
}
