#include "display_mode.h"
#include "../../driver/video_pipeline_driver/video_pipeline_driver.h"
#include "../../driver/dma_driver/dma_driver.h"
#include "xil_printf.h"
#include "xtime_l.h"
#include "sleep.h"
#include "xstatus.h"

#define BRAM_LOADING_ADDR       XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#define ROM_PREVIEW_SECONDS     3U

static unsigned int rom_preview_active = 0U;
static XTime rom_preview_start_time;

int SwitchDisplayToLive(void)
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

void RomPreviewStart(void)
{
	/* MM2S switches to this fixed source at the next frame boundary. */
	vdma_set_fixed_source(&vdma, BRAM_LOADING_ADDR);
	XTime_GetTime(&rom_preview_start_time);
	rom_preview_active = 1U;
	xil_printf("ROM display enabled for %u seconds at %08X\r\n",
		(unsigned)ROM_PREVIEW_SECONDS, (unsigned)BRAM_LOADING_ADDR);
}

void RomPreviewPoll(void)
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
