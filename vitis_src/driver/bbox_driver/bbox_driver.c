#include "bbox_driver.h"
#include "../capture_driver/capture_driver.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "BBOX.h"

int BboxInitialize(void)
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

void BboxToggle(void)
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

void BboxDisable(void)
{
    BBOX_mWriteReg(
        BBOX_BASEADDR,
        BBOX_S00_AXI_SLV_REG0_OFFSET,
        0U
    );
}
