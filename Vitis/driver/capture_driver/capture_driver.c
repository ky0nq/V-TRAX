#include "capture_driver.h"
#include "xil_io.h"

u32 CaptureStatus(void)
{
	return Xil_In32(CAPTURE_BASEADDR + CAPTURE_STS_OFFSET);
}

void CaptureIrqClear(void)
{
	/* ACK only clears irq_pending; CAPTURE_DONE remains readable. */
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_CTRL_OFFSET, CAPTURE_CTRL_IRQ_CLEAR);
}

void CaptureSelectReadPort(unsigned int cpu_read)
{
	/* READ_SELECT also stores the interrupt enable bit. */
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_READ_SELECT_OFFSET,
		CAPTURE_IRQ_ENABLE | (cpu_read ? CAPTURE_READ_CPU_ENABLE : 0U));
}

u32 CaptureReadPixel(unsigned int x, unsigned int y)
{
	u32 index = y * CAPTURE_RESULT_SIZE + x;
	Xil_Out32(CAPTURE_BASEADDR + CAPTURE_PIXEL_INDEX_OFFSET, index);
	return Xil_In32(CAPTURE_BASEADDR + CAPTURE_PIXEL_DATA_OFFSET) & 0x00FFFFFFU;
}
