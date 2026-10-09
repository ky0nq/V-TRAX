#include "cnn_driver.h"
#include "xil_io.h"

u32 CnnStatus(void)
{
	return Xil_In32(CNN_BASEADDR + CNN_STATUS_OFFSET);
}

u32 CnnResult(void)
{
	return Xil_In32(CNN_BASEADDR + CNN_RESULT_OFFSET);
}

void CnnWriteCommand(u32 command)
{
	/* CONTROL bit 2 is stored by the RTL. Keep it set on every command so
	 * START or IRQ_CLEAR writes do not accidentally disable the IRQ output. */
	Xil_Out32(CNN_BASEADDR + CNN_CTRL_OFFSET,
		command | CNN_CTRL_IRQ_ENABLE);
}
