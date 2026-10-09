#include "timer_driver.h"
#include "xil_io.h"

void TimerInitialize(void)
{
	Xil_Out16(TIMER_BASEADDR + TIMER_CR_OFFSET, 0U);
	Xil_Out16(TIMER_BASEADDR + TIMER_PSC_OFFSET, TIMER_PSC_VALUE);
	Xil_Out16(TIMER_BASEADDR + TIMER_ARR_OFFSET, TIMER_ARR_VALUE);
	TimerIrqClear();
	/* RTL retains counter state when stopped; initialize once after reset. */
}

void TimerStart(void)
{
	/* CR bit0: count enable, bit1: IRQ enable. */
	Xil_Out16(TIMER_BASEADDR + TIMER_CR_OFFSET, 3U);
}

u32 TimerIrqStatus(void)
{
	return Xil_In16(TIMER_BASEADDR + TIMER_IRQ_OFFSET) & 1U;
}

void TimerIrqClear(void)
{
	/* Pending bit is write-one-to-clear. */
	Xil_Out16(TIMER_BASEADDR + TIMER_IRQ_OFFSET, 1U);
}
