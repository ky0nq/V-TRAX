#ifndef TIMER_DRIVER_H
#define TIMER_DRIVER_H

#include "xparameters.h"
#include "xil_types.h"

#define TIMER_INTR_ID XPAR_FABRIC_APB_TIMER_0_O_IRQ_INTR
#define TIMER_BASEADDR (XPAR_AXI_APB_0_BASEADDR + 0x5000U)
#define TIMER_CR_OFFSET 0x00U
#define TIMER_PSC_OFFSET 0x02U
#define TIMER_ARR_OFFSET 0x04U
#define TIMER_IRQ_OFFSET 0x06U
/* PCLK = PS FCLK_CLK0 = 100 MHz in the current block design.
 * Period = (PSC + 1) * (ARR + 1) / PCLK = 10 ms. */
#define TIMER_PSC_VALUE 99U
#define TIMER_ARR_VALUE 9999U

void TimerInitialize(void);
void TimerStart(void);
u32 TimerIrqStatus(void);
void TimerIrqClear(void);

#endif
