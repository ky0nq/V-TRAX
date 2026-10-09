/*
 * gpio_driver.c
 *
 *  Created on: 2026. 9. 30.
 *      Author: kccistc
 */

#include "gpio_driver.h"

#include "xil_io.h"
#include "xparameters.h"


/* The AXI-to-APB bridge selects GPIO when PADDR[14:12] is 3.
 * APB_to_GPIO: CR at +0, IDR at +2; CR bit 0 = 0 selects input. */
#define GPIO_APB_BASEADDR (XPAR_AXI_APB_0_BASEADDR + 0x3000U)
#define GPIO_CR_OFFSET   0x0000U
#define GPIO_IDR_OFFSET  0x0002U
#define GPIO_IRQ_OFFSET 0x0006U

void gpio_ja1_init_input(void)
{

	u16 direction = Xil_In16(GPIO_APB_BASEADDR + GPIO_CR_OFFSET);
	Xil_Out16(GPIO_APB_BASEADDR + GPIO_CR_OFFSET,
		(u16)(direction & (u16)~GPIO_JA1_BIT_MASK));
}

u32 gpio_ja1_read_level(void)
{
	return (Xil_In16(GPIO_APB_BASEADDR + GPIO_IDR_OFFSET) &
		GPIO_JA1_BIT_MASK) != 0U;
}


u32 gpio_ja1_irq_status(void)
{
    return Xil_In16(GPIO_APB_BASEADDR + GPIO_IRQ_OFFSET)
           & GPIO_JA1_BIT_MASK;
}

void gpio_ja1_irq_clear(void)
{
    /* Write 1 to clear the IRQ status. */
    Xil_Out16(GPIO_APB_BASEADDR + GPIO_IRQ_OFFSET,
              (u16)GPIO_JA1_BIT_MASK);
}

void gpio_ja2_init_input(void)
{
    u16 direction = Xil_In16(GPIO_APB_BASEADDR + GPIO_CR_OFFSET);

    Xil_Out16(GPIO_APB_BASEADDR + GPIO_CR_OFFSET,
              (u16)(direction & (u16)~GPIO_JA2_BIT_MASK));
}

void gpio_ja3_init_input(void)
{
    u16 direction = Xil_In16(GPIO_APB_BASEADDR + GPIO_CR_OFFSET);

    Xil_Out16(GPIO_APB_BASEADDR + GPIO_CR_OFFSET,
              (u16)(direction & (u16)~GPIO_JA3_BIT_MASK));
}

u32 gpio_ja2_read_level(void)
{
    return (Xil_In16(GPIO_APB_BASEADDR + GPIO_IDR_OFFSET) &
            GPIO_JA2_BIT_MASK) != 0U;
}

u32 gpio_ja3_read_level(void)
{
    return (Xil_In16(GPIO_APB_BASEADDR + GPIO_IDR_OFFSET) &
            GPIO_JA3_BIT_MASK) != 0U;
}

u32 gpio_ja2_irq_status(void)
{
    return Xil_In16(GPIO_APB_BASEADDR + GPIO_IRQ_OFFSET) &
           GPIO_JA2_BIT_MASK;
}

u32 gpio_ja3_irq_status(void)
{
    return Xil_In16(GPIO_APB_BASEADDR + GPIO_IRQ_OFFSET) &
           GPIO_JA3_BIT_MASK;
}

void gpio_ja2_irq_clear(void)
{
    Xil_Out16(GPIO_APB_BASEADDR + GPIO_IRQ_OFFSET,
              (u16)GPIO_JA2_BIT_MASK);
}

void gpio_ja3_irq_clear(void)
{
    Xil_Out16(GPIO_APB_BASEADDR + GPIO_IRQ_OFFSET,
              (u16)GPIO_JA3_BIT_MASK);
}
