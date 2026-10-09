/*
 * gpio_driver.h
 *
 *  Created on: 2026. 9. 30.
 *      Author: kccistc
 */

#ifndef SRC_GPIO_GPIO_H_
#define SRC_GPIO_GPIO_H_

#include "xil_types.h"

/* External JA1 signal: io_port_0[0] on package pin N15. */
#define GPIO_JA1_BIT_MASK 0x0001U
#define GPIO_JA2_BIT_MASK 0x0002U
#define GPIO_JA3_BIT_MASK 0x0004U
/* Configure the selected JA pin as an input while preserving
 * the direction of the other GPIO pins. */

void gpio_ja1_init_input(void);
void gpio_ja2_init_input(void);
void gpio_ja3_init_input(void);

/* Returns the raw voltage level: 0 = low, 1 = high. */
u32 gpio_ja1_read_level(void);
u32 gpio_ja2_read_level(void);
u32 gpio_ja3_read_level(void);

u32 gpio_ja1_irq_status(void);
u32 gpio_ja2_irq_status(void);
u32 gpio_ja3_irq_status(void);

void gpio_ja1_irq_clear(void);
void gpio_ja2_irq_clear(void);
void gpio_ja3_irq_clear(void);

#endif /* SRC_GPIO_GPIO_H_ */
