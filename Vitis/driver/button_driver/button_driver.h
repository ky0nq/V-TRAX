#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include "xparameters.h"

/* AXI GPIO channel 1 is connected to the four active-high board buttons. */
#define BUTTON_GPIO_BASEADDR     XPAR_AXI_GPIO_0_BASEADDR
#define BUTTON_GPIO_DATA_OFFSET  0x00U
#define BUTTON_MASK              0x0FU

#endif /* BUTTON_DRIVER_H */
