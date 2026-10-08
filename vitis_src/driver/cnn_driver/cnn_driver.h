#ifndef CNN_DRIVER_H
#define CNN_DRIVER_H

#include "xil_types.h"
#include "xparameters.h"

#define CNN_BASEADDR             XPAR_IP_CNN_0_BASEADDR
#define CNN_CTRL_OFFSET          0x00U
#define CNN_STATUS_OFFSET        0x04U
#define CNN_RESULT_OFFSET        0x08U
#define CNN_CTRL_START           0x01U
#define CNN_CTRL_IRQ_CLEAR       0x02U
#define CNN_CTRL_IRQ_ENABLE      0x04U
#define CNN_STATUS_BUSY          0x01U
#define CNN_STATUS_START_READY   0x02U
#define CNN_STATUS_DONE          0x04U
#define CNN_STATUS_START_PENDING 0x08U

u32 CnnStatus(void);
u32 CnnResult(void);
void CnnWriteCommand(u32 command);

#endif /* CNN_DRIVER_H */
