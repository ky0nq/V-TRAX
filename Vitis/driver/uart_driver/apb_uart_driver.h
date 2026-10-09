#ifndef APB_UART_DRIVER_H
#define APB_UART_DRIVER_H

#include "xparameters.h"
#include "xil_io.h"
#include "xstatus.h"
#include "xtime_l.h"

/* AXI_APB_0 occupies 0x43C00000..0x43C0FFFF in the current XSA.
 * The custom bridge selects UART when PADDR[14:12] is 1. */
#define APB_UART_BASE             (XPAR_AXI_APB_0_BASEADDR + 0x1000U)
#define APB_UART_RX_DATA_OFF      0x0U
#define APB_UART_TX_DATA_OFF      0x2U
#define APB_UART_STATUS_OFF       0x4U
#define APB_UART_IRQ_EN_OFF       0x6U

/* A byte at 115200 baud takes about 87 us. Bound the wait in case TX stalls. */
#define APB_UART_TX_TIMEOUT_TICKS (COUNTS_PER_SECOND / 500U)

static inline void apb_uart_init(void)
{
    Xil_Out16(APB_UART_BASE + APB_UART_IRQ_EN_OFF, 0U);
}

static inline int apb_uart_tx_busy(void)
{
    return (Xil_In16(APB_UART_BASE + APB_UART_STATUS_OFF) & 1U) != 0U;
}

static inline int apb_uart_send_byte(u8 data)
{
    XTime start, now;

    XTime_GetTime(&start);
    while (apb_uart_tx_busy()) {
        XTime_GetTime(&now);
        if ((now - start) >= APB_UART_TX_TIMEOUT_TICKS)
            return XST_FAILURE;
    }

    Xil_Out16(APB_UART_BASE + APB_UART_TX_DATA_OFF,
              (u16)(((u16)data << 1) | 1U));
    return XST_SUCCESS;
}

static inline int apb_uart_send_buf(const u8 *buf, unsigned int len)
{
    unsigned int i;

    for (i = 0U; i < len; ++i) {
        if (apb_uart_send_byte(buf[i]) != XST_SUCCESS)
            return XST_FAILURE;
    }
    return XST_SUCCESS;
}

#endif /* APB_UART_DRIVER_H */
