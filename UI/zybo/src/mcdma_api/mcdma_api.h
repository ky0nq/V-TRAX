#ifndef CUSTOM_VDMA_API_H
#define CUSTOM_VDMA_API_H

#include "xil_types.h"
#include "xstatus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Custom VDMA AXI-Lite register map. */
#define VDMA_MM2S_CR_OFFSET        0x00U
#define VDMA_MM2S_SR_OFFSET        0x04U
#define VDMA_MM2S_SA_OFFSET        0x18U
#define VDMA_MM2S_READ_ERR_OFFSET  0x1CU
#define VDMA_MM2S_BTT_OFFSET       0x28U
#define VDMA_MM2S_BURST_OFFSET     0x30U
#define VDMA_MM2S_NUM_BUF_OFFSET   0x38U
#define VDMA_MM2S_SW_IDX_OFFSET    0x3CU
#define VDMA_S2MM_CR_OFFSET        0x40U
#define VDMA_S2MM_SR_OFFSET        0x44U
#define VDMA_S2MM_DA0_OFFSET       0x48U
#define VDMA_S2MM_DA1_OFFSET       0x4CU
#define VDMA_S2MM_DA2_OFFSET       0x50U
#define VDMA_S2MM_START_OFFSET     0x54U
#define VDMA_S2MM_WRITE_ERR_OFFSET 0x58U

#define VDMA_CR_ABORT_MASK         (1U << 2)
#define VDMA_CR_CYCLIC_MASK        (1U << 4)
#define VDMA_CR_LIVE_MASK          (1U << 5)
#define VDMA_CR_IDX_SW_MASK        (1U << 6)
#define VDMA_CR_IOC_IRQEN_MASK     (1U << 12)
#define VDMA_CR_ERR_IRQEN_MASK     (1U << 14)

#define VDMA_SR_BUSY_MASK          (1U << 0)
#define VDMA_SR_IDLE_MASK          (1U << 1)
#define VDMA_SR_ERROR_MASK         (1U << 4)
#define VDMA_SR_INDEX_MASK         (7U << 8)
#define VDMA_SR_INDEX_SHIFT        8U
#define VDMA_SR_IOC_IRQ_MASK       (1U << 12)
#define VDMA_SR_ERR_IRQ_MASK       (1U << 14)
#define VDMA_SR_IRQ_MASK           (VDMA_SR_IOC_IRQ_MASK | VDMA_SR_ERR_IRQ_MASK)

#define VDMA_MM2S_DEFAULT_BURST    0x10FU /* INCR, ARLEN=15: 16 beats */
#define VDMA_BUFFER_COUNT          3U

typedef struct {
	UINTPTR base_address;
	u32 frame_size_bytes;
	u32 num_buffers;
	UINTPTR buffer_address[VDMA_BUFFER_COUNT];
	volatile u32 newest_rx_idx;
	volatile u32 mm2s_cur_idx;
	volatile u32 mm2s_fixed_source;
	volatile UINTPTR mm2s_fixed_addr;
} VdmaHandle;

int vdma_configure(VdmaHandle *ctx, UINTPTR base_address,
		u32 frame_size_bytes, const UINTPTR *buffer_addresses,
		u32 num_buffers, UINTPTR initial_mm2s_address);
int vdma_start(VdmaHandle *ctx);

u32 vdma_read_mm2s_status(const VdmaHandle *ctx);
u32 vdma_read_s2mm_status(const VdmaHandle *ctx);
u32 vdma_ack_mm2s_irq(VdmaHandle *ctx);
u32 vdma_ack_s2mm_irq(VdmaHandle *ctx);

void vdma_set_fixed_source(VdmaHandle *ctx, UINTPTR address);
void vdma_set_live(VdmaHandle *ctx);
void vdma_dump_status(VdmaHandle *ctx);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_VDMA_API_H */
