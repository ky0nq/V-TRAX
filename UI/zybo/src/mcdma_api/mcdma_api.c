#include "mcdma_api.h"

#include <string.h>

#include "xil_cache.h"
#include "xil_io.h"
#include "xil_printf.h"

static u32 vdma_read(const VdmaHandle *ctx, u32 offset)
{
	return Xil_In32(ctx->base_address + offset);
}

static void vdma_write(const VdmaHandle *ctx, u32 offset, u32 value)
{
	Xil_Out32(ctx->base_address + offset, value);
}

static u32 vdma_mm2s_control(const VdmaHandle *ctx)
{
	return vdma_read(ctx, VDMA_MM2S_CR_OFFSET);
}

int vdma_configure(VdmaHandle *ctx, UINTPTR base_address,
		u32 frame_size_bytes, const UINTPTR *buffer_addresses,
		u32 num_buffers, UINTPTR initial_mm2s_address)
{
	u32 i;

	if (ctx == NULL || buffer_addresses == NULL || base_address == 0U ||
	    frame_size_bytes == 0U || (frame_size_bytes & 3U) != 0U ||
	    num_buffers != VDMA_BUFFER_COUNT ||
	    (initial_mm2s_address & 3U) != 0U)
		return XST_INVALID_PARAM;

	memset(ctx, 0, sizeof(*ctx));
	ctx->base_address = base_address;
	ctx->frame_size_bytes = frame_size_bytes;
	ctx->num_buffers = num_buffers;
	ctx->mm2s_fixed_source = 1U;
	ctx->mm2s_fixed_addr = initial_mm2s_address;

	for (i = 0U; i < num_buffers; ++i) {
		if ((buffer_addresses[i] & 3U) != 0U)
			return XST_INVALID_PARAM;
		ctx->buffer_address[i] = buffer_addresses[i];
		/* Remove dirty CPU cache lines before S2MM owns the buffers. */
		Xil_DCacheFlushRange((INTPTR)buffer_addresses[i], frame_size_bytes);
	}

	/* Quiesce MM2S and clear stale level-sensitive interrupt status. */
	vdma_write(ctx, VDMA_MM2S_CR_OFFSET, VDMA_CR_ABORT_MASK);
	vdma_write(ctx, VDMA_MM2S_CR_OFFSET, 0U);
	vdma_write(ctx, VDMA_MM2S_SR_OFFSET, VDMA_SR_IRQ_MASK);
	vdma_write(ctx, VDMA_S2MM_CR_OFFSET, 0U);
	vdma_write(ctx, VDMA_S2MM_SR_OFFSET, VDMA_SR_IRQ_MASK);

	vdma_write(ctx, VDMA_S2MM_DA0_OFFSET, (u32)buffer_addresses[0]);
	vdma_write(ctx, VDMA_S2MM_DA1_OFFSET, (u32)buffer_addresses[1]);
	vdma_write(ctx, VDMA_S2MM_DA2_OFFSET, (u32)buffer_addresses[2]);

	vdma_write(ctx, VDMA_MM2S_SA_OFFSET, (u32)initial_mm2s_address);
	vdma_write(ctx, VDMA_MM2S_BURST_OFFSET, VDMA_MM2S_DEFAULT_BURST);
	vdma_write(ctx, VDMA_MM2S_NUM_BUF_OFFSET, num_buffers);
	vdma_write(ctx, VDMA_MM2S_SW_IDX_OFFSET, 0U);

	return XST_SUCCESS;
}

int vdma_start(VdmaHandle *ctx)
{
	u32 irq_enable;

	if (ctx == NULL || ctx->base_address == 0U ||
	    ctx->num_buffers != VDMA_BUFFER_COUNT)
		return XST_INVALID_PARAM;

	irq_enable = VDMA_CR_IOC_IRQEN_MASK | VDMA_CR_ERR_IRQEN_MASK;

	/* S2MM runs continuously through DA0, DA1 and DA2 after one start pulse. */
	vdma_write(ctx, VDMA_S2MM_CR_OFFSET, irq_enable);
	vdma_write(ctx, VDMA_S2MM_START_OFFSET, 1U);

	/* Start on the fixed loading image. LIVE is enabled later at a frame edge. */
	vdma_write(ctx, VDMA_MM2S_CR_OFFSET,
		VDMA_CR_CYCLIC_MASK | irq_enable);
	vdma_write(ctx, VDMA_MM2S_BTT_OFFSET, ctx->frame_size_bytes);

	return XST_SUCCESS;
}

u32 vdma_read_mm2s_status(const VdmaHandle *ctx)
{
	return vdma_read(ctx, VDMA_MM2S_SR_OFFSET);
}

u32 vdma_read_s2mm_status(const VdmaHandle *ctx)
{
	return vdma_read(ctx, VDMA_S2MM_SR_OFFSET);
}

u32 vdma_ack_mm2s_irq(VdmaHandle *ctx)
{
	u32 status = vdma_read_mm2s_status(ctx);
	u32 pending = status & VDMA_SR_IRQ_MASK;

	ctx->mm2s_cur_idx = (status & VDMA_SR_INDEX_MASK) >> VDMA_SR_INDEX_SHIFT;
	if (pending != 0U)
		vdma_write(ctx, VDMA_MM2S_SR_OFFSET, pending);
	return status;
}

u32 vdma_ack_s2mm_irq(VdmaHandle *ctx)
{
	u32 status = vdma_read_s2mm_status(ctx);
	u32 pending = status & VDMA_SR_IRQ_MASK;

	ctx->newest_rx_idx = (status & VDMA_SR_INDEX_MASK) >> VDMA_SR_INDEX_SHIFT;
	if (pending != 0U)
		vdma_write(ctx, VDMA_S2MM_SR_OFFSET, pending);
	return status;
}

void vdma_set_fixed_source(VdmaHandle *ctx, UINTPTR address)
{
	u32 control;

	if (ctx == NULL || (address & 3U) != 0U)
		return;

	vdma_write(ctx, VDMA_MM2S_SA_OFFSET, (u32)address);
	control = vdma_mm2s_control(ctx);
	control |= VDMA_CR_CYCLIC_MASK;
	control &= ~(VDMA_CR_LIVE_MASK | VDMA_CR_IDX_SW_MASK | VDMA_CR_ABORT_MASK);
	vdma_write(ctx, VDMA_MM2S_CR_OFFSET, control);
	ctx->mm2s_fixed_addr = address;
	ctx->mm2s_fixed_source = 1U;
}

void vdma_set_live(VdmaHandle *ctx)
{
	u32 control;

	if (ctx == NULL)
		return;

	control = vdma_mm2s_control(ctx);
	control |= VDMA_CR_CYCLIC_MASK | VDMA_CR_LIVE_MASK;
	control &= ~(VDMA_CR_IDX_SW_MASK | VDMA_CR_ABORT_MASK);
	vdma_write(ctx, VDMA_MM2S_CR_OFFSET, control);
	ctx->mm2s_fixed_source = 0U;
}

void vdma_dump_status(VdmaHandle *ctx)
{
	u32 mm2s_cr;
	u32 mm2s_sr;
	u32 s2mm_cr;
	u32 s2mm_sr;

	if (ctx == NULL)
		return;

	mm2s_cr = vdma_read(ctx, VDMA_MM2S_CR_OFFSET);
	mm2s_sr = vdma_read_mm2s_status(ctx);
	s2mm_cr = vdma_read(ctx, VDMA_S2MM_CR_OFFSET);
	s2mm_sr = vdma_read_s2mm_status(ctx);

	xil_printf("VDMA MM2S: CR=%08X SR=%08X SA=%08X BTT=%08X "
		"READ_ERR=%08X\r\n",
		(unsigned)mm2s_cr, (unsigned)mm2s_sr,
		(unsigned)vdma_read(ctx, VDMA_MM2S_SA_OFFSET),
		(unsigned)vdma_read(ctx, VDMA_MM2S_BTT_OFFSET),
		(unsigned)vdma_read(ctx, VDMA_MM2S_READ_ERR_OFFSET));
	xil_printf("VDMA S2MM: CR=%08X SR=%08X DA=%08X/%08X/%08X "
		"WRITE_ERR=%08X\r\n",
		(unsigned)s2mm_cr, (unsigned)s2mm_sr,
		(unsigned)vdma_read(ctx, VDMA_S2MM_DA0_OFFSET),
		(unsigned)vdma_read(ctx, VDMA_S2MM_DA1_OFFSET),
		(unsigned)vdma_read(ctx, VDMA_S2MM_DA2_OFFSET),
		(unsigned)vdma_read(ctx, VDMA_S2MM_WRITE_ERR_OFFSET));
}
