#include "video_pipeline_driver.h"
#include "../timer_driver/timer_driver.h"
#include "../capture_driver/capture_driver.h"
#include "../cnn_driver/cnn_driver.h"
#include "../../application/capture_cnn_ctrl/capture_cnn_ctrl.h"     /* capture_download_active, capture_waiting, ... */
#include "../gpio_driver/gpio_driver.h"                     /* gpio_ja{1,2,3}_irq_status/clear/read_level */
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_io.h"
#include "xil_printf.h"

VdmaHandle   vdma;
XScuGic      IntcInstance;
volatile u32 ja1_irq_count = 0U;
volatile u32 ja2_irq_count = 0U;
volatile u32 ja3_irq_count = 0U;
volatile unsigned int cnn_irq_pending = 0U;
volatile unsigned int capture_irq_pending = 0U;
volatile u32 timer_irq_count = 0U;
volatile unsigned int capture_irq_count = 0U;
volatile unsigned int capture_irq_frame_count = 0U;
volatile unsigned int cnn_irq_count = 0U;
volatile u32 cnn_irq_last_status = 0U;
volatile u32 cnn_irq_last_result = 0U;
volatile XTime cnn_irq_done_time = 0;

volatile unsigned int s2mm_done_count = 0;
volatile unsigned int s2mm_valid_count = 0;
volatile unsigned int mm2s_done_count = 0;
volatile unsigned int s2mm_error_count = 0;
volatile unsigned int mm2s_error_count = 0;
volatile u32 s2mm_last_error = 0;
volatile u32 mm2s_last_error = 0;
volatile u32 s2mm_last_status = 0;
volatile u32 mm2s_last_status = 0;
volatile unsigned int s2mm_last_buffer_idx = 0;
volatile unsigned int mm2s_last_buffer_idx = 0;

#define FRAME_BUFFER_0_ADDR ((UINTPTR)0x02000000U)
#define FRAME_BUFFER_1_ADDR ((UINTPTR)0x03000000U)
#define FRAME_BUFFER_2_ADDR ((UINTPTR)0x04000000U)

const UINTPTR frame_buffer_addresses[NUM_FRAME_BUFFERS] = {
	FRAME_BUFFER_0_ADDR,
	FRAME_BUFFER_1_ADDR,
	FRAME_BUFFER_2_ADDR
};

static void S2MM_IntrHandler(void *CallBackRef)
{
	VdmaHandle *ctx = (VdmaHandle *)CallBackRef;
	u32 status = vdma_ack_s2mm_irq(ctx);

	s2mm_last_status = status;
	if (status & VDMA_SR_IOC_IRQ_MASK) {
		s2mm_last_buffer_idx = ctx->newest_rx_idx;
		++s2mm_done_count;
		if ((status & VDMA_SR_ERROR_MASK) == 0U)
			++s2mm_valid_count;
	}
	if (status & VDMA_SR_ERR_IRQ_MASK) {
		s2mm_last_error = Xil_In32(ctx->base_address +
			VDMA_S2MM_WRITE_ERR_OFFSET);
		++s2mm_error_count;
	}
}

static void MM2S_IntrHandler(void *CallBackRef)
{
	VdmaHandle *ctx = (VdmaHandle *)CallBackRef;
	u32 status = vdma_ack_mm2s_irq(ctx);

	mm2s_last_status = status;
	if (status & VDMA_SR_IOC_IRQ_MASK) {
		mm2s_last_buffer_idx = ctx->mm2s_fixed_source ?
			NUM_FRAME_BUFFERS : ctx->mm2s_cur_idx;
		++mm2s_done_count;
	}
	if (status & VDMA_SR_ERR_IRQ_MASK) {
		mm2s_last_error = Xil_In32(ctx->base_address +
			VDMA_MM2S_READ_ERR_OFFSET);
		++mm2s_error_count;
	}
}

static void CnnIntrHandler(void *CallBackRef)
{
	u32 status;
	XTime completed_at;

	(void)CallBackRef;
	status = CnnStatus();
	cnn_irq_last_status = status;
	if ((status & CNN_STATUS_DONE) != 0U) {
		XTime_GetTime(&completed_at);
		/* Store the result before clearing the level-sensitive source.  The
		 * foreground task consumes it after seeing cnn_irq_pending. */
		cnn_irq_last_result = CnnResult();
		cnn_irq_done_time = completed_at;
		++cnn_irq_count;
		cnn_irq_pending = 1U;
	}
	CnnWriteCommand(CNN_CTRL_IRQ_CLEAR);
}

static void TimerIntrHandler(void *CallBackRef)
{
	(void)CallBackRef;
	if (TimerIrqStatus() != 0U) {
		TimerIrqClear();
		++timer_irq_count;
	}
}

static void CaptureIntrHandler(void *CallBackRef)
{
	u32 status = CaptureStatus();
	(void)CallBackRef;
	if ((status & (CAPTURE_STS_DONE | CAPTURE_STS_BUSY)) == CAPTURE_STS_DONE) {
		capture_irq_frame_count = s2mm_done_count;
		++capture_irq_count;
		capture_irq_pending = 1U;
	}
	CaptureIrqClear();
}

/* UART output is deliberately deferred out of the interrupt handlers. */
void VdmaLogPoll(void)
{
	static unsigned int s2mm_done_seen = 0U;
	static unsigned int mm2s_done_seen = 0U;
	static unsigned int s2mm_error_seen = 0U;
	static unsigned int mm2s_error_seen = 0U;
	unsigned int count;

	if (capture_download_active)
		return;

	count = s2mm_done_count;
	if (count != s2mm_done_seen) {
		if (count <= 3U || (count % 60U) == 0U){}
//			xil_printf("VDMA S2MM done=%lu valid=%lu buf=%lu\r\n",
//				(unsigned long)count, (unsigned long)s2mm_valid_count,
//				(unsigned long)s2mm_last_buffer_idx);
		s2mm_done_seen = count;
	}
	count = mm2s_done_count;
	if (count != mm2s_done_seen) {
		if (count <= 3U || (count % 60U) == 0U){}
//			xil_printf("VDMA MM2S done=%lu buf=%lu\r\n",
//				(unsigned long)count,
//				(unsigned long)mm2s_last_buffer_idx);
		mm2s_done_seen = count;
	}

	count = s2mm_error_count;
	if (count != s2mm_error_seen) {
		if (count <= 3U || (count % 60U) == 0U)
			xil_printf("VDMA S2MM ERROR count=%lu addr=%08X sr=%08X\r\n",
				(unsigned long)count, (unsigned)s2mm_last_error,
				(unsigned)s2mm_last_status);
		s2mm_error_seen = count;
	}
	count = mm2s_error_count;
	if (count != mm2s_error_seen) {
		if (count <= 3U || (count % 60U) == 0U)
			xil_printf("VDMA MM2S ERROR count=%lu addr=%08X sr=%08X\r\n",
				(unsigned long)count, (unsigned)mm2s_last_error,
				(unsigned)mm2s_last_status);
		mm2s_error_seen = count;
	}
}

/* JA press event state: accept the first IRQ immediately, then ignore
 * further IRQs on the same pin for 300 ms after the last accepted event.
 * Rejected IRQs never extend this interval. GPIO handlers own timestamps;
 * only the foreground consumer owns seen/consumed/discarded.
 */
#define JA_COOLDOWN_COUNTS ((XTime)COUNTS_PER_SECOND * JA_GPIO_COOLDOWN_MS / 1000U)
typedef struct {
    unsigned int has_accepted;
    XTime last_accepted;
    volatile u32 isr_entry;
    volatile u32 status_zero;
    volatile u32 received;
    volatile u32 rejected;
    u32 seen;
    u32 consumed;
    u32 discarded;
} JaPressGate;

static JaPressGate ja1_press_gate;
static JaPressGate ja2_press_gate;

static int JaCanAccept(const JaPressGate *gate, XTime now)
{
    return !gate->has_accepted ||
        now - gate->last_accepted >= JA_COOLDOWN_COUNTS;
}

static void JaAcceptPress(JaPressGate *gate, volatile u32 *accepted)
{
    XTime now;

    XTime_GetTime(&now);
    ++gate->received;
    if (JaCanAccept(gate, now)) {
        gate->last_accepted = now;
        gate->has_accepted = 1U;
        ++*accepted;
    } else {
        ++gate->rejected;
    }
}

/* Counters retain every accepted event, even if release precedes consumption.
 * Unsigned arithmetic also preserves pending events across counter wrap.
 */
static int JaPressTake(JaPressGate *gate, u32 accepted)
{
    if (gate->seen == accepted)
        return 0;
    ++gate->seen;
    ++gate->consumed;
    return 1;
}

static void JaPressDiscard(JaPressGate *gate, u32 accepted)
{
    gate->discarded += accepted - gate->seen;
    gate->seen = accepted;
}

int Ja1PressTake(void)
{
    return JaPressTake(&ja1_press_gate, ja1_irq_count);
}

int Ja2PressTake(void)
{
    return JaPressTake(&ja2_press_gate, ja2_irq_count);
}

void Ja1PressDiscard(void)
{
    JaPressDiscard(&ja1_press_gate, ja1_irq_count);
}

void Ja2PressDiscard(void)
{
    JaPressDiscard(&ja2_press_gate, ja2_irq_count);
}

static void Ja1GpioIntrHandler(void *ref)
{
    (void)ref;
    /* Count entry before the peripheral status check, including empty IRQs. */
    ++ja1_press_gate.isr_entry;
    if (gpio_ja1_irq_status() != 0U) {
        gpio_ja1_irq_clear();
        JaAcceptPress(&ja1_press_gate, &ja1_irq_count);
    } else {
        ++ja1_press_gate.status_zero;
    }
}

static void Ja2GpioIntrHandler(void *ref)
{
    (void)ref;
    /* Count entry before the peripheral status check, including empty IRQs. */
    ++ja2_press_gate.isr_entry;
    if (gpio_ja2_irq_status() != 0U) {
        gpio_ja2_irq_clear();
        JaAcceptPress(&ja2_press_gate, &ja2_irq_count);
    } else {
        ++ja2_press_gate.status_zero;
    }
}

/* End JA press event state. */

static void Ja3GpioIntrHandler(void *ref)
{
    (void)ref;

    if (gpio_ja3_irq_status() != 0U) {
        gpio_ja3_irq_clear();
        ++ja3_irq_count;
    }
}

/* UART remains in the foreground; limit diagnostics to 10 Hz.
 * hw_pending is the pin's GPIO IRQ bit sampled by the foreground, whereas
 * pending is the software event queue. Neither read clears the GPIO IRQ.
 * Snapshots can straddle an IRQ, so adjacent counter values may differ briefly.
 */
static void JaPrintStats(unsigned int pin, JaPressGate *gate, u32 accepted,
    u32 level, u32 hw_pending, u32 *entry_seen, u32 *level_seen,
    u32 *hw_pending_seen, u32 *progress_seen, XTime *last_print)
{
    XTime now;
    u32 entry = gate->isr_entry;
    u32 zero = gate->status_zero;
    u32 received = gate->received;

    if (entry == *entry_seen && level == *level_seen &&
        hw_pending == *hw_pending_seen && gate->seen == *progress_seen)
        return;
    XTime_GetTime(&now);
    if (now - *last_print < (XTime)COUNTS_PER_SECOND / 10U)
        return;
    *last_print = now;
    *entry_seen = entry;
    *level_seen = level;
    *hw_pending_seen = hw_pending;
    *progress_seen = gate->seen;
    xil_printf("JA%u level=%lu IRQ isr_entry=%lu status_zero=%lu "
        "hw_pending=%lu rx=%lu accepted=%lu rejected=%lu "
        "consumed=%lu discarded=%lu pending=%lu cooldown_ms=%u\r\n",
        pin, (unsigned long)level, (unsigned long)entry, (unsigned long)zero,
        (unsigned long)hw_pending, (unsigned long)received,
        (unsigned long)accepted, (unsigned long)gate->rejected,
        (unsigned long)gate->consumed, (unsigned long)gate->discarded,
        (unsigned long)(accepted - gate->seen), JA_GPIO_COOLDOWN_MS);
}

void Ja1GpioPrintPoll(void)
{
    static u32 entry_seen = 0U;
    static u32 level_seen = 2U;
    static u32 hw_pending_seen = 2U;
    static u32 progress_seen = 0U;
    static XTime last_print = 0;

    JaPrintStats(1U, &ja1_press_gate, ja1_irq_count, gpio_ja1_read_level(),
        gpio_ja1_irq_status() != 0U, &entry_seen, &level_seen,
        &hw_pending_seen, &progress_seen, &last_print);
}

void Ja2GpioPrintPoll(void)
{
    static u32 entry_seen = 0U;
    static u32 level_seen = 2U;
    static u32 hw_pending_seen = 2U;
    static u32 progress_seen = 0U;
    static XTime last_print = 0;

    JaPrintStats(2U, &ja2_press_gate, ja2_irq_count, gpio_ja2_read_level(),
        gpio_ja2_irq_status() != 0U, &entry_seen, &level_seen,
        &hw_pending_seen, &progress_seen, &last_print);
}

void Ja3GpioPrintPoll(void)
{
    static u32 printed_count = 0U;
    static u32 last_level = 2U;
    u32 count = ja3_irq_count;
    u32 level = gpio_ja3_read_level();

    if (level != last_level) {
        last_level = level;
        xil_printf("JA3 level=%lu\r\n", (unsigned long)level);
    }

    if (count != printed_count) {
        printed_count = count;
        xil_printf("GPIO input detected: JA3, IRQ count=%lu\r\n",
                   (unsigned long)count);
    }
}

int SetupVdmaInterrupts(void)
{
	int Status;
	XScuGic_Config *IntcConfig;

	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
	if (!IntcConfig) return XST_FAILURE;

	Status = XScuGic_CfgInitialize(&IntcInstance, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) return XST_FAILURE;

	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			&IntcInstance);

	/* Clear a stale DONE level and enable the CNN interrupt at the IP. */
	cnn_irq_pending = 0U;
	CnnWriteCommand(CNN_CTRL_IRQ_CLEAR);

	Status = XScuGic_Connect(&IntcInstance, VDMA_S2MM_INTR_ID,
			(Xil_ExceptionHandler)S2MM_IntrHandler, &vdma);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, VDMA_MM2S_INTR_ID,
			(Xil_ExceptionHandler)MM2S_IntrHandler, &vdma);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, JA1_GPIO_INTR_ID,
			(Xil_ExceptionHandler)Ja1GpioIntrHandler, 0);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, JA2_GPIO_INTR_ID,
			(Xil_ExceptionHandler)Ja2GpioIntrHandler, 0);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, JA3_GPIO_INTR_ID,
			(Xil_ExceptionHandler)Ja3GpioIntrHandler, 0);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, CNN_INTR_ID,
			(Xil_ExceptionHandler)CnnIntrHandler, 0);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	Status = XScuGic_Connect(&IntcInstance, CAPTURE_INTR_ID,
			(Xil_ExceptionHandler)CaptureIntrHandler, 0);
	if (Status != XST_SUCCESS) return XST_FAILURE;
	CaptureIrqClear();
	capture_irq_pending = 0U;
	CaptureSelectReadPort(0U);
	TimerInitialize();
	Status = XScuGic_Connect(&IntcInstance, TIMER_INTR_ID,
			(Xil_ExceptionHandler)TimerIntrHandler, 0);
	if (Status != XST_SUCCESS) return XST_FAILURE;

	XScuGic_SetPriorityTriggerType(&IntcInstance, VDMA_S2MM_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_SetPriorityTriggerType(&IntcInstance, VDMA_MM2S_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_SetPriorityTriggerType(&IntcInstance, JA1_GPIO_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_SetPriorityTriggerType(&IntcInstance, JA2_GPIO_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_SetPriorityTriggerType(&IntcInstance, JA3_GPIO_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_SetPriorityTriggerType(&IntcInstance, CNN_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_Enable(&IntcInstance, VDMA_S2MM_INTR_ID);
	XScuGic_Enable(&IntcInstance, VDMA_MM2S_INTR_ID);
	XScuGic_Enable(&IntcInstance, JA1_GPIO_INTR_ID);
	XScuGic_Enable(&IntcInstance, JA2_GPIO_INTR_ID);
	XScuGic_Enable(&IntcInstance, JA3_GPIO_INTR_ID);
	XScuGic_Enable(&IntcInstance, CNN_INTR_ID);
	XScuGic_SetPriorityTriggerType(&IntcInstance, CAPTURE_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_Enable(&IntcInstance, CAPTURE_INTR_ID);
	XScuGic_SetPriorityTriggerType(&IntcInstance, TIMER_INTR_ID,
			0xA0U, 0x1U);
	XScuGic_Enable(&IntcInstance, TIMER_INTR_ID);
	Xil_ExceptionEnable();
	TimerStart();

	return XST_SUCCESS;
}

void PrintVdmaDiagnostics(void)
{
	xil_printf("APB Timer: irq=%lu pending=%lu period=10ms\r\n",
		(unsigned long)timer_irq_count, (unsigned long)TimerIrqStatus());
	xil_printf("VDMA counters: S2MM=%lu valid=%lu MM2S=%lu "
		"errors=%lu/%lu error_addr=%08X/%08X status=%08X/%08X\r\n",
		(unsigned long)s2mm_done_count, (unsigned long)s2mm_valid_count,
		(unsigned long)mm2s_done_count,
		(unsigned long)s2mm_error_count, (unsigned long)mm2s_error_count,
		(unsigned)s2mm_last_error, (unsigned)mm2s_last_error,
		(unsigned)s2mm_last_status, (unsigned)mm2s_last_status);
	xil_printf("capture: status=%08X waiting=%u src=%08X irq=%u pending=%u\r\n",
		(unsigned)CaptureStatus(), capture_waiting,
		(unsigned)capture_source_addr, capture_irq_count, capture_irq_pending);
	xil_printf("CNN: status=%08X waiting=%u starts=%u done=%u timeout=%u "
		"irq=%u pending=%u irq_status=%08X result_valid=%u result=%d\r\n",
		(unsigned)CnnStatus(), cnn_waiting, cnn_start_count,
		cnn_done_count, cnn_timeout_count, cnn_irq_count,
		cnn_irq_pending, (unsigned)cnn_irq_last_status,
		cnn_result_valid, (int)cnn_last_result);
	vdma_dump_status(&vdma);
}
