#include "button_input.h"
#include "../../driver/button_driver/button_driver.h"
#include "../../driver/cnn_driver/cnn_driver.h"
#include "../capture_cnn_ctrl/capture_cnn_ctrl.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xtime_l.h"

/* BTN0 = capture+CNN, BTN1 = print CNN result. */
#define BUTTON_CAPTURE_MASK      0x01U
#define BUTTON_CNN_RESULT_MASK   0x02U
#define BUTTON_DEBOUNCE_TICKS    (COUNTS_PER_SECOND / 50U) /* 20 ms */

static u32 button_raw_state = 0U;
static u32 button_stable_state = 0U;
static XTime button_raw_change_time;

void ButtonInit(void)
{
	button_raw_state = Xil_In32(BUTTON_GPIO_BASEADDR +
		BUTTON_GPIO_DATA_OFFSET) & BUTTON_MASK;
	button_stable_state = button_raw_state;
	XTime_GetTime(&button_raw_change_time);
	xil_printf("buttons: BTN0=capture+CNN, BTN1=print CNN result (state=%X)\r\n",
		(unsigned)button_stable_state);
}

void ButtonPrintCnnResult(void)
{
	u32 status = CnnStatus();
	u32 raw = Xil_In32(CNN_BASEADDR + CNN_RESULT_OFFSET);

	xil_printf("BTN1 CNN: status=%08X waiting=%u valid=%u raw=%02X "
		"result=%d starts=%u done=%u timeout=%u\r\n",
		(unsigned)status, cnn_waiting, cnn_result_valid,
		(unsigned)(raw & 0xFFU), (int)(s8)(raw & 0xFFU),
		cnn_start_count, cnn_done_count, cnn_timeout_count);
}

void ButtonPoll(void)
{
	u32 current;
	u32 pressed;
	XTime now;

	current = Xil_In32(BUTTON_GPIO_BASEADDR +
		BUTTON_GPIO_DATA_OFFSET) & BUTTON_MASK;
	XTime_GetTime(&now);

	if (current != button_raw_state) {
		button_raw_state = current;
		button_raw_change_time = now;
		return;
	}
	if (now - button_raw_change_time < BUTTON_DEBOUNCE_TICKS ||
	    current == button_stable_state)
		return;

	pressed = (current ^ button_stable_state) & current;
	button_stable_state = current;

	if (pressed & BUTTON_CAPTURE_MASK) {
		xil_printf("BTN0: capture requested\r\n");
		CaptureStartFromLastFrame();
	}
	if (pressed & BUTTON_CNN_RESULT_MASK)
		ButtonPrintCnnResult();
}
