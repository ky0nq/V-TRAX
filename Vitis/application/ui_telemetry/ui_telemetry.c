#include "ui_telemetry.h"
#include "ui_telemetry_format.h"
#include "../capture_cnn_ctrl/capture_cnn_ctrl.h"
#include "xil_printf.h"
#include "xtime_l.h"

/* Compact 10 Hz console USB-UART telemetry; no Ethernet or DDR copying.
 * Main loop only: keep video/CNN interrupts free of serial output. */
void UiApplicationService(void)
{
    static XTime last;
    XTime now;
    unsigned age;
    VehicleUiState state;
    char frame[160];
    XTime_GetTime(&now);
    if (now - last < (XTime)COUNTS_PER_SECOND / 10U) return;
    last = now;
    age = cnn_done_count ?
        (unsigned)((now - ui_cnn_done_time) / (COUNTS_PER_SECOND / 1000U)) : 0U;
    VehicleReadUiState(&state);
    if (UiSerialFormat(frame, sizeof frame, ui_pressure_percent, (int)cnn_last_result,
            cnn_result_valid && age < 1500U, age, &state,
            CaptureGetMode() == CAPTURE_MODE_DEMO) >= 0)
        xil_printf("\r\n%s\r\n", frame);
}
