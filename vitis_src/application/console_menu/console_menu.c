#include "console_menu.h"
#include "../../driver/video_pipeline_driver/video_pipeline_driver.h"
#include "../capture_cnn_ctrl/capture_cnn_ctrl.h"
#include "../display_mode/display_mode.h"
#include "../../driver/bbox_driver/bbox_driver.h"
#include "../camera_profile/camera_profile.h"
#include "../../driver/cam_ae/cam_ae.h"
#include "../../driver/gamma/gamma.h"
#include "../../driver/uart_driver/apb_uart_driver.h"
#include "xil_printf.h"
#include "xuartps_hw.h"

#if ENABLE_CAMERA_CONSOLE_MENU
void menu_help(){
	xil_printf("\r\n--- keys ------------------------------\r\n");
	xil_printf("  mode: %s (boot default: TEST)\r\n",
		CaptureGetMode() == CAPTURE_MODE_TEST ? "TEST" : "DEMO");
	xil_printf("  T  : enter DEMO, capture on each 10ms timer tick when idle\r\n");
	xil_printf("  d  : dump custom VDMA status and counters \r\n");
	xil_printf("  c  : TEST only, capture last DDR frame then run CNN\r\n");
	xil_printf("  CNN starts automatically after a valid Capture DONE IRQ\r\n");
	xil_printf("  BTN0: capture last DDR frame and run CNN\r\n");
	xil_printf("  BTN1: print the latest CNN accelerator result\r\n");
	xil_printf("  r  : read captured 64x64 RGB pixels\r\n");
	xil_printf("  x  : TEST only, download completed 64x64 RGB888 as HEX\r\n");
	xil_printf("  l  : show BRAM ROM image for 3 seconds\r\n");
	xil_printf("  j  : toggle BBOX overlay ON/OFF\r\n");
	xil_printf("  y : next camera AE/gamma profile (0..9)\r\n");
	xil_printf("  Y : restore startup camera profile (2)\r\n");
	xil_printf("  u : undo camera AE/gamma change (up to 16)\r\n");
	xil_printf("  t : lock current exposure/gain (AE/AGC off), print status\r\n");
	xil_printf("      y/Y/u may still change gamma; reset restores auto mode\r\n");
	xil_printf("  ?  : menu help \r\n");
}

void menu_run()
{
	char c;

	/* In DEMO, UART1 RX belongs exclusively to the vehicle parser. */
	if (CaptureGetMode() == CAPTURE_MODE_DEMO)
		return;

	if (!XUartPs_IsReceiveData(STDIN_BASEADDRESS)){
		return;
	}
	c = (char)XUartPs_RecvByte(STDIN_BASEADDRESS);

	switch(c){
		case	'd' : PrintVdmaDiagnostics(); break;
		case 'T':
			CaptureEnterDemoMode();
			xil_printf("DEMO mode: 10ms timer capture enabled; busy ticks skipped\r\n");
			break;
		case 'c':
			if (CaptureGetMode() == CAPTURE_MODE_TEST) {
				CaptureStartFromLastFrame();
				xil_printf("capture result\n");
			}
			else
				xil_printf("c is available only in TEST mode\r\n");
			break;
		case	'r' : CapturePrintPixels(); break;
		case 'x':
			if (CaptureGetMode() == CAPTURE_MODE_TEST)
				CaptureDownloadPixels();
			else
				xil_printf("x is available only in TEST mode\r\n");
			break;
		case 'l': RomPreviewStart(); break;
		case 'j': BboxToggle(); break;
		case 'y':
			CameraNextColorProfile();
			break;
		case 'Y':
			CameraSetColorProfile(COLOR_PROFILE_DEFAULT);
			break;
		case 'u':
			CameraUndoColorProfile();
			break;
		case 't':
			(void)cam_ae_lock();
			xil_printf("PL UART base=%08X busy=%d\r\n",
				(unsigned)APB_UART_BASE, apb_uart_tx_busy());
			xil_printf("camera color profile=%u/9 AE=%s gamma=%s undo=%u\r\n",
				color_profile, cam_ae_name(cam_ae_get()),
				gamma_name(gamma_get()), camera_history_count);
			break;
		case	'?' : menu_help();		break;
		default : break;
	}
}

#endif
