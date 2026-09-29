#ifndef UI_STREAM_CONFIG_H
#define UI_STREAM_CONFIG_H
/* Direct cable: PC wired adapter 192.168.10.1/24. UDP broadcast needs no PC MAC or ARP. */
#define UI_BOARD_IP 192,168,10,2
#define UI_PC_IP 192,168,10,255
#define UI_NETMASK 255,255,255,0
#define UI_GATEWAY 0,0,0,0
#define UI_MAC {0x02,0x53,0x4f,0x43,0x00,0x02}
#define UI_DATA_PORT 7000
#define UI_VIDEO_PORT 7001
/* CNN-view crop from the 1280x720 DDR camera frames. The hardware CNN keeps
 * its existing capture path; this is only the PC display copy. */
#define UI_VIDEO_WIDTH 256U
#define UI_VIDEO_HEIGHT 256U
#define UI_VIDEO_FPS 30U
#define UI_CROP_X 0U
#define UI_CROP_Y 464U
#define UI_CROP_WIDTH 256U
#define UI_CROP_HEIGHT 256U
#define UI_COPY_ROWS_PER_POLL 128U
#define UI_PACKETS_PER_POLL 24U
#define UI_SEND_BUDGET_US 1500U
#define UI_FRAME_DEADLINE_MS 100U
#define UI_PAYLOAD_BYTES 1200U
/* Existing filter_sw.c specifies G,B,R bytes in DDR. */
#define UI_DDR_R_OFFSET 2U
#define UI_DDR_G_OFFSET 0U
#define UI_DDR_B_OFFSET 1U
/* UI_NETWORK_ONLY=1: skip camera/CNN/HDMI, test ping and JSON first. */
#ifndef UI_NETWORK_ONLY
#define UI_NETWORK_ONLY 0
#endif
#define UI_TELEMETRY_HZ 20U
#define UI_AUTO_CNN 0
#define UI_CNN_PERIOD_MS 200U
#define UI_CNN_FRESH_MS 1500U
/* Replace UiReadPressurePercent() before changing this flag to zero. */
#define UI_PRESSURE_IS_TEST 1
#define UI_TEST_PRESSURE_PERCENT 0
#endif
