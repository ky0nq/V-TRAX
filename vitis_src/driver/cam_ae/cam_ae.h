/*
 *  cam_ae.h
 *
 *  OV5640 auto exposure (AE) target control - to keep bright areas from
 *  turning pure white
 *
 *  ---------------------------------------------------------------------
 *  WHY THIS MODULE IS NEEDED
 *
 *  The Pcam path runs the sensor in RAW mode and skips the ISP. But exposure
 *  time and analog gain come before the ISP, so they still work.
 *  In other words, AE is still running.
 *
 *  However, both Digilent's code and our code only set these three
 *  registers in the 0x3A range:
 *
 *      {0x3a13, 0x43}   pre-gain
 *      {0x3a18, 0x00}   gain ceiling, high byte
 *      {0x3a19, 0xf8}   gain ceiling = 248/16 = 15.5x
 *
 *  The exposure target registers (0x3A0F / 0x3A10 / 0x3A1B / 0x3A1E /
 *  0x3A11 / 0x3A1F) are never written, so they keep the sensor reset values.
 *
 *      Reset default      : 0x78 / 0x68 / 0x78 / 0x68 / 0xD0 / 0x40
 *      OmniVision suggests: 0x30 / 0x28 / 0x30 / 0x26 / 0x60 / 0x14
 *
 *  The default is more than 2x brighter than the suggested value. AE raises
 *  the exposure to reach this target, so bright areas become saturated
 *  (maxed out) in the sensor. Once a signal is saturated, the gamma curve
 *  later in the pipeline cannot bring it back.
 *
 *  ---------------------------------------------------------------------
 *  WHAT THE REGISTERS MEAN (standard OV5640 register map)
 *
 *      0x3A0F  AEC CTRL0F   stable range, upper limit (WPT)
 *      0x3A10  AEC CTRL10   stable range, lower limit (BPT)
 *      0x3A1B  AEC CTRL1B   stable range, upper limit (WPT2)
 *      0x3A1E  AEC CTRL1E   stable range, lower limit (BPT2)
 *      0x3A11  AEC CTRL11   fast zone, upper limit
 *      0x3A1F  AEC CTRL1F   fast zone, lower limit
 *
 *  WPT/BPT make the target brightness window. The fast zone is an outer
 *  limit: when the image is far outside the window, AE moves faster.
 *  Lower values mean a darker target, so exposure goes down and bright
 *  areas keep their detail.
 *
 *  ! Honest limit : only level 0 (suggested value) and level +2 (reset
 *    default) come from a clear source. The other four levels are guesses
 *    in between. So this module does not fix one value. Instead you can
 *    move it up and down live over UART. Tuning while watching the screen
 *    is the safest way.
 *
 *  ---------------------------------------------------------------------
 *  HOW TO USE
 *
 *      cam_ae_init();          // once at boot. Sets level 0
 *      cam_ae_down();          // darker (protects bright areas)
 *      cam_ae_up();            // brighter
 *      cam_ae_dump();          // show target values + real exposure/gain
 *
 *  AE needs a few frames to settle. After a change, wait about 0.5 s
 *  before you judge the result.
 */

#ifndef CAM_AE_H
#define CAM_AE_H

#include "xil_types.h"

typedef enum {
    AE_LEVEL_M3 = 0,    /* darkest - best protection for bright areas */
    AE_LEVEL_M2,
    AE_LEVEL_M1,
    AE_LEVEL_0,         /* OmniVision suggested value (default) */
    AE_LEVEL_P1,
    AE_LEVEL_P2,        /* sensor reset default = state before this module */
    AE_LEVEL_COUNT
} Ae_level;

/* Sets level 0 (suggested value). Call this after OV5640_SetMode720p(). */
void        cam_ae_init(void);

/* Sets the given level. Does nothing if the level is out of range. */
void        cam_ae_set(Ae_level lv);

/* Freeze current exposure and gain (AE/AGC off). Repeated calls stay locked.
 * Gamma/AWB are unchanged. Returns 0 on success, -1 on error.
 * A sensor reset restores the startup automatic mode. */
int         cam_ae_lock(void);

/* One step darker / brighter. Stays at the end if already there. */
void        cam_ae_down(void);
void        cam_ae_up(void);

Ae_level    cam_ae_get(void);
const char *cam_ae_name(Ae_level lv);

/*
 *  Prints the current AE state over UART.
 *
 *    - reads back the 6 target registers to check they were really written
 *    - the exposure (in lines) and analog gain (x times) AE is using now
 *    - the gain ceiling
 *
 *  If bright areas are white, look at the gain:
 *    gain near 1.0x but exposure is high -> the lighting is too bright.
 *    gain is high -> AE is exposing for the dark areas and giving up the
 *    bright ones.
 */
void        cam_ae_dump(void);

#endif /* CAM_AE_H */
