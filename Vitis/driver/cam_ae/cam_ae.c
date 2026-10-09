/*
 *  cam_ae.c
 *
 *  OV5640 AE target control. Read cam_ae.h first to see what this does and why.
 */

#include "cam_ae.h"
#include "xil_printf.h"
#include "../ov5640/OV5640.h"
#include "../iic_sccb_cfg/iic_sccb_cfg.h"

/*===========================================================================
 *  Registers
 *===========================================================================*/

/* AE target brightness */
#define REG_AEC_CTRL0F      0x3A0F      /* stable range, upper (WPT)  */
#define REG_AEC_CTRL10      0x3A10      /* stable range, lower (BPT)  */
#define REG_AEC_CTRL1B      0x3A1B      /* stable range, upper (WPT2) */
#define REG_AEC_CTRL1E      0x3A1E      /* stable range, lower (BPT2) */
#define REG_AEC_CTRL11      0x3A11      /* fast zone, upper           */
#define REG_AEC_CTRL1F      0x3A1F      /* fast zone, lower           */

/* Values AE is using now (we only read these) */
#define REG_AEC_PK_EXP_H    0x3500      /* [3:0] = exposure[19:16]  */
#define REG_AEC_PK_EXP_M    0x3501      /*         exposure[15:8]   */
#define REG_AEC_PK_EXP_L    0x3502      /*         exposure[7:0]    */
#define REG_AEC_PK_MANUAL   0x3503
#define AEC_MANUAL_MASK     0x03U      /* bit0: manual exposure, bit1: manual gain */
#define REG_AEC_PK_GAIN_H   0x350A      /* [1:0] = gain[9:8]        */
#define REG_AEC_PK_GAIN_L   0x350B      /*         gain[7:0]        */

/* Gain ceiling (max gain) */
#define REG_AEC_GAIN_CEIL_H 0x3A18
#define REG_AEC_GAIN_CEIL_L 0x3A19

/*===========================================================================
 *  Level table
 *
 *  Level 0  : OmniVision suggested start value. Comes from a clear source.
 *  Level +2 : sensor reset default. Where our system was before this module.
 *  Others   : guesses in between. Tune them by testing.
 *
 *  Every row must keep these rules:
 *    upper (WPT) > lower (BPT)
 *    fast zone lower < BPT < WPT < fast zone upper
 *===========================================================================*/
typedef struct {
    u8          wpt;        /* 0x3A0F */
    u8          bpt;        /* 0x3A10 */
    u8          wpt2;       /* 0x3A1B */
    u8          bpt2;       /* 0x3A1E */
    u8          vpt_hi;     /* 0x3A11 */
    u8          vpt_lo;     /* 0x3A1F */
    const char *name;
} Ae_target;

static const Ae_target ae_tab[AE_LEVEL_COUNT] = {
    /*  WPT   BPT   WPT2  BPT2  VPThi VPTlo */
    {  0x18, 0x10, 0x18, 0x0E, 0x30, 0x08, "-3  darkest (max highlight protection)" },
    {  0x20, 0x18, 0x20, 0x16, 0x40, 0x0C, "-2  darker"                             },
    {  0x28, 0x20, 0x28, 0x1E, 0x50, 0x10, "-1  slightly darker"                        },
    {  0x30, 0x28, 0x30, 0x26, 0x60, 0x14, " 0  OmniVision recommended"                  },
    {  0x48, 0x40, 0x48, 0x3E, 0x90, 0x20, "+1  brighter"                               },
    {  0x78, 0x68, 0x78, 0x68, 0xD0, 0x40, "+2  sensor reset default (before tuning)"    },
};

static Ae_level cur = AE_LEVEL_0;

int cam_ae_lock(void)
{
    static const u16 registers[] = {
        REG_AEC_PK_EXP_H, REG_AEC_PK_EXP_M, REG_AEC_PK_EXP_L,
        REG_AEC_PK_GAIN_H, REG_AEC_PK_GAIN_L
    };
    u8 original_control, control, values[5];
    u32 exposure;
    u16 gain;
    unsigned int i;
    int changed;

    if (iic_sccb_read(REG_AEC_PK_MANUAL, &original_control) != IIC_SCCB_OK) {
        xil_printf("AE lock failed: cannot read manual control\r\n");
        return -1;
    }

    changed = (original_control & AEC_MANUAL_MASK) != AEC_MANUAL_MASK;
    /* Freeze both automatic controllers BEFORE reading the multi-byte values.
     * Keep the existing exposure/gain registers and all other control bits.
     * Repeated calls only verify and report; they never unlock the sensor.
     * Reference: Linux drivers/media/i2c/ov5640.c autoexposure/autogain bits. */
    if (changed && iic_sccb_write(REG_AEC_PK_MANUAL,
                                 (u8)(original_control | AEC_MANUAL_MASK)) !=
                       IIC_SCCB_OK)
        goto failed;

    if (iic_sccb_read(REG_AEC_PK_MANUAL, &control) != IIC_SCCB_OK ||
        (control & AEC_MANUAL_MASK) != AEC_MANUAL_MASK)
        goto failed;

    for (i = 0U; i < sizeof(registers) / sizeof(registers[0]); ++i) {
        if (iic_sccb_read(registers[i], &values[i]) != IIC_SCCB_OK)
            goto failed;
    }

    exposure = ((u32)(values[0] & 0x0FU) << 16) |
               ((u32)values[1] << 8) | values[2];
    gain = (u16)(((u16)(values[3] & 0x03U) << 8) | values[4]);
    xil_printf("AE/AGC %s: manual control=%02X\r\n",
               changed ? "locked" : "already locked", (unsigned)control);
    xil_printf("  exposure=%lu (1/16 line), gain=%u.%02u x (raw=%u)\r\n",
               (unsigned long)exposure, (unsigned)(gain / 16U),
               (unsigned)((gain % 16U) * 100U / 16U), (unsigned)gain);
    xil_printf("  Reset to restore startup auto mode; gamma/AWB unchanged.\r\n");
    return 0;

failed:
    xil_printf("AE lock failed: SCCB error or manual-control mismatch\r\n");
    /* A failed verification must not be reported as a successful lock. */
    if (changed) {
        if (iic_sccb_write(REG_AEC_PK_MANUAL, original_control) != IIC_SCCB_OK ||
            iic_sccb_read(REG_AEC_PK_MANUAL, &control) != IIC_SCCB_OK ||
            control != original_control)
            xil_printf("AE control restore failed; sensor mode unknown, reset required\r\n");
    }
    return -1;
}

/*===========================================================================
 *  Apply
 *===========================================================================*/

void cam_ae_set(Ae_level lv)
{
    const Ae_target *t;
    u32 errors_before;

    if (lv >= AE_LEVEL_COUNT) {
        return;
    }
    t = &ae_tab[lv];
    errors_before = iic_sccb_error_count();

    /*
     * Write all six as one group. If we write them one by one, a new frame
     * can start in the middle. Then for one frame the upper limit is already
     * low but the lower limit is still high. You would see a flicker.
     */
    OV5640_GroupBegin();
    OV5640_WriteSCCB(REG_AEC_CTRL0F, t->wpt);
    OV5640_WriteSCCB(REG_AEC_CTRL10, t->bpt);
    OV5640_WriteSCCB(REG_AEC_CTRL1B, t->wpt2);
    OV5640_WriteSCCB(REG_AEC_CTRL1E, t->bpt2);
    OV5640_WriteSCCB(REG_AEC_CTRL11, t->vpt_hi);
    OV5640_WriteSCCB(REG_AEC_CTRL1F, t->vpt_lo);
    OV5640_GroupCommit();

    if (iic_sccb_error_count() != errors_before) {
        xil_printf("AE target write failed; keeping state at %s\r\n",
                   ae_tab[cur].name);
        return;
    }

    cur = lv;

    xil_printf("AE target : %s\r\n", t->name);
    xil_printf("  (takes a few frames to settle - wait before judging)\r\n");
}

void cam_ae_init(void)
{
    cam_ae_set(AE_LEVEL_0);
}

void cam_ae_down(void)
{
    if (cur == AE_LEVEL_M3) {
        xil_printf("AE target : already at the darkest level (%s)\r\n", ae_tab[cur].name);
        xil_printf("  For more, dim the lighting or go to manual exposure.\r\n");
        return;
    }
    cam_ae_set((Ae_level)(cur - 1));
}

void cam_ae_up(void)
{
    if (cur == AE_LEVEL_P2) {
        xil_printf("AE target : already at the brightest level (%s)\r\n", ae_tab[cur].name);
        return;
    }
    cam_ae_set((Ae_level)(cur + 1));
}

Ae_level cam_ae_get(void)
{
    return cur;
}

const char *cam_ae_name(Ae_level lv)
{
    if (lv >= AE_LEVEL_COUNT) {
        return "?";
    }
    return ae_tab[lv].name;
}

/*===========================================================================
 *  Status print
 *===========================================================================*/

void cam_ae_dump(void)
{
    const Ae_target *t = &ae_tab[cur];
    u8  r0f, r10, r1b, r1e, r11, r1f;
    u32 exp_raw, exp_lines;
    u16 gain_raw, ceil_raw;
    int ok;

    xil_printf("\r\n--- AE status -----------------------------------\r\n");
    xil_printf("level : %s\r\n", t->name);

    /*-------------------------------------------------------------------
     *  Read back the target registers
     *
     *  If we write without checking, and SCCB fails without any message,
     *  we see "I changed the value but the screen did not change" and
     *  cannot find the cause.
     *-------------------------------------------------------------------*/
    r0f = OV5640_ReadSCCB(REG_AEC_CTRL0F);
    r10 = OV5640_ReadSCCB(REG_AEC_CTRL10);
    r1b = OV5640_ReadSCCB(REG_AEC_CTRL1B);
    r1e = OV5640_ReadSCCB(REG_AEC_CTRL1E);
    r11 = OV5640_ReadSCCB(REG_AEC_CTRL11);
    r1f = OV5640_ReadSCCB(REG_AEC_CTRL1F);

    ok = (r0f == t->wpt) && (r10 == t->bpt) &&
         (r1b == t->wpt2) && (r1e == t->bpt2) &&
         (r11 == t->vpt_hi) && (r1f == t->vpt_lo);

    xil_printf("target registers (expected -> read back)\r\n");
    xil_printf("  0x3A0F WPT   0x%02X -> 0x%02X\r\n", t->wpt,    r0f);
    xil_printf("  0x3A10 BPT   0x%02X -> 0x%02X\r\n", t->bpt,    r10);
    xil_printf("  0x3A1B WPT2  0x%02X -> 0x%02X\r\n", t->wpt2,   r1b);
    xil_printf("  0x3A1E BPT2  0x%02X -> 0x%02X\r\n", t->bpt2,   r1e);
    xil_printf("  0x3A11 VPThi 0x%02X -> 0x%02X\r\n", t->vpt_hi, r11);
    xil_printf("  0x3A1F VPTlo 0x%02X -> 0x%02X\r\n", t->vpt_lo, r1f);
    xil_printf("  => %s\r\n", ok ? "match" : "!! MISMATCH - suspect SCCB");

    /*-------------------------------------------------------------------
     *  Values AE is using now
     *
     *  Exposure is 20 bits. The unit is 1/16 line.
     *  Gain is 10 bits. The real gain is value / 16.
     *-------------------------------------------------------------------*/
    exp_raw = ((u32)(OV5640_ReadSCCB(REG_AEC_PK_EXP_H) & 0x0F) << 16)
            | ((u32) OV5640_ReadSCCB(REG_AEC_PK_EXP_M)         <<  8)
            |  (u32) OV5640_ReadSCCB(REG_AEC_PK_EXP_L);
    exp_lines = exp_raw >> 4;

    gain_raw = ((u16)(OV5640_ReadSCCB(REG_AEC_PK_GAIN_H) & 0x03) << 8)
             |  (u16) OV5640_ReadSCCB(REG_AEC_PK_GAIN_L);

    ceil_raw = ((u16)OV5640_ReadSCCB(REG_AEC_GAIN_CEIL_H) << 8)
             |  (u16)OV5640_ReadSCCB(REG_AEC_GAIN_CEIL_L);

    xil_printf("current AE operating point\r\n");
    xil_printf("  exposure     : %d lines (raw %d, unit 1/16 line)\r\n",
               (int)exp_lines, (int)exp_raw);
    xil_printf("  gain         : %d.%02d x (raw %d)\r\n",
               (int)(gain_raw / 16), (int)((gain_raw % 16) * 100 / 16),
               (int)gain_raw);
    xil_printf("  gain ceiling : %d.%02d x (raw %d)\r\n",
               (int)(ceil_raw / 16), (int)((ceil_raw % 16) * 100 / 16),
               (int)ceil_raw);

    /*-------------------------------------------------------------------
     *  How to read this
     *-------------------------------------------------------------------*/
    xil_printf("how to read this\r\n");
    if (gain_raw <= 20) {
        xil_printf("  Gain is near 1x, so the scene is bright enough.\r\n");
        xil_printf("  If highlights are still blown, lower the target ('a').\r\n");
    } else {
        xil_printf("  Gain is raised. AE is exposing for the dark areas and\r\n");
        xil_printf("  giving up the bright ones. Lower the target, or even\r\n");
        xil_printf("  out the lighting.\r\n");
    }
    xil_printf("-------------------------------------------------\r\n");
}
