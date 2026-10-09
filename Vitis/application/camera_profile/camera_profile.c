#include "camera_profile.h"
#include "../../driver/cam_ae/cam_ae.h"
#include "../../driver/gamma/gamma.h"
#include "../../driver/iic_sccb_cfg/iic_sccb_cfg.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "sleep.h"

#define COLOR_PROFILE_COUNT     10U
#define CAMERA_HISTORY_DEPTH    16U

unsigned int color_profile = COLOR_PROFILE_DEFAULT;
typedef struct {
    Ae_level ae;
    Gamma_factor gamma;
    unsigned int profile;
} CameraSettings;
static CameraSettings camera_history[CAMERA_HISTORY_DEPTH];
unsigned int camera_history_count = 0U;

static void CameraSaveSettings(void)
{
    unsigned int i;

    if (camera_history_count == CAMERA_HISTORY_DEPTH) {
        for (i = 1U; i < CAMERA_HISTORY_DEPTH; ++i)
            camera_history[i - 1U] = camera_history[i];
        --camera_history_count;
    }
    camera_history[camera_history_count].ae = cam_ae_get();
    camera_history[camera_history_count].gamma = gamma_get();
    camera_history[camera_history_count].profile = color_profile;
    ++camera_history_count;
}

static int CameraApplySettings(Ae_level ae, Gamma_factor gamma,
                               unsigned int profile)
{
    u32 errors_before;

    errors_before = iic_sccb_error_count();
    cam_ae_set(ae);
    if (iic_sccb_error_count() != errors_before) {
        xil_printf("camera profile %u: SCCB error changing AE target; press u to retry previous settings\r\n",
                   profile);
        return 0;
    }
    gamma_set(gamma);
    color_profile = profile;
    xil_printf("camera profile %u/9: AE %s, gamma %s (undo=%u)\r\n",
               profile, cam_ae_name(ae), gamma_name(gamma),
               camera_history_count);
    return 1;
}

void CameraSetColorProfile(unsigned int index)
{
    static const struct {
        Ae_level ae;
        Gamma_factor gamma;
    } profiles[COLOR_PROFILE_COUNT] = {
        { AE_LEVEL_0,  GAMMA_1_1_8 }, /* previous startup settings */
        { AE_LEVEL_P1, GAMMA_1_1_8 },
        { AE_LEVEL_P2, GAMMA_1_1_8 }, /* startup settings */
        { AE_LEVEL_0,  GAMMA_1_1_5 },
        { AE_LEVEL_P1, GAMMA_1_1_5 },
        { AE_LEVEL_P2, GAMMA_1_1_5 },
        { AE_LEVEL_M1, GAMMA_1_1_5 },
        { AE_LEVEL_0,  GAMMA_1_1_2 },
        { AE_LEVEL_P1, GAMMA_1_1_2 },
        { AE_LEVEL_P2, GAMMA_1_1_2 }
    };
    if (index >= COLOR_PROFILE_COUNT)
        return;
    if (color_profile == index && cam_ae_get() == profiles[index].ae &&
        gamma_get() == profiles[index].gamma)
        return;
    CameraSaveSettings();
    CameraApplySettings(profiles[index].ae, profiles[index].gamma, index);
}

void CameraNextColorProfile(void)
{
    CameraSetColorProfile((color_profile + 1U) % COLOR_PROFILE_COUNT);
}

void CameraUndoColorProfile(void)
{
    CameraSettings previous;

    if (camera_history_count == 0U) {
        xil_printf("camera profile: no previous settings\r\n");
        return;
    }
    previous = camera_history[camera_history_count - 1U];
    --camera_history_count;
    if (!CameraApplySettings(previous.ae, previous.gamma, previous.profile))
        ++camera_history_count;
}
