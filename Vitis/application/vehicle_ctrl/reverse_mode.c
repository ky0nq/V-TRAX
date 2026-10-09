#include "reverse_mode.h"

#include "../../driver/gpio_driver/gpio_driver.h"

#include "../../driver/video_pipeline_driver/video_pipeline_driver.h"


// ==================================================
// REVERSE-MODE FSR THRESHOLDS
//
// Edit these independently from the forward-mode thresholds in vehicle_ctrl.c.
//
// Mapping:
//   raw < threshold[0] -> level 0
//   raw < threshold[1] -> level 1
//   raw < threshold[2] -> level 2
//   raw < threshold[3] -> level 3
//   raw < threshold[4] -> level 4
//   otherwise          -> level 5
//
// Initial values are copied from the current forward calibration.
// ==================================================
static const int16_t reverseAccelThreshold[5] =
{
    7000,
    10000,
    14000,
    17000,
    20000
};


static const int16_t reverseBrakeThreshold[5] =
{
    10000,
    11000,
    12000,
    13000,
    14000
};


/* JA2 consumes latched IRQ events with an independent 300 ms cooldown. */
static AdditionalFeatureMode currentMode = ADDITIONAL_FEATURE_NORMAL;
static unsigned int initialized = 0U;

static uint8_t AdditionalFeatureRawToLevel(
    int16_t raw,
    const int16_t threshold[5]
)
{
    if (
        raw
        <
        0
    )
    {
        raw =
            0;
    }


    if (
        raw
        <
        threshold[0]
    )
    {
        return 0U;
    }


    if (
        raw
        <
        threshold[1]
    )
    {
        return 1U;
    }


    if (
        raw
        <
        threshold[2]
    )
    {
        return 2U;
    }


    if (
        raw
        <
        threshold[3]
    )
    {
        return 3U;
    }


    if (
        raw
        <
        threshold[4]
    )
    {
        return 4U;
    }


    return 5U;
}


// ==================================================
// Public API
// ==================================================
void AdditionalFeatureInit(void)
{
    gpio_ja2_init_input();
    Ja2PressDiscard();
    currentMode = ADDITIONAL_FEATURE_NORMAL;
    initialized = 1U;
}

int AdditionalFeaturePoll(void)
{
    if (!initialized || !Ja2PressTake())
        return 0;

    currentMode = currentMode == ADDITIONAL_FEATURE_NORMAL
        ? ADDITIONAL_FEATURE_REVERSE : ADDITIONAL_FEATURE_NORMAL;
    return 1;
}

int AdditionalFeatureIsActive(void)
{
    return
        currentMode
        ==
        ADDITIONAL_FEATURE_REVERSE;
}


AdditionalFeatureMode AdditionalFeatureGetMode(void)
{
    return
        currentMode;
}


const char *AdditionalFeatureModeName(void)
{
    return
        AdditionalFeatureIsActive()
            ? "REVERSE"
            : "NORMAL";
}


uint8_t AdditionalFeatureAccelLevel(
    int16_t raw
)
{
    return
        AdditionalFeatureRawToLevel(
            raw,
            reverseAccelThreshold
        );
}


uint8_t AdditionalFeatureBrakeLevel(
    int16_t raw
)
{
    return
        AdditionalFeatureRawToLevel(
            raw,
            reverseBrakeThreshold
        );
}


void AdditionalFeatureApply(
    int16_t accelRaw,
    int16_t brakeRaw,
    int8_t cnnSteering,
    int8_t *steering,
    uint8_t *accelLevel,
    uint8_t *brakeLevel,
    uint8_t *flags
)
{
    if (
        !AdditionalFeatureIsActive()
    )
    {
        return;
    }


    if (
        steering == 0
        ||
        accelLevel == 0
        ||
        brakeLevel == 0
        ||
        flags == 0
    )
    {
        return;
    }


    // ==================================================
    // Steering
    // ==================================================
#if ADDITIONAL_FEATURE_INVERT_CNN_STEERING

    *steering =
        (int8_t)(
            -cnnSteering
        );

#else

    *steering =
        cnnSteering;

#endif


    // ==================================================
    // Reverse-mode pedal mapping
    // ==================================================
    *accelLevel =
        AdditionalFeatureAccelLevel(
            accelRaw
        );


    *brakeLevel =
        AdditionalFeatureBrakeLevel(
            brakeRaw
        );


    // Brake always wins over accelerator.
    if (
        *brakeLevel
        >
        0U
    )
    {
        *accelLevel =
            0U;
    }


    // ==================================================
    // Keep the same 8-byte CommandPacket.
    //
    // ESP32 #2 must interpret bit 1 as reverse direction.
    // ==================================================
    *flags |=
        FLAG_REVERSE_MODE;
}
