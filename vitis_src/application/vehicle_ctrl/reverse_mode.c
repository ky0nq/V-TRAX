#include "reverse_mode.h"

#include "../../driver/gpio_driver/gpio_driver.h"

#include "xtime_l.h"


// ==================================================
// REVERSE-MODE FSR THRESHOLDS
//
// Edit these independently from vehicle_task.c.
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
    19000,
    22000
};


static const int16_t reverseBrakeThreshold[5] =
{
    13000,
    17000,
    19000,
    21000,
    23000
};


// ==================================================
// JA2-only mode state
//
// This module intentionally does NOT use:
//   - JA1
//   - vehicle_armed
//   - vehicle_start_pending
//   - vehicle_stop_pending
//   - ja1_irq_count
//
// It only polls the raw JA2 level through gpio.c.
// ==================================================
static AdditionalFeatureMode currentMode =
    ADDITIONAL_FEATURE_NORMAL;

static unsigned int initialized = 0U;

static u32 ja2RawState = 0U;

static u32 ja2StableState = 0U;

/*
 * A new press is accepted only after JA2 has first been observed
 * released for the debounce interval.
 */
static unsigned int released = 0U;

static XTime rawChangedAt;


// ==================================================
// Internal helpers
// ==================================================
static XTime AdditionalFeatureDebounceCounts(void)
{
    return
        (XTime)COUNTS_PER_SECOND
        *
        ADDITIONAL_FEATURE_JA2_DEBOUNCE_MS
        /
        1000U;
}


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
    XTime now;

    /*
     * JA2 only.
     * Calling init here is harmless even though main.c also
     * configures JA2 as an input during board bring-up.
     */
    gpio_ja2_init_input();

    ja2RawState =
        gpio_ja2_read_level();

    ja2StableState =
        ja2RawState;

    XTime_GetTime(
        &now
    );

    rawChangedAt =
        now;

    released =
        0U;

    /*
     * Always boot in the existing NORMAL driving mode.
     *
     * If JA2 is held high during boot, it must be released
     * for the debounce interval and pressed again.
     */
    currentMode =
        ADDITIONAL_FEATURE_NORMAL;

    initialized =
        1U;
}


int AdditionalFeaturePoll(void)
{
    u32 raw;
    XTime now;
    XTime debounceCounts;

    if (
        !initialized
    )
    {
        return 0;
    }


    raw =
        gpio_ja2_read_level();

    XTime_GetTime(
        &now
    );

    debounceCounts =
        AdditionalFeatureDebounceCounts();


    // --------------------------------------------------
    // Raw electrical edge:
    // restart the debounce timer.
    // --------------------------------------------------
    if (
        raw
        !=
        ja2RawState
    )
    {
        ja2RawState =
            raw;

        rawChangedAt =
            now;

        return 0;
    }


    // --------------------------------------------------
    // Wait until the raw level has remained unchanged
    // for the complete debounce interval.
    // --------------------------------------------------
    if (
        now - rawChangedAt
        <
        debounceCounts
    )
    {
        return 0;
    }


    // Already accepted this stable level.
    if (
        ja2StableState
        ==
        ja2RawState
    )
    {
        /*
         * A stable LOW arms the next press.
         *
         * This is what prevents a button held during boot from
         * immediately changing the mode.
         */
        if (
            ja2StableState
            ==
            0U
        )
        {
            released =
                1U;
        }

        return 0;
    }


    // Accept the new stable electrical state.
    ja2StableState =
        ja2RawState;


    // --------------------------------------------------
    // Stable LOW = release.
    // Release never toggles the driving mode.
    // --------------------------------------------------
    if (
        ja2StableState
        ==
        0U
    )
    {
        released =
            1U;

        return 0;
    }


    // --------------------------------------------------
    // Stable HIGH = new press.
    //
    // Only accept it if a valid release was seen first.
    // --------------------------------------------------
    if (
        !released
    )
    {
        return 0;
    }


    released =
        0U;


    currentMode =
        (
            currentMode
            ==
            ADDITIONAL_FEATURE_NORMAL
        )
            ? ADDITIONAL_FEATURE_REVERSE
            : ADDITIONAL_FEATURE_NORMAL;


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
