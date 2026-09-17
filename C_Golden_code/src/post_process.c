#include "post_process.h"

/*
 * For connection testing only:
 * Scale factor 1, zero-point 0, INT8 saturation.
 * Real model multiplier/shift-based requantization is not implemented yet.
 */
output_t post_process(
    accum_t acc,
    bias_t bias,
    int relu_en
)
{
    // Use a wider type to avoid INT32 overflow during bias addition
    int64_t value = (int64_t)acc + (int64_t)bias;

    // Saturate to INT8 for test purposes
    if (value > INT8_MAX) {
        value = INT8_MAX;
    } else if (value < INT8_MIN) {
        value = INT8_MIN;
    }

    // For this test, the output zero-point is 0
    if (relu_en && value < 0) {
        value = 0;
    }

    return (output_t)value;
}