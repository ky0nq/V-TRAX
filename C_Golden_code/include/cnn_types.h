/*  design structure */
// Activation  : signed INT8
// Weight      : signed INT8
// Multiply    : signed INT16
// Accumulator : signed INT32
// Bias        : signed INT32
// Output      : INT8

#ifndef CNN_TYPES_H
#define CNN_TYPES_H

#include <stdint.h>

typedef int8_t  act_t;     // Activation
typedef int8_t  weight_t;  // Weight
typedef int16_t mult_t;    // Multiply
typedef int32_t accum_t;     // Accumulator
typedef int32_t bias_t;    // Bias
typedef int8_t  output_t;     // Output
#endif