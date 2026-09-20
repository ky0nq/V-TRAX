#ifndef PE_ARRAY_H
#define PE_ARRAY_H

#include "cnn_types.h"

void pe_mac(
    const act_t *activation,
    const weight_t *weight,
    int K,
    accum_t *result
);

#endif