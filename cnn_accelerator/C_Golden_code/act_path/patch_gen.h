#ifndef PATCH_GEN_H
#define PATCH_GEN_H

#include "cnn_types.h"
#include "cnn_layer.h"

void patch_generate(
    const act_t *input,
    act_t *patch,
    const LayerConfig *layer,
    int output_x,
    int output_y
);

#endif