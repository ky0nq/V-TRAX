#ifndef POST_PROCESS_H
#define POST_PROCESS_H

#include "cnn_types.h"

output_t post_process(
    accum_t acc,
    bias_t bias,
    int relu_en
);

#endif