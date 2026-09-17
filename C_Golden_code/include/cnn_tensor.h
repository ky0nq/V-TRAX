#ifndef CNN_TENSOR_H
#define CNN_TENSOR_H

#include "cnn_types.h"

typedef struct {
    int h;
    int w;
    int c;

    act_t *data;
} Tensor;

typedef struct {
    int len;

    act_t *data;
} Vector;

static inline int tensor_index_hwc(
    int y,
    int x,
    int c,
    int width,
    int channels
)
{
    return (y * width + x) * channels + c;
}

#endif