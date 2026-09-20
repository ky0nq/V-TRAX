/* Layer types custom */
#ifndef CNN_LAYER_H
#define CNN_LAYER_H
typedef enum {
    LAYER_CONV,
    LAYER_FC
} LayerType;

typedef struct {
    int input_h;
    int input_w;
    int input_c;

    int output_h;
    int output_w;
    int output_c;

    int kernel_h;
    int kernel_w;

    int stride;
    int padding;

    //FC
    int input_size;
    int output_size;

    int relu_en; // 0: disabled, 1: enabled

} LayerConfig;
#endif