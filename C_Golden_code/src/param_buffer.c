// Bias,Quant Buffer
typedef struct {

    bias_t bias;

    int32_t multiplier;
    int shift;

} QuantParam;