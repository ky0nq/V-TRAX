#include "cnn_types.h"
#include "cnn_layer.h"
#include "pe_array.h"
#include "post_process.h"

void fc_golden(
    const act_t *input,
    const weight_t *weight,
    const bias_t *bias,
    output_t *output,
    const LayerConfig *layer
)
{
    int input_len = layer->input_len;
    int output_len = layer->output_len;

    for (int oc = 0; oc < output_len; oc++) {

        accum_t acc = 0;

        for (int k = 0; k < input_len; k++) {

            acc +=
                (accum_t)input[k] *
                (accum_t)weight[oc * input_len + k];
        }

        output[oc] =
            post_process(
                acc,
                bias[oc],
                layer->relu_en
            );
    }
}