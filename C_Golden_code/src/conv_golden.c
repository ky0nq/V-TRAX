#include "cnn_types.h"
#include "cnn_layer.h"
#include "patch_gen.h"
#include "pe_array.h"
#include "post_process.h"

void conv_golden(
    const act_t *input,
    const weight_t *weight,
    const bias_t *bias,
    output_t *output,
    const LayerConfig *layer
)
{
    int output_h = layer->output_h;
    int output_w = layer->output_w;
    int output_c = layer->output_c;

    int K = layer->kernel_h *
            layer->kernel_w *
            layer->input_c;

    act_t patch[K];

    for (int oy = 0; oy < output_h; oy++) {

        for (int ox = 0; ox < output_w; ox++) {

            // Patch Generator
            patch_generate(
                input,
                patch,
                layer,
                ox,
                oy
            );

            // Output Channel
            for (int oc = 0; oc < output_c; oc++) {

                accum_t acc;

                // ③ PE Array
                pe_mac(
                    patch,
                    &weight[oc * K],
                    K,
                    &acc
                );

                // Post Process
                output[
                    (oy * output_w + ox) * output_c + oc
                ] =
                    post_process(
                        acc,
                        bias[oc],
                        layer->relu_en
                    );
            }
        }
    }
}