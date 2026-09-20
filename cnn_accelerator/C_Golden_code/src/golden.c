#include "golden.h"
#include "cnn_layer.h"
#include "cnn_types.h"

void cnn_golden(
    const act_t *input,
    LayerConfig *layers,
    int num_layers
)
{
    const act_t *current_input = input;

    act_t *current_output = NULL;

    for (int i = 0; i < num_layers; i++) {

        LayerConfig *layer = &layers[i];

        if (layer->type == LAYER_CONV) {

            /*
             * output buffer ready
             */

            conv_golden(
                current_input,
                ...,
                current_output,
                layer
            );

        }
        else if (layer->type == LAYER_FC) {

            fc_golden(
                current_input,
                ...,
                current_output,
                layer
            );
        }

        current_input = current_output;
    }
}