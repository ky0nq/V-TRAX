// Patch generator
// Compute the 3x3 kernel input coordinates for a given output position
// Assumes zero padding; INT8 zero-point should be checked if needed
#include "patch_gen.h"

void patch_generate(
    const act_t *input,
    act_t *patch,
    const LayerConfig *layer,
    int output_x,
    int output_y
)
{
    int k = 0;

    for (int ky = 0; ky < 3; ky++) {
        for (int kx = 0; kx < 3; kx++) {
            // Apply zero padding for out-of-bound spatial locations
            int input_x =
                output_x * layer->stride 
                + kx 
                - layer->padding;

            int input_y =
                output_y * layer->stride
                + ky
                - layer->padding;

            for (int c = 0; c < layer->input_c; c++) {

                if (input_x < 0 ||
                    input_x >= layer->input_w ||
                    input_y < 0 ||
                    input_y >= layer->input_h) {

                    patch[k++] = 0;

                } else {

                    int index =
                        (input_y * layer->input_w
                         + input_x)
                        * layer->input_c + c;

                    patch[k++] = input[index];
                }
            }
        }
    }
}