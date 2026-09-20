//golden Reference calculation
#include "pe_array.h"

void pe_mac(
    const act_t *activation,
    const weight_t *weight,
    int K,
    accum_t *result
)
{
    accum_t acc = 0;

    for (int k = 0; k < K; k++) {

        mult_t product =
            (mult_t)activation[k]
            * (mult_t)weight[k];

        acc += (accum_t)product;
    }

    *result = acc;
}