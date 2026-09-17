//Weight Buffer
void weight_buffer_read(
    const weight_t *weights,
    weight_t *weight_out,
    int out_channel,
    int k,
    int K
)
{
    weight_out[0] =
        weights[(out_channel + 0) * K + k];

    weight_out[1] =
        weights[(out_channel + 1) * K + k];

    weight_out[2] =
        weights[(out_channel + 2) * K + k];
}