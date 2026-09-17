void flatten_hwc(
    const Tensor *input,
    act_t *vector
)
{
    int index = 0;

    for (int y = 0; y < input->h; y++) {
        for (int x = 0; x < input->w; x++) {
            for (int c = 0; c < input->c; c++) {

                vector[index++] =
                    input->data[
                        (y * input->w + x) * input->c + c
                    ];
            }
        }
    }
}