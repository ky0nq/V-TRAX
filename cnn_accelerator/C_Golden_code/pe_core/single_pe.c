/*
Single PE Recived One Activation and One Weight
→ multiply two object
→ acc multiply result
*/

#include <stdbool.h> 
#include <stdint.h> 
#include <inttypes.h> // printf() -> int32_t value
#include <stdio.h> 

// RTL Input Port
typedef struct {
    bool acc_clear;     // Start accumulation from zero when asserted // reset acc for calculate new MAC
    bool mac_valid; 
    bool mac_last;      // input last 'K' (K-1)

    int8_t act_in;      // PE input / signed INT8 Activation
    int8_t weight_in;   // PE input / signed INT8 Weight
} PEInput;

// RTL Output Port
typedef struct {
    int8_t act_out;      // PE Output / Move to Right PE (Activation)
    int8_t weight_out;   // PE Output / Move to Down PE (Weight)
    bool valid_out;      // PE Output Valid

    int32_t result_data; // PE Output / Result signed INT32
    bool result_valid;   // PE Result Valid
} PEOutput;


// register
typedef struct {
    int32_t acc_reg;    // save acc result (for Calculating)
    int32_t result_reg; // save recent result 
} PEState;


// Emulate 32-bit two's-complement wrap-around
static int32_t wrap_i32(int64_t value)
{
    uint32_t bits;

    bits = (uint32_t)((uint64_t)value & UINT64_C(0xFFFFFFFF));

    if (bits & UINT32_C(0x80000000)) {
        return (int32_t)((int64_t)bits - INT64_C(0x100000000));
    }

    return (int32_t)bits;
}

// PE Initial
static void pe_init(PEState *pe)
{
    pe->acc_reg = 0;
    pe->result_reg = 0;
}

// PE 1clk Action
static void pe_step(
    PEState *pe,
    const PEInput *in,
    PEOutput *out
)
{
    int16_t product;    // Activation * Weight Result
    int32_t base_acc;   // c_acc | before add MAC
    int32_t acc_next;   // n_acc | add MAC Result // acc_next = base_acc + product

    // setting Output 
    out->act_out = in->act_in;
    out->weight_out = in->weight_in;
    out->valid_out = in->mac_valid;

    out->result_data = pe->result_reg;
    out->result_valid = false;

    // reset acc
    if (in->acc_clear) {
        pe->acc_reg = 0;
    }

    // vaild == 0 => MAC X
    if (!in->mac_valid) {
        return;
    }

    // INT8 Activation * INT8 Weight => INT16 product
    product = (int16_t)((int16_t)in->act_in *(int16_t)in->weight_in);

    base_acc = pe->acc_reg;

    // INT16 product -> INT32(INT64 + INT64)
    acc_next = wrap_i32((int64_t)base_acc + (int64_t)product);

    pe->acc_reg = acc_next;

    // when last MAC Situation
    if (in->mac_last) {
        pe->result_reg = acc_next;

        out->result_data = acc_next;
        out->result_valid = true;
    }
}

// main
int main(void)
{
    const int8_t activation[] = {2, -1, 3};
    const int8_t weight[]     = {4, 5, -2};

    const size_t k_len = sizeof(activation) / sizeof(activation[0]);

    PEState pe;
    PEInput in = {0};
    PEOutput out = {0};

    // 1. PE Init
    pe_init(&pe);

    // 2. acc clear
    in.acc_clear = true;
    pe_step(&pe, &in, &out);

    printf("clear: acc = %" PRId32 "\n", pe.acc_reg);

    // 3. input activation, weight in 1clk
    for (size_t k = 0; k < k_len; ++k) {
        in.acc_clear = false;
        in.mac_valid = true;
        in.mac_last  = (k == k_len - 1);

        in.act_in    = activation[k];
        in.weight_in = weight[k];

        pe_step(&pe, &in, &out);

        printf(
            "MAC %zu: act=%d, weight=%d, "
            "acc=%" PRId32 ", result_valid=%d\n",
            k + 1,
            (int)in.act_in,
            (int)in.weight_in,
            pe.acc_reg,
            (int)out.result_valid
        );
    }

    // 4. final result after last MAC
    const int32_t expected = -3;

    printf(
        "expected result=%" PRId32 ", PE result=%" PRId32 "\n",
        expected,
        out.result_data
    );

    if (out.result_valid && out.result_data == expected) {
        printf("PASS\n");
        return 0;
    }

    printf("FAIL\n");
    return 1;
}
