`timescale 1ns / 1ps

module pe_array (
    input wire clk,
    input wire rst_n,

    input wire i_step_en,
    input wire i_acc_clear,

    // Activation
    input wire [23:0] i_act_data,

    // Weight
    input wire [23:0] i_wgt_data,

    // mac control
    input wire [8:0] i_mac_valid,
    input wire [8:0] i_mac_last,

    // pe result
    output wire [287:0] o_result_data,

    // PE result valid
    output wire [8:0] o_result_valid
);

    /*
i_mac_valid[0], i_mac_last[0], o_result_valid[0] : PE00
i_mac_valid[1], i_mac_last[1], o_result_valid[1] : PE01
i_mac_valid[2], i_mac_last[2], o_result_valid[2] : PE02
i_mac_valid[3], i_mac_last[3], o_result_valid[3] : PE10
i_mac_valid[4], i_mac_last[4], o_result_valid[4] : PE11
i_mac_valid[5], i_mac_last[5], o_result_valid[5] : PE12
i_mac_valid[6], i_mac_last[6], o_result_valid[6] : PE20
i_mac_valid[7], i_mac_last[7], o_result_valid[7] : PE21
i_mac_valid[8], i_mac_last[8], o_result_valid[8] : PE22
*/

    // Activation 내부 연결
    wire signed [7:0] act_00_01;
    wire signed [7:0] act_01_02;

    wire signed [7:0] act_10_11;
    wire signed [7:0] act_11_12;

    wire signed [7:0] act_20_21;
    wire signed [7:0] act_21_22;

    // Weight 내부 연결
    wire signed [7:0] weight_00_10;
    wire signed [7:0] weight_10_20;

    wire signed [7:0] weight_01_11;
    wire signed [7:0] weight_11_21;

    wire signed [7:0] weight_02_12;
    wire signed [7:0] weight_12_22;

    single_pe u_single_pe00 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[0]),
        .i_mac_last    (i_mac_last[0]),
        .act_in        (i_act_data[7:0]),
        .weight_in     (i_wgt_data[7:0]),
        .act_out       (act_00_01),
        .weight_out    (weight_00_10),
        .result_data   ($signed(o_result_data[31:0])),
        .o_result_valid(o_result_valid[0])
    );

    single_pe u_single_pe01 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[1]),
        .i_mac_last    (i_mac_last[1]),
        .act_in        (act_00_01),
        .weight_in     (i_wgt_data[15:8]),
        .act_out       (act_01_02),
        .weight_out    (weight_01_11),
        .result_data   ($signed(o_result_data[63:32])),
        .o_result_valid(o_result_valid[1])
    );

    single_pe u_single_pe02 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[2]),
        .i_mac_last    (i_mac_last[2]),
        .act_in        (act_01_02),
        .weight_in     (i_wgt_data[23:16]),
        .act_out       (),
        .weight_out    (weight_02_12),
        .result_data   ($signed(o_result_data[95:64])),
        .o_result_valid(o_result_valid[2])
    );

    single_pe u_single_pe10 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[3]),
        .i_mac_last    (i_mac_last[3]),
        .act_in        (i_act_data[15:8]),
        .weight_in     (weight_00_10),
        .act_out       (act_10_11),
        .weight_out    (weight_10_20),
        .result_data   ($signed(o_result_data[127:96])),
        .o_result_valid(o_result_valid[3])
    );

    single_pe u_single_pe11 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[4]),
        .i_mac_last    (i_mac_last[4]),
        .act_in        (act_10_11),
        .weight_in     (weight_01_11),
        .act_out       (act_11_12),
        .weight_out    (weight_11_21),
        .result_data   ($signed(o_result_data[159:128])),
        .o_result_valid(o_result_valid[4])
    );

    single_pe u_single_pe12 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[5]),
        .i_mac_last    (i_mac_last[5]),
        .act_in        (act_11_12),
        .weight_in     (weight_02_12),
        .act_out       (),
        .weight_out    (weight_12_22),
        .result_data   ($signed(o_result_data[191:160])),
        .o_result_valid(o_result_valid[5])
    );

    single_pe u_single_pe20 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[6]),
        .i_mac_last    (i_mac_last[6]),
        .act_in        (i_act_data[23:16]),
        .weight_in     (weight_10_20),
        .act_out       (act_20_21),
        .weight_out    (),
        .result_data   ($signed(o_result_data[223:192])),
        .o_result_valid(o_result_valid[6])
    );

    single_pe u_single_pe21 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[7]),
        .i_mac_last    (i_mac_last[7]),
        .act_in        (act_20_21),
        .weight_in     (weight_11_21),
        .act_out       (act_21_22),
        .weight_out    (),
        .result_data   ($signed(o_result_data[255:224])),
        .o_result_valid(o_result_valid[7])
    );

    single_pe u_single_pe22 (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_mac_valid   (i_mac_valid[8]),
        .i_mac_last    (i_mac_last[8]),
        .act_in        (act_21_22),
        .weight_in     (weight_12_22),
        .act_out       (),
        .weight_out    (),
        .result_data   ($signed(o_result_data[287:256])),
        .o_result_valid(o_result_valid[8])
    );
endmodule
