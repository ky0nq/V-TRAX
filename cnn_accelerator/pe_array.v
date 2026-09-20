`timescale 1ns / 1ps

module pe_array (
    input wire clk,
    input wire rst_n,

    input wire step_en,
    input wire acc_clear,

    // Activation: 각 행의 왼쪽 입력
    input wire signed [7:0] act_in0,
    input wire signed [7:0] act_in1,
    input wire signed [7:0] act_in2,

    // Weight: 각 열의 위쪽 입력
    input wire signed [7:0] weight_in0,
    input wire signed [7:0] weight_in1,
    input wire signed [7:0] weight_in2,

    // 각 PE별 MAC 제어
    input wire [8:0] mac_valid,
    input wire [8:0] mac_last,

    // 각 PE 결과
    output wire signed [31:0] result_data00,
    output wire signed [31:0] result_data01,
    output wire signed [31:0] result_data02,

    output wire signed [31:0] result_data10,
    output wire signed [31:0] result_data11,
    output wire signed [31:0] result_data12,

    output wire signed [31:0] result_data20,
    output wire signed [31:0] result_data21,
    output wire signed [31:0] result_data22,

    // 각 PE 결과 valid
    output wire [8:0] result_valid
);

/*
mac_valid[0], mac_last[0], result_valid[0] : PE00
mac_valid[1], mac_last[1], result_valid[1] : PE01
mac_valid[2], mac_last[2], result_valid[2] : PE02
mac_valid[3], mac_last[3], result_valid[3] : PE10
mac_valid[4], mac_last[4], result_valid[4] : PE11
mac_valid[5], mac_last[5], result_valid[5] : PE12
mac_valid[6], mac_last[6], result_valid[6] : PE20
mac_valid[7], mac_last[7], result_valid[7] : PE21
mac_valid[8], mac_last[8], result_valid[8] : PE22
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
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[0]),
        .mac_last    (mac_last[0]),
        .act_in      (act_in0),
        .weight_in   (weight_in0),
        .act_out     (act_00_01),
        .weight_out  (weight_00_10),
        .result_data (result_data00),
        .result_valid(result_valid[0])
    );

    single_pe u_single_pe01 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[1]),
        .mac_last    (mac_last[1]),
        .act_in      (act_00_01),
        .weight_in   (weight_in1),
        .act_out     (act_01_02),
        .weight_out  (weight_01_11),
        .result_data (result_data01),
        .result_valid(result_valid[1])
    );

    single_pe u_single_pe02 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[2]),
        .mac_last    (mac_last[2]),
        .act_in      (act_01_02),
        .weight_in   (weight_in2),
        .act_out     (),
        .weight_out  (weight_02_12),
        .result_data (result_data02),
        .result_valid(result_valid[2])
    );

    single_pe u_single_pe10 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[3]),
        .mac_last    (mac_last[3]),
        .act_in      (act_in1),
        .weight_in   (weight_00_10),
        .act_out     (act_10_11),
        .weight_out  (weight_10_20),
        .result_data (result_data10),
        .result_valid(result_valid[3])
    );

    single_pe u_single_pe11 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[4]),
        .mac_last    (mac_last[4]),
        .act_in      (act_10_11),
        .weight_in   (weight_01_11),
        .act_out     (act_11_12),
        .weight_out  (weight_11_21),
        .result_data (result_data11),
        .result_valid(result_valid[4])
    );

    single_pe u_single_pe12 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[5]),
        .mac_last    (mac_last[5]),
        .act_in      (act_11_12),
        .weight_in   (weight_02_12),
        .act_out     (),
        .weight_out  (weight_12_22),
        .result_data (result_data12),
        .result_valid(result_valid[5])
    );

    single_pe u_single_pe20 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[6]),
        .mac_last    (mac_last[6]),
        .act_in      (act_in2),
        .weight_in   (weight_10_20),
        .act_out     (act_20_21),
        .weight_out  (),
        .result_data (result_data20),
        .result_valid(result_valid[6])
    );

    single_pe u_single_pe21 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[7]),
        .mac_last    (mac_last[7]),
        .act_in      (act_20_21),
        .weight_in   (weight_11_21),
        .act_out     (act_21_22),
        .weight_out  (),
        .result_data (result_data21),
        .result_valid(result_valid[7])
    );

    single_pe u_single_pe22 (
        .clk         (clk),
        .rst_n       (rst_n),
        .step_en     (step_en),
        .acc_clear   (acc_clear),
        .mac_valid   (mac_valid[8]),
        .mac_last    (mac_last[8]),
        .act_in      (act_21_22),
        .weight_in   (weight_12_22),
        .act_out     (),
        .weight_out  (),
        .result_data (result_data22),
        .result_valid(result_valid[8])
    );
endmodule