`timescale 1ns / 1ps

module pe_core (
    input wire clk,
    input wire rst_n,

    input wire [23:0] i_act_data,
    input wire        i_act_valid,

    output wire o_act_ready,

    input  wire [23:0] i_wgt_data,
    input  wire        i_wgt_valid,

    output wire o_wgt_ready,

    input wire i_step_en,
    input wire i_feed_valid,

    input wire       i_askew_clear,
    input wire       i_wskew_clear,
    input wire       i_acc_clear,
    input wire [8:0] i_mac_valid,
    input wire [8:0] i_mac_last,

    output wire [287:0] o_result_data,
    output wire [  8:0] o_result_valid
);
    wire [23:0] act_data, wgt_data;

    act_skew u_act_skew (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_clear     (i_askew_clear),
        .i_data      (i_act_data),
        .i_valid     (i_act_valid),
        .o_ready     (o_act_ready),
        .i_step_en   (i_step_en),
        .i_feed_valid(i_feed_valid),
        .o_data      (act_data)
    );

    wgt_skew u_wgt_skew (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_clear     (i_wskew_clear),
        .i_data      (i_wgt_data),
        .i_valid     (i_wgt_valid),
        .o_ready     (o_wgt_ready),
        .i_step_en   (i_step_en),
        .i_feed_valid(i_feed_valid),
        .o_data      (wgt_data)
    );

    pe_array u_pe_array (
        .clk           (clk),
        .rst_n         (rst_n),
        .i_step_en     (i_step_en),
        .i_acc_clear   (i_acc_clear),
        .i_act_data    (act_data),
        .i_wgt_data    (wgt_data),
        .i_mac_valid   (i_mac_valid),
        .i_mac_last    (i_mac_last),
        .o_result_data (o_result_data),
        .o_result_valid(o_result_valid)
    );

endmodule
