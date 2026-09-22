`timescale 1ns / 1ps

module pe_core_out_path_top (
    input  wire        clk,
    input  wire        rst_n,

    // =========================================================
    // PE Core - Activation / Weight Stream
    // =========================================================
    input  wire [23:0] i_act_data,
    input  wire        i_act_valid,
    output wire        o_act_ready,

    input  wire [23:0] i_wgt_data,
    input  wire        i_wgt_valid,
    output wire        o_wgt_ready,

    // =========================================================
    // PE Core - Control
    // =========================================================
    input  wire        i_step_en,
    input  wire        i_feed_valid,

    input  wire        i_askew_clear,
    input  wire        i_wskew_clear,
    input  wire        i_acc_clear,

    input  wire [ 8:0] i_mac_valid,
    input  wire [ 8:0] i_mac_last,

    // =========================================================
    // Out Path - Layer Configuration
    // =========================================================
    input  wire               i_layer_cfg_valid,
    input  wire               i_is_fc,
    input  wire               i_is_final_layer,
    input  wire               i_pool_en,
    input  wire        [ 6:0] i_conv_w,
    input  wire        [ 6:0] i_conv_h,
    input  wire        [ 5:0] i_out_channels,
    input  wire        [13:0] i_dst_base,
    input  wire               i_relu_en,
    input  wire signed [31:0] i_quant_multiplier,
    input  wire        [ 5:0] i_quant_shift,
    input  wire        [ 5:0] i_bias_base,

    // Final FC2 angle requantization
    input  wire signed [31:0] i_angle_multiplier,
    input  wire        [ 5:0] i_angle_shift,

    // =========================================================
    // Out Path - Tile Configuration
    // =========================================================
    input  wire        i_tile_cfg_valid,
    input  wire [11:0] i_patch_base,
    input  wire [ 4:0] i_out_ch_base,
    input  wire [ 2:0] i_row_mask,
    input  wire [ 2:0] i_col_mask,
    input  wire        i_tile_last,

    output wire        o_result_space_ready,

    // =========================================================
    // Out Path - Parameter Load
    // =========================================================
    input  wire               i_param_load_start,
    input  wire               i_param_wr_en,
    input  wire        [ 5:0] i_param_wr_addr,
    input  wire signed [31:0] i_param_wr_data,

    output wire               o_param_load_done,
    output wire               o_params_ready,

    // =========================================================
    // Out Path - Input Buffer Write Interface
    // =========================================================
    input  wire        i_write_grant,

    output wire        o_wr_en,
    output wire [13:0] o_wr_addr,
    output wire [23:0] o_wr_data,
    output wire [ 2:0] o_wr_be,

    // =========================================================
    // Out Path - IRQ / Completion
    // =========================================================
    input  wire        i_irq_en,
    input  wire        i_irq_clear,

    output wire        o_tile_in_done,
    output wire        o_layer_done,

    output wire signed [31:0] o_final_result,
    output wire               o_done_status,
    output wire               o_irq
);

    // =========================================================
    // pe_core -> out_path
    // =========================================================
    wire [287:0] pe_result_data;
    wire [  8:0] pe_result_valid;

    // =========================================================
    // PE Core
    // =========================================================
    pe_core u_pe_core (
        .clk           (clk),
        .rst_n         (rst_n),

        .i_act_data    (i_act_data),
        .i_act_valid   (i_act_valid),
        .o_act_ready   (o_act_ready),

        .i_wgt_data    (i_wgt_data),
        .i_wgt_valid   (i_wgt_valid),
        .o_wgt_ready   (o_wgt_ready),

        .i_step_en     (i_step_en),
        .i_feed_valid  (i_feed_valid),

        .i_askew_clear (i_askew_clear),
        .i_wskew_clear (i_wskew_clear),
        .i_acc_clear   (i_acc_clear),

        .i_mac_valid   (i_mac_valid),
        .i_mac_last    (i_mac_last),

        .o_result_data (pe_result_data),
        .o_result_valid(pe_result_valid)
    );

    // =========================================================
    // Output Path
    // =========================================================
    out_path u_out_path (
        .clk               (clk),
        .rst_n             (rst_n),

        // Layer Configuration
        .i_layer_cfg_valid (i_layer_cfg_valid),
        .i_is_fc           (i_is_fc),
        .i_is_final_layer  (i_is_final_layer),
        .i_pool_en         (i_pool_en),
        .i_conv_w          (i_conv_w),
        .i_conv_h          (i_conv_h),
        .i_out_channels    (i_out_channels),
        .i_dst_base        (i_dst_base),
        .i_relu_en         (i_relu_en),
        .i_quant_multiplier(i_quant_multiplier),
        .i_quant_shift     (i_quant_shift),
        .i_bias_base       (i_bias_base),

        .i_angle_multiplier(i_angle_multiplier),
        .i_angle_shift     (i_angle_shift),

        // Tile Configuration
        .i_tile_cfg_valid  (i_tile_cfg_valid),
        .i_patch_base      (i_patch_base),
        .i_out_ch_base     (i_out_ch_base),
        .i_row_mask        (i_row_mask),
        .i_col_mask        (i_col_mask),
        .i_tile_last       (i_tile_last),

        // PE Result
        .i_result_data     (pe_result_data),
        .i_result_valid    (pe_result_valid),

        .o_result_space_ready(o_result_space_ready),

        // Parameter Load
        .i_param_load_start(i_param_load_start),
        .i_param_wr_en     (i_param_wr_en),
        .i_param_wr_addr   (i_param_wr_addr),
        .i_param_wr_data   (i_param_wr_data),

        .o_param_load_done (o_param_load_done),
        .o_params_ready    (o_params_ready),

        // Input Buffer Write Interface
        .i_write_grant     (i_write_grant),

        .o_wr_en           (o_wr_en),
        .o_wr_addr         (o_wr_addr),
        .o_wr_data         (o_wr_data),
        .o_wr_be           (o_wr_be),

        // IRQ / Completion
        .i_irq_en          (i_irq_en),
        .i_irq_clear       (i_irq_clear),

        .o_tile_in_done    (o_tile_in_done),
        .o_layer_done      (o_layer_done),

        .o_final_result    (o_final_result),
        .o_done_status     (o_done_status),
        .o_irq             (o_irq)
    );

endmodule
