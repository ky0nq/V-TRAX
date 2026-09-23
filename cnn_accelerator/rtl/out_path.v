`timescale 1ns / 1ps

module out_path (
    input wire clk,
    input wire rst_n,

    input wire               i_layer_cfg_valid,
    input wire               i_is_fc,
    input wire               i_is_final_layer,
    input wire               i_pool_en,
    input wire        [ 6:0] i_conv_w,
    input wire        [ 6:0] i_conv_h,
    input wire        [ 5:0] i_out_channels,
    input wire        [13:0] i_dst_base,
    input wire               i_relu_en,
    input wire signed [31:0] i_quant_multiplier,
    input wire        [ 5:0] i_quant_shift,
    input wire        [ 5:0] i_bias_base,

    // Final FC2 angle requantization
    input wire signed [31:0] i_angle_multiplier,
    input wire        [ 5:0] i_angle_shift,

    input wire        i_tile_cfg_valid,
    input wire [11:0] i_patch_base,
    input wire [ 4:0] i_out_ch_base,
    input wire [ 2:0] i_row_mask,
    input wire [ 2:0] i_col_mask,
    input wire        i_tile_last,

    input wire [287:0] i_result_data,
    input wire [  8:0] i_result_valid,

    output wire o_result_space_ready,

    input wire i_param_load_start,

    input wire               i_param_wr_en,
    input wire        [ 5:0] i_param_wr_addr,
    input wire signed [31:0] i_param_wr_data,

    output wire o_param_load_done,

    output wire o_params_ready,

    input wire i_write_grant,

    output wire        o_wr_en,
    output wire [13:0] o_wr_addr,
    output wire [23:0] o_wr_data,
    output wire [ 2:0] o_wr_be,

    input wire i_irq_en,
    input wire i_irq_clear,

    output wire o_tile_in_done,
    output wire o_layer_done,

    output wire signed [31:0] o_final_result,
    output wire               o_done_status,
    output wire               o_irq
);
    // =========================================================
    // Layer Configuration Registers
    // =========================================================
    reg               is_fc_reg;
    reg               is_final_layer_reg;
    reg               pool_en_reg;
    reg        [ 6:0] conv_w_reg;
    reg        [ 5:0] out_channels_reg;
    reg        [13:0] dst_base_reg;
    reg               relu_en_reg;
    reg signed [31:0] quant_multiplier_reg;
    reg        [ 5:0] quant_shift_reg;
    reg        [ 5:0] bias_base_reg;

    reg signed [31:0] angle_multiplier_reg;
    reg        [ 5:0] angle_shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_fc_reg            <= 1'b0;
            is_final_layer_reg   <= 1'b0;
            pool_en_reg          <= 1'b0;
            conv_w_reg           <= 7'd0;
            out_channels_reg     <= 6'd0;
            dst_base_reg         <= 14'd0;
            relu_en_reg          <= 1'b0;
            quant_multiplier_reg <= 32'd0;
            quant_shift_reg      <= 6'd0;
            bias_base_reg        <= 6'd0;
            angle_multiplier_reg <= 32'd0;
            angle_shift_reg      <= 6'd0;
        end else if (i_layer_cfg_valid) begin
            is_fc_reg            <= i_is_fc;
            is_final_layer_reg   <= i_is_final_layer;
            pool_en_reg          <= i_pool_en;
            conv_w_reg           <= i_conv_w;
            out_channels_reg     <= i_out_channels;
            dst_base_reg         <= i_dst_base;
            relu_en_reg          <= i_relu_en;
            quant_multiplier_reg <= i_quant_multiplier;
            quant_shift_reg      <= i_quant_shift;
            bias_base_reg        <= i_bias_base;
            angle_multiplier_reg <= i_angle_multiplier;
            angle_shift_reg      <= i_angle_shift;
        end
    end

    // =========================================================
    // Tile Configuration : 직접 전달 (핸드셰이크 정리 2026-09-23)
    //   cnn_cntl 은 이전 타일의 tile_in_done 뒤, pe_cntl 이 IDLE 이고 수집기가 빈 것을 본 뒤에만
    //   i_tile_cfg_valid 를 낸다 (요구사항 4 / 14). 그 시점에 output_fifo (tile_active=0) 와
    //   post_process (READY, cfg_change_allowed=1) 는 항상 받을 수 있으므로 여기서 다시 버퍼링하지
    //   않는다. 아래 두 ready 는 검증용으로만 남긴다 (둘 다 1 이어야 정상).
    // =========================================================
    wire        fifo_cfg_ready;
    wire        pp_cfg_ready;
    wire        tile_cfg_fire = i_tile_cfg_valid;

    // =========================================================
    // Output FIFO Interface Wires
    // =========================================================

    wire        [95:0] fifo_data;
    wire               fifo_valid;
    wire        [ 2:0] fifo_keep;
    wire        [18:0] fifo_meta;
    wire               fifo_ready;

    // =========================================================
    // Post Process Interface Wires
    // =========================================================
    wire               pp_params_ready;

    wire               pp_bias_req_valid;
    wire               pp_bias_req_ready;
    wire        [ 5:0] pp_bias_addr;

    wire signed [31:0] pp_bias_data;
    wire               pp_bias_rsp_valid;
    wire               pp_bias_rsp_ready;

    wire        [23:0] pp_data;
    wire               pp_valid;
    wire        [ 2:0] pp_keep;
    wire        [18:0] pp_meta;
    wire               pp_ready;

    wire signed [ 7:0] pp_final_data;
    wire               pp_final_valid;
    wire               pp_final_ready;

    wire               params_loaded;

    assign o_params_ready = pp_params_ready;

    // =========================================================
    // Pooling Unit Interface Wires
    // =========================================================
    wire        [23:0] pool_data;
    wire               pool_valid;
    wire        [ 2:0] pool_keep;
    wire        [18:0] pool_meta;
    wire               pool_ready;

    wire               pool_tile_in_done;
    wire               result_final_tile_done;

    param_buf #(
        .PARAM_TOTAL(43)
    ) u_param_buf (
        .clk  (clk),
        .rst_n(rst_n),

        // cnn_cntl -> param_buf
        .i_load_start(i_param_load_start),
        .i_wr_en     (i_param_wr_en),
        .i_wr_addr   (i_param_wr_addr),
        .i_wr_data   (i_param_wr_data),

        // Load Status
        .o_load_done(o_param_load_done),
        .o_loaded   (params_loaded),

        // post_process -> param_buf : Bias Read Request
        .i_rd_req_valid(pp_bias_req_valid),
        .o_rd_req_ready(pp_bias_req_ready),
        .i_rd_addr     (pp_bias_addr),

        // param_buf -> post_process : Bias Read Response
        .o_rd_data     (pp_bias_data),
        .o_rd_rsp_valid(pp_bias_rsp_valid),
        .i_rd_rsp_ready(pp_bias_rsp_ready)
    );


    output_fifo u_output_fifo (
        .clk  (clk),
        .rst_n(rst_n),

        .i_cfg_valid  (tile_cfg_fire),
        .i_patch_base (i_patch_base),
        .i_out_ch_base(i_out_ch_base),
        .i_row_mask   (i_row_mask),
        .i_col_mask   (i_col_mask),
        .i_tile_last  (i_tile_last),

        .i_is_fc (is_fc_reg),
        .i_conv_w(conv_w_reg),

        .i_result_data (i_result_data),
        .i_result_valid(i_result_valid),

        .o_cfg_ready         (fifo_cfg_ready),
        .o_result_space_ready(o_result_space_ready),

        .o_data (fifo_data),
        .o_valid(fifo_valid),
        .i_ready(fifo_ready),
        .o_keep (fifo_keep),
        .o_meta (fifo_meta)
    );

    post_process u_post_process (
        .clk  (clk),
        .rst_n(rst_n),

        // Tile / Layer Configuration
        .i_cfg_valid(tile_cfg_fire),
        .o_cfg_ready(pp_cfg_ready),

        .i_col_mask   (i_col_mask),
        .i_out_ch_base(i_out_ch_base),

        .i_bias_base     (bias_base_reg),
        .i_relu_en       (relu_en_reg),
        .i_is_final_layer(is_final_layer_reg),

        .i_quant_multiplier(quant_multiplier_reg),
        .i_quant_shift     (quant_shift_reg),

        .i_angle_multiplier(angle_multiplier_reg),
        .i_angle_shift     (angle_shift_reg),

        // Parameter Buffer Interface
        .i_params_loaded(params_loaded),
        .o_params_ready (pp_params_ready),

        // Bias Read Request
        .o_bias_req_valid(pp_bias_req_valid),
        .i_bias_req_ready(pp_bias_req_ready),
        .o_bias_addr     (pp_bias_addr),

        // Bias Read Response
        .i_bias_data     (pp_bias_data),
        .i_bias_rsp_valid(pp_bias_rsp_valid),
        .o_bias_rsp_ready(pp_bias_rsp_ready),

        // output_fifo -> post_process
        .i_data (fifo_data),
        .i_valid(fifo_valid),
        .o_ready(fifo_ready),
        .i_keep (fifo_keep),
        .i_meta (fifo_meta),

        // Normal Output
        .o_data (pp_data),
        .o_valid(pp_valid),
        .i_ready(pp_ready),
        .o_keep (pp_keep),
        .o_meta (pp_meta),

        // Final Output
        .o_final_data (pp_final_data),
        .o_final_valid(pp_final_valid),
        .i_final_ready(pp_final_ready)
    );

    pooling_unit u_pooling_unit (
        .clk  (clk),
        .rst_n(rst_n),

        // Layer Configuration
        .i_cfg_valid(i_layer_cfg_valid),
        .i_pool_en  (i_pool_en),
        .i_in_w     (i_conv_w),
        .i_in_h     (i_conv_h),
        .i_channels (i_out_channels),

        // post_process -> pooling_unit
        .i_data (pp_data),
        .i_valid(pp_valid),
        .o_ready(pp_ready),
        .i_keep (pp_keep),
        .i_meta (pp_meta),

        // pooling_unit -> result_buf
        .o_data (pool_data),
        .o_valid(pool_valid),
        .i_ready(pool_ready),
        .o_keep (pool_keep),
        .o_meta (pool_meta),

        // Tile Input Completion
        .o_tile_in_done(pool_tile_in_done)
    );

    result_buf u_result_buf (
        .clk  (clk),
        .rst_n(rst_n),

        // Layer Configuration
        .i_cfg_valid      (i_layer_cfg_valid),
        .i_is_final_layer (i_is_final_layer),
        .i_dst_base       (i_dst_base),
        .i_dst_channels   (i_out_channels),

        // pooling_unit -> result_buf
        .i_data       (pool_data),
        .i_valid      (pool_valid),
        .o_ready      (pool_ready),
        .i_keep       (pool_keep),
        .i_meta       (pool_meta),

        // Input Buffer Write Port
        .i_write_grant(i_write_grant),
        .o_wr_en      (o_wr_en),
        .o_wr_addr    (o_wr_addr),
        .o_wr_data    (o_wr_data),
        .o_wr_be      (o_wr_be),

        // post_process Final Scalar -> result_buf
        .i_final_data (pp_final_data),
        .i_final_valid(pp_final_valid),
        .o_final_ready(pp_final_ready),

        // Final Tile Completion
        .o_final_tile_done(result_final_tile_done),

        // Layer Done / CSR / IRQ
        .o_layer_done  (o_layer_done),
        .i_irq_en      (i_irq_en),
        .i_irq_clear   (i_irq_clear),
        .o_final_result(o_final_result),
        .o_done_status (o_done_status),
        .o_irq         (o_irq)
    );

    assign o_tile_in_done = pool_tile_in_done | result_final_tile_done;

endmodule
