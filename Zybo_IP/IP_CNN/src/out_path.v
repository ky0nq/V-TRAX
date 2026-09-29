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

module output_fifo (
    input wire clk,
    input wire rst_n,

    // Tile configuration
    input wire        i_cfg_valid,
    input wire [11:0] i_patch_base,
    input wire [ 4:0] i_out_ch_base,
    input wire [ 2:0] i_row_mask,
    input wire [ 2:0] i_col_mask,
    input wire        i_tile_last,

    input wire       i_is_fc,
    input wire [6:0] i_conv_w,

    // PE result input
    input wire [287:0] i_result_data,
    input wire [  8:0] i_result_valid,

    // Tile configuration handshake
    output wire o_cfg_ready,

    // PE result space status
    output wire o_result_space_ready,

    // Output
    output reg  [95:0] o_data,
    output reg         o_valid,
    input  wire        i_ready,

    output reg  [ 2:0] o_keep,
    output wire [18:0] o_meta
);

    // result save register
    reg  [95:0] row0_data;
    reg  [95:0] row1_data;
    reg  [95:0] row2_data;

    reg  [ 2:0] row0_valid;
    reg  [ 2:0] row1_valid;
    reg  [ 2:0] row2_valid;

    // tile set register
    reg  [11:0] patch_base_reg;
    reg  [ 4:0] out_ch_base_reg;
    reg  [ 2:0] row_mask_reg;
    reg  [ 2:0] col_mask_reg;
    reg         tile_last_reg;
    reg         is_fc_reg;
    reg  [ 6:0] conv_w_reg;
    reg  [ 2:0] conv_w_sh_reg;   // log2(conv_w). W 는 2 의 거듭제곱 (64 / 32 / 16)

    // 2 의 거듭제곱 (1 ~ 64) 의 log2. 그 밖의 값은 최상위 1 의 자리 (타이밍 : 나눗셈 대신 shift 용)
    function [2:0] f_log2_7;
        input [6:0] v;
        begin
            casez (v)
                7'b1??????: f_log2_7 = 3'd6;
                7'b01?????: f_log2_7 = 3'd5;
                7'b001????: f_log2_7 = 3'd4;
                7'b0001???: f_log2_7 = 3'd3;
                7'b00001??: f_log2_7 = 3'd2;
                7'b000001?: f_log2_7 = 3'd1;
                default:    f_log2_7 = 3'd0;
            endcase
        end
    endfunction

    // output state register
    reg         tile_active;

    // output row num
    reg  [ 1:0] out_row;

    // handshake
    wire        cfg_fire;
    wire        out_fire;

    assign o_cfg_ready          = rst_n && !tile_active;
    assign o_result_space_ready = rst_n && !tile_active;

    wire cfg_mask_valid;

    assign cfg_mask_valid =
    (
        (i_row_mask == 3'b001) ||
        (i_row_mask == 3'b011) ||
        (i_row_mask == 3'b111)
    ) &&
    (
        (i_col_mask == 3'b001) ||
        (i_col_mask == 3'b011) ||
        (i_col_mask == 3'b111)
    );
    assign cfg_fire = i_cfg_valid && o_cfg_ready && cfg_mask_valid;
    assign out_fire = o_valid && i_ready;

    // select first effective row
    reg [1:0] first_row;

    always @(*) begin
        first_row = 2'd3;

        if (i_row_mask[0]) begin
            first_row = 2'd0;
        end else if (i_row_mask[1]) begin
            first_row = 2'd1;
        end else if (i_row_mask[2]) begin
            first_row = 2'd2;
        end
    end

    // select next effective row
    reg [1:0] next_row;

    always @(*) begin
        next_row = 2'd3;

        case (out_row)
            2'd0: begin
                if (row_mask_reg[1]) next_row = 2'd1;
                else if (row_mask_reg[2]) next_row = 2'd2;
            end
            2'd1: begin
                if (row_mask_reg[2]) next_row = 2'd2;
            end
            2'd2: begin
                next_row = 2'd3;
            end
            default: begin
                next_row = 2'd3;
            end
        endcase
    end

    wire row0_complete;
    wire row1_complete;
    wire row2_complete;

    assign row0_complete = row_mask_reg[0] && ((row0_valid & col_mask_reg) == col_mask_reg);
    assign row1_complete = row_mask_reg[1] && ((row1_valid & col_mask_reg) == col_mask_reg);
    assign row2_complete = row_mask_reg[2] && ((row2_valid & col_mask_reg) == col_mask_reg);

    wire last_valid_row;

    wire last_valid_0 = (out_row == 2'd0) && row_mask_reg[0] && !row_mask_reg[1] && !row_mask_reg[2];
    wire last_valid_1 = (out_row == 2'd1) && row_mask_reg[1] && !row_mask_reg[2];
    wire last_valid_2 = (out_row == 2'd2) && row_mask_reg[2];

    assign last_valid_row = last_valid_0 || last_valid_1 || last_valid_2;

    // select out_row data
    reg [95:0] selected_row_data;
    reg        selected_row_complete;

    always @(*) begin
        selected_row_data     = 96'd0;
        selected_row_complete = 1'b0;

        case (out_row)
            2'd0: begin
                selected_row_data     = row0_data;
                selected_row_complete = row0_complete;
            end
            2'd1: begin
                selected_row_data     = row1_data;
                selected_row_complete = row1_complete;
            end
            2'd2: begin
                selected_row_data     = row2_data;
                selected_row_complete = row2_complete;
            end
            default: begin
                selected_row_data     = 96'd0;
                selected_row_complete = 1'b0;
            end
        endcase
    end


    // output
    always @(*) begin
        o_data  = 96'd0;
        o_keep  = 3'b000;
        o_valid = 1'b0;

        if (tile_active && selected_row_complete) begin
            o_keep  = col_mask_reg;
            o_valid = 1'b1;

            case (col_mask_reg)
                3'b001: begin
                    o_data = {64'd0, selected_row_data[31:0]};
                end
                3'b011: begin
                    o_data = {32'd0, selected_row_data[63:0]};
                end
                3'b111: begin
                    o_data = selected_row_data;
                end
                default: begin
                    o_data  = 96'd0;
                    o_keep  = 3'b000;
                    o_valid = 1'b0;
                end
            endcase
        end
    end

    wire [11:0] current_position;

    // 패치 순번 (2x2 블록 Z 순서, 명세 R369 / R483 : W=64 이면 패치 0..8 -> pos 0,1,64,65,2,3,66,67,4)
    // -> 화소 위치 y * W + x.  W 가 2 의 거듭제곱이라 shift 로 계산한다 (2026-09-23. 그 전에는 raster 로
    // 내서 pooling 의 q 순서 검사에서 막혔다)
    wire [11:0] cur_patch = patch_base_reg + {10'd0, out_row};
    wire [ 9:0] blk       = cur_patch[11:2];                                   // 2x2 블록 번호
    wire [ 1:0] q         = cur_patch[1:0];                                    // 블록 안 : 0 (0,0) 1 (1,0) 2 (0,1) 3 (1,1)
    wire [ 2:0] wpr_sh    = (conv_w_sh_reg == 3'd0) ? 3'd0 : (conv_w_sh_reg - 3'd1);   // log2(W/2)
    wire [ 9:0] by        = blk >> wpr_sh;
    wire [ 9:0] bx        = blk & ((10'd1 << wpr_sh) - 10'd1);
    wire [11:0] pos_y     = {1'b0, by, 1'b0} + {11'd0, q[1]};
    wire [11:0] pos_x     = {1'b0, bx, 1'b0} + {11'd0, q[0]};

    assign current_position = is_fc_reg ? 12'd0 : ((pos_y << conv_w_sh_reg) | pos_x);
    assign o_meta = {
        tile_last_reg && last_valid_row,  // [18] layer_end
        last_valid_row,  // [17] tile_end
        out_ch_base_reg,  // [16:12] out_ch_base
        current_position  // [11:0] position
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row0_data       <= 96'd0;
            row1_data       <= 96'd0;
            row2_data       <= 96'd0;

            row0_valid      <= 3'b000;
            row1_valid      <= 3'b000;
            row2_valid      <= 3'b000;

            patch_base_reg  <= 12'd0;
            out_ch_base_reg <= 5'd0;
            row_mask_reg    <= 3'b000;
            col_mask_reg    <= 3'b000;
            tile_last_reg   <= 1'b0;
            is_fc_reg       <= 1'b0;
            conv_w_reg      <= 7'd0;
            conv_w_sh_reg   <= 3'd0;

            tile_active     <= 1'b0;
            out_row         <= 2'd0;
        end else begin
            if (cfg_fire) begin
                patch_base_reg  <= i_patch_base;
                out_ch_base_reg <= i_out_ch_base;
                row_mask_reg    <= i_row_mask;
                col_mask_reg    <= i_col_mask;
                tile_last_reg   <= i_tile_last;
                is_fc_reg       <= i_is_fc;
                conv_w_reg      <= i_conv_w;
                conv_w_sh_reg   <= f_log2_7(i_conv_w);

                tile_active     <= 1'b1;

                out_row         <= first_row;

                row0_valid      <= 3'b000;
                row1_valid      <= 3'b000;
                row2_valid      <= 3'b000;
            end else begin
                if (tile_active) begin
                    //row 0
                    if (i_result_valid[0]) begin
                        row0_data[31:0] <= i_result_data[31:0];
                        row0_valid[0]   <= 1'b1;
                    end
                    if (i_result_valid[1]) begin
                        row0_data[63:32] <= i_result_data[63:32];
                        row0_valid[1]    <= 1'b1;
                    end
                    if (i_result_valid[2]) begin
                        row0_data[95:64] <= i_result_data[95:64];
                        row0_valid[2]    <= 1'b1;
                    end

                    //row 1
                    if (i_result_valid[3]) begin
                        row1_data[31:0] <= i_result_data[127:96];
                        row1_valid[0]   <= 1'b1;
                    end
                    if (i_result_valid[4]) begin
                        row1_data[63:32] <= i_result_data[159:128];
                        row1_valid[1]    <= 1'b1;
                    end
                    if (i_result_valid[5]) begin
                        row1_data[95:64] <= i_result_data[191:160];
                        row1_valid[2]    <= 1'b1;
                    end

                    // row 2
                    if (i_result_valid[6]) begin
                        row2_data[31:0] <= i_result_data[223:192];
                        row2_valid[0]   <= 1'b1;
                    end
                    if (i_result_valid[7]) begin
                        row2_data[63:32] <= i_result_data[255:224];
                        row2_valid[1]    <= 1'b1;
                    end
                    if (i_result_valid[8]) begin
                        row2_data[95:64] <= i_result_data[287:256];
                        row2_valid[2]    <= 1'b1;
                    end
                end

                // output handshake
                if (out_fire) begin
                    case (out_row)
                        2'd0: row0_valid <= 3'b000;
                        2'd1: row1_valid <= 3'b000;
                        2'd2: row2_valid <= 3'b000;
                        default: begin
                            row0_valid <= row0_valid;
                            row1_valid <= row1_valid;
                            row2_valid <= row2_valid;
                        end
                    endcase
                    if (last_valid_row) begin
                        tile_active <= 1'b0;
                        out_row     <= 2'd0;
                    end else begin
                        out_row <= next_row;
                    end
                end
            end
        end
    end

endmodule

module param_buf #(
    parameter integer PARAM_TOTAL = 43      // 43 : Conv0 6 + Conv1 4 + fc1 32 + fc2 1
) (
    input  wire        clk,
    input  wire        rst_n,

    // cnn_cntl -> out_path
    input  wire               i_load_start,
    input  wire               i_wr_en,
    input  wire        [ 5:0] i_wr_addr,
    input  wire signed [31:0] i_wr_data,
    output reg                o_load_done,
    output reg                o_loaded,

    // -> post_process
    input  wire               i_rd_req_valid,
    output wire               o_rd_req_ready,
    input  wire        [ 5:0] i_rd_addr,
    output reg  signed [31:0] o_rd_data,
    output reg                o_rd_rsp_valid,
    input  wire               i_rd_rsp_ready      
);

    reg signed [31:0] mem [0:63];                // 64-depth. Using only 0 ~ 42
    reg        loading;                     // i_load_start ~ last store = High-state
    reg [ 5:0] wr_cnt;                      // Load counter 0 ~ 43

    wire wr_fire = loading && !i_load_start && i_wr_en;
    wire wr_last   = wr_fire && (wr_cnt == PARAM_TOTAL - 1); // 43th store
    wire req_fire  = i_rd_req_valid && o_rd_req_ready;
    wire rsp_fire  = o_rd_rsp_valid && i_rd_rsp_ready;

    // Read request Access state condition
    // 응답이 같은 클럭에 수락되면 다음 요청을 바로 받는다 (연속 읽기. 핸드셰이크 정리 2026-09-23)
    assign o_rd_req_ready = o_loaded && !loading && !i_load_start && (!o_rd_rsp_valid || i_rd_rsp_ready);

    // memory Read/Write
    always @(posedge clk) begin
        if (wr_fire)  mem[i_wr_addr] <= i_wr_data;
        if (req_fire) o_rd_data      <= mem[i_rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            loading        <= 1'b0;
            wr_cnt         <= 6'd0;
            o_load_done    <= 1'b0;
            o_loaded       <= 1'b0;
            o_rd_rsp_valid <= 1'b0;
        end else begin
            o_load_done <= 1'b0;                 

            if (i_load_start) begin // Load Start
                loading        <= 1'b1;
                wr_cnt         <= 6'd0;
                o_loaded       <= 1'b0;
                o_rd_rsp_valid <= 1'b0;
            end else if (wr_fire) begin
                wr_cnt <= wr_cnt + 6'd1;
                if (wr_last) begin // Last Parameter Store
                    loading     <= 1'b0;
                    o_loaded    <= 1'b1;
                    o_load_done <= 1'b1;
                end
            end

            // Read response
            if      (req_fire)      o_rd_rsp_valid <= 1'b1;
            else if (rsp_fire)      o_rd_rsp_valid <= 1'b0;
        end
    end

endmodule

module post_process (
    input wire clk,
    input wire rst_n,

    // Tile / Layer Configuration
    input wire i_cfg_valid,
    output wire o_cfg_ready,

    input wire [2:0] i_col_mask,
    input wire [4:0] i_out_ch_base,

    input wire [5:0] i_bias_base,
    input wire       i_relu_en,
    input wire       i_is_final_layer,

    input wire signed [31:0] i_quant_multiplier,
    input wire        [ 5:0] i_quant_shift,

    input wire signed [31:0] i_angle_multiplier,
    input wire        [ 5:0] i_angle_shift,

    // Parameter Buffer Interface
    input  wire i_params_loaded,
    output reg  o_params_ready,

    // Bias Read Request
    output reg        o_bias_req_valid,
    input  wire       i_bias_req_ready,
    output reg  [5:0] o_bias_addr,

    // Bias Read Response
    input  wire signed [31:0] i_bias_data,
    input  wire               i_bias_rsp_valid,
    output reg                o_bias_rsp_ready,

    // Input Stream from Output FIFO
    input  wire [95:0] i_data,
    input  wire        i_valid,
    output reg         o_ready,

    input wire [ 2:0] i_keep,
    input wire [18:0] i_meta,

    // Normal Output Stream to Pooling Unit
    // 3 lanes x INT8 = 24-bit
    output reg  [23:0] o_data,
    output reg         o_valid,
    input  wire        i_ready,

    output reg [ 2:0] o_keep,
    output reg [18:0] o_meta,

    // Final Layer Output to Result Buffer
    // INT32 scalar path
    output reg signed [7:0] o_final_data,
    output reg              o_final_valid,
    input  wire             i_final_ready
);
    // =========================================================
    // Internal Configuration Registers
    // =========================================================

    // 현재 Tile에서 유효한 출력 channel lane을 저장
    // 001 : lane0
    // 011 : lane0~1
    // 111 : lane0~2
    reg [2:0] col_mask_reg;

    // 현재 Tile의 첫 번째 output channel 번호 저장
    // Bias 주소 계산 시 사용
    reg [4:0] out_ch_base_reg;

    // 현재 Layer의 Bias 시작 주소 저장
    // 실제 Bias 주소 = bias_base + out_ch_base + lane
    reg [5:0] bias_base_reg;

    // 현재 Layer에서 ReLU 적용 여부 저장
    reg relu_en_reg;

    // 현재 Tile이 Final Layer인지 저장
    // Normal path와 Final path를 분기하기 위해 사용
    reg is_final_layer_reg;

    // 중간 Layer requantization에 사용하는 multiplier 저장
    reg signed [31:0] quant_multiplier_reg;

    // 중간 Layer requantization에 사용하는 right shift 값 저장
    reg        [ 5:0] quant_shift_reg;

    // Final Layer의 INT32 결과를 signed INT8 각도로
    // 변환할 때 사용하는 multiplier 저장
    reg signed [31:0] angle_multiplier_reg;

    // Final Layer 각도 변환 시 사용하는 right shift 값 저장
    reg        [ 5:0] angle_shift_reg;

    // 새로운 Tile configuration을 받아도 되는 상태인지 표시
    reg cfg_change_allowed;


    // =========================================================
    // Bias Cache
    // =========================================================

    // 현재 output channel tile에서 사용하는 Bias를 저장
    // Bias는 Tile 설정 시 param_buf에서 한 번 읽고,
    // 해당 Tile의 여러 row 처리 동안 반복 사용
    reg signed [31:0] bias_cache0;
    reg signed [31:0] bias_cache1;
    reg signed [31:0] bias_cache2;

    // ---- Bias 읽기 (핸드셰이크 정리 2026-09-23) ----
    //   요청은 lane 0..n-1 을 연속으로 내고 (req_idx), 응답은 오는 대로 받는다 (rsp_idx, ready 항상 1).
    //   전에는 lane 마다 요청 -> 응답 대기 를 반복해서 타일마다 7 clk 이 들었다.
    reg [1:0] bias_req_idx;      // 다음에 요청할 lane
    reg [1:0] bias_rsp_idx;      // 다음에 받을 lane
    reg [1:0] bias_n_lanes;      // 이 타일에 필요한 lane 수 (col_mask 001/011/111 -> 1/2/3)

    // ---- Bias 캐시 2 entry ----
    //   같은 (bias_base, out_ch_base, col_mask) 타일이 다시 오면 param_buf 를 읽지 않는다.
    //   Conv0 / Conv1 은 채널 그룹 2 개가 번갈아 오므로 첫 두 타일 뒤로는 전부 hit (타일당 -7 clk).
    //   param_buf 가 다시 적재되면 (i_params_loaded=0) 비운다.
    reg        [ 1:0] bc_valid;
    reg        [13:0] bc_key0, bc_key1;           // {bias_base[5:0], out_ch_base[4:0], col_mask[2:0]}
    reg signed [31:0] bc0_b0, bc0_b1, bc0_b2;
    reg signed [31:0] bc1_b0, bc1_b1, bc1_b2;
    reg               bc_next;                    // 다음에 채울 entry
    wire       [13:0] cfg_key  = {i_bias_base, i_out_ch_base, i_col_mask};
    wire              cfg_hit0 = bc_valid[0] && (bc_key0 == cfg_key);
    wire              cfg_hit1 = bc_valid[1] && (bc_key1 == cfg_key);
    wire              cfg_hit  = cfg_hit0 || cfg_hit1;


    // =========================================================
    // Input Beat Buffer
    // =========================================================

    // output_fifo에서 받은 96-bit beat 전체를 저장
    // 한 beat를 저장한 뒤 lane0~2를 순차적으로 처리하기 위해 필요
    reg [95:0] data_reg;

    // 저장된 beat에서 실제 유효한 lane 정보
    // data_reg와 동일한 handshake에서 함께 저장
    reg [2:0] keep_reg;

    // position / channel / tile_end / layer_end 정보를
    // 처리 완료 후 downstream으로 그대로 전달하기 위해 저장
    reg [18:0] meta_reg;


    // =========================================================
    // Input Lane Split
    // =========================================================

    // 96-bit 입력 beat를 3개의 signed INT32 lane으로 해석
    // 별도 32-bit reg 3개를 두지 않고 data_reg에서 직접 분리
    wire signed [31:0] lane0_data;
    wire signed [31:0] lane1_data;
    wire signed [31:0] lane2_data;

    assign lane0_data = $signed(data_reg[31:0]);
    assign lane1_data = $signed(data_reg[63:32]);
    assign lane2_data = $signed(data_reg[95:64]);


    // =========================================================
    // Lane Processing Control
    // =========================================================

    // 현재 처리 중인 lane 번호
    // Requant 연산기 하나를 lane0~2가 공유하기 위해 사용
    reg [1:0] lane_idx;

    // lane_idx에 따라 선택된 현재 PE 결과
    // lane0_data / lane1_data / lane2_data 중 하나를 선택
    reg signed [31:0] current_lane_data;

    // 현재 lane에 대응하는 Bias
    // bias_cache0~2 중 하나를 선택
    reg signed [31:0] current_bias;

    // 현재 lane이 실제 유효한 lane인지 확인
    // keep_reg[lane_idx]를 사용하여 invalid lane 계산을 건너뜀
    reg current_lane_valid;

    // =========================================================
    // Current Lane Selector
    // =========================================================
    always @(*) begin
        // 기본값
        // 잘못된 lane_idx가 들어와도 latch가 생기지 않도록 설정
        current_lane_data  = 32'd0;
        current_bias       = 32'd0;
        current_lane_valid = 1'b0;

        case (lane_idx)

            2'd0: begin
                current_lane_data  = lane0_data;
                current_bias       = bias_cache0;
                current_lane_valid = keep_reg[0];
            end

            2'd1: begin
                current_lane_data  = lane1_data;
                current_bias       = bias_cache1;
                current_lane_valid = keep_reg[1];
            end

            2'd2: begin
                current_lane_data  = lane2_data;
                current_bias       = bias_cache2;
                current_lane_valid = keep_reg[2];
            end

            default: begin
                current_lane_data  = 32'd0;
                current_bias       = 32'd0;
                current_lane_valid = 1'b0;
            end
        endcase
    end

    // =========================================================
    // Requant 파이프라인 (타이밍 : 2026-09-23)
    //   한 클럭에 있던 bias 덧셈 -> 32x32 곱 -> 반올림 shift -> 포화 를 4 단으로 나눴다.
    //   PROCESS 에서 lane 을 한 클럭에 하나씩 넣고 (s0), 3 클럭 뒤 (s3) 결과가 result_lane 에 써진다.
    //   lane 순서 / keep / meta 처리는 그대로. 한 beat 에 (lane 수 + 3) 클럭이 든다.
    // =========================================================

    // ---- s0 : bias 덧셈 + INT32 포화 (조합, PROCESS 의 현재 lane) ----
    reg signed [32:0] bias_sum_ext;   // 33-bit 로 더해서 overflow 를 본다
    reg signed [31:0] acc32_reg;      // Golden 의 A32

    always @(*) begin
        bias_sum_ext = {current_lane_data[31], current_lane_data} + {current_bias[31], current_bias};
    end

    always @(*) begin
        case (bias_sum_ext[32:31])
            2'b01:   acc32_reg = 32'h7FFF_FFFF;   // 양의 overflow -> INT32 최댓값
            2'b10:   acc32_reg = 32'h8000_0000;   // 음의 overflow -> INT32 최솟값
            default: acc32_reg = bias_sum_ext[31:0];
        endcase
    end

    // 현재 레이어의 M / S.  Final FC2 는 각도 변환용 M / S
    reg signed [31:0] current_multiplier;
    reg        [ 5:0] current_shift;

    always @(*) begin
        if (is_final_layer_reg) begin
            current_multiplier = angle_multiplier_reg;
            current_shift      = angle_shift_reg;
        end
        else begin
            current_multiplier = quant_multiplier_reg;
            current_shift      = quant_shift_reg;
        end
    end

    // ---- 파이프라인 레지스터 ----
    //   s1 : acc32, M (DSP 입력 레지스터)   s2 : 64-bit 곱   s3 : 곱 + 반올림 오프셋
    //   s3 에서 shift + 포화 해서 result_lane / final_result_reg 에 쓴다
    reg               s1_valid, s2_valid, s3_valid;   // 그 단에 유효 lane 이 있다
    reg        [ 1:0] s1_lane,  s2_lane,  s3_lane;    // 그 lane 번호
    reg               s1_last,  s2_last,  s3_last;    // 이 beat 의 마지막 lane 이다
    reg signed [31:0] s1_acc32;
    reg signed [31:0] s1_mult;
    reg signed [63:0] s2_prod;
    reg signed [63:0] s3_sum;

    // 반올림 : round-to-nearest, ties away from zero
    //   P >= 0 : (P + 2^(S-1)) >> S
    //   P <  0 : (P + 2^(S-1) - 1) >>> S      (= -((|P| + 2^(S-1)) >> S) 와 같다)
    //   S = 0  : P 그대로
    //   |P| <= 2^62 (INT32 x INT32) 이고 S <= 30 이라 64-bit 에서 넘치지 않는다
    wire        [63:0] round_half = (current_shift == 6'd0) ? 64'd0 : (64'd1 << (current_shift - 6'd1));
    wire signed [63:0] round_off  = (current_shift == 6'd0) ? 64'sd0 :
                                    (s2_prod[63] ? ($signed(round_half) - 64'sd1) : $signed(round_half));

    // s3 : 산술 우측 shift 뒤 값 (부호 포함)
    wire signed [63:0] rounded_result = s3_sum >>> current_shift;

    // 일반 레이어 : ReLU + signed INT8 포화
    reg signed [7:0] normal_int8_result;

    always @(*) begin
        if (relu_en_reg && rounded_result[63])        normal_int8_result = 8'd0;    // ReLU
        else if (rounded_result > $signed(64'd127))   normal_int8_result = 8'h7F;   // 최댓값 초과
        else if (rounded_result < -$signed(64'd128))  normal_int8_result = 8'h80;   // 최솟값 미만
        else                                          normal_int8_result = rounded_result[7:0];
    end

    // Final FC2 : ReLU 없이 signed INT8 포화 (1 도 단위 각도)
    reg signed [7:0] final_int8_result;

    always @(*) begin
        if (rounded_result > $signed(64'd127))        final_int8_result = 8'h7F;
        else if (rounded_result < -$signed(64'd128))  final_int8_result = 8'h80;
        else                                          final_int8_result = rounded_result[7:0];
    end

    // Final 결과를 handshake 완료까지 유지
    reg signed [7:0] final_result_reg;

    // =========================================================
    // Normal INT8 Output Lane Registers
    // =========================================================

    // 각 lane의 post-process 완료 INT8 결과 저장
    // 3개 lane 처리가 모두 끝난 뒤 24-bit o_data로 묶어서 출력
    reg signed [7:0] result_lane0;
    reg signed [7:0] result_lane1;
    reg signed [7:0] result_lane2;

    // 이 beat 의 마지막 lane 을 파이프라인에 넣었다 (넣은 뒤 결과가 나올 때까지 기다린다)
    reg issue_done;

    // =========================================================
    // FSM State
    // =========================================================
    reg [2:0] state;
    reg [2:0] next_state;

    // FSM state
    localparam IDLE       = 3'd0; // Tile configuration 대기
    localparam BIAS_REQ   = 3'd1; // 필요한 Bias read request 발생
    localparam BIAS_WAIT  = 3'd2; // Bias response 대기
    localparam READY      = 3'd3; // Bias 준비 완료, FIFO 입력 대기
    localparam PROCESS    = 3'd4; // lane0~2 순차 post-process
    localparam OUT_WAIT   = 3'd5; // Normal INT8 output 수락 대기
    localparam FINAL_WAIT = 3'd6; // Final output 수락 대기

    // =========================================================
    // Handshake Wires
    // =========================================================

    // output_fifo → post_process 실제 입력 수락
    wire input_fire;

    // post_process → pooling_unit 실제 출력 전달
    wire output_fire;

    // post_process → result_buf Final 결과 실제 전달
    wire final_fire;

    // param_buf Bias read request 실제 수락
    wire bias_req_fire;

    // param_buf Bias read response 실제 수락
    wire bias_rsp_fire;

    // Tile configuration을 실제로 수락하는 조건
    // IDLE 또는 READY 상태에서 params가 모두 적재된 경우에만 수락
    wire cfg_fire;

    // Tile configuration의 channel mask 유효성 확인
    // Normal Layer : 001 / 011 / 111
    // Final FC2    : 001만 허용
    wire cfg_mask_valid;

    assign cfg_mask_valid = i_is_final_layer ? (i_col_mask == 3'b001) : 
            ((i_col_mask == 3'b001) ||
             (i_col_mask == 3'b011) ||
             (i_col_mask == 3'b111));

    // 현재 Tile에서 사용하게 될 마지막 Bias 주소
    wire [6:0] cfg_bias_last_addr;

    // Bias address가 전체 Bias parameter 범위 0~42 안인지 확인
    wire cfg_bias_addr_valid;

    // Final FC2 configuration 유효성 확인
    // FC2 : output channel 1개, bias addr 42, ReLU 미사용
    wire cfg_final_valid;

    assign cfg_final_valid = !i_is_final_layer || (
            (i_out_ch_base == 5'd0) &&
            (i_bias_base   == 6'd42) &&
            (i_relu_en     == 1'b0));

    assign cfg_bias_last_addr = {1'b0, i_bias_base} + {2'b00, i_out_ch_base} + 
        ((i_col_mask == 3'b111) ? 7'd2 : (i_col_mask == 3'b011) ? 7'd1 : 7'd0);

    assign cfg_bias_addr_valid = (cfg_bias_last_addr <= 7'd42);

    // 입력 beat의 keep가 현재 Tile 설정과 일치하는지 확인
    // 정상 keep는 001 / 011 / 111만 허용
    wire input_keep_valid;

    // Final FC2 입력 형식 확인
    // Final Layer는 lane0 하나만 사용하며
    // 마지막 Tile이자 마지막 Layer 결과여야 함
    wire final_input_valid;

    // 입력 beat의 metadata가 현재 Tile 설정과 일치하는지 확인
    wire input_meta_valid;

    assign input_meta_valid = (i_meta[16:12] == out_ch_base_reg) && (!i_meta[18] || i_meta[17]);

    assign final_input_valid = !is_final_layer_reg || ((i_keep    == 3'b001) && (i_meta[17] == 1'b1) && (i_meta[18] == 1'b1));
    assign input_keep_valid = ((i_keep == 3'b001) || (i_keep == 3'b011) || (i_keep == 3'b111)) && (i_keep == col_mask_reg);
    
    
    assign cfg_fire = i_cfg_valid && o_cfg_ready;
    assign o_cfg_ready =
        i_params_loaded &&
        cfg_mask_valid &&
        cfg_bias_addr_valid &&
        cfg_final_valid &&
        cfg_change_allowed &&
        ((state == IDLE) || (state == READY));

    assign input_fire    = i_valid          && o_ready;
    assign output_fire   = o_valid          && i_ready;
    assign final_fire    = o_final_valid    && i_final_ready;
    assign bias_req_fire = o_bias_req_valid && i_bias_req_ready;
    assign bias_rsp_fire = i_bias_rsp_valid && o_bias_rsp_ready;

    // 이번 응답이 이 타일의 마지막 lane 이다
    wire bias_rsp_last = bias_rsp_fire && (bias_rsp_idx == bias_n_lanes - 2'd1);

    // =========================================================
    // Lane 넣기 / 완료 판정
    // =========================================================

    // 이 beat 에서 마지막으로 넣을 lane (keep 은 001 / 011 / 111 로 lane0 부터 연속)
    wire [1:0] last_lane_idx = keep_reg[2] ? 2'd2 : (keep_reg[1] ? 2'd1 : 2'd0);

    // s0 에 마지막 lane 을 넣는 클럭
    wire       issue_last    = (state == PROCESS) && !issue_done && (lane_idx == last_lane_idx);

    // 마지막 lane 의 결과가 result_lane 에 써지는 클럭. 다음 클럭에 OUT_WAIT / FINAL_WAIT
    wire       process_done  = (state == PROCESS) && s3_last;

    // =========================================================
    // FSM State Register
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // =========================================================
    // FSM Next State Logic
    // =========================================================
    always @(*) begin
        next_state = state;

        case (state)
            // -------------------------------------------------
            // Tile configuration 대기
            // -------------------------------------------------
            IDLE: begin
                if (cfg_fire)
                    next_state = cfg_hit ? READY : BIAS_REQ;   // 캐시 hit 이면 바로 READY
            end


            // -------------------------------------------------
            // param_buf에 현재 lane Bias 요청
            // -------------------------------------------------
            BIAS_REQ: begin
                // lane 0..n-1 요청을 연속으로 내고, 마지막 응답이 오면 READY
                if (bias_rsp_last)
                    next_state = READY;
            end


            // -------------------------------------------------
            // Bias 응답 대기
            // -------------------------------------------------
            BIAS_WAIT: begin
                // 쓰지 않는다 (연속 읽기로 바꾸면서 BIAS_REQ 에 합쳤다)
                next_state = READY;
            end


            // -------------------------------------------------
            // output_fifo 입력 대기
            // -------------------------------------------------
            READY: begin
                if (cfg_fire) begin
                    next_state = cfg_hit ? READY : BIAS_REQ;
                end
                else if (input_fire) begin
                    next_state = PROCESS;
                end
            end


            // -------------------------------------------------
            // 현재 beat의 lane 순차 처리
            // -------------------------------------------------
            PROCESS: begin
                if (process_done) begin
                    if (is_final_layer_reg)
                        next_state = FINAL_WAIT;
                    else
                        next_state = OUT_WAIT;
                end
            end


            // -------------------------------------------------
            // Normal output이 pooling_unit에 수락되기를 대기
            // -------------------------------------------------
            OUT_WAIT: begin
                if (output_fire)
                    next_state = READY;
            end


            // -------------------------------------------------
            // Final angle이 result_buf에 수락되기를 대기
            // -------------------------------------------------
            FINAL_WAIT: begin
                if (final_fire)
                    next_state = READY;
            end


            default: begin
                next_state = IDLE;
            end

        endcase
    end


    // =========================================================
    // FSM Output / Control Logic
    // =========================================================
    always @(*) begin
        // -----------------------------------------------------
        // 기본값
        // -----------------------------------------------------
        o_params_ready   = 1'b0;

        o_bias_req_valid = 1'b0;
        o_bias_addr      = 6'd0;
        o_bias_rsp_ready = 1'b0;

        o_ready          = 1'b0;

        // 데이터 출력은 state 로 게이팅하지 않고 레지스터를 그대로 낸다 (타이밍 2026-09-23 :
        // state -> o_meta -> pooling 위치 계산 -> o_ready 가 한 경로였다). 유효 여부는 valid 만 본다
        o_data           = {result_lane2, result_lane1, result_lane0};
        o_valid          = 1'b0;
        o_keep           = keep_reg;
        o_meta           = meta_reg;
        o_final_data     = final_result_reg;
        o_final_valid    = 1'b0;


        case (state)

            IDLE: begin
                // cfg 대기
            end

            BIAS_REQ: begin
                // 남은 lane 이 있으면 요청, 응답은 항상 받는다
                o_bias_req_valid = (bias_req_idx < bias_n_lanes);
                o_bias_addr      = bias_base_reg + out_ch_base_reg + bias_req_idx;
                o_bias_rsp_ready = 1'b1;
            end

            BIAS_WAIT: begin
            end

            READY: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;
                // cfg 수락 cycle은 입력 금지
                // valid beat가 들어온 경우 keep도 정상이어야 수락
                o_ready = !cfg_change_allowed && !cfg_fire && (!i_valid || (input_keep_valid && input_meta_valid && final_input_valid));
            end

            PROCESS: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;

                // lane 연산 중이므로 새로운 입력은 받지 않음
            end

            OUT_WAIT: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;
                // 처리 완료된 3개의 INT8 lane을 24-bit로 출력
                o_data  = {result_lane2, result_lane1, result_lane0};
                // 현재 beat에서 실제 유효했던 lane 정보 전달
                o_keep  = keep_reg;
                // position / channel / tile_end / layer_end 전달
                o_meta  = meta_reg;
                // pooling_unit에 출력 유효 표시
                o_valid = 1'b1;
            end

            FINAL_WAIT: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;

                // Final FC2 결과는 signed INT8 각도값
                o_final_data  = final_result_reg;
                o_final_valid = 1'b1;
            end

            default: begin
            end

        endcase
    end

    // =========================================================
    // Configuration Register Update
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_mask_reg         <= 3'b000;
            out_ch_base_reg      <= 5'd0;
            bias_base_reg        <= 6'd0;
            relu_en_reg          <= 1'b0;
            is_final_layer_reg   <= 1'b0;

            quant_multiplier_reg <= 32'd0;
            quant_shift_reg      <= 6'd0;

            angle_multiplier_reg <= 32'd0;
            angle_shift_reg      <= 6'd0;
        end
        else if (cfg_fire) begin
            col_mask_reg         <= i_col_mask;
            out_ch_base_reg      <= i_out_ch_base;
            bias_base_reg        <= i_bias_base;
            relu_en_reg          <= i_relu_en;
            is_final_layer_reg   <= i_is_final_layer;

            quant_multiplier_reg <= i_quant_multiplier;
            quant_shift_reg      <= i_quant_shift;

            angle_multiplier_reg <= i_angle_multiplier;
            angle_shift_reg      <= i_angle_shift;
        end
    end

    // =========================================================
    // Tile Configuration Lock
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // reset 후 첫 Tile configuration 허용
            cfg_change_allowed <= 1'b1;
        end
        else if (cfg_fire) begin
            // 현재 Tile configuration을 받았으므로
            // Tile이 끝날 때까지 새로운 cfg 금지
            cfg_change_allowed <= 1'b0;
        end
        else if (output_fire && meta_reg[17]) begin
            // Normal path의 마지막 row가 실제 수락되면
            // 다음 Tile configuration 허용
            cfg_change_allowed <= 1'b1;
        end
        else if (final_fire) begin
            // Final FC2 결과가 실제 수락되면
            // 현재 Tile 종료
            cfg_change_allowed <= 1'b1;
        end
    end

    // =========================================================
    // Bias 읽기 / 캐시 갱신
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bias_cache0   <= 32'd0;
            bias_cache1   <= 32'd0;
            bias_cache2   <= 32'd0;
            bias_req_idx  <= 2'd0;
            bias_rsp_idx  <= 2'd0;
            bias_n_lanes  <= 2'd1;
            bc_valid      <= 2'b00;
            bc_key0       <= 14'd0;  bc_key1 <= 14'd0;
            bc0_b0 <= 32'd0; bc0_b1 <= 32'd0; bc0_b2 <= 32'd0;
            bc1_b0 <= 32'd0; bc1_b1 <= 32'd0; bc1_b2 <= 32'd0;
            bc_next       <= 1'b0;
        end
        else begin
            // param_buf 가 다시 적재되면 (새 추론) 캐시를 비운다
            if (!i_params_loaded) bc_valid <= 2'b00;

            if (cfg_fire) begin
                bias_req_idx <= 2'd0;
                bias_rsp_idx <= 2'd0;
                bias_n_lanes <= i_col_mask[2] ? 2'd3 : (i_col_mask[1] ? 2'd2 : 2'd1);
                if (cfg_hit0) begin
                    bias_cache0 <= bc0_b0;  bias_cache1 <= bc0_b1;  bias_cache2 <= bc0_b2;
                end
                else if (cfg_hit1) begin
                    bias_cache0 <= bc1_b0;  bias_cache1 <= bc1_b1;  bias_cache2 <= bc1_b2;
                end
                else begin
                    bias_cache0 <= 32'd0;   bias_cache1 <= 32'd0;   bias_cache2 <= 32'd0;
                end
            end
            else begin
                if (bias_req_fire) bias_req_idx <= bias_req_idx + 2'd1;
                if (bias_rsp_fire) begin
                    bias_rsp_idx <= bias_rsp_idx + 2'd1;
                    case (bias_rsp_idx)
                        2'd0:    bias_cache0 <= i_bias_data;
                        2'd1:    bias_cache1 <= i_bias_data;
                        default: bias_cache2 <= i_bias_data;
                    endcase
                end
                // 마지막 응답 : 이 타일의 bias 세 개를 캐시에 넣는다
                if (bias_rsp_last && i_params_loaded) begin
                    if (!bc_next) begin
                        bc_key0 <= {bias_base_reg, out_ch_base_reg, col_mask_reg};
                        bc0_b0  <= (bias_rsp_idx == 2'd0) ? i_bias_data : bias_cache0;
                        bc0_b1  <= (bias_rsp_idx == 2'd1) ? i_bias_data : bias_cache1;
                        bc0_b2  <= (bias_rsp_idx == 2'd2) ? i_bias_data : bias_cache2;
                        bc_valid[0] <= 1'b1;
                    end
                    else begin
                        bc_key1 <= {bias_base_reg, out_ch_base_reg, col_mask_reg};
                        bc1_b0  <= (bias_rsp_idx == 2'd0) ? i_bias_data : bias_cache0;
                        bc1_b1  <= (bias_rsp_idx == 2'd1) ? i_bias_data : bias_cache1;
                        bc1_b2  <= (bias_rsp_idx == 2'd2) ? i_bias_data : bias_cache2;
                        bc_valid[1] <= 1'b1;
                    end
                    bc_next <= ~bc_next;
                end
            end
        end
    end


    // =========================================================
    // Input Beat Buffer Update
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_reg <= 96'd0;
            keep_reg <= 3'b000;
            meta_reg <= 19'd0;
        end
        else if (input_fire) begin
            // output_fifo에서 실제 handshake가 성립한 경우에만
            // 현재 96-bit PE result beat와 부가 정보를 저장
            data_reg <= i_data;
            keep_reg <= i_keep;
            meta_reg <= i_meta;
        end
    end

    // =========================================================
    // Lane 파이프라인 진행
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane_idx         <= 2'd0;
            issue_done       <= 1'b0;
            s1_valid <= 1'b0;  s2_valid <= 1'b0;  s3_valid <= 1'b0;
            s1_lane  <= 2'd0;  s2_lane  <= 2'd0;  s3_lane  <= 2'd0;
            s1_last  <= 1'b0;  s2_last  <= 1'b0;  s3_last  <= 1'b0;
            s1_acc32 <= 32'd0; s1_mult  <= 32'd0;
            s2_prod  <= 64'd0; s3_sum   <= 64'd0;
            result_lane0     <= 8'd0;
            result_lane1     <= 8'd0;
            result_lane2     <= 8'd0;
            final_result_reg <= 8'd0;
        end
        else begin
            // ---- 파이프라인은 매 클럭 전진한다 ----
            // s0 -> s1 : PROCESS 에서 현재 lane 을 넣는다 (무효 lane 은 valid=0 으로 지나간다)
            s1_valid <= (state == PROCESS) && !issue_done && current_lane_valid;
            s1_lane  <= lane_idx;
            s1_last  <= issue_last;
            s1_acc32 <= acc32_reg;
            s1_mult  <= current_multiplier;

            // s1 -> s2 : signed 32 x 32 곱 (DSP)
            s2_valid <= s1_valid;
            s2_lane  <= s1_lane;
            s2_last  <= s1_last;
            s2_prod  <= s1_acc32 * s1_mult;

            // s2 -> s3 : 반올림 오프셋 덧셈
            s3_valid <= s2_valid;
            s3_lane  <= s2_lane;
            s3_last  <= s2_last;
            s3_sum   <= s2_prod + round_off;

            // ---- s3 : shift + 포화 결과 저장 ----
            if (s3_valid) begin
                if (!is_final_layer_reg) begin
                    case (s3_lane)
                        2'd0:    result_lane0 <= normal_int8_result;
                        2'd1:    result_lane1 <= normal_int8_result;
                        2'd2:    result_lane2 <= normal_int8_result;
                        default: begin end
                    endcase
                end
                else if (s3_lane == 2'd0) begin
                    // Final FC2 는 lane0 하나
                    final_result_reg <= final_int8_result;
                end
            end

            // ---- lane 넣기 ----
            if (input_fire) begin
                // 새 beat : lane0 부터. 무효 lane 자리에 이전 결과가 남지 않도록 지운다
                lane_idx         <= 2'd0;
                issue_done       <= 1'b0;
                result_lane0     <= 8'd0;
                result_lane1     <= 8'd0;
                result_lane2     <= 8'd0;
                final_result_reg <= 8'd0;
            end
            else if ((state == PROCESS) && !issue_done) begin
                if (issue_last) issue_done <= 1'b1;
                else            lane_idx   <= lane_idx + 2'd1;
            end
        end
    end

endmodule

module pooling_unit (
    input wire clk,
    input wire rst_n,

    // =========================================================
    // Layer Configuration
    // =========================================================
    input wire       i_cfg_valid,
    input wire       i_pool_en,
    input wire [6:0] i_in_w,
    input wire [6:0] i_in_h,
    input wire [5:0] i_channels,

    // =========================================================
    // Input Stream from post_process
    // =========================================================
    input  wire [23:0] i_data,
    input  wire        i_valid,
    output wire        o_ready,

    input wire [ 2:0] i_keep,
    input wire [18:0] i_meta,

    // =========================================================
    // Output Stream to result_buf
    // =========================================================
    output wire [23:0] o_data,
    output wire        o_valid,
    input  wire        i_ready,

    output wire [ 2:0] o_keep,
    output wire [18:0] o_meta,

    // =========================================================
    // Tile Input Completion
    // =========================================================
    output reg o_tile_in_done
);

    // =========================================================
    // Layer Configuration Registers
    // =========================================================
    reg       pool_en_reg;
    reg [6:0] in_w_reg;
    reg [6:0] in_h_reg;
    reg [5:0] channels_reg;
    reg [2:0] in_w_sh_reg;     // log2(in_w).  in_w 는 2 의 거듭제곱 (64 / 32 / 16)
    reg [6:0] in_w_mask_reg;   // in_w - 1

    // 2 의 거듭제곱 (1 ~ 64) 의 log2. 그 밖의 값은 최상위 1 의 자리 (타이밍 : 나눗셈 대신 shift 용)
    function [2:0] f_log2_7;
        input [6:0] v;
        begin
            casez (v)
                7'b1??????: f_log2_7 = 3'd6;
                7'b01?????: f_log2_7 = 3'd5;
                7'b001????: f_log2_7 = 3'd4;
                7'b0001???: f_log2_7 = 3'd3;
                7'b00001??: f_log2_7 = 3'd2;
                7'b000001?: f_log2_7 = 3'd1;
                default:    f_log2_7 = 3'd0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pool_en_reg   <= 1'b0;
            in_w_reg      <= 7'd0;
            in_h_reg      <= 7'd0;
            channels_reg  <= 6'd0;
            in_w_sh_reg   <= 3'd0;
            in_w_mask_reg <= 7'd0;
        end else if (i_cfg_valid) begin
            pool_en_reg   <= i_pool_en;
            in_w_reg      <= i_in_w;
            in_h_reg      <= i_in_h;
            channels_reg  <= i_channels;
            in_w_sh_reg   <= f_log2_7(i_in_w);
            in_w_mask_reg <= i_in_w - 7'd1;
        end
    end

    // =========================================================
    // Input Metadata Decode
    // =========================================================

    // i_meta[11:0]  : input spatial position
    // i_meta[16:12] : output channel base
    // i_meta[17]    : tile_end
    // i_meta[18]    : layer_end
    wire [11:0] input_pos;
    wire [ 4:0] input_out_ch_base;
    wire        input_tile_end;
    wire        input_layer_end;

    assign input_pos         = i_meta[11:0];
    assign input_out_ch_base = i_meta[16:12];
    assign input_tile_end    = i_meta[17];
    assign input_layer_end   = i_meta[18];

    // 채널 그룹 선택. 1792 줄 always 블록보다 앞에 있어야 해서 여기로 올렸다 (사용 전 선언).
    // Current MaxPool layer has 6 output channels.
    //
    // group 0 : channel 0 ~ 2, out_ch_base = 0
    // group 1 : channel 3 ~ 5, out_ch_base = 3
    wire pool_group_sel;

    assign pool_group_sel = (input_out_ch_base == 5'd3);

    // =========================================================
    // Position Decode
    // =========================================================

    // pos = y * in_w + x
    wire [11:0] input_y_full;
    wire [11:0] input_x_full;

    // 나눗셈 대신 shift / mask (in_w 는 2 의 거듭제곱). 조합 나눗셈은 100 MHz 에서 -20 ns 였다 (2026-09-23)
    assign input_y_full = input_pos >> in_w_sh_reg;
    assign input_x_full = input_pos & {5'd0, in_w_mask_reg};

    wire [6:0] input_y;
    wire [6:0] input_x;

    assign input_y = input_y_full[6:0];
    assign input_x = input_x_full[6:0];

    // =========================================================
    // 2x2 Pool Window Position
    // =========================================================

    // q = 2 * (y % 2) + (x % 2)
    //
    // q=0 : top-left
    // q=1 : top-right
    // q=2 : bottom-left
    // q=3 : bottom-right
    wire [1:0] pool_q;

    assign pool_q = {input_y[0], input_x[0]};

    // Pool output position
    // pool_pos = (y/2) * (in_w/2) + (x/2)
    wire [11:0] pool_position;

    // (y/2) * (in_w/2) + (x/2)  =  (y/2) << (log2(in_w) - 1)  |  (x/2)
    wire [11:0] pool_y2 = {6'd0, input_y[6:1]};
    wire [11:0] pool_x2 = {6'd0, input_x[6:1]};
    assign pool_position = (in_w_sh_reg == 3'd0) ? 12'd0 : ((pool_y2 << (in_w_sh_reg - 3'd1)) | pool_x2);

    // 동일 channel group에서 하나의 2x2 window를 구분하는 ID.
    // 현재 구조에서는 pool output position과 동일하다.
    wire [11:0] pool_window_id;

    assign pool_window_id = pool_position;

    // =========================================================
    // Input / Output Handshake
    // =========================================================

    // result_buf로 전달할 출력 1개를 보관하는 register
    reg  [23:0] out_data_reg;
    reg         out_valid_reg;
    reg  [ 2:0] out_keep_reg;
    reg  [18:0] out_meta_reg;

    wire        input_fire;
    wire        output_fire;
    wire        pool_state_ready;

    // Output register가 비어 있거나,
    // 현재 output이 같은 cycle에 downstream으로 전달되면
    // 새로운 input을 받을 수 있다.
    assign o_ready =
        rst_n &&
        !i_cfg_valid &&
        (!out_valid_reg || i_ready) &&
        (!i_valid || pool_state_ready);

    assign input_fire = i_valid && o_ready;
    assign output_fire = out_valid_reg && i_ready;

    // Output stream
    assign o_data = out_data_reg;
    assign o_valid = out_valid_reg;
    assign o_keep = out_keep_reg;
    assign o_meta = out_meta_reg;

    // =========================================================
    // Input Lane Split
    // =========================================================

    wire signed [7:0] input_lane0;
    wire signed [7:0] input_lane1;
    wire signed [7:0] input_lane2;

    assign input_lane0 = $signed(i_data[7:0]);
    assign input_lane1 = $signed(i_data[15:8]);
    assign input_lane2 = $signed(i_data[23:16]);

    // =========================================================
    // Pool Channel Group Select
    // =========================================================



    // =========================================================
    // Group 0 Pool State : Channel 0 ~ 2
    // =========================================================

    // 현재 처리 중인 2x2 window가 존재하는지 표시
    reg               group0_window_valid;

    // 현재 그룹이 처리 중인 window ID
    reg        [11:0] group0_window_id;

    // 다음에 받아야 하는 q
    // 0 -> 1 -> 2 -> 3
    reg        [ 1:0] group0_next_q;

    // 각 channel lane의 현재 Max 값
    reg signed [ 7:0] group0_max0;
    reg signed [ 7:0] group0_max1;
    reg signed [ 7:0] group0_max2;
    reg [2:0] group0_keep_reg;

    // =========================================================
    // Group 1 Pool State : Channel 3 ~ 5
    // =========================================================

    reg               group1_window_valid;
    reg        [11:0] group1_window_id;
    reg        [ 1:0] group1_next_q;

    reg signed [ 7:0] group1_max0;
    reg signed [ 7:0] group1_max1;
    reg signed [ 7:0] group1_max2;
    reg [2:0] group1_keep_reg;

    // =========================================================
    // Final Max for q=3
    // =========================================================

    wire signed [7:0] group0_final_max0;
    wire signed [7:0] group0_final_max1;
    wire signed [7:0] group0_final_max2;

    wire signed [7:0] group1_final_max0;
    wire signed [7:0] group1_final_max1;
    wire signed [7:0] group1_final_max2;

    assign group0_final_max0 =
        (group0_keep_reg[0] && (input_lane0 > group0_max0)) ?
        input_lane0 : group0_max0;

    assign group0_final_max1 =
        (group0_keep_reg[1] && (input_lane1 > group0_max1)) ?
        input_lane1 : group0_max1;

    assign group0_final_max2 =
        (group0_keep_reg[2] && (input_lane2 > group0_max2)) ?
        input_lane2 : group0_max2;

    assign group1_final_max0 =
        (group1_keep_reg[0] && (input_lane0 > group1_max0)) ?
        input_lane0 : group1_max0;

    assign group1_final_max1 =
        (group1_keep_reg[1] && (input_lane1 > group1_max1)) ?
        input_lane1 : group1_max1;

    assign group1_final_max2 =
        (group1_keep_reg[2] && (input_lane2 > group1_max2)) ?
        input_lane2 : group1_max2;

    // =========================================================
    // Last Pool Output Detection
    // =========================================================

    wire [2:0] input_valid_lane_count;
    wire [6:0] input_group_end_ch;

    wire pool_last_window;
    wire pool_last_group;
    wire pool_output_layer_end;

    assign input_valid_lane_count =
        {2'd0, i_keep[0]} +
        {2'd0, i_keep[1]} +
        {2'd0, i_keep[2]};

    // exclusive end channel
    // ex) base=3, keep=111 -> 3+3=6
    assign input_group_end_ch =
        {2'd0, input_out_ch_base} +
        {4'd0, input_valid_lane_count};

    assign pool_last_window =
        (in_w_reg != 7'd0) &&
        (in_h_reg != 7'd0) &&
        (input_x == (in_w_reg - 7'd1)) &&
        (input_y == (in_h_reg - 7'd1));

    assign pool_last_group =
        (input_group_end_ch >= {1'b0, channels_reg});

    assign pool_output_layer_end =
        input_layer_end &&
        pool_last_window &&
        pool_last_group;

    // =========================================================
    // Input Validity Check
    // =========================================================
    wire input_keep_valid;
    wire input_position_valid;
    wire pool_group_valid;
    wire pool_cfg_valid;
    wire pool_layer_end_valid;
    // =========================================================
    // Pool Configuration Validity
    // =========================================================

    // 2x2 MaxPool 조건
    // - width / height는 최소 2
    // - width / height는 짝수
    // - channel 수는 1 이상, 최대 6
    assign pool_cfg_valid =
        (in_w_reg >= 7'd2) &&
        (in_h_reg >= 7'd2) &&
        (in_w_reg[0] == 1'b0) &&
        (in_h_reg[0] == 1'b0) &&
        ((in_w_reg & (in_w_reg - 7'd1)) == 7'd0) &&   // in_w 는 2 의 거듭제곱 (위치를 shift 로 계산)
        (channels_reg >= 6'd1) &&
        (channels_reg <= 6'd6);

    // Pool 경로에서 layer_end가 들어온 경우에는
    // 반드시 마지막 2x2 window의 q=3,
    // 그리고 마지막 channel group이어야 한다.
    assign pool_layer_end_valid =
        !input_layer_end ||
        (
            (pool_q == 2'd3) &&
            pool_last_window &&
            pool_last_group
        );

    assign input_position_valid =
        (in_w_reg != 7'd0) &&
        (in_h_reg != 7'd0) &&
        (input_y_full < {5'd0, in_h_reg}) &&
        (input_x_full < {5'd0, in_w_reg});

    assign input_keep_valid =
        (i_keep == 3'b001) ||
        (i_keep == 3'b011) ||
        (i_keep == 3'b111);

    // 현재 MaxPool Layer는 6채널이므로
    // channel group base는 0 또는 3만 허용
    assign pool_group_valid =
        (input_out_ch_base == 5'd0) ||
        (input_out_ch_base == 5'd3);

    // =========================================================
    // Pool Input Sequence Check
    // =========================================================

    // Bypass에서는 항상 입력 가능.
    //
    // MaxPool에서는 각 channel group마다
    // q=0 -> q=1 -> q=2 -> q=3 순서를 강제한다.
    //
    // q=0 : 해당 group에 진행 중인 window가 없어야 함
    // q=1~3 : 동일 window_id이고 next_q와 일치해야 함
    assign pool_state_ready =
        !pool_en_reg ?
        (
            input_keep_valid
        )
        :
        (
            input_keep_valid &&
            input_position_valid &&
            pool_group_valid &&
            pool_cfg_valid &&
            pool_layer_end_valid &&
            (
                !pool_group_sel ?
                (
                    (pool_q == 2'd0) ?
                        !group0_window_valid
                    :
                        (
                            group0_window_valid &&
                            (group0_window_id == pool_window_id) &&
                            (group0_next_q == pool_q) &&
                            (group0_keep_reg == i_keep)
                        )
                )
                :
                (
                    (pool_q == 2'd0) ?
                        !group1_window_valid
                    :
                        (
                            group1_window_valid &&
                            (group1_window_id == pool_window_id) &&
                            (group1_next_q == pool_q) &&
                            (group1_keep_reg == i_keep)
                        )
                )
            )
        );
        
    // (아래 always 는 group*_final_max / *_keep_reg / pool_output_layer_end 를 쓰므로 그 선언들 뒤로 옮겼다 : 사용 전 선언)
    // =========================================================
    // Output Register / Tile Input Done
    // =========================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_data_reg      <= 24'd0;
            out_valid_reg     <= 1'b0;
            out_keep_reg      <= 3'b000;
            out_meta_reg      <= 19'd0;

            o_tile_in_done    <= 1'b0;
        end
        else begin
            // pulse 기본값
            o_tile_in_done <= 1'b0;

            // 새 Layer 시작 시 pending output 제거
            if (i_cfg_valid) begin
                out_valid_reg <= 1'b0;
            end
            else begin

                // 현재 output이 downstream에 수락되면 제거
                if (output_fire) begin
                    out_valid_reg <= 1'b0;
                end

                // -------------------------------------------------
                // 새로운 input 수락
                // -------------------------------------------------
                if (input_fire) begin

                    // =============================================
                    // Bypass
                    // =============================================
                    if (!pool_en_reg) begin
                        out_data_reg  <= i_data;
                        out_keep_reg  <= i_keep;
                        out_meta_reg  <= i_meta;
                        out_valid_reg <= 1'b1;
                    end

                    // =============================================
                    // 2x2 MaxPool
                    // q=3에서만 output 생성
                    // =============================================
                    else if (pool_q == 2'd3) begin

                        if (!pool_group_sel) begin
                            out_data_reg <= {
                                group0_final_max2,
                                group0_final_max1,
                                group0_final_max0
                            };
                        
                            out_keep_reg <= group0_keep_reg;
                        end
                        else begin
                            out_data_reg <= {
                                group1_final_max2,
                                group1_final_max1,
                                group1_final_max0
                            };
                        
                            out_keep_reg <= group1_keep_reg;
                        end

                        // Pool output metadata
                        //
                        // [18] layer_end
                        // [17] tile_end = 항상 0
                        // [16:12] out_ch_base
                        // [11:0] pooled position
                        out_meta_reg <= {
                            pool_output_layer_end,
                            1'b0,
                            input_out_ch_base,
                            pool_position
                        };

                        out_valid_reg <= 1'b1;
                    end

                    // Tile의 마지막 input beat를 수락한 순간
                    if (input_tile_end) begin
                        o_tile_in_done <= 1'b1;
                    end
                end
            end
        end
    end
    

    // =========================================================
    // Pool State Update
    // =========================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            group0_window_valid <= 1'b0;
            group0_window_id    <= 12'd0;
            group0_next_q       <= 2'd0;

            group0_max0         <= 8'd0;
            group0_max1         <= 8'd0;
            group0_max2         <= 8'd0;

            group1_window_valid <= 1'b0;
            group1_window_id    <= 12'd0;
            group1_next_q       <= 2'd0;

            group1_max0         <= 8'd0;
            group1_max1         <= 8'd0;
            group1_max2         <= 8'd0;
            group0_keep_reg     <= 3'b000;
            group1_keep_reg     <= 3'b000;
        end
        else if (i_cfg_valid) begin
            // Layer 변경 시에만 Pool 상태 clear
            group0_window_valid <= 1'b0;
            group0_window_id    <= 12'd0;
            group0_next_q       <= 2'd0;

            group0_max0         <= 8'd0;
            group0_max1         <= 8'd0;
            group0_max2         <= 8'd0;

            group1_window_valid <= 1'b0;
            group1_window_id    <= 12'd0;
            group1_next_q       <= 2'd0;

            group1_max0         <= 8'd0;
            group1_max1         <= 8'd0;
            group1_max2         <= 8'd0;
            group0_keep_reg     <= 3'b000;
            group1_keep_reg     <= 3'b000;
        end
        else if (input_fire && pool_en_reg) begin

            // =================================================
            // Group 0 : Channel 0 ~ 2
            // =================================================
            if (!pool_group_sel) begin
                case (pool_q)

                    // q=0 : 실제 첫 입력값으로 Max 초기화
                    2'd0: begin
                        group0_window_valid <= 1'b1;
                        group0_window_id    <= pool_window_id;
                        group0_next_q       <= 2'd1;

                        group0_keep_reg <= i_keep;

                        group0_max0 <= input_lane0;
                        group0_max1 <= input_lane1;
                        group0_max2 <= input_lane2;
                    end

                    // q=1
                    2'd1: begin
                        if (group0_keep_reg[0] && (input_lane0 > group0_max0))
                            group0_max0 <= input_lane0;

                        if (group0_keep_reg[1] && (input_lane1 > group0_max1))
                            group0_max1 <= input_lane1;

                        if (group0_keep_reg[2] && (input_lane2 > group0_max2))
                            group0_max2 <= input_lane2;

                        group0_next_q <= 2'd2;
                    end

                    // q=2
                    2'd2: begin
                        if (group0_keep_reg[0] && (input_lane0 > group0_max0))
                            group0_max0 <= input_lane0;

                        if (group0_keep_reg[1] && (input_lane1 > group0_max1))
                            group0_max1 <= input_lane1;

                        if (group0_keep_reg[2] && (input_lane2 > group0_max2))
                            group0_max2 <= input_lane2;

                        group0_next_q <= 2'd3;
                    end

                    // q=3 :
                    // 마지막 비교 결과는 다음 단계에서
                    // output register에 직접 넣는다.
                    2'd3: begin
                        group0_window_valid <= 1'b0;
                        group0_next_q       <= 2'd0;
                    end

                    default: begin
                    end
                endcase
            end

            // =================================================
            // Group 1 : Channel 3 ~ 5
            // =================================================
            else begin
                case (pool_q)

                    2'd0: begin
                        group1_window_valid <= 1'b1;
                        group1_window_id    <= pool_window_id;
                        group1_next_q       <= 2'd1;

                        group1_keep_reg <= i_keep;

                        group1_max0 <= input_lane0;
                        group1_max1 <= input_lane1;
                        group1_max2 <= input_lane2;
                    end

                    2'd1: begin
                        if (group1_keep_reg[0] && (input_lane0 > group1_max0))
                            group1_max0 <= input_lane0;

                        if (group1_keep_reg[1] && (input_lane1 > group1_max1))
                            group1_max1 <= input_lane1;

                        if (group1_keep_reg[2] && (input_lane2 > group1_max2))
                            group1_max2 <= input_lane2;

                        group1_next_q <= 2'd2;
                    end

                    2'd2: begin
                        if (group1_keep_reg[0] && (input_lane0 > group1_max0))
                            group1_max0 <= input_lane0;

                        if (group1_keep_reg[1] && (input_lane1 > group1_max1))
                            group1_max1 <= input_lane1;

                        if (group1_keep_reg[2] && (input_lane2 > group1_max2))
                            group1_max2 <= input_lane2;

                        group1_next_q <= 2'd3;
                    end

                    2'd3: begin
                        group1_window_valid <= 1'b0;
                        group1_next_q       <= 2'd0;
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

endmodule

module result_buf (
    input  wire        clk,
    input  wire        rst_n,

    // ---- layer config ----
    input  wire        i_cfg_valid,         
    input  wire        i_is_final_layer,    
    input  wire [13:0] i_dst_base,          
    input  wire [ 5:0] i_dst_channels,      

    // ---- pooling_unit ----
    input  wire [23:0] i_data,            
    input  wire        i_valid,           
    output wire        o_ready,           
    input  wire [ 2:0] i_keep,            
    input  wire [18:0] i_meta,            
    input  wire        i_write_grant,     
    output wire        o_wr_en,           
    output wire [13:0] o_wr_addr,         
    output wire [23:0] o_wr_data,         
    output wire [ 2:0] o_wr_be,           

    // ---- post_process scalar ----
    input wire signed [7:0] i_final_data,  
    input  wire        i_final_valid,      
    output wire        o_final_ready,      
    output reg         o_final_tile_done,  

    // ---- Done / CSR ----
    output reg               o_layer_done,       
    input  wire              i_irq_en,           
    input  wire              i_irq_clear,        
    output reg signed [31:0] o_final_result,  
    output reg               o_done_status,      
    output wire              o_irq               
);

    // ---- cfg ----
    reg        is_final;
    reg [13:0] dst_base;
    reg [ 3:0] wpp;                         // ceil(C / 3) : C <= 32 -> 1 ~ 11
    reg        final_stored;                // 이 레이어의 scalar 를 저장했다
    reg        layer_done_sent;             // 이 레이어의 layer_done 을 냈다 (중복 금지, R472)

    function [3:0] ceil_div3;               // (C + 2) / 3, C 는 6bit
        input [5:0] c;
        begin ceil_div3 = ({2'b00, c} + 8'd2) / 8'd3; end
    endfunction

    // ---- 중간 레이어 쓰기 ----
    wire [11:0] m_pos  = i_meta[11:0];
    wire [ 4:0] m_ocb  = i_meta[16:12];
    wire        m_lend = i_meta[18];

    wire [15:0] pix_off = m_pos * {12'd0, wpp};                 // pos * ceil(C/3), 최대 4095 * 11
    wire [ 3:0] ch_off  = m_ocb / 5'd3;                         // out_ch_base 는 3 의 배수 -> 그룹 번호 0 ~ 10

    assign o_ready = rst_n && !i_cfg_valid && !is_final && !layer_done_sent && i_write_grant;         // 큐 없음 : grant 가 있으면 바로 쓴다
    assign o_wr_en   = i_valid && o_ready;
    assign o_wr_addr = dst_base + pix_off[13:0] + {10'd0, ch_off};
    assign o_wr_data = i_data;
    assign o_wr_be   = i_keep;

    wire wr_fire     = o_wr_en;
    wire lend_fire   = wr_fire && m_lend && !layer_done_sent;  

    assign o_final_ready = rst_n && !i_cfg_valid && is_final && !final_stored;
    wire   final_fire    = i_final_valid && o_final_ready;

    assign o_irq = i_irq_en && o_done_status;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_final          <= 1'b0;
            dst_base          <= 14'd0;
            wpp               <= 4'd1;
            final_stored      <= 1'b0;
            layer_done_sent   <= 1'b0;
            o_final_tile_done <= 1'b0;
            o_layer_done      <= 1'b0;
            o_final_result    <= 32'd0;
            o_done_status     <= 1'b0;
        end else begin
            o_final_tile_done <= 1'b0;
            o_layer_done      <= 1'b0;

            if (i_cfg_valid) begin
                is_final        <= i_is_final_layer;
                dst_base        <= i_dst_base;
                wpp             <= ceil_div3(i_dst_channels);
                final_stored    <= 1'b0;
                layer_done_sent <= 1'b0;
            end

            // Layer Change : next clk = layer_done High-state
            if (lend_fire) begin
                o_layer_done    <= 1'b1;
                layer_done_sent <= 1'b1;
            end

            // Final : angle store
            if (final_fire) begin
                o_final_result    <= {{24{i_final_data[7]}}, i_final_data};
                final_stored      <= 1'b1;
                layer_done_sent   <= 1'b1;
                o_final_tile_done <= 1'b1;
                o_layer_done      <= 1'b1;
                o_done_status     <= 1'b1;
            end else if (i_irq_clear) begin
                o_done_status     <= 1'b0;
            end
        end
    end

endmodule