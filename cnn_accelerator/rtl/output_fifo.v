`timescale 1ns / 1ps

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

    assign current_position = is_fc_reg ? 12'd0 : patch_base_reg + out_row;
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
