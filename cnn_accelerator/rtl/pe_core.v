`timescale 1ns / 1ps

module pe_core (
    input wire clk,
    input wire rst_n,

    input wire [23:0] i_act_data,
    input wire        i_act_valid,

    output wire o_act_ready,

    input wire [23:0] i_wgt_data,
    input wire        i_wgt_valid,

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

/*
============================================
SINGLE PE
============================================
*/
module single_pe (
    input wire clk,
    input wire rst_n,
    input wire i_step_en,
    input wire i_acc_clear,
    input wire i_mac_valid,
    input wire i_mac_last,

    input wire signed [7:0] act_in,
    input wire signed [7:0] weight_in,

    output reg signed [ 7:0] act_out,
    output reg signed [ 7:0] weight_out,
    output reg signed [31:0] result_data,
    output reg               o_result_valid
);

    reg signed [31:0] acc_reg;

    reg signed [15:0] product;
    reg signed [31:0] product_ext;
    reg signed [31:0] acc_next;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            acc_reg        <= 32'd0;
            result_data    <= 32'd0;
            o_result_valid <= 1'b0;
            act_out        <= 8'd0;
            weight_out     <= 8'd0;
        end else begin
            if (i_acc_clear) begin
                acc_reg        <= 32'd0;
                result_data    <= 32'd0;
                o_result_valid <= 1'b0;
            end else begin
                o_result_valid <= 1'b0;  // 1 pulse

                if (i_step_en) begin
                    act_out    <= act_in;
                    weight_out <= weight_in;
                    // i_step_en = 0 : pe stop
                    if (i_mac_valid) begin
                        acc_reg <= acc_next;
                        if (i_mac_last) begin
                            result_data <= acc_next;
                            o_result_valid <= 1'b1;
                        end
                    end
                end
            end
        end
    end

    always @(*) begin
        product     = $signed(act_in) * $signed(weight_in);
        product_ext = {{16{product[15]}}, product};
        acc_next    = acc_reg + product_ext;
    end
endmodule

/*
============================================
PE ARRAY
============================================
*/
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
    // Activation Connect
    wire signed [7:0] act_00_01;
    wire signed [7:0] act_01_02;

    wire signed [7:0] act_10_11;
    wire signed [7:0] act_11_12;

    wire signed [7:0] act_20_21;
    wire signed [7:0] act_21_22;

    // Weight Connect
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
        .result_data   (o_result_data[31:0]),
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
        .result_data   (o_result_data[63:32]),
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
        .result_data   (o_result_data[95:64]),
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
        .result_data   (o_result_data[127:96]),
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
        .result_data   (o_result_data[159:128]),
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
        .result_data   (o_result_data[191:160]),
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
        .result_data   (o_result_data[223:192]),
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
        .result_data   (o_result_data[255:224]),
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
        .result_data   (o_result_data[287:256]),
        .o_result_valid(o_result_valid[8])
    );
endmodule

/*
============================================
ACT SKEW
============================================
*/
module act_skew (
    input wire clk,
    input wire rst_n,

    input wire        i_clear,
    input wire [23:0] i_data,
    input wire        i_valid,

    output wire o_ready,

    input wire i_step_en,
    input wire i_feed_valid,

    output wire [23:0] o_data
);
    // lane0 1 단, lane1 2 단, lane2 3 단.  pe_cntl 의 mac_valid (valid_pipe[r+c], 레지스터) 가 inject 다음 step 부터
    // 유효하므로 PE(r,c) 는 beat k 를 step k+1+r+c 에 봐야 한다 (2026-09-22 수정 : 원래는 lane0 이 조합 통과라 beat 0 이 빠졌다)
    reg [7:0] data_d0_1, data_d1_1, data_d1_2, data_d2_1, data_d2_2, data_d2_3;

    wire [23:0] feed_data;

    assign o_ready   = rst_n && !i_clear && i_step_en && i_feed_valid;

    assign feed_data = (i_feed_valid && i_valid) ? i_data : 24'd0;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            data_d0_1 <= 8'd0;
            data_d1_1 <= 8'd0;
            data_d1_2 <= 8'd0;
            data_d2_1 <= 8'd0;
            data_d2_2 <= 8'd0;
            data_d2_3 <= 8'd0;
        end else begin
            if (i_clear) begin
                data_d0_1 <= 8'd0;
                data_d1_1 <= 8'd0;
                data_d1_2 <= 8'd0;
                data_d2_1 <= 8'd0;
                data_d2_2 <= 8'd0;
                data_d2_3 <= 8'd0;
            end else begin
                if (i_step_en) begin
                    data_d0_1 <= feed_data[7:0];  // lane0 : 1 단
                    data_d1_1 <= feed_data[15:8];
                    data_d1_2 <= data_d1_1;  // lane1 : 2 단
                    data_d2_1 <= feed_data[23:16];
                    data_d2_2 <= data_d2_1;
                    data_d2_3 <= data_d2_2;  // lane2 : 3 단
                end
            end
        end
    end

    assign o_data = (!rst_n || i_clear) ? 24'd0 :
        {data_d2_3, data_d1_2, data_d0_1};
endmodule

/*
============================================
WGT SKEW
============================================
*/
module wgt_skew (
    input wire clk,
    input wire rst_n,

    input wire        i_clear,
    input wire [23:0] i_data,
    input wire        i_valid,

    output wire o_ready,

    input wire i_step_en,
    input wire i_feed_valid,

    output wire [23:0] o_data
);
    // lane0 1 단, lane1 2 단, lane2 3 단.  pe_cntl 의 mac_valid (valid_pipe[r+c], 레지스터) 가 inject 다음 step 부터
    // 유효하므로 PE(r,c) 는 beat k 를 step k+1+r+c 에 봐야 한다 (2026-09-22 수정 : 원래는 lane0 이 조합 통과라 beat 0 이 빠졌다)
    reg [7:0] data_d0_1, data_d1_1, data_d1_2, data_d2_1, data_d2_2, data_d2_3;

    wire [23:0] feed_data;

    assign o_ready   = rst_n && !i_clear && i_step_en && i_feed_valid;

    assign feed_data = (i_feed_valid && i_valid) ? i_data : 24'd0;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            data_d0_1 <= 8'd0;
            data_d1_1 <= 8'd0;
            data_d1_2 <= 8'd0;
            data_d2_1 <= 8'd0;
            data_d2_2 <= 8'd0;
            data_d2_3 <= 8'd0;
        end else begin
            if (i_clear) begin
                data_d0_1 <= 8'd0;
                data_d1_1 <= 8'd0;
                data_d1_2 <= 8'd0;
                data_d2_1 <= 8'd0;
                data_d2_2 <= 8'd0;
                data_d2_3 <= 8'd0;
            end else begin
                if (i_step_en) begin
                    data_d0_1 <= feed_data[7:0];  // lane0 : 1 단
                    data_d1_1 <= feed_data[15:8];
                    data_d1_2 <= data_d1_1;  // lane1 : 2 단
                    data_d2_1 <= feed_data[23:16];
                    data_d2_2 <= data_d2_1;
                    data_d2_3 <= data_d2_2;  // lane2 : 3 단
                end
            end
        end
    end

    assign o_data = (!rst_n || i_clear) ? 24'd0 : {data_d2_3, data_d1_2, data_d0_1};
endmodule
