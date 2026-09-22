`timescale 1ns / 1ps

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
    reg [7:0] data_d1_1, data_d1_2, data_d2_2;

    wire [23:0] feed_data;

    assign o_ready   = rst_n && !i_clear && i_step_en && i_feed_valid;

    assign feed_data = (i_feed_valid && i_valid) ? i_data : 24'd0;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            data_d1_1 <= 8'd0;
            data_d1_2 <= 8'd0;
            data_d2_2 <= 8'd0;
        end else begin
            if (i_clear) begin
                data_d1_1 <= 8'd0;
                data_d1_2 <= 8'd0;
                data_d2_2 <= 8'd0;
            end else begin
                if (i_step_en) begin
                    data_d1_1 <= feed_data[15:8];
                    data_d1_2 <= feed_data[23:16];
                    data_d2_2 <= data_d1_2;
                end
            end
        end
    end

    assign o_data = (!rst_n || i_clear) ? 24'd0 :
        {data_d2_2, data_d1_1, feed_data[7:0]};

endmodule