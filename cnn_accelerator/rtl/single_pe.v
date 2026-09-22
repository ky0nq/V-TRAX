`timescale 1ns / 1ps

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
