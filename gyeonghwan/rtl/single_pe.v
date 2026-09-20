`timescale 1ns / 1ps

module single_pe (
    input wire clk,
    input wire rst_n,
    input wire step_en,
    input wire acc_clear,
    input wire mac_valid,
    input wire mac_last,

    input wire signed [7:0] act_in,
    input wire signed [7:0] weight_in,

    output reg signed [ 7:0] act_out,
    output reg signed [ 7:0] weight_out,
    output reg signed [31:0] result_data,
    output reg               result_valid
);

    reg signed [31:0] acc_reg;

    reg signed [15:0] product;
    reg signed [31:0] product_ext;
    reg signed [31:0] acc_next;

    always @(posedge clk, negedge rst_n) begin
        if (!rst_n) begin
            acc_reg      <= 32'd0;
            result_data  <= 32'd0;
            result_valid <= 0;
            act_out      <= 8'd0;
            weight_out   <= 8'd0;
        end else begin
            if (acc_clear) begin
                acc_reg      <= 32'd0;
                result_data  <= 32'd0;
                result_valid <= 1'b0;
            end else begin
                result_valid <= 1'b0;  // 1 pulse

                if (step_en) begin
                    act_out    <= act_in;
                    weight_out <= weight_in;
                    // step_en = 0 : pe stop
                    if (mac_valid) begin
                        acc_reg <= acc_next;
                        if (mac_last) begin
                            result_data  <= acc_next;
                            result_valid <= 1'b1;
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