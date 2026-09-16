`timescale 1ns / 1ps


module tick_gen #(
    parameter COUNT = 1000
) (  //10,24,48Mhz
    input  wire clk,  // FPGA System Clock
    input  wire rst,
    output wire tick
);
    reg [$clog2(COUNT)-1:0] cnt;
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            cnt <= 0;
        end else begin
            if (cnt >= (COUNT - 1)) cnt <= 0;
            else cnt <= cnt + 1;
        end
    end
    assign tick = (cnt == (COUNT - 1)) ? 1 : 0;
endmodule

