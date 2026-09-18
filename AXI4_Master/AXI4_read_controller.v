`timescale 1ns / 1ps

module AXI4_read_controller #(
    parameter ADDR_WIDTH = 15
)(
    input                           clk,
    input                           rst_n,

    // DMA Register Map
    //   ERROR register (0x1C) is done by the register map, which owns the
    //   layout : [29:15] read error address, [14:0] write error address.
    input                           start,
    output reg                      busy,
    output reg                      done,
    output reg                      error,
    output reg  [ADDR_WIDTH-1:0]    error_addr,

    // Datapath
    input                           fifo_full,
    input                           r_hs,
    input                           xfer_done,
    input                           err_valid,  // RRESP error, already qualified by the datapath
    input                           cfg_err,    // alignment / burst configuration fault
    input       [ADDR_WIDTH-1:0]    err_addr,   // base address of the failing burst

    // Datapath control signal
    output                          en,         // state == S_DATA
    output reg                      init        // IDLE -> DATA start 1 pulse
);

    // State
    localparam S_IDLE = 1'b0;
    localparam S_DATA = 1'b1;

    reg state;

    assign en = (state == S_DATA);

    // state register
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            init  <= 1'b0;
        end
        else begin
            init <= 1'b0; // 1 pulse
            case (state)
                S_IDLE:
                    if (!fifo_full && start) begin  // remaining state & start signal
                        state <= S_DATA;            // state transition
                        init  <= 1'b1;              // initialize
                    end
                S_DATA:
                    if (xfer_done) state <= S_IDLE; // transfer done -> state transition
                default: state <= S_IDLE;
            endcase
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            error_addr <= {ADDR_WIDTH{1'b0}};
        end
        else if (state == S_IDLE) begin
            if (!fifo_full && start) begin  // transfer start -> signal setting
                busy  <= 1'b1;
                done  <= 1'b0;
                error <= 1'b0;
            end
        end
        else begin
            // The datapath owns error detection : it checks RRESP[1] and
            // registers err_addr, so sampling err_valid here one cycle later
            // reads a stable address.
            // DONE and ERROR are independent. DONE = 1 with ERROR = 1 means
            // the transfer finished but the destination is not trustworthy.
            if ((err_valid || cfg_err) && !error) begin
                error      <= 1'b1;
                error_addr <= err_addr;      // no shifting here, register map places it
            end
            if (xfer_done) begin             // transfer finished
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

endmodule