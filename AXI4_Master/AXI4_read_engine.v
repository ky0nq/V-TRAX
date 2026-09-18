`timescale 1ns / 1ps

module AXI4_read_engine #(
    parameter ADDR_WIDTH      = 15,
    parameter DATA_WIDTH      = 32,
    parameter LEN_WIDTH       = 32,
    parameter BURST_WIDTH     = 8,

    parameter [ADDR_WIDTH-1:0] ROM_BASE    = 15'h0000,
    parameter [ADDR_WIDTH-1:0] ROM_SIZE    = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_BASE  = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_SIZE  = 15'h4000,
    parameter [ADDR_WIDTH-1:0] RAM_BASE    = 15'h4400,
    parameter [ADDR_WIDTH-1:0] RAM_SIZE    = 15'h0400,

    parameter MAX_BURST_BYTES = 16       // 16B = 4 beats
)(
    input                           clk,
    input                           rst_n,

    input                           start,
    input                           abort,      // CTRL.ABORT
    input       [ADDR_WIDTH-1:0]    src_addr,
    input       [LEN_WIDTH-1:0]     length,
    input       [BURST_WIDTH+1:0]   burst_cfg,  // [7:0] ARLEN, [9:8] burst type
    output                          busy,
    output                          done,
    output                          error,
    output      [ADDR_WIDTH-1:0]    error_addr,

    output                          fifo_wr_en,
    output      [DATA_WIDTH-1:0]    fifo_wr_data,
    input                           fifo_full,

    // AXI4-Full AR/R channel
    output      [3:0]               arid,
    output      [ADDR_WIDTH-1:0]    araddr,
    output      [BURST_WIDTH-1:0]   arlen,
    output      [2:0]               arsize,
    output      [1:0]               arburst,
    output                          arvalid,
    input                           arready,
    input       [DATA_WIDTH-1:0]    rdata,
    input                           rvalid,
    input                           rlast,
    input       [3:0]               rid,
    input       [1:0]               rresp,
    output                          rready
);

    wire                  en, init, r_hs, xfer_done;
    wire [ADDR_WIDTH-1:0] err_addr_w;
    wire                  err_valid_w, cfg_err_w;

    AXI4_read_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) U_READ_CNTL (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .busy       (busy),
        .done       (done),
        .error      (error),
        .error_addr (error_addr),
        .fifo_full  (fifo_full),
        .r_hs       (r_hs),
        .xfer_done  (xfer_done),
        .err_valid  (err_valid_w), 
        .cfg_err    (cfg_err_w),
        .err_addr   (err_addr_w),
        .en         (en),
        .init       (init)
    );

    AXI4_read_datapath #(
        .ADDR_WIDTH      (ADDR_WIDTH),
        .DATA_WIDTH      (DATA_WIDTH),
        .LEN_WIDTH       (LEN_WIDTH),
        .BURST_WIDTH     (BURST_WIDTH),
        .ROM_BASE        (ROM_BASE),
        .ROM_SIZE        (ROM_SIZE),
        .DDR3L_BASE      (DDR3L_BASE),
        .DDR3L_SIZE      (DDR3L_SIZE),
        .RAM_BASE        (RAM_BASE),
        .RAM_SIZE        (RAM_SIZE),
        .MAX_BURST_BYTES (MAX_BURST_BYTES)
    ) U_READ_DATAPATH (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .init         (init),
        .abort        (abort),
        .src_addr     (src_addr),
        .length       (length),
        .burst_cfg    (burst_cfg),
        .r_hs         (r_hs),
        .xfer_done    (xfer_done),
        .err_addr     (err_addr_w),
        .err_valid    (err_valid_w),
        .cfg_err      (cfg_err_w),
        .fifo_wr_en   (fifo_wr_en),
        .fifo_wr_data (fifo_wr_data),
        .fifo_full    (fifo_full),
        .arid         (arid),
        .araddr       (araddr),
        .arlen        (arlen),
        .arsize       (arsize),
        .arburst      (arburst),
        .arvalid      (arvalid),
        .arready      (arready),
        .rdata        (rdata),
        .rvalid       (rvalid),
        .rid          (rid),
        .rlast        (rlast),
        .rresp        (rresp),
        .rready       (rready)
    );

endmodule