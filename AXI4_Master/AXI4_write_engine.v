//=====================================================================
// axi4_write_engine.v
//
// axi4_write_controller(제어)와 axi4_write_datapath(데이터패스)를
// 인스턴스화해서 연결하는 최상위 wrapper.
//
// ★ 이번 수정 사항 ★
// 1) burst_cfg 포트 폭 치명적 버그 수정
//    기존: [BURST_WIDTH-1:0]  (8bit) <- burst_type(상위 2bit)가 통째로 0으로
//          잘려서 항상 FIXED(미구현)로 읽히던 버그. INCR이 영원히 안 걸림.
//    수정: [BURST_WIDTH+1:0]  (10bit, datapath가 기대하는 폭과 일치)
// 2) 3슬레이브(ROM/DDR3L/RAM) 주소맵 파라미터 추가 + pass-through
// 3) region_err 신호를 Controller까지 연결
//=====================================================================
module AXI4_write_engine #(
    parameter ADDR_WIDTH  = 15,
    parameter DATA_WIDTH  = 32,
    parameter LEN_WIDTH   = 32,
    parameter BURST_WIDTH = 8,

    // ===== 신규: 3슬레이브 주소맵 (Interconnect / Read Engine과 동일 값이어야 함) =====
    parameter [ADDR_WIDTH-1:0] ROM_BASE    = 15'h0000,
    parameter [ADDR_WIDTH-1:0] ROM_SIZE    = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_BASE  = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_SIZE  = 15'h4000,
    parameter [ADDR_WIDTH-1:0] RAM_BASE    = 15'h4400,
    parameter [ADDR_WIDTH-1:0] RAM_SIZE    = 15'h0400
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- Register Map I/F ----
    input  wire                    start,
    input  wire [ADDR_WIDTH-1:0]   dst_addr,
    input  wire [LEN_WIDTH-1:0]    length,
    input  wire [BURST_WIDTH+1:0]  burst_cfg,   // ★ 폭 수정: BURST_WIDTH-1:0 -> BURST_WIDTH+1:0
    output wire                    busy,
    output wire                    done,
    output wire                    error,
    output wire [ADDR_WIDTH-1:0]   error_addr,   // ★ 폭 수정: 32 -> ADDR_WIDTH

    // ---- FIFO I/F (consumer) ----
    output wire                    fifo_rd_en,
    input  wire [DATA_WIDTH-1:0]   fifo_rd_data,
    input  wire                    fifo_empty,

    // ---- AXI4 AW/W/B 채널 ----
    output wire [3:0]              awid,
    output wire [ADDR_WIDTH-1:0]   awaddr,
    output wire [BURST_WIDTH-1:0]  awlen,
    output wire [2:0]              awsize,
    output wire [1:0]              awburst,
    output wire                    awvalid,
    input  wire                    awready,

    output wire [DATA_WIDTH-1:0]   wdata,
    output wire [DATA_WIDTH/8-1:0] wstrb,
    output wire                    wlast,
    output wire                    wvalid,
    input  wire                    wready,

    input  wire [3:0]              bid,
    input  wire [1:0]              bresp,
    input  wire                    bvalid,
    output wire                    bready
);

    wire en, init, b_hs, xfer_done, region_err;

    AXI4_write_controller #(
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .busy       (busy),
        .done       (done),
        .error      (error),
        .error_addr (error_addr),
        .b_hs       (b_hs),
        .xfer_done  (xfer_done),
        .bresp      (bresp),
        .awaddr     (awaddr),
        .region_err (region_err),   // ★ 신규 연결
        .en         (en),
        .init       (init)
    );

    AXI4_write_datapath #(
        .ADDR_WIDTH  (ADDR_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .BURST_WIDTH (BURST_WIDTH),
        .ROM_BASE    (ROM_BASE),
        .ROM_SIZE    (ROM_SIZE),
        .DDR3L_BASE  (DDR3L_BASE),
        .DDR3L_SIZE  (DDR3L_SIZE),
        .RAM_BASE    (RAM_BASE),
        .RAM_SIZE    (RAM_SIZE)
    ) u_dp (
        .clk          (clk),
        .rst_n        (rst_n),
        .en           (en),
        .init         (init),
        .dst_addr     (dst_addr),
        .length       (length),
        .burst_cfg    (burst_cfg),
        .b_hs         (b_hs),
        .xfer_done    (xfer_done),
        .region_err   (region_err),   // ★ 신규 연결
        .fifo_rd_en   (fifo_rd_en),
        .fifo_rd_data (fifo_rd_data),
        .fifo_empty   (fifo_empty),
        .awid         (awid),
        .awaddr       (awaddr),
        .awlen        (awlen),
        .awsize       (awsize),
        .awburst      (awburst),
        .awvalid      (awvalid),
        .awready      (awready),
        .wdata        (wdata),
        .wstrb        (wstrb),
        .wlast        (wlast),
        .wvalid       (wvalid),
        .wready       (wready),
        .bid          (bid),
        .bresp        (bresp),
        .bvalid       (bvalid),
        .bready       (bready)
    );

endmodule