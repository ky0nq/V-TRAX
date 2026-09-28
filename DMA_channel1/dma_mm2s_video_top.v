`timescale 1ns / 1ps
//
// DMA MM2S video top
//   = dma_mm2s_top (기존 그대로) + axis_frame_depacketizer
//
//   CPU --(S_AXI_LITE)--> dma_mm2s_top --(M_AXI)--> axi_mem_intercon (DDR / BRAM)
//                              |
//                              | 32bit packed bytes (내부 연결)
//                              v
//                     axis_frame_depacketizer
//                              |
//                              v
//                     M_AXIS_VIDEO (24bit RGB, 1 pixel/beat) --> v_axi4s_vid_out.video_in
//
//   출력 스트림 규약 (AXI4-Stream Video)
//     tuser : 프레임 첫 픽셀 (SOF)
//     tlast : 줄의 마지막 픽셀 (EOL)

module dma_mm2s_video_top #(
    parameter integer FIFO_DEPTH      = 64,
    parameter integer MAX_BURST_BYTES = 64,
    parameter integer PIXELS_PER_LINE = 1280,
    parameter integer FRAME_HEIGHT    = 720
)(
    input  wire         aclk,
    input  wire         aresetn,

    output wire         irq,
    input  wire [2:0]   s2mm_newest_idx,

    // ================= S_AXI_LITE (Register Map) =================
    input  wire [5:0]   s_axi_lite_awaddr,
    input  wire [2:0]   s_axi_lite_awprot,
    input  wire         s_axi_lite_awvalid,
    output wire         s_axi_lite_awready,
    input  wire [31:0]  s_axi_lite_wdata,
    input  wire [3:0]   s_axi_lite_wstrb,
    input  wire         s_axi_lite_wvalid,
    output wire         s_axi_lite_wready,
    output wire [1:0]   s_axi_lite_bresp,
    output wire         s_axi_lite_bvalid,
    input  wire         s_axi_lite_bready,
    input  wire [5:0]   s_axi_lite_araddr,
    input  wire [2:0]   s_axi_lite_arprot,
    input  wire         s_axi_lite_arvalid,
    output wire         s_axi_lite_arready,
    output wire [31:0]  s_axi_lite_rdata,
    output wire [1:0]   s_axi_lite_rresp,
    output wire         s_axi_lite_rvalid,
    input  wire         s_axi_lite_rready,

    // ================= M_AXI (AXI4-Full, read only) =================
    output wire [4:0]   m_axi_arid,
    output wire [31:0]  m_axi_araddr,
    output wire [7:0]   m_axi_arlen,
    output wire [2:0]   m_axi_arsize,
    output wire [1:0]   m_axi_arburst,
    output wire         m_axi_arlock,
    output wire [3:0]   m_axi_arcache,
    output wire [2:0]   m_axi_arprot,
    output wire [3:0]   m_axi_arqos,
    output wire         m_axi_arvalid,
    input  wire         m_axi_arready,
    input  wire [4:0]   m_axi_rid,
    input  wire [31:0]  m_axi_rdata,
    input  wire [1:0]   m_axi_rresp,
    input  wire         m_axi_rlast,
    input  wire         m_axi_rvalid,
    output wire         m_axi_rready,

    // ================= M_AXIS_VIDEO (24bit RGB pixel) =================
    output wire [23:0]  m_axis_video_tdata,
    output wire [2:0]   m_axis_video_tkeep,
    output wire         m_axis_video_tuser,
    output wire         m_axis_video_tlast,
    output wire         m_axis_video_tvalid,
    input  wire         m_axis_video_tready
);

    // ------------------------------------------------------------------
    // 내부 32bit packed 스트림 (dma_mm2s_top -> depacketizer)
    // ------------------------------------------------------------------
    wire [31:0] pk_tdata;
    wire [3:0]  pk_tkeep;
    wire        pk_tlast;
    wire        pk_tvalid;
    wire        pk_tready;

    // ------------------------------------------------------------------
    // DMA MM2S (기존 top 그대로)
    // ------------------------------------------------------------------
    dma_mm2s_top #(
        .FIFO_DEPTH      (FIFO_DEPTH),
        .MAX_BURST_BYTES (MAX_BURST_BYTES)
    ) U_DMA_MM2S (
        .aclk               (aclk),
        .aresetn            (aresetn),
        .irq                (irq),
        .s2mm_newest_idx    (s2mm_newest_idx),

        .s_axi_lite_awaddr  (s_axi_lite_awaddr),
        .s_axi_lite_awprot  (s_axi_lite_awprot),
        .s_axi_lite_awvalid (s_axi_lite_awvalid),
        .s_axi_lite_awready (s_axi_lite_awready),
        .s_axi_lite_wdata   (s_axi_lite_wdata),
        .s_axi_lite_wstrb   (s_axi_lite_wstrb),
        .s_axi_lite_wvalid  (s_axi_lite_wvalid),
        .s_axi_lite_wready  (s_axi_lite_wready),
        .s_axi_lite_bresp   (s_axi_lite_bresp),
        .s_axi_lite_bvalid  (s_axi_lite_bvalid),
        .s_axi_lite_bready  (s_axi_lite_bready),
        .s_axi_lite_araddr  (s_axi_lite_araddr),
        .s_axi_lite_arprot  (s_axi_lite_arprot),
        .s_axi_lite_arvalid (s_axi_lite_arvalid),
        .s_axi_lite_arready (s_axi_lite_arready),
        .s_axi_lite_rdata   (s_axi_lite_rdata),
        .s_axi_lite_rresp   (s_axi_lite_rresp),
        .s_axi_lite_rvalid  (s_axi_lite_rvalid),
        .s_axi_lite_rready  (s_axi_lite_rready),

        .m_axi_arid         (m_axi_arid),
        .m_axi_araddr       (m_axi_araddr),
        .m_axi_arlen        (m_axi_arlen),
        .m_axi_arsize       (m_axi_arsize),
        .m_axi_arburst      (m_axi_arburst),
        .m_axi_arlock       (m_axi_arlock),
        .m_axi_arcache      (m_axi_arcache),
        .m_axi_arprot       (m_axi_arprot),
        .m_axi_arqos        (m_axi_arqos),
        .m_axi_arvalid      (m_axi_arvalid),
        .m_axi_arready      (m_axi_arready),
        .m_axi_rid          (m_axi_rid),
        .m_axi_rdata        (m_axi_rdata),
        .m_axi_rresp        (m_axi_rresp),
        .m_axi_rlast        (m_axi_rlast),
        .m_axi_rvalid       (m_axi_rvalid),
        .m_axi_rready       (m_axi_rready),

        .m_axis_tdata       (pk_tdata),
        .m_axis_tkeep       (pk_tkeep),
        .m_axis_tlast       (pk_tlast),
        .m_axis_tvalid      (pk_tvalid),
        .m_axis_tready      (pk_tready)
    );

    // ------------------------------------------------------------------
    // Depacketizer : 32bit packed bytes -> 24bit RGB pixel
    //   tuser/tid/tdest 입력은 DMA 쪽에서 안 쓰므로 0
    // ------------------------------------------------------------------
    axis_frame_depacketizer #(
        .TUSER_WIDTH     (16),
        .TID_WIDTH       (8),
        .TDEST_WIDTH     (4),
        .PIXELS_PER_LINE (PIXELS_PER_LINE),
        .FRAME_HEIGHT    (FRAME_HEIGHT)
    ) U_DEPACKETIZER (
        .aclk          (aclk),
        .aresetn       (aresetn),
        .s_axis_tdata  (pk_tdata),
        .s_axis_tkeep  (pk_tkeep),
        .s_axis_tuser  (16'd0),
        .s_axis_tid    (8'd0),
        .s_axis_tdest  (4'd0),
        .s_axis_tlast  (pk_tlast),
        .s_axis_tvalid (pk_tvalid),
        .s_axis_tready (pk_tready),

        .m_axis_tdata  (m_axis_video_tdata),
        .m_axis_tkeep  (m_axis_video_tkeep),
        .m_axis_tuser  (m_axis_video_tuser),
        .m_axis_tlast  (m_axis_video_tlast),
        .m_axis_tvalid (m_axis_video_tvalid),
        .m_axis_tready (m_axis_video_tready)
    );

endmodule