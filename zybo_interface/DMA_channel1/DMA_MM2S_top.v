`timescale 1ns / 1ps
//
// DMA MM2S top  (MCDMA MM2S 대체)
//
//   CPU --(S_AXI_LITE)--> AXI_Lite_Slave_v1_0 (레지스터 맵, irq)
//                              |
//                        [frame select]  LIVE=0 : SA
//                              |         LIVE=1 : FB_BASE + newest_idx x BTT
//                              v
//                         AXI4_read_engine --(M_AXI AR/R, 32bit 주소)--> axi_mem_intercon
//                              |                                           ├─ HP0 (DDR)
//                              v                                           └─ axi_bram_ctrl (로딩화면)
//                            fifo (32bit)
//                              v
//                         M_AXIS (32bit packed) --> axis_frame_depacketizer --> v_axi4s_vid_out
//
// 동작 요약 (MCDMA + ISR 에서 하던 일을 하드웨어로)
//   - CYCLIC=1 : 프레임이 끝나면 CPU 개입 없이 바로 다음 프레임
//   - LIVE=0   : SA 하나만 반복 (로딩화면 0x4400_0000, 필터 결과 고정 등) = mm2s_override
//   - LIVE=1   : 매 프레임 시작마다 S2MM 이 가장 최근 완성한 버퍼를 읽음 = newest_rx_idx
//   - LIVE / SA 변경은 다음 프레임 경계에서 자동 반영 (채널 정지/재구성 불필요)
//
// 클럭은 aclk 하나 (AXI-Lite, AXI4, AXIS, s2mm_newest_idx 모두 동일 클럭 전제)
//
module dma_mm2s_top #(
    parameter integer FIFO_DEPTH      = 64,
    parameter integer MAX_BURST_BYTES = 64
)(
    input  wire         aclk,
    input  wire         aresetn,

    output wire         irq,

    // S2MM(카메라 쓰기) 채널이 알려주는 "가장 최근에 다 쓴 버퍼 번호"
    // 아직 없으면 0 으로 묶고 CR.IDX_SW=1 로 CPU 가 SW_IDX 에 써주면 됨
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

    // ================= M_AXIS (32bit packed bytes) =================
    output wire [31:0]  m_axis_tdata,
    output wire [3:0]   m_axis_tkeep,
    output wire         m_axis_tlast,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready
);

    // ------------------------------------------------------------------
    // Register Map
    // ------------------------------------------------------------------
    wire [31:0] SA_reg, DA_reg, BTT_reg, CDMACR_reg, FB_BASE_reg;
    wire [9:0]  BURST_CFG_reg;
    wire [3:0]  NUM_BUF_reg;
    wire [2:0]  SW_IDX_reg;
    wire        start_pulse, abort_pulse;

    wire        dma_busy, dma_done, dma_error, frame_done;
    wire [31:0] dma_error_addr;
    reg  [2:0]  cur_buf_idx;

    AXI_Lite_Slave_v1_0 #(
        .C_S00_AXI_DATA_WIDTH (32),
        .C_S00_AXI_ADDR_WIDTH (6)
    ) U_REGMAP (
        .SA_reg          (SA_reg),
        .DA_reg          (DA_reg),
        .BTT_reg         (BTT_reg),
        .CDMACR_reg      (CDMACR_reg),
        .BURST_CFG_reg   (BURST_CFG_reg),
        .FB_BASE_reg     (FB_BASE_reg),
        .NUM_BUF_reg     (NUM_BUF_reg),
        .SW_IDX_reg      (SW_IDX_reg),
        .start_pulse     (start_pulse),
        .abort_pulse     (abort_pulse),
        .irq             (irq),
        .dma_busy        (dma_busy),
        .frame_done      (frame_done),
        .read_error      (dma_error),
        .read_error_addr (dma_error_addr),
        .cur_buf_idx     (cur_buf_idx),

        .s00_axi_aclk    (aclk),
        .s00_axi_aresetn (aresetn),
        .s00_axi_awaddr  (s_axi_lite_awaddr),
        .s00_axi_awprot  (s_axi_lite_awprot),
        .s00_axi_awvalid (s_axi_lite_awvalid),
        .s00_axi_awready (s_axi_lite_awready),
        .s00_axi_wdata   (s_axi_lite_wdata),
        .s00_axi_wstrb   (s_axi_lite_wstrb),
        .s00_axi_wvalid  (s_axi_lite_wvalid),
        .s00_axi_wready  (s_axi_lite_wready),
        .s00_axi_bresp   (s_axi_lite_bresp),
        .s00_axi_bvalid  (s_axi_lite_bvalid),
        .s00_axi_bready  (s_axi_lite_bready),
        .s00_axi_araddr  (s_axi_lite_araddr),
        .s00_axi_arprot  (s_axi_lite_arprot),
        .s00_axi_arvalid (s_axi_lite_arvalid),
        .s00_axi_arready (s_axi_lite_arready),
        .s00_axi_rdata   (s_axi_lite_rdata),
        .s00_axi_rresp   (s_axi_lite_rresp),
        .s00_axi_rvalid  (s_axi_lite_rvalid),
        .s00_axi_rready  (s_axi_lite_rready)
    );

    // ------------------------------------------------------------------
    // Frame select (MCDMA ISR 의 newest_rx_idx / mm2s_override 역할)
    // ------------------------------------------------------------------
    wire cr_cyclic = CDMACR_reg[4];
    wire cr_live   = CDMACR_reg[5];
    wire cr_idx_sw = CDMACR_reg[6];

    wire [2:0] newest_raw = cr_idx_sw ? SW_IDX_reg : s2mm_newest_idx;
    wire [2:0] newest_idx = ({1'b0, newest_raw} < NUM_BUF_reg) ? newest_raw : 3'd0;

    wire [31:0] live_addr = FB_BASE_reg + (newest_idx * BTT_reg);
    wire [31:0] frame_src = cr_live ? live_addr : SA_reg;

    // 지금 읽는 버퍼 번호 (디버깅용, SR[10:8])
    // datapath 가 src_addr 를 래치하는 순간(init) 과 같은 클럭에 같이 잡음
    reg start_q;
    always @(posedge aclk) begin
        if (!aresetn) begin
            start_q     <= 1'b0;
            cur_buf_idx <= 3'd0;
        end else begin
            start_q <= start_pulse && !dma_busy;
            if (start_q || frame_done)
                cur_buf_idx <= newest_idx;
        end
    end

    // ------------------------------------------------------------------
    // Read Engine
    // ------------------------------------------------------------------
    wire        fifo_wr_en;
    wire [31:0] fifo_wr_data;
    wire        fifo_full;

    AXI4_read_engine #(
        .ADDR_WIDTH      (32),
        .DATA_WIDTH      (32),
        .LEN_WIDTH       (32),
        .BURST_WIDTH     (8),
        .R0_BASE         (32'h0000_0000),   // DDR (HP0)
        .R0_SIZE         (32'h4000_0000),
        .R1_BASE         (32'h8000_0000),   // BRAM 프레임 창 (axi_bram_ctrl)
        .R1_SIZE         (32'h002A_3000),
        .MAX_BURST_BYTES (MAX_BURST_BYTES)
    ) U_READ_ENGINE (
        .clk          (aclk),
        .rst_n        (aresetn),
        .start        (start_pulse),
        .abort        (abort_pulse),
        .cyclic       (cr_cyclic),
        .src_addr     (frame_src),
        .length       (BTT_reg),
        .burst_cfg    (BURST_CFG_reg),
        .busy         (dma_busy),
        .done         (dma_done),
        .error        (dma_error),
        .error_addr   (dma_error_addr),
        .frame_done   (frame_done),
        .fifo_wr_en   (fifo_wr_en),
        .fifo_wr_data (fifo_wr_data),
        .fifo_full    (fifo_full),
        .arid         (m_axi_arid),
        .araddr       (m_axi_araddr),
        .arlen        (m_axi_arlen),
        .arsize       (m_axi_arsize),
        .arburst      (m_axi_arburst),
        .arvalid      (m_axi_arvalid),
        .arready      (m_axi_arready),
        .rdata        (m_axi_rdata),
        .rvalid       (m_axi_rvalid),
        .rlast        (m_axi_rlast),
        .rid          (m_axi_rid),
        .rresp        (m_axi_rresp),
        .rready       (m_axi_rready)
    );

    assign m_axi_arlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;

    // ------------------------------------------------------------------
    // FIFO
    // ------------------------------------------------------------------
    wire        fifo_rd_en;
    wire [31:0] fifo_rd_data;
    wire        fifo_empty;

    fifo #(
        .DATA_WIDTH (32),
        .DEPTH      (FIFO_DEPTH)
    ) U_FIFO (
        .clk          (aclk),
        .rst_n        (aresetn),
        .fifo_wr_en   (fifo_wr_en),
        .fifo_wr_data (fifo_wr_data),
        .fifo_full    (fifo_full),
        .fifo_rd_en   (fifo_rd_en),
        .fifo_rd_data (fifo_rd_data),
        .fifo_empty   (fifo_empty),
        .fifo_count   ()
    );

    // ------------------------------------------------------------------
    // FIFO -> AXI-Stream
    // ------------------------------------------------------------------
    assign m_axis_tvalid = !fifo_empty;
    assign m_axis_tdata  = fifo_rd_data;
    assign m_axis_tkeep  = 4'b1111;
    assign fifo_rd_en    = m_axis_tvalid && m_axis_tready;

    reg [29:0] out_beat_cnt;
    reg [29:0] total_beats_q;

    always @(posedge aclk) begin
        if (!aresetn)
            total_beats_q <= 30'd0;
        else if (start_pulse && !dma_busy)
            total_beats_q <= BTT_reg[31:2];
    end

    assign m_axis_tlast = (out_beat_cnt == total_beats_q - 1'b1);

    always @(posedge aclk) begin
        if (!aresetn)
            out_beat_cnt <= 30'd0;
        else if (fifo_rd_en)
            out_beat_cnt <= m_axis_tlast ? 30'd0 : out_beat_cnt + 1'b1;
    end

endmodule