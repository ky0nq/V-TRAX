`timescale 1ns / 1ps
`include "uvm_macros.svh"
import uvm_pkg::*;
import dma_pkg::*;

module tb_top;
    logic aclk;
    logic aresetn;

    initial aclk = 0;
    always #5 aclk = ~aclk;          // 100MHz

    initial begin
        aresetn = 0;
        repeat (10) @(posedge aclk);
        aresetn = 1;
    end

    // ---------------- interfaces ----------------
    axil_interface #(.AW(7), .DW(32))             axil  (aclk, aresetn);
    mm2s_interface #(.IDW(5), .AW(32), .DW(32))   mm2s  (aclk, aresetn);
    s2mm_interface #(.IDW(4), .AW(32), .DW(32))   s2mm  (aclk, aresetn);
    axis_interface #(.DW(24), .KW(3), .UW(1))     vin   (aclk, aresetn);  // 카메라 -> DUT
    axis_interface #(.DW(24), .KW(3), .UW(1))     vout  (aclk, aresetn);  // DUT -> 출력

    // ---------------- DUT ----------------
    // 시뮬레이션 시간 줄이려고 해상도 작게
    dma_top #(
        .PIXELS_PER_LINE (64),
        .FRAME_HEIGHT    (4)
    ) dut (
        .aclk                (aclk),
        .aresetn             (aresetn),
        .mm2s_irq            (axil.mm2s_irq),
        .s2mm_irq            (axil.s2mm_irq),

        // AXI-Lite
        .s_axi_lite_awaddr   (axil.awaddr),
        .s_axi_lite_awprot   (axil.awprot),
        .s_axi_lite_awvalid  (axil.awvalid),
        .s_axi_lite_awready  (axil.awready),
        .s_axi_lite_wdata    (axil.wdata),
        .s_axi_lite_wstrb    (axil.wstrb),
        .s_axi_lite_wvalid   (axil.wvalid),
        .s_axi_lite_wready   (axil.wready),
        .s_axi_lite_bresp    (axil.bresp),
        .s_axi_lite_bvalid   (axil.bvalid),
        .s_axi_lite_bready   (axil.bready),
        .s_axi_lite_araddr   (axil.araddr),
        .s_axi_lite_arprot   (axil.arprot),
        .s_axi_lite_arvalid  (axil.arvalid),
        .s_axi_lite_arready  (axil.arready),
        .s_axi_lite_rdata    (axil.rdata),
        .s_axi_lite_rresp    (axil.rresp),
        .s_axi_lite_rvalid   (axil.rvalid),
        .s_axi_lite_rready   (axil.rready),

        // video in
        .s_axis_video_tdata  (vin.tdata),
        .s_axis_video_tvalid (vin.tvalid),
        .s_axis_video_tready (vin.tready),
        .s_axis_video_tlast  (vin.tlast),
        .s_axis_video_tuser  (vin.tuser[0]),

        // S2MM (AXI3 write)
        .m_axi_s2mm_awid     (s2mm.awid),
        .m_axi_s2mm_awaddr   (s2mm.awaddr),
        .m_axi_s2mm_awlen    (s2mm.awlen),
        .m_axi_s2mm_awsize   (s2mm.awsize),
        .m_axi_s2mm_awburst  (s2mm.awburst),
        .m_axi_s2mm_awlock   (s2mm.awlock),
        .m_axi_s2mm_awcache  (s2mm.awcache),
        .m_axi_s2mm_awprot   (s2mm.awprot),
        .m_axi_s2mm_awvalid  (s2mm.awvalid),
        .m_axi_s2mm_awready  (s2mm.awready),
        .m_axi_s2mm_wid      (s2mm.wid),
        .m_axi_s2mm_wdata    (s2mm.wdata),
        .m_axi_s2mm_wstrb    (s2mm.wstrb),
        .m_axi_s2mm_wlast    (s2mm.wlast),
        .m_axi_s2mm_wvalid   (s2mm.wvalid),
        .m_axi_s2mm_wready   (s2mm.wready),
        .m_axi_s2mm_bid      (s2mm.bid),
        .m_axi_s2mm_bresp    (s2mm.bresp),
        .m_axi_s2mm_bvalid   (s2mm.bvalid),
        .m_axi_s2mm_bready   (s2mm.bready),

        // MM2S (AXI4 read)
        .m_axi_mm2s_arid     (mm2s.arid),
        .m_axi_mm2s_araddr   (mm2s.araddr),
        .m_axi_mm2s_arlen    (mm2s.arlen),
        .m_axi_mm2s_arsize   (mm2s.arsize),
        .m_axi_mm2s_arburst  (mm2s.arburst),
        .m_axi_mm2s_arlock   (mm2s.arlock),
        .m_axi_mm2s_arcache  (mm2s.arcache),
        .m_axi_mm2s_arprot   (mm2s.arprot),
        .m_axi_mm2s_arqos    (mm2s.arqos),
        .m_axi_mm2s_arvalid  (mm2s.arvalid),
        .m_axi_mm2s_arready  (mm2s.arready),
        .m_axi_mm2s_rid      (mm2s.rid),
        .m_axi_mm2s_rdata    (mm2s.rdata),
        .m_axi_mm2s_rresp    (mm2s.rresp),
        .m_axi_mm2s_rlast    (mm2s.rlast),
        .m_axi_mm2s_rvalid   (mm2s.rvalid),
        .m_axi_mm2s_rready   (mm2s.rready),

        // video out
        .m_axis_video_tdata  (vout.tdata),
        .m_axis_video_tkeep  (vout.tkeep),
        .m_axis_video_tuser  (vout.tuser[0]),
        .m_axis_video_tlast  (vout.tlast),
        .m_axis_video_tvalid (vout.tvalid),
        .m_axis_video_tready (vout.tready)
    );

    // +WAVE=파일명 을 주면 그 이름으로 파형 저장 (안 주면 저장 안 함 -> 평소엔 빠르게)
    initial begin
        string fsdb_name;
        if ($value$plusargs("WAVE=%s", fsdb_name)) begin
            $fsdbDumpfile(fsdb_name);
            $fsdbDumpvars(0, tb_top, "+all");
        end
    end

    // ---------------- config_db ----------------
    // 모든 driver/monitor 가 "vif" 라는 이름으로 get 함
    // axis 는 같은 타입이 두 개라서 agent 경로(*camera_agt*, *disp_agt*)로 구분
    initial begin
        uvm_config_db#(virtual axil_interface)::set(null, "*axil_agt*",   "axil_vif", axil);
        uvm_config_db#(virtual axis_interface)::set(null, "*camera_agt*", "axis_vif", vin);
        uvm_config_db#(virtual axis_interface)::set(null, "*disp_agt*",   "axis_vif", vout);
        uvm_config_db#(virtual mm2s_interface)::set(null, "*mm2s_agt*",   "mm2s_vif", mm2s);
        uvm_config_db#(virtual s2mm_interface)::set(null, "*s2mm_agt*",   "s2mm_vif", s2mm);
        run_test();
    end
endmodule