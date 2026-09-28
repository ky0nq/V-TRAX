// AXI4 Read (MM2S) : DUT = master, TB = slave (메모리 모델 역할)

interface mm2s_interface #(parameter int IDW = 5, parameter int AW = 32, parameter int DW = 32)
                     (input logic clk, input logic rst_n);
    // AR
    logic [IDW-1:0] arid;
    logic [AW-1:0]  araddr;
    logic [7:0]     arlen;
    logic [2:0]     arsize;
    logic [1:0]     arburst;
    logic           arlock;
    logic [3:0]     arcache;
    logic [2:0]     arprot;
    logic [3:0]     arqos;
    logic           arvalid;
    logic           arready;
    // R
    logic [IDW-1:0] rid;
    logic [DW-1:0]  rdata;
    logic [1:0]     rresp;
    logic           rlast;
    logic           rvalid;
    logic           rready;

    clocking drv_cb @(posedge clk);
        default input #1step output #0;
        input  arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arvalid;
        output arready;
        output rid, rdata, rresp, rlast, rvalid;
        input  rready;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arvalid, arready;
        input rid, rdata, rresp, rlast, rvalid, rready;
    endclocking

    modport DRV(clocking drv_cb, input clk, input rst_n);
    modport MON(clocking mon_cb, input clk, input rst_n);
endinterface