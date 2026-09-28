// AXI3 Write (S2MM) : DUT = master, TB = slave (메모리 모델 역할)

interface s2mm_interface #(parameter int IDW = 4, parameter int AW = 32, parameter int DW = 32)
                     (input logic clk, input logic rst_n);
    // AW
    logic [IDW-1:0]  awid;
    logic [AW-1:0]   awaddr;
    logic [3:0]      awlen;     // AXI3 : 4bit
    logic [2:0]      awsize;
    logic [1:0]      awburst;
    logic [1:0]      awlock;    // AXI3 : 2bit
    logic [3:0]      awcache;
    logic [2:0]      awprot;
    logic            awvalid;
    logic            awready;
    // W
    logic [IDW-1:0]  wid;       // AXI3 전용
    logic [DW-1:0]   wdata;
    logic [DW/8-1:0] wstrb;
    logic            wlast;
    logic            wvalid;
    logic            wready;
    // B
    logic [IDW-1:0]  bid;
    logic [1:0]      bresp;
    logic            bvalid;
    logic            bready;

    clocking drv_cb @(posedge clk);
        default input #1step output #0;
        input  awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awvalid;
        output awready;
        input  wid, wdata, wstrb, wlast, wvalid;
        output wready;
        output bid, bresp, bvalid;
        input  bready;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awvalid, awready;
        input wid, wdata, wstrb, wlast, wvalid, wready;
        input bid, bresp, bvalid, bready;
    endclocking

    modport DRV(clocking drv_cb, input clk, input rst_n);
    modport MON(clocking mon_cb, input clk, input rst_n);
endinterface