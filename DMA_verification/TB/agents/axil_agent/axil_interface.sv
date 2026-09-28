// AXI4-Lite : TB = master (CPU 역할, 레지스터 read/write)

interface axil_interface #(parameter int AW = 7, parameter int DW = 32)
                   (input logic clk, input logic rst_n);
    // AW
    logic [AW-1:0]   awaddr;
    logic [2:0]      awprot;
    logic            awvalid;
    logic            awready;
    // W
    logic [DW-1:0]   wdata;
    logic [DW/8-1:0] wstrb;
    logic            wvalid;
    logic            wready;
    // B
    logic [1:0]      bresp;
    logic            bvalid;
    logic            bready;
    // AR
    logic [AW-1:0]   araddr;
    logic [2:0]      arprot;
    logic            arvalid;
    logic            arready;
    // R
    logic [DW-1:0]   rdata;
    logic [1:0]      rresp;
    logic            rvalid;
    logic            rready;
    // 인터럽트
    logic            mm2s_irq;
    logic            s2mm_irq;

    clocking drv_cb @(posedge clk);
        default input #1step output #0;
        output awaddr, awprot, awvalid, wdata, wstrb, wvalid, bready;
        output araddr, arprot, arvalid, rready;
        input  awready, wready, bresp, bvalid;
        input  arready, rdata, rresp, rvalid;
        input  mm2s_irq, s2mm_irq;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input awaddr, awprot, awvalid, awready;
        input wdata, wstrb, wvalid, wready;
        input bresp, bvalid, bready;
        input araddr, arprot, arvalid, arready;
        input rdata, rresp, rvalid, rready;
        input mm2s_irq, s2mm_irq;
    endclocking

    modport DRV(clocking drv_cb, input clk, input rst_n);
    modport MON(clocking mon_cb, input clk, input rst_n);
endinterface