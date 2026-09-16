`timescale 1ns / 1ps

module GPIO_top (
    input  logic        PCLK,
    input  logic        PRESETn,

    // ===== Bridge와 연결되는 APB 신호 =====
    input  logic        PSEL,
    input  logic        PENABLE,
    input  logic        PWRITE,
    input  logic [31:0] PADDR,
    input  logic [15:0] PWDATA,
    input  logic [1:0]  PSTRB,
    input  logic [2:0]  PPROT,
    output logic [15:0] PRDATA,
    output logic        PREADY,
    output logic        PSLVERR,
    output logic [7:0]  o_irq,

    // ===== 물리 핀 =====
    inout  wire  [15:0] io_port
);

    logic [15:0] w_cr;
    logic [15:0] w_aodr;
    logic [15:0] w_idr;

    APB_to_GPIO U_BRIDGE (
        .PCLK(PCLK),
        .PRESETn(PRESETn),
        .PSEL(PSEL),
        .PENABLE(PENABLE),
        .PWRITE(PWRITE),
        .PADDR(PADDR),
        .PWDATA(PWDATA),
        .PSTRB(PSTRB),
        .PPROT(PPROT),
        .PRDATA(PRDATA),
        .PREADY(PREADY),
        .PSLVERR(PSLVERR),

        .o_cr  (w_cr),
        .o_aodr(w_aodr),
        .i_idr (w_idr),

        .o_irq(o_irq)
    );

    gpio U_GPIO (
        .cr (w_cr),
        .idr(w_idr),
        .aodr(w_aodr),
        .io_port(io_port)
    );

endmodule