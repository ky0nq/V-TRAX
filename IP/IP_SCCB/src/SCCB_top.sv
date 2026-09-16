`timescale 1ns / 1ps

module SCCB_top #(
    parameter len_x = 320,
    parameter len_y = 240,
    parameter BYTE_SWAP = 0
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        pclk,
    input  logic        href,
    input  logic        vsync,
    input  logic [7:0]  cam_data,

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
    output logic        o_irq,

    // ===== 카메라 캡처 결과 (VGA/DMA쪽으로) =====
    output logic [15:0]              pixel_data,
    output logic                     pixel_valid,
    output logic [$clog2(len_x)-1:0] pixel_x,
    output logic [$clog2(len_y)-1:0] pixel_y,
    output logic                     vga_start,

    // ===== SCCB 물리 핀 =====
    inout  wire         siod,
    output logic        sioc,
    output logic        pwdn
);

    logic       w_wstart, w_rstart, w_op;
    logic [7:0] w_reg_addr_to_seq, w_reg_data_to_seq;

    logic       w_rw, w_start;
    logic [6:0] w_addr;
    logic [7:0] w_reg_addr_to_eng, w_reg_data_to_eng;

    logic       w_engine_done;
    logic [7:0] w_read_data;

    logic       w_seq_done;

    logic       w_siod_i, w_siod_en, w_siod_o;
    assign siod     = w_siod_en ? w_siod_o : 1'bz;
    assign w_siod_i = siod;

    APB_to_SCCB U_WRAPPER (
        .PCLK(clk),
        .PRESETn(~rst),

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

        .o_wstart   (w_wstart),
        .o_rstart   (w_rstart),
        .o_op       (w_op),
        .o_reg_addr (w_reg_addr_to_seq),
        .o_reg_data (w_reg_data_to_seq),

        .i_done     (w_seq_done),
        .i_read_data(w_read_data),

        .o_irq      (o_irq)
    );

    SCCB_sequencer #(
        .BYTE_SWAP(BYTE_SWAP)
    ) U_SEQ (
        .clk       (clk),
        .rst       (rst),

        .i_wstart  (w_wstart),
        .i_rstart  (w_rstart),
        .i_op      (w_op),
        .i_reg_addr(w_reg_addr_to_seq),
        .i_reg_data(w_reg_data_to_seq),
        .i_done    (w_engine_done),

        .o_rw      (w_rw),
        .o_start   (w_start),
        .o_addr    (w_addr),
        .o_reg_addr(w_reg_addr_to_eng),
        .o_reg_data(w_reg_data_to_eng),

        .o_seq_done(w_seq_done)
    );

    OV7670_engine #(
        .len_x     (len_x),
        .len_y     (len_y)
    ) U_ENGINE (
        .clk       (clk),
        .rst       (rst),
        .pclk      (pclk),
        .i_href    (href),
        .i_vsync   (vsync),
        .i_cam_data(cam_data),

        .i_rw      (w_rw),
        .i_start   (w_start),
        .i_addr    (w_addr),
        .i_reg_addr(w_reg_addr_to_eng),
        .i_reg_data(w_reg_data_to_eng),

        .o_pixel_data (pixel_data),
        .o_pixel_valid(pixel_valid),
        .o_pixel_x    (pixel_x),
        .o_pixel_y    (pixel_y),
        .o_reg_data   (w_read_data),
        .o_done       (w_engine_done),

        .i_siod     (w_siod_i),
        .o_sioc     (sioc),
        .o_siod_en  (w_siod_en),
        .o_siod     (w_siod_o),
        .o_vga_start(vga_start),
        .o_pwdn     (pwdn)
    );

endmodule