`timescale 1ns / 1ps

module VIDEO_top #(
    parameter cam_x = 320,
    parameter cam_y = 240,
    parameter vga_x = 640,
    parameter vga_y = 480,
    parameter BYTE_SWAP = 0
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        pclk,
    input  logic        href,
    input  logic        cam_vsync,
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

    // ===== SCCB 물리 핀 =====
    inout  wire          siod,
    output logic         sioc,
    output logic         pwdn,

    // ===== VGA 물리 핀 =====
    output logic       hsync,
    output logic       vga_vsync,
    output logic [3:0] vgaRed,
    output logic [3:0] vgaGreen,
    output logic [3:0] vgaBlue,
    output logic       xclk
);

    logic [15:0] w_cam_pixel_data;
    logic        w_cam_pixel_valid;
    logic [$clog2(cam_x)-1:0] w_cam_pixel_x;
    logic [$clog2(cam_y)-1:0] w_cam_pixel_y;
    logic        w_vga_start;

    logic [15:0] w_fb_pixel_data;
    logic [$clog2(vga_x)-1:0] w_vga_pixel_x;
    logic [$clog2(vga_y)-1:0] w_vga_pixel_y;

    SCCB_top #(
        .len_x(cam_x),
        .len_y(cam_y),
        .BYTE_SWAP(BYTE_SWAP)
    ) U_SCCB (
        .clk(clk),
        .rst(rst),
        .pclk(pclk),
        .href(href),
        .vsync(cam_vsync),
        .cam_data(cam_data),

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
        .o_irq(o_irq),

        .pixel_data(w_cam_pixel_data),
        .pixel_valid(w_cam_pixel_valid),
        .pixel_x(w_cam_pixel_x),
        .pixel_y(w_cam_pixel_y),
        .vga_start(w_vga_start),

        .siod(siod),
        .sioc(sioc),
        .pwdn(pwdn)
    );

    frame_buffer #(
        .len_x(vga_x),
        .len_y(vga_y)
    ) U_FB (
        .clk(clk),
        .pclk(pclk),
        .rst(rst),

        .i_pixel_data (w_cam_pixel_data),
        .i_cam_x      (w_cam_pixel_x),
        .i_cam_y      (w_cam_pixel_y),
        .i_pixel_valid(w_cam_pixel_valid),

        .i_vga_x(w_vga_pixel_x),
        .i_vga_y(w_vga_pixel_y),
        .o_pixel_data(w_fb_pixel_data),

        .i_crop_read_en(1'b0),
        .i_crop_read_addr('0),
        .o_gray_read_data()
    );

    VGA_driver #(
        .len_x(vga_x),
        .len_y(vga_y)
    ) U_VGA (
        .clk(clk),
        .rst(rst),

        .i_pixel_data(w_fb_pixel_data),
        .i_vga_start(w_vga_start),

        .hsync(hsync),
        .vsync(vga_vsync),
        .vgaRed(vgaRed),
        .vgaGreen(vgaGreen),
        .vgaBlue(vgaBlue),
        .xclk(xclk),

        .o_pixel_x(w_vga_pixel_x),
        .o_pixel_y(w_vga_pixel_y)
    );

endmodule