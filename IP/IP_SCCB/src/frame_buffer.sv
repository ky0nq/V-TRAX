`timescale 1ns / 1ps

module frame_buffer #(
    parameter integer len_x            = 640,
    parameter integer len_y            = 480,
    parameter integer FOREGROUND_DARK  = 1,
    parameter integer BINARY_THRESHOLD = 4
) (
    input  logic                        clk,
    input  logic                        pclk,
    input  logic                        rst,

    // ===== 카메라 쪽  =====
    input  logic [                   15:0] i_pixel_data,
    input  logic [   $clog2(len_x>>1)-1:0] i_cam_x,
    input  logic [   $clog2(len_y>>1)-1:0] i_cam_y,
    input  logic                           i_pixel_valid,

    // ===== VGA 쪽  =====
    input  logic [   $clog2(len_x)-1:0] i_vga_x,
    input  logic [   $clog2(len_y)-1:0] i_vga_y,
    output logic [                15:0] o_pixel_data,

    // ===== AI가속기 쪽 =====
    input  logic                                     i_crop_read_en,
    input  logic [$clog2((len_x>>1)*(len_y>>1))-1:0] i_crop_read_addr,
    output logic [                              3:0] o_gray_read_data,

    output logic [ 3:0] o_gray_pixel
);

    localparam integer CAM_WIDTH  = len_x >> 1;
    localparam integer CAM_HEIGHT = len_y >> 1;
    localparam integer FRAME_PIXELS = CAM_WIDTH * CAM_HEIGHT;
    localparam integer FRAME_ADDR_WIDTH = $clog2(FRAME_PIXELS);

    logic [3:0] ram_rdata;
    logic [7:0] r_GrayRed;
    logic [7:0] r_GrayGreen;
    logic [7:0] r_GrayBlue;
    logic [15:0] r_Gray;

    (* ram_style = "block" *)
    logic [3:0] gray_bram[0:FRAME_PIXELS-1];

    logic [FRAME_ADDR_WIDTH-1:0] camera_write_addr;
    logic [FRAME_ADDR_WIDTH-1:0] vga_read_addr;
    logic [FRAME_ADDR_WIDTH-1:0] selected_read_addr;
    logic                        vga_foreground_binary;

    assign r_GrayRed   = {i_pixel_data[15:12], i_pixel_data[15:12]};
    assign r_GrayGreen = {i_pixel_data[10:7],  i_pixel_data[10:7]};
    assign r_GrayBlue  = {i_pixel_data[4:1],   i_pixel_data[4:1]};
    assign r_Gray      = r_GrayRed * 8'd77 + r_GrayGreen * 8'd150 + r_GrayBlue * 8'd29;

    // grayscale 명도 높게 수정
    logic [3:0] gray_raw;
    logic [3:0] enhanced_gray;

    assign gray_raw = r_Gray[15:12];

    always @(*) begin
        if (gray_raw <= 4'd4)
            enhanced_gray = 4'd0;
        else if (gray_raw >= 4'd12)
            enhanced_gray = 4'd15;
        else
            enhanced_gray = (gray_raw - 4'd4) << 1;
    end

    assign o_gray_pixel = enhanced_gray;
    assign camera_write_addr = i_cam_x + i_cam_y * CAM_WIDTH;
    assign vga_read_addr = (i_vga_x >> 1) + (i_vga_y >> 1) * CAM_WIDTH;
    assign selected_read_addr = i_crop_read_en ? i_crop_read_addr : vga_read_addr;
    assign o_gray_read_data = ram_rdata;
    assign vga_foreground_binary = (FOREGROUND_DARK != 0) ?
                                   (ram_rdata <= BINARY_THRESHOLD) :
                                   (ram_rdata >= BINARY_THRESHOLD);

    always @(posedge pclk) begin
        if (i_pixel_valid) gray_bram[camera_write_addr] <= o_gray_pixel;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) ram_rdata <= 4'd0;
        else if (i_crop_read_en ||
                 (((i_vga_x >> 1) <= (CAM_WIDTH - 1)) && ((i_vga_y >> 1) <= (CAM_HEIGHT - 1))))
            ram_rdata <= gray_bram[selected_read_addr];
    end

    always @(*) begin
        o_pixel_data = vga_foreground_binary ? 16'h0000 : 16'hFFFF;
    end

endmodule