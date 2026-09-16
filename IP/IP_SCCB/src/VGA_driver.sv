`timescale 1ns / 1ps

module VGA_driver #(
    parameter len_x = 640,
    parameter len_y = 480
) (
    input  logic        clk,         // 100MHz, 물리/시스템 클럭
    input  logic        rst,

    // ===== FrameBuffer / SCCB_top에서 들어오는 논리신호 =====
    input  logic [15:0]  i_pixel_data, 
    input  logic         i_vga_start, 

    // ===== 모니터로 나가는 물리 핀 =====
    output logic       hsync,
    output logic       vsync,
    output logic [3:0] vgaRed,
    output logic [3:0] vgaGreen,
    output logic [3:0] vgaBlue,
    output logic       xclk,

    // ===== frame buffer에서 나가는 신호 =====
    output logic [$clog2(len_x)-1:0] o_pixel_x,
    output logic [$clog2(len_y)-1:0] o_pixel_y
);
    localparam HFRONT = 16, HSYNC = 96, HBACK = 48;
    localparam VFRONT = 10, VSYNC = 2, VBACK = 33;
    localparam IDLE = 0, DATA = 1, WWAIT = 2, RWAIT = 3;

    logic [1:0] state;
    logic [9:0] htick_cnt;
    logic [5:0] vtick_cnt;
    logic       start;
    logic       xclk_tick;
    logic       xclk_pe;
    logic       xclk_d;

    tick_gen #(
        .COUNT(2)
    ) tick_2 (
        .clk (clk),
        .rst (rst),
        .tick(xclk_tick)
    );  //For 250MHz

    always @(posedge xclk_tick, posedge rst) begin
        if (rst) begin
            xclk <= 0;
        end else begin
            xclk <= ~xclk;
        end
    end
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            xclk_d <= 0;
        end else begin
            xclk_d <= xclk;
        end
    end
    assign xclk_pe = (!xclk_d && xclk) ? 1 : 0;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            o_pixel_x <= 0;
            o_pixel_y <= 0;
            vgaRed <= 0;
            vgaGreen <= 0;
            vgaBlue <= 0;
            htick_cnt <= 0;
            vtick_cnt <= 0;
            hsync <= 1;
            vsync <= 1;
            state <= IDLE;
            start <= 0;
        end else begin
            case (state)
                IDLE: begin
                    o_pixel_x <= 0;
                    o_pixel_y <= 0;
                    vgaRed <= 0;
                    vgaGreen <= 0;
                    vgaBlue <= 0;
                    htick_cnt <= 0;
                    vtick_cnt <= 0;
                    hsync <= 1;
                    vsync <= 1;
                    if (i_vga_start) begin
                        start <= 1;
                    end
                    if (start && xclk_pe) begin
                        vgaRed <= i_pixel_data[15:12];
                        vgaGreen <= i_pixel_data[10:7];
                        vgaBlue <= i_pixel_data[4:1];
                        state <= DATA;
                    end

                end
                DATA: begin
                    if (xclk_pe) begin
                        vgaRed   <= i_pixel_data[15:12];
                        vgaGreen <= i_pixel_data[10:7];
                        vgaBlue  <= i_pixel_data[4:1];
                        if (o_pixel_x >= len_x - 1) begin
                            o_pixel_x <= 0;
                            state   <= WWAIT;
                        end else begin
                            o_pixel_x <= o_pixel_x + 1;
                            state   <= DATA;
                        end
                    end
                end
                WWAIT: begin
                    if (xclk_pe) begin
                        vgaRed <= 0;
                        vgaGreen <= 0;
                        vgaBlue <= 0;
                        htick_cnt <= htick_cnt + 1;
                        if (htick_cnt == 16 - 1) hsync <= 0;
                        else if (htick_cnt == 16 + 96 - 1) hsync <= 1;
                        else if (htick_cnt == 16 + 96 + 48 - 1) begin
                            state <= DATA;
                            htick_cnt <= 0;
                            if (o_pixel_y >= len_y - 1) begin
                                o_pixel_y <= 0;
                                state   <= RWAIT;
                            end else begin
                                o_pixel_y <= o_pixel_y + 1;
                            end
                        end
                    end
                end
                RWAIT: begin
                    if (xclk_pe) begin
                        vgaRed <= 0;
                        vgaGreen <= 0;
                        vgaBlue <= 0;
                        htick_cnt <= htick_cnt + 1;
                        if (htick_cnt == len_x + 16 - 1) hsync <= 0;
                        else if (htick_cnt == len_x + 16 + 96 - 1) hsync <= 1;
                        else if (htick_cnt == len_x + 16 + 96 + 48 - 1) begin
                            htick_cnt <= 0;
                            vtick_cnt <= vtick_cnt + 1;
                            if (vtick_cnt == 9) vsync <= 0;
                            else if (vtick_cnt == 11) vsync <= 1;
                            else if (vtick_cnt == 44) begin
                                state <= DATA;
                                vtick_cnt <= 0;
                            end
                        end
                    end
                end
                default: begin
                    state <= IDLE;
                    o_pixel_x <= 0;
                    o_pixel_y <= 0;
                    vgaRed <= 0;
                    vgaGreen <= 0;
                    vgaBlue <= 0;
                    htick_cnt <= 0;
                    vtick_cnt <= 0;
                    hsync <= 1;
                    vsync <= 1;
                end
            endcase
        end
    end
endmodule