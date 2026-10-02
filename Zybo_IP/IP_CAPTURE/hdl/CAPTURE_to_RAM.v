`timescale 1ns / 1ps

module CAPTURE #(
    parameter DATA_WIDTH = 24,
    parameter F_X        = 1280,
    parameter F_Y        = 720,
    parameter CAP        = 256,
    parameter CAP_RAM    = 64
)(
    input  wire                  ACLK,
    input  wire                  RESETN,
    input  wire                  CAPTURE_START,
    input  wire [10:0]           CROP_X,
    input  wire [9:0]            CROP_Y,
    input  wire [DATA_WIDTH-1:0] BUFF_DATA,
    input  wire                  BUFF_VALID,
    output reg                   CAPTURE_DONE,

    input  wire [$clog2(4096)-1:0] read_addr,
    output reg  [DATA_WIDTH-1:0] read_data
);
    reg [DATA_WIDTH-1:0] capture_data;
    reg                  capture_valid;

    reg [10:0] CAPTURE_X_0;
    reg [9:0]  CAPTURE_Y_0;
    wire [31:0] CAPTURE_X_1 = CAPTURE_X_0 + CAP - 1;
    wire [31:0] CAPTURE_Y_1 = CAPTURE_Y_0 + CAP - 1;
    // The video/DDR path stores pixels as R-B-G:
    //   [23:16] = R, [15:8] = B, [7:0] = G.
    // This capture path reduces each 4x4 block and writes standard RGB888.
    localparam integer REDUCE = 4;

    reg [$clog2(4096)-1:0] ram_w_addr;
    reg capture_start_d;

    wire start_pulse;

    assign start_pulse = CAPTURE_START && !capture_start_d;

    always @(posedge ACLK) begin
        if (!RESETN) begin
            CAPTURE_X_0 <= 0;
            CAPTURE_Y_0 <= 0;
        end else if (start_pulse) begin
            CAPTURE_X_0 <= CROP_X;
            CAPTURE_Y_0 <= CROP_Y;
        end
    end

    reg [$clog2(F_X)-1:0] Stream_data_Ax;
    reg [$clog2(F_Y)-1:0] Stream_data_Ay;

    wire [$clog2(CAP)-1:0] capture_x;
    wire [$clog2(CAP)-1:0] capture_y;

    assign capture_x = Stream_data_Ax - CAPTURE_X_0;
    assign capture_y = Stream_data_Ay - CAPTURE_Y_0;

    // The last three pixels are enough to form each horizontal 4-pixel sum.
    reg [DATA_WIDTH-1:0] pixel_prev0;
    reg [DATA_WIDTH-1:0] pixel_prev1;
    reg [DATA_WIDTH-1:0] pixel_prev2;
    reg [9:0] sum_r_x;
    reg [9:0] sum_g_x;
    reg [9:0] sum_b_x;
    reg sum_x_valid;
    reg [$clog2(CAP)-1:0] capture_x_d;
    reg [$clog2(CAP)-1:0] capture_y_d;

    // A column group is revisited only on the next input row (256 cycles later).
    reg [11:0] sum_r_y [0:CAP_RAM-1];
    reg [11:0] sum_g_y [0:CAP_RAM-1];
    reg [11:0] sum_b_y [0:CAP_RAM-1];
    wire [$clog2(CAP_RAM)-1:0] capture_block_x = capture_x_d >> 2;
    wire [11:0] total_r = sum_r_y[capture_block_x] + {2'b00, sum_r_x};
    wire [11:0] total_g = sum_g_y[capture_block_x] + {2'b00, sum_g_x};
    wire [11:0] total_b = sum_b_y[capture_block_x] + {2'b00, sum_b_x};

    wire pixel_in_crop = BUFF_VALID &&
        (Stream_data_Ax >= CAPTURE_X_0) &&
        (Stream_data_Ax <= CAPTURE_X_1) &&
        (Stream_data_Ay >= CAPTURE_Y_0) &&
        (Stream_data_Ay <= CAPTURE_Y_1);

    wire [8:0] red_pair0 = {1'b0, pixel_prev2[23:16]} +
                           {1'b0, pixel_prev1[23:16]};
    wire [8:0] red_pair1 = {1'b0, pixel_prev0[23:16]} +
                           {1'b0, BUFF_DATA[23:16]};
    wire [8:0] green_pair0 = {1'b0, pixel_prev2[7:0]} +
                             {1'b0, pixel_prev1[7:0]};
    wire [8:0] green_pair1 = {1'b0, pixel_prev0[7:0]} +
                             {1'b0, BUFF_DATA[7:0]};
    wire [8:0] blue_pair0 = {1'b0, pixel_prev2[15:8]} +
                            {1'b0, pixel_prev1[15:8]};
    wire [8:0] blue_pair1 = {1'b0, pixel_prev0[15:8]} +
                            {1'b0, BUFF_DATA[15:8]};

    reg [DATA_WIDTH-1:0] CAPTURE_RAM [0:4095];

    always @(posedge ACLK) begin
        if (!RESETN)
            capture_start_d <= 1'b0;
        else
            capture_start_d <= CAPTURE_START;
    end

    always @(posedge ACLK) begin
        if (!RESETN) begin
            Stream_data_Ax <= 0;
            Stream_data_Ay <= 0;
        end else if (start_pulse) begin
            Stream_data_Ax <= 0;
            Stream_data_Ay <= 0;
        end else if (BUFF_VALID) begin
            if (Stream_data_Ax >= F_X-1) begin
                Stream_data_Ax <= 0;
                if (Stream_data_Ay >= F_Y-1)
                    Stream_data_Ay <= 0;
                else
                    Stream_data_Ay <= Stream_data_Ay + 1'b1;
            end else begin
                Stream_data_Ax <= Stream_data_Ax + 1'b1;
            end
        end
    end

    always @(posedge ACLK) begin
        if (!RESETN || start_pulse) begin
            pixel_prev0 <= 0;
            pixel_prev1 <= 0;
            pixel_prev2 <= 0;
            sum_r_x <= 0;
            sum_g_x <= 0;
            sum_b_x <= 0;
            sum_x_valid <= 1'b0;
            capture_x_d <= 0;
            capture_y_d <= 0;
        end else begin
            sum_x_valid <= 1'b0;
            if (pixel_in_crop) begin
                pixel_prev2 <= pixel_prev1;
                pixel_prev1 <= pixel_prev0;
                pixel_prev0 <= BUFF_DATA;
                if (capture_x[1:0] == REDUCE-1) begin
                    sum_r_x <= {1'b0, red_pair0} + {1'b0, red_pair1};
                    sum_g_x <= {1'b0, green_pair0} + {1'b0, green_pair1};
                    sum_b_x <= {1'b0, blue_pair0} + {1'b0, blue_pair1};
                    capture_x_d <= capture_x;
                    capture_y_d <= capture_y;
                    sum_x_valid <= 1'b1;
                end
            end
        end
    end

    always @(posedge ACLK) begin
        if (!RESETN || start_pulse) begin
            capture_data <= 0;
            capture_valid <= 1'b0;
        end else begin
            capture_valid <= 1'b0;
            if (sum_x_valid) begin
                if (capture_y_d[1:0] == 2'd0) begin
                    sum_r_y[capture_block_x] <= {2'b00, sum_r_x};
                    sum_g_y[capture_block_x] <= {2'b00, sum_g_x};
                    sum_b_y[capture_block_x] <= {2'b00, sum_b_x};
                end else if (capture_y_d[1:0] == REDUCE-1) begin
                    capture_data <= {total_r[11:4], total_g[11:4], total_b[11:4]};
                    capture_valid <= 1'b1;
                end else begin
                    sum_r_y[capture_block_x] <= total_r;
                    sum_g_y[capture_block_x] <= total_g;
                    sum_b_y[capture_block_x] <= total_b;
                end
            end
        end
    end

    always @(posedge ACLK) begin
        if (!RESETN) begin
            ram_w_addr <= 0;
            CAPTURE_DONE <= 1'b0;
        end else if (start_pulse) begin
            ram_w_addr <= 0;
            CAPTURE_DONE <= 1'b0;
        end else if (capture_valid) begin
            CAPTURE_RAM[ram_w_addr] <= capture_data;

            if (ram_w_addr == 4095) begin
                ram_w_addr <= 0;
                CAPTURE_DONE <= 1'b1;
            end else begin
                ram_w_addr <= ram_w_addr + 1'b1;
            end
        end
    end

    always @(posedge ACLK) begin
        if (!RESETN)
            read_data <= 0;
        else
            read_data <= CAPTURE_RAM[read_addr];
    end

endmodule
