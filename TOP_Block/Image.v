`timescale 1ns / 1ps
//
// image_control (ram_bridge) : front end of RAM(Weight) BRAM port A
//                              -- 2-stage pipelined version --
//                              -- 64x64 source image, 4x upscale -> 256x256 --
//
// Pipeline (same 3-cycle read latency as before)
//   Stage A (comb) : ctrl_addr -> word/960 divide             -> reg (y_coord_q, ...)
//   Stage B (comb) : box test, sx = wr/3, {sy,sx} + base       -> reg (bram_addr)
//
// !!! REQUIRED : AXI BRAM Controller read latency = 3 !!!
//   ctrl_en at cycle t -> Stage A regs at t+1 -> Stage B / BRAM port at t+2
//   -> BRAM data back at t+3.
//   (Tcl: set_property CONFIG.READ_LATENCY {3} [get_bd_cells axi_bram_ctrl_0])
//   Keep blk_mem_gen Port A output registers OFF.
//
// Stripe fix (vs. previous version)
//   Before, sel_q1/sel_q2 advanced on EVERY clock, while the BRAM output
//   only changes when it is enabled. When the controller paused between
//   bursts (ctrl_en low), the BRAM kept its data but sel dropped to 0, so
//   the first word of each 64-byte burst came back as BG_COLOR
//   -> coloured vertical lines every 16 words.
//   Now the Stage-B registers (bram_addr, sel_q1) only load while en_q=1,
//   and sel_q2 only loads when bram_en=1, so the select path holds its value
//   exactly like the BRAM output does.
//
// BRAM layout (32bit words, Weight_Bias_Image coe, packed without gaps)
//   0x0000 - 0xB0C1 (0     - 45249) : weights, 1 per word [23:0]
//   0xB0C2 - 0xB0EC (45250 - 45292) : params, 1 per word [31:0]
//   0xB0ED - 0xC0EC (45293 - 49388) : loading image, 64x64 gray,
//                                     1 word per source pixel, all 4 bytes = gray
//                                     address = base + sy*64 + sx
//
// Why 1 word per source pixel
//   4x horizontal upscale at 3B/px : 4 output pixels = 12 bytes = 3 words,
//   all made of the same gray byte. So output word wr (0..191 in the row)
//   is simply source pixel sx = wr/3, sent as-is.
//
// Screen position (1280x720 frame, 3B/px, 960 words per line)
//   START_X must be a multiple of 4 px (4 px = 3 words).
//   x = START_X .. START_X+255, y = START_Y .. START_Y+255
//   Default (444, 144) is estimated from the displayed crop window;
//   adjust these two parameters to match the real crop origin.
//
// AXI window (AXI BRAM Controller byte address, 22bit)
//   0x00_0000 - 0x2A_2FFF : frame window (1280x720x3B), read only, for DMA MM2S
//   0x30_0000 - 0x33_FFFF : CPU direct window, BRAM mapped 1:1
//                           writes allowed only below LOAD_IMG_BASE
//
module image_control #(
    parameter [15:0] WEIGHT_BASE   = 16'h0000,    // documentation only
    parameter [15:0] PARAM_BASE    = 16'hB0C2,    // documentation only
    parameter [15:0] LOAD_IMG_BASE = 16'hB0ED,    // 45293 : loading image start
    parameter        START_X       = 444,         // multiple of 4
    parameter        START_Y       = 144
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        ctrl_en,
    input  wire [3:0]  ctrl_we,
    input  wire [21:0] ctrl_addr,
    input  wire [31:0] ctrl_din,
    output wire [31:0] ctrl_dout,

    output reg         bram_en,
    output reg  [0:0]  bram_we,     // BRAM wea is 1 bit
    output reg  [15:0] bram_addr,
    output reg  [31:0] bram_din,
    input  wire [31:0] bram_dout
);

    // ------------------------------------------------------------------
    // screen / box parameters
    // ------------------------------------------------------------------
    localparam WORDS_PER_LINE = 960;               // 1280px * 3B / 4B
    localparam BOX_WORD_START = START_X * 3 / 4;   // 444 -> 333
    localparam BOX_WORD_LEN   = 192;               // 256px * 3B / 4B
    localparam BOX_H          = 256;               // 64 source rows x4

    localparam [31:0] BG_COLOR = 32'h00000000;     // keep all 4 bytes equal

    // ==================================================================
    // Stage A (combinational) : decode request + the first multiply
    //   word / 960 = ((word >> 6) * 17477) >> 18   (exact for 0 .. 691199)
    // ==================================================================
    wire [15:0] direct_addr_a = ctrl_addr[17:2];                   // BRAM word address
    wire is_direct_a  = (ctrl_addr[21:18] == 4'b1100);              // 0x30_0000 - 0x33_FFFF
    wire is_write_a   = (ctrl_we != 4'b0000);
    wire direct_wr_a  = ctrl_en &&  is_write_a &&  is_direct_a
                       && (direct_addr_a < LOAD_IMG_BASE);          // protect loading image

    wire [19:0] word_a    = ctrl_addr[21:2];
    wire [13:0] word_64_a = word_a[19:6];                           // word / 64
    wire [28:0] prod_a    = word_64_a * 15'd17477;                  // stage A's one multiply
    wire [10:0] y_coord_a = prod_a[28:18];                          // word / 960

    // ==================================================================
    // Stage A registers -> Stage B inputs
    // ==================================================================
    reg         en_q, direct_q, wr_q;
    reg  [15:0] direct_addr_q;
    reg  [19:0] word_a_q;
    reg  [10:0] y_coord_q;
    reg  [31:0] din_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            en_q          <= 1'b0;
            direct_q      <= 1'b0;
            wr_q          <= 1'b0;
            direct_addr_q <= 16'd0;
            word_a_q      <= 20'd0;
            y_coord_q     <= 11'd0;
            din_q         <= 32'd0;
        end else begin
            en_q          <= ctrl_en;
            direct_q      <= is_direct_a;
            wr_q          <= direct_wr_a;
            direct_addr_q <= direct_addr_a;
            word_a_q      <= word_a;
            y_coord_q     <= y_coord_a;
            din_q         <= ctrl_din;
        end
    end

    // ==================================================================
    // Stage B (combinational) : box test + source pixel address
    // ==================================================================
    wire [19:0] y_x960_b  = ({9'd0, y_coord_q} << 10) - ({9'd0, y_coord_q} << 6);  // y*960
    wire [19:0] x_full_b  = word_a_q - y_x960_b;                    // 0 .. 959
    wire [9:0]  word_in_line_b = x_full_b[9:0];

    wire is_box_row_b  = (y_coord_q >= START_Y) && (y_coord_q < START_Y + BOX_H);
    wire is_box_word_b = is_box_row_b &&
                          (word_in_line_b >= BOX_WORD_START) &&
                          (word_in_line_b <  BOX_WORD_START + BOX_WORD_LEN);

    wire [10:0] sy_full_b  = y_coord_q - START_Y;                   // 0 .. 255
    wire [5:0]  sy_b       = sy_full_b[7:2];                        // /4 -> 0 .. 63
    wire [9:0]  wr_full_b  = word_in_line_b - BOX_WORD_START;       // 0 .. 191
    wire [7:0]  wr_b       = wr_full_b[7:0];
    // wr/3 = (wr*171) >> 9  (exact for 0 .. 191), 171 = 128+32+8+2+1 (shift-add)
    wire [15:0] wr_x171_b  = ({8'd0, wr_b} << 7) + ({8'd0, wr_b} << 5)
                           + ({8'd0, wr_b} << 3) + ({8'd0, wr_b} << 1)
                           +  {8'd0, wr_b};
    wire [5:0]  sx_b       = wr_x171_b[14:9];                       // 0 .. 63

    wire [15:0] img_off_b   = is_box_word_b ? {4'd0, sy_b, sx_b} : 16'h0;   // sy*64 + sx
    wire [15:0] addr_next_b = direct_q ? direct_addr_q : (LOAD_IMG_BASE + img_off_b);
    wire        sel_b       = direct_q || is_box_word_b;

    // ==================================================================
    // Stage B registers -> drive the BRAM port (cycle t+2)
    //   address / select only load on an enabled request, so they hold
    //   their value through controller stalls (like a real BRAM port).
    // ==================================================================
    reg sel_q1, sel_q2;

    always @(posedge clk) begin
        if (!rst_n) begin
            bram_en   <= 1'b0;
            bram_we   <= 1'b0;
            bram_addr <= 16'd0;
            bram_din  <= 32'd0;
            sel_q1    <= 1'b0;
        end else begin
            bram_en   <= en_q;
            bram_we   <= wr_q;
            bram_din  <= din_q;
            if (en_q) begin
                bram_addr <= addr_next_b;
                sel_q1    <= sel_b;
            end
        end
    end

    // ==================================================================
    // Read data select, aligned to the BRAM output
    //   sel_q2 changes on exactly the same clock edges as bram_dout
    //   (only when the BRAM is enabled), so it can never drift.
    // ==================================================================
    always @(posedge clk) begin
        if (!rst_n)
            sel_q2 <= 1'b0;
        else if (bram_en)
            sel_q2 <= sel_q1;
    end

    assign ctrl_dout = sel_q2 ? bram_dout : BG_COLOR;

endmodule
