`timescale 1ns / 1ps
//
// image_control (ram_bridge) : front end of RAM(Weight) BRAM port A
//                              -- 2-stage pipelined version --
//
// Why 2 stages
//   A single register stage was not enough: the two multiplies
//   (word/960 via *17477>>18, then sy*300) still totalled ~9.8 ns of
//   combinational logic in one cycle (WNS -4.1 ns at 10 ns / 100 MHz).
//   The math is now split at the halfway point:
//     Stage A (comb) : ctrl_addr -> word/960 divide          -> reg (y_coord_q, ...)
//     Stage B (comb) : box test, sy*300 multiply, add offset -> reg (bram_addr)
//   Each stage is now well under 10 ns.
//
// !!! REQUIRED : AXI BRAM Controller read latency = 3 !!!
//   ctrl_en at cycle t -> Stage A regs at t+1 -> Stage B / BRAM port at t+2
//   -> BRAM data back at t+3.
//   Set axi_bram_ctrl_0 "Read Latency" to 3
//   (Tcl: set_property CONFIG.READ_LATENCY {3} [get_bd_cells axi_bram_ctrl_0])
//   Keep blk_mem_gen Port A output registers OFF (BRAM itself still adds 1
//   cycle, on top of the 2 pipeline cycles here = 3 total).
//
// BRAM layout (32bit words, Weight_Bias_Image coe, packed without gaps)
//   0x0000 - 0xB0C1 (0     - 45249) : weights, 1 per word [23:0]
//   0xB0C2 - 0xB0EC (45250 - 45292) : params, 1 per word [31:0]
//   0xB0ED - 0xEA58 (45293 - 59992) : loading image, 400x49, 3B/px packed,
//                                     300 words per row x 49 rows
//
// AXI window (AXI BRAM Controller byte address, 22bit)
//   0x00_0000 - 0x2A_2FFF : frame window (1280x720x3B), read only, for DMA MM2S
//   0x30_0000 - 0x33_FFFF : CPU direct window, BRAM mapped 1:1
//                           writes allowed only below LOAD_IMG_BASE
//
module image_control #(
    parameter [15:0] WEIGHT_BASE   = 16'h0000,    // documentation only
    parameter [15:0] PARAM_BASE    = 16'hB0C2,    // documentation only
    parameter [15:0] LOAD_IMG_BASE = 16'hB0ED     // 45293 : loading image start
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
    localparam WORDS_PER_LINE = 960;   // 1280px * 3B / 4B
    localparam BOX_WORD_START = 330;   // x = 440px
    localparam BOX_WORD_LEN   = 300;   // 400px
    localparam START_Y        = 311;
    localparam BOX_H          = 98;    // 49 source rows, doubled vertically

    localparam [31:0] BG_COLOR = 32'h00000000;   // keep all 4 bytes equal

    // ==================================================================
    // Stage A (combinational) : decode request + the first multiply
    //   word / 960 = ((word >> 6) * 17477) >> 18   (exact for 0 .. 691199)
    // ==================================================================
    wire [15:0] direct_addr_a = ctrl_addr[17:2];                   // BRAM word address
    wire is_direct_a  = (ctrl_addr[21:18] == 4'b1100);              // 0x30_0000 - 0x33_FFFF
    wire is_write_a   = (ctrl_we != 4'b0000);
    wire direct_wr_a  = ctrl_en &&  is_write_a &&  is_direct_a
                       && (direct_addr_a < LOAD_IMG_BASE);          // protect loading image
    wire is_read_a    = ctrl_en && !is_write_a;

    wire [19:0] word_a    = ctrl_addr[21:2];
    wire [13:0] word_64_a = word_a[19:6];                           // word / 64
    wire [28:0] prod_a    = word_64_a * 15'd17477;                  // stage A's one multiply
    wire [10:0] y_coord_a = prod_a[28:18];                          // word / 960

    // ==================================================================
    // Stage A registers -> Stage B inputs
    // ==================================================================
    reg         en_q, we_q, direct_q, wr_q;
    reg  [15:0] direct_addr_q;
    reg  [19:0] word_a_q;
    reg  [10:0] y_coord_q;
    reg  [31:0] din_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            en_q          <= 1'b0;
            we_q          <= 1'b0;
            direct_q      <= 1'b0;
            wr_q          <= 1'b0;
            direct_addr_q <= 16'd0;
            word_a_q      <= 20'd0;
            y_coord_q     <= 11'd0;
            din_q         <= 32'd0;
        end else begin
            en_q          <= ctrl_en;
            we_q          <= is_write_a;
            direct_q      <= is_direct_a;
            wr_q          <= direct_wr_a;
            direct_addr_q <= direct_addr_a;
            word_a_q      <= word_a;
            y_coord_q     <= y_coord_a;
            din_q         <= ctrl_din;
        end
    end

    // ==================================================================
    // Stage B (combinational) : box test + second multiply (sy*300) + add
    // ==================================================================
    wire [19:0] y_x960_b  = ({9'd0, y_coord_q} << 10) - ({9'd0, y_coord_q} << 6);  // y*960
    wire [19:0] x_full_b  = word_a_q - y_x960_b;                    // 0 .. 959
    wire [9:0]  word_in_line_b = x_full_b[9:0];

    wire is_box_row_b  = (y_coord_q >= START_Y) && (y_coord_q < START_Y + BOX_H);
    wire is_box_word_b = is_box_row_b &&
                          (word_in_line_b >= BOX_WORD_START) &&
                          (word_in_line_b <  BOX_WORD_START + BOX_WORD_LEN);

    wire [10:0] sy_full_b     = y_coord_q - START_Y;
    wire [9:0]  sy_b          = sy_full_b[10:1];                    // vertical 2x
    wire [9:0]  word_in_row_b = word_in_line_b - BOX_WORD_START;    // 0 .. 299
    wire [15:0] img_off_b     = is_box_word_b ? (sy_b * 300 + word_in_row_b) : 16'h0;  // stage B's one multiply

    wire [15:0] addr_next_b   = direct_q ? direct_addr_q : (LOAD_IMG_BASE + img_off_b);
    wire        sel_b         = direct_q || is_box_word_b;

    // ==================================================================
    // Stage B registers -> drive the BRAM port (cycle t+2)
    // ==================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            bram_en   <= 1'b0;
            bram_we   <= 1'b0;
            bram_addr <= 16'd0;
            bram_din  <= 32'd0;
        end else begin
            bram_en   <= en_q;
            bram_we   <= wr_q;
            bram_addr <= addr_next_b;
            bram_din  <= din_q;
        end
    end

    // ==================================================================
    // Read data select, aligned to the BRAM output (3 cycles after ctrl_en)
    //   sel_q1 : captured together with the Stage-B registers (cycle t+2,
    //            matches bram_en/bram_addr being presented to the BRAM)
    //   sel_q2 : one more cycle (cycle t+3, matches bram_dout being valid)
    // ==================================================================
    reg sel_q1, sel_q2;
    reg read_a_q;   // was this a read, sampled alongside en_q/we_q (available t+1)

    always @(posedge clk) begin
        if (!rst_n)
            read_a_q <= 1'b0;
        else
            read_a_q <= is_read_a;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            sel_q1 <= 1'b0;
            sel_q2 <= 1'b0;
        end else begin
            sel_q1 <= read_a_q ? sel_b : 1'b0;
            sel_q2 <= sel_q1;
        end
    end

    assign ctrl_dout = sel_q2 ? bram_dout : BG_COLOR;

endmodule
