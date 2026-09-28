`timescale 1 ns / 1 ps

module CAPTURE_AXI_HP1_MASTER #(
    parameter integer ID_WIDTH   = 6,
    parameter integer ADDR_WIDTH = 32,
    parameter integer DATA_WIDTH = 64,
    parameter integer FRAME_X    = 1280,
    parameter integer FRAME_Y    = 720,
    parameter integer CAP_SIZE   = 256
)(
    input wire clk,
    input wire RESETN,

    output reg [ID_WIDTH-1:0] CAPTURE_ARID,
    output reg [ADDR_WIDTH-1:0] CAPTURE_ARADDR,
    output reg [7:0] CAPTURE_ARLEN,
    output reg [2:0] CAPTURE_ARSIZE,
    output reg [1:0] CAPTURE_ARBURST,
    output reg CAPTURE_ARLOCK,
    output reg [3:0] CAPTURE_ARCACHE,
    output reg [2:0] CAPTURE_ARPROT,
    output reg [3:0] CAPTURE_ARQOS,
    output reg CAPTURE_ARVALID,
    input wire CAPTURE_ARREADY,

    input wire [ID_WIDTH-1:0] CAPTURE_RID,
    input wire [DATA_WIDTH-1:0] CAPTURE_RDATA,
    input wire [1:0] CAPTURE_RRESP,
    input wire CAPTURE_RLAST,
    input wire CAPTURE_RVALID,
    output wire CAPTURE_RREADY,

    input wire CAPTURE_START,
    input wire [ADDR_WIDTH-1:0] BASE_ADDR,
    input wire [10:0] CROP_X,
    input wire [9:0] CROP_Y,

    output wire CAPTURE_BUSY,

    output reg [23:0] BUFF_DATA,
    output reg BUFF_VALID
);

    localparam integer PIXEL_BYTES   = 3;
    localparam integer BEAT_BYTES    = DATA_WIDTH / 8;
    localparam integer ROW_STRIDE    = FRAME_X * PIXEL_BYTES;
    localparam integer ROI_ROW_BYTES = CAP_SIZE * PIXEL_BYTES;
    localparam integer BEATS_PER_ROW = ROI_ROW_BYTES / BEAT_BYTES;
    localparam integer MAX_CROP_X    = FRAME_X - CAP_SIZE;
    localparam integer MAX_CROP_Y    = FRAME_Y - CAP_SIZE;

    localparam IDLE = 2'd0;
    localparam AR = 2'd1;
    localparam RDATA = 2'd2;
    localparam DRAIN = 2'd3;

    reg [1:0] state;
    reg [9:0] row_count;
    reg [ADDR_WIDTH-1:0] row_addr;
    reg [ADDR_WIDTH-1:0] burst_addr;
    reg [8:0] remaining_beats;
    reg [4:0] burst_beats;
    reg capture_active;
    reg capture_start_d;

    // Eight bytes arrive on each AXI read beat. The existing video pipeline
    // stores each pixel as R-B-G; CAPTURE converts it to RGB888 after unpacking.
    // Emit one packed pixel per cycle and stall RREADY when the byte queue is
    // almost full.
    reg [127:0] byte_buffer;
    reg [4:0] byte_count;
    wire pixel_emit = byte_count >= 5'd3;
    wire [4:0] bytes_after_emit = byte_count - (pixel_emit ? 5'd3 : 5'd0);
    wire [127:0] shifted_buffer =
        pixel_emit ? (byte_buffer >> 24) : byte_buffer;
    wire [63:0] read_bytes =
        (CAPTURE_RRESP == 2'b00) ? CAPTURE_RDATA : 64'd0;
    wire [127:0] merged_buffer = shifted_buffer |
        ({64'd0, read_bytes} << (bytes_after_emit * 8));

    wire start_pulse;
    wire AR_HAND_SHAKE;
    wire R_HAND_SHAKE;
    wire [ADDR_WIDTH-1:0] crop_start_addr;
    assign start_pulse = CAPTURE_START && !capture_start_d;
    assign AR_HAND_SHAKE = CAPTURE_ARVALID && CAPTURE_ARREADY;
    assign R_HAND_SHAKE = CAPTURE_RVALID && CAPTURE_RREADY;
    assign CAPTURE_BUSY = capture_active;
    //assign crop_start_addr = BASE_ADDR +
    //                         CROP_Y * ROW_STRIDE +
    //                         CROP_X * PIXEL_BYTES;
    assign crop_start_addr = BASE_ADDR +
                             (CROP_Y<<8)+ (CROP_Y<<9)+ (CROP_Y<<10) +(CROP_Y<<11)+
                             (CROP_X<<1)+ CROP_X;
    assign CAPTURE_RREADY = (state == RDATA) &&
                            (bytes_after_emit <= 5'd8);

    // HP1 is AXI3: at most 16 beats per burst, never across 4 KiB.
    function [4:0] get_burst_beats;
        input [ADDR_WIDTH-1:0] addr;
        input [8:0] beats_left;
        reg [9:0] beats_to_4k;
        begin
            if (addr[11:0] == 12'h000) beats_to_4k = 10'd512;
            else beats_to_4k = (13'h1000 - {1'b0, addr[11:0]}) >> 3;
            if ((beats_left >= 9'd16) && (beats_to_4k >= 10'd16)) get_burst_beats = 5'd16;
            else if (beats_left <= beats_to_4k) get_burst_beats = beats_left[4:0];
            else								get_burst_beats = beats_to_4k[4:0];
        end
    endfunction

    always @(posedge clk) begin
        if (!RESETN)
            capture_start_d <= 1'b0;
        else
            capture_start_d <= CAPTURE_START;
    end

    always @(posedge clk) begin
        if (!RESETN) begin
            CAPTURE_ARID    <= {ID_WIDTH{1'b0}};
            CAPTURE_ARADDR  <= {ADDR_WIDTH{1'b0}};
            CAPTURE_ARLEN   <= 8'd0;
            CAPTURE_ARSIZE  <= 3'd3;
            CAPTURE_ARBURST <= 2'b01;
            CAPTURE_ARLOCK  <= 1'b0;
            CAPTURE_ARCACHE <= 4'b0000;
            CAPTURE_ARPROT  <= 3'b000;
            CAPTURE_ARQOS   <= 4'b0000;
            CAPTURE_ARVALID <= 1'b0;
            BUFF_DATA       <= 24'd0;
            BUFF_VALID      <= 1'b0;
            byte_buffer     <= 128'd0;
            byte_count      <= 5'd0;
            state           <= IDLE;
            row_count       <= 10'd0;
            row_addr        <= {ADDR_WIDTH{1'b0}};
            burst_addr      <= {ADDR_WIDTH{1'b0}};
            remaining_beats <= 9'd0;
            burst_beats     <= 5'd0;
            capture_active  <= 1'b0;
        end else begin
            BUFF_VALID <= 1'b0;

            if (state != IDLE) begin
                if (pixel_emit) begin
                    BUFF_DATA  <= byte_buffer[23:0];
                    BUFF_VALID <= 1'b1;
                end
                byte_buffer <= R_HAND_SHAKE ? merged_buffer : shifted_buffer;
                byte_count <= bytes_after_emit +
                              (R_HAND_SHAKE ? 5'd8 : 5'd0);
            end

            case (state)
                IDLE: begin
                    CAPTURE_ARVALID <= 1'b0;
                    capture_active <= 1'b0;
                    if (start_pulse &&
                        (CROP_X <= MAX_CROP_X) &&
                        (CROP_Y <= MAX_CROP_Y) &&
                        (crop_start_addr[2:0] == 3'b000)) begin
                        byte_buffer     <= 128'd0;
                        byte_count      <= 5'd0;
                        capture_active  <= 1'b1;
                        row_count       <= 10'd0;
                        row_addr        <= crop_start_addr;
                        burst_addr      <= crop_start_addr;
                        remaining_beats <= BEATS_PER_ROW;
                        burst_beats     <= get_burst_beats(crop_start_addr, BEATS_PER_ROW);
                        CAPTURE_ARADDR  <= crop_start_addr;
                        CAPTURE_ARLEN   <= get_burst_beats(crop_start_addr,
                                                            BEATS_PER_ROW) - 1'b1;
                        CAPTURE_ARVALID <= 1'b1;
                        state           <= AR;
                    end
                end

                AR: begin
                    if (AR_HAND_SHAKE) begin
                        CAPTURE_ARVALID <= 1'b0;
                        state <= RDATA;
                    end
                end

                RDATA: begin
                    if (R_HAND_SHAKE && CAPTURE_RLAST) begin
                        if (remaining_beats > burst_beats) begin
                            remaining_beats <= remaining_beats - burst_beats;
                            burst_addr <= burst_addr + burst_beats * BEAT_BYTES;
                            burst_beats <= get_burst_beats( burst_addr + burst_beats * BEAT_BYTES, remaining_beats - burst_beats);
                            CAPTURE_ARADDR <= burst_addr + burst_beats * BEAT_BYTES;
                            CAPTURE_ARLEN <= get_burst_beats( burst_addr + burst_beats * BEAT_BYTES, remaining_beats - burst_beats) - 1'b1;
                            CAPTURE_ARVALID <= 1'b1;
                            state <= AR;
                        end else if (row_count == CAP_SIZE - 1) begin
                            CAPTURE_ARVALID <= 1'b0;
                            remaining_beats <= 9'd0;
                            burst_beats <= 5'd0;
                            state <= DRAIN;
                        end else begin
                            row_count <= row_count + 1'b1;
                            row_addr <= row_addr + ROW_STRIDE;
                            burst_addr <= row_addr + ROW_STRIDE;
                            remaining_beats <= BEATS_PER_ROW;
                            burst_beats <= get_burst_beats( row_addr + ROW_STRIDE, BEATS_PER_ROW);
                            CAPTURE_ARADDR <= row_addr + ROW_STRIDE;
                            CAPTURE_ARLEN <= get_burst_beats( row_addr + ROW_STRIDE, BEATS_PER_ROW) - 1'b1;
                            CAPTURE_ARVALID <= 1'b1;
                            state <= AR;
                        end
                    end
                end

                DRAIN: begin
                    if (bytes_after_emit == 5'd0) begin
                        capture_active <= 1'b0;
                        state <= IDLE;
                    end
                end

                default: begin
                    CAPTURE_ARVALID <= 1'b0;
                    capture_active <= 1'b0;
                    state <= IDLE;
                end
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        if (DATA_WIDTH != 64)
            $error("CAPTURE_AXI_HP1_MASTER requires a 64-bit HP1 data bus");
        if ((ROW_STRIDE % BEAT_BYTES) != 0 ||
            (ROI_ROW_BYTES % BEAT_BYTES) != 0)
            $error("Packed RGB888 frame and ROI rows must align to eight bytes");
        if (BEATS_PER_ROW > 511)
            $error("BEATS_PER_ROW does not fit in remaining_beats");
    end
`endif
endmodule
