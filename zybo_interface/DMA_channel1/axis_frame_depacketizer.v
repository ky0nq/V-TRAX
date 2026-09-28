`timescale 1ns / 1ps


/*
 * MCDMA MM2S: 32-bit packed bytes
 * gray_filter: one 24-bit RGB pixel per transfer
 */
module axis_frame_depacketizer #(
    parameter integer TUSER_WIDTH     = 16,
    parameter integer TID_WIDTH       = 8,
    parameter integer TDEST_WIDTH     = 4,
    parameter integer PIXELS_PER_LINE = 1280,
    parameter integer FRAME_HEIGHT    = 720
)(
    input  wire                   aclk,
    input  wire                   aresetn,
    input  wire [31:0]            s_axis_tdata,
    input  wire [3:0]             s_axis_tkeep,
    input  wire [TUSER_WIDTH-1:0] s_axis_tuser,
    input  wire [TID_WIDTH-1:0]   s_axis_tid,
    input  wire [TDEST_WIDTH-1:0] s_axis_tdest,
    input  wire                   s_axis_tlast,
    input  wire                   s_axis_tvalid,
    output wire                   s_axis_tready,

    output wire [23:0]            m_axis_tdata,
    output wire [2:0]             m_axis_tkeep,
    output wire                   m_axis_tuser,
    output wire                   m_axis_tlast,
    output wire                   m_axis_tvalid,
    input  wire                   m_axis_tready
);
    localparam integer PIXEL_COUNT_WIDTH =
        (PIXELS_PER_LINE <= 1) ? 1 : $clog2(PIXELS_PER_LINE);
    localparam integer LINE_COUNT_WIDTH =
        (FRAME_HEIGHT <= 1) ? 1 : $clog2(FRAME_HEIGHT);

    reg [63:0] byte_buffer;
    reg [3:0] byte_count;
    reg [PIXEL_COUNT_WIDTH-1:0] pixel_count;
    reg [LINE_COUNT_WIDTH-1:0] line_count;

    wire output_transfer = m_axis_tvalid && m_axis_tready;
    wire [3:0] bytes_after_output =
        byte_count - (output_transfer ? 4'd3 : 4'd0);

    wire input_transfer = s_axis_tvalid && s_axis_tready;
    wire [63:0] shifted_buffer =
        output_transfer ? (byte_buffer >> 24) : byte_buffer;
    wire [63:0] input_bytes = {32'b0, s_axis_tdata};
    wire [63:0] merged_buffer =
        shifted_buffer | (input_bytes << (bytes_after_output * 8));

    assign m_axis_tvalid = (byte_count >= 3);
    assign m_axis_tdata  = byte_buffer[23:0];
    assign m_axis_tkeep  = 3'b111;
    assign m_axis_tuser  = (pixel_count == 0) && (line_count == 0);
    assign m_axis_tlast  = (pixel_count == PIXELS_PER_LINE - 1);
    assign s_axis_tready = (bytes_after_output <= 4);

    always @(posedge aclk) begin
        if (!aresetn) begin
            byte_buffer <= 64'b0;
            byte_count  <= 4'd0;
            pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};
            line_count  <= {LINE_COUNT_WIDTH{1'b0}};
        end else begin
            if (input_transfer)
                byte_buffer <= merged_buffer;
            else if (output_transfer)
                byte_buffer <= shifted_buffer;

            byte_count <= bytes_after_output +
                          (input_transfer ? 4'd4 : 4'd0);

            if (output_transfer) begin
                if (pixel_count == PIXELS_PER_LINE - 1) begin
                    pixel_count <= {PIXEL_COUNT_WIDTH{1'b0}};

                    if (line_count == FRAME_HEIGHT - 1)
                        line_count <= {LINE_COUNT_WIDTH{1'b0}};
                    else
                        line_count <= line_count + 1'b1;
                end else begin
                    pixel_count <= pixel_count + 1'b1;
                end
            end
        end
    end

`ifndef SYNTHESIS
    always @(posedge aclk) begin
        if (aresetn && input_transfer && s_axis_tkeep != 4'b1111)
            $error("axis_frame_depacketizer requires full 32-bit input beats");
    end
`endif
endmodule
