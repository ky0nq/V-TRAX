`timescale 1 ns / 1 ps

module BBOX_v1_0 #
(
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 4
)
(
    // AXI4-Stream Video clock/reset
    input  wire        axis_aclk,
    input  wire        axis_aresetn,

    // DMA_0/m_axis_video -> BBOX/s_axis_video
    input  wire [23:0] s_axis_video_tdata,
    input  wire [2:0]  s_axis_video_tkeep,
    input  wire        s_axis_video_tlast,
    input  wire        s_axis_video_tuser,
    input  wire        s_axis_video_tvalid,
    output wire        s_axis_video_tready,

    // BBOX/m_axis_video -> v_axi4s_vid_out_0/video_in
    output wire [23:0] m_axis_video_tdata,
    output wire        m_axis_video_tlast,
    output wire        m_axis_video_tuser,
    output wire        m_axis_video_tvalid,
    input  wire        m_axis_video_tready,

    // AXI-Lite S00_AXI
    input  wire                                      s00_axi_aclk,
    input  wire                                      s00_axi_aresetn,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]          s00_axi_awaddr,
    input  wire [2:0]                               s00_axi_awprot,
    input  wire                                      s00_axi_awvalid,
    output wire                                      s00_axi_awready,
    input  wire [C_S00_AXI_DATA_WIDTH-1:0]          s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0]      s00_axi_wstrb,
    input  wire                                      s00_axi_wvalid,
    output wire                                      s00_axi_wready,
    output wire [1:0]                               s00_axi_bresp,
    output wire                                      s00_axi_bvalid,
    input  wire                                      s00_axi_bready,
    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]          s00_axi_araddr,
    input  wire [2:0]                               s00_axi_arprot,
    input  wire                                      s00_axi_arvalid,
    output wire                                      s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1:0]          s00_axi_rdata,
    output wire [1:0]                               s00_axi_rresp,
    output wire                                      s00_axi_rvalid,
    input  wire                                      s00_axi_rready
);

    wire        cfg_enable;
    wire [10:0] cfg_x;
    wire [8:0]  cfg_y;
    wire [23:0] cfg_color;

    BBOX_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) BBOX_v1_0_S00_AXI_inst (
        .bbox_enable   (cfg_enable),
        .bbox_x        (cfg_x),
        .bbox_y        (cfg_y),
        .bbox_color    (cfg_color),

        .S_AXI_ACLK    (s00_axi_aclk),
        .S_AXI_ARESETN (s00_axi_aresetn),
        .S_AXI_AWADDR  (s00_axi_awaddr),
        .S_AXI_AWPROT  (s00_axi_awprot),
        .S_AXI_AWVALID (s00_axi_awvalid),
        .S_AXI_AWREADY (s00_axi_awready),
        .S_AXI_WDATA   (s00_axi_wdata),
        .S_AXI_WSTRB   (s00_axi_wstrb),
        .S_AXI_WVALID  (s00_axi_wvalid),
        .S_AXI_WREADY  (s00_axi_wready),
        .S_AXI_BRESP   (s00_axi_bresp),
        .S_AXI_BVALID  (s00_axi_bvalid),
        .S_AXI_BREADY  (s00_axi_bready),
        .S_AXI_ARADDR  (s00_axi_araddr),
        .S_AXI_ARPROT  (s00_axi_arprot),
        .S_AXI_ARVALID (s00_axi_arvalid),
        .S_AXI_ARREADY (s00_axi_arready),
        .S_AXI_RDATA   (s00_axi_rdata),
        .S_AXI_RRESP   (s00_axi_rresp),
        .S_AXI_RVALID  (s00_axi_rvalid),
        .S_AXI_RREADY  (s00_axi_rready)
    );

    // 1280x720 video, fixed 256x256 BBOX matching the capture ROI.
    // Clamp the programmable origin so the complete BBOX remains on screen.
    wire [10:0] cfg_x_safe = (cfg_x > 11'd1024) ? 11'd1024 : cfg_x;
    wire [8:0]  cfg_y_safe = (cfg_y > 9'd464)    ? 9'd464    : cfg_y;

    reg        bbox_enable;
    reg [10:0] bbox_x;
    reg [8:0]  bbox_y;
    reg [23:0] bbox_color;

    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;

    wire video_fire = s_axis_video_tvalid && s_axis_video_tready;

    // AXI-Lite and AXIS are connected to the same 10 MHz clock in BD.
    // New settings are applied only at SOF.
    always @(posedge axis_aclk) begin
        if (!axis_aresetn) begin
            bbox_enable <= 1'b0;
            bbox_x      <= 11'd0;
            bbox_y      <= 9'd0;
            bbox_color  <= 24'hFF0000;
        end else if (video_fire && s_axis_video_tuser) begin
            bbox_enable <= cfg_enable;
            bbox_x      <= cfg_x_safe;
            bbox_y      <= cfg_y_safe;
            bbox_color  <= cfg_color;
        end
    end

    wire [10:0] pixel_x = s_axis_video_tuser ? 11'd0 : x_cnt;
    wire [9:0]  pixel_y = s_axis_video_tuser ? 10'd0 : y_cnt;

    wire [10:0] bbox_x_start = bbox_x;
    wire [9:0]  bbox_y_start = {1'b0, bbox_y};
    wire [10:0] bbox_x_end = bbox_x_start + 11'd255;
    wire [9:0]  bbox_y_end = bbox_y_start + 10'd255;

    wire inside_x = (pixel_x >= bbox_x_start) && (pixel_x <= bbox_x_end);
    wire inside_y = (pixel_y >= bbox_y_start) && (pixel_y <= bbox_y_end);

    wire bbox_border =
        bbox_enable &&
        ((((pixel_x == bbox_x_start) || (pixel_x == bbox_x_end)) && inside_y) ||
         (((pixel_y == bbox_y_start) || (pixel_y == bbox_y_end)) && inside_x));

    assign s_axis_video_tready = m_axis_video_tready;

    assign m_axis_video_tdata  = bbox_border ? bbox_color : s_axis_video_tdata;
    assign m_axis_video_tlast  = s_axis_video_tlast;
    assign m_axis_video_tuser  = s_axis_video_tuser;
    assign m_axis_video_tvalid = s_axis_video_tvalid;

    always @(posedge axis_aclk) begin
        if (!axis_aresetn) begin
            x_cnt <= 11'd0;
            y_cnt <= 10'd0;
        end else if (video_fire) begin
            if (s_axis_video_tuser) begin
                x_cnt <= s_axis_video_tlast ? 11'd0 : 11'd1;
                y_cnt <= s_axis_video_tlast ? 10'd1 : 10'd0;
            end else if (s_axis_video_tlast) begin
                x_cnt <= 11'd0;
                y_cnt <= y_cnt + 10'd1;
            end else begin
                x_cnt <= x_cnt + 11'd1;
            end
        end
    end

    // DMA provides TKEEP[2:0], but v_axi4s_vid_out/video_in does not use TKEEP.
    wire _unused_tkeep = &s_axis_video_tkeep;

endmodule
