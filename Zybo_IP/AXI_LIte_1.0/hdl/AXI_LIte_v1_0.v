`timescale 1 ns / 1 ps

module AXI_LIte_v1_0 #(
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 5,
    parameter integer ID_WIDTH   = 6,
    parameter integer ADDR_WIDTH = 32,
    parameter integer DATA_WIDTH = 64
)(
    input  wire s00_axi_aclk,
    input  wire s00_axi_aresetn,

    input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_awaddr,
    input  wire [2:0] s00_axi_awprot,
    input  wire s00_axi_awvalid,
    output wire s00_axi_awready,

    input  wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
    input  wire s00_axi_wvalid,
    output wire s00_axi_wready,

    output wire [1:0] s00_axi_bresp,
    output wire s00_axi_bvalid,
    input  wire s00_axi_bready,

    input  wire [C_S00_AXI_ADDR_WIDTH-1:0] s00_axi_araddr,
    input  wire [2:0] s00_axi_arprot,
    input  wire s00_axi_arvalid,
    output wire s00_axi_arready,

    output wire [C_S00_AXI_DATA_WIDTH-1:0] s00_axi_rdata,
    output wire [1:0] s00_axi_rresp,
    output wire s00_axi_rvalid,
    input  wire s00_axi_rready,

    output wire [ID_WIDTH-1:0] m_axi_arid,
    output wire [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0] m_axi_arlen,
    output wire [2:0] m_axi_arsize,
    output wire [1:0] m_axi_arburst,
    output wire m_axi_arlock,
    output wire [3:0] m_axi_arcache,
    output wire [2:0] m_axi_arprot,
    output wire [3:0] m_axi_arqos,
    output wire m_axi_arvalid,
    input  wire m_axi_arready,

    input  wire [ID_WIDTH-1:0] m_axi_rid,
    input  wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0] m_axi_rresp,
    input  wire m_axi_rlast,
    input  wire m_axi_rvalid,
    output wire m_axi_rready,
    output wire [23:0] read_data,
    input wire [11:0] read_addr
);

    wire CAPTURE_START;
    wire [31:0] BASE_ADDR;
    wire [10:0] CROP_X;
    wire [9:0] CROP_Y;
    reg CAPTURE_BUSY;
    reg capture_start_d;
    wire accepted_start;
    wire capture_cfg_valid;
    wire master_busy;
    wire CAPTURE_DONE;
    wire [23:0] BUFF_DATA;
    wire BUFF_VALID;
    wire [11:0] debug_read_addr;
    wire debug_read_enable;
    // The AI-side RAM read address remains internal until the accelerator is
    // instantiated here. CPU reads select debug_read_addr through AXI-Lite.
    wire [11:0] capture_read_addr;
    assign capture_read_addr = debug_read_enable ? debug_read_addr : read_addr;
    assign capture_cfg_valid = (CROP_X <= 11'd1024) && (CROP_Y <= 10'd464) && (CROP_X[2:0] == 3'b000) && (BASE_ADDR[2:0] == 3'b000);
    assign accepted_start = CAPTURE_START && !capture_start_d && !CAPTURE_BUSY && capture_cfg_valid;

    always @(posedge s00_axi_aclk) begin
        if (!s00_axi_aresetn) begin
            capture_start_d <= 1'b0;
            CAPTURE_BUSY <= 1'b0;
        end else begin
            capture_start_d <= CAPTURE_START;
            if (accepted_start) CAPTURE_BUSY <= 1'b1;
            else if (CAPTURE_DONE && !master_busy) CAPTURE_BUSY <= 1'b0;
        end
    end

    AXI_LIte_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) AXI_LIte_v1_0_S00_AXI_inst (
        .CAPTURE_START(CAPTURE_START),
        .BASE_ADDR(BASE_ADDR),
        .CROP_X(CROP_X),
        .CROP_Y(CROP_Y),
        .CAPTURE_BUSY(CAPTURE_BUSY),
        .CAPTURE_DONE(CAPTURE_DONE),
        .DEBUG_READ_ADDR(debug_read_addr),
        .DEBUG_READ_ENABLE(debug_read_enable),
        .DEBUG_READ_DATA(read_data),

        .S_AXI_ACLK(s00_axi_aclk),
        .S_AXI_ARESETN(s00_axi_aresetn),

        .S_AXI_AWADDR(s00_axi_awaddr),
        .S_AXI_AWPROT(s00_axi_awprot),
        .S_AXI_AWVALID(s00_axi_awvalid),
        .S_AXI_AWREADY(s00_axi_awready),

        .S_AXI_WDATA(s00_axi_wdata),
        .S_AXI_WSTRB(s00_axi_wstrb),
        .S_AXI_WVALID(s00_axi_wvalid),
        .S_AXI_WREADY(s00_axi_wready),

        .S_AXI_BRESP(s00_axi_bresp),
        .S_AXI_BVALID(s00_axi_bvalid),
        .S_AXI_BREADY(s00_axi_bready),

        .S_AXI_ARADDR(s00_axi_araddr),
        .S_AXI_ARPROT(s00_axi_arprot),
        .S_AXI_ARVALID(s00_axi_arvalid),
        .S_AXI_ARREADY(s00_axi_arready),

        .S_AXI_RDATA(s00_axi_rdata),
        .S_AXI_RRESP(s00_axi_rresp),
        .S_AXI_RVALID(s00_axi_rvalid),
        .S_AXI_RREADY(s00_axi_rready)
    );

    CAPTURE_AXI_HP1_MASTER #(
        .ID_WIDTH(ID_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) CAPTURE_AXI_HP1_MASTER_inst (
        .clk(s00_axi_aclk),
        .RESETN(s00_axi_aresetn),

        .CAPTURE_ARID(m_axi_arid),
        .CAPTURE_ARADDR(m_axi_araddr),
        .CAPTURE_ARLEN(m_axi_arlen),
        .CAPTURE_ARSIZE(m_axi_arsize),
        .CAPTURE_ARBURST(m_axi_arburst),
        .CAPTURE_ARLOCK(m_axi_arlock),
        .CAPTURE_ARCACHE(m_axi_arcache),
        .CAPTURE_ARPROT(m_axi_arprot),
        .CAPTURE_ARQOS(m_axi_arqos),
        .CAPTURE_ARVALID(m_axi_arvalid),
        .CAPTURE_ARREADY(m_axi_arready),

        .CAPTURE_RID(m_axi_rid),
        .CAPTURE_RDATA(m_axi_rdata),
        .CAPTURE_RRESP(m_axi_rresp),
        .CAPTURE_RLAST(m_axi_rlast),
        .CAPTURE_RVALID(m_axi_rvalid),
        .CAPTURE_RREADY(m_axi_rready),

        .CAPTURE_START(accepted_start),
        .BASE_ADDR(BASE_ADDR),
        .CROP_X(CROP_X),
        .CROP_Y(CROP_Y),

        .CAPTURE_BUSY(master_busy),

        .BUFF_DATA(BUFF_DATA),
        .BUFF_VALID(BUFF_VALID)
    );

    CAPTURE #(
        .DATA_WIDTH(24),
        .F_X(256),
        .F_Y(256),
        .CAP(256),
        .CAP_RAM(64)
    ) CAPTURE_inst (
        .ACLK(s00_axi_aclk),
		.RESETN(s00_axi_aresetn),
    	.CAPTURE_START(accepted_start),
        .CROP_X(11'd0),
        .CROP_Y(10'd0),
    	.BUFF_DATA(BUFF_DATA),
    	.BUFF_VALID(BUFF_VALID),
    	.CAPTURE_DONE(CAPTURE_DONE),
	.read_addr(capture_read_addr),
	.read_data(read_data)
    );

endmodule
