
`timescale 1 ns / 1 ps

module AXI4_Lite_interconnect_v1_0 #(
    // Users to add parameters here

    // User parameters ends
    // Do not modify the parameters beyond this line


    // Parameters of Axi Slave Bus Interface S00_AXI
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 4
) (
    // Users to add ports here
    output wire o_irq,

    // Image RAM
    output wire [11:0] o_img_rd_addr,
    input  wire [23:0] i_img_rdata,



    // Shared RAM (Weight + Bias) : data must arrive 1 clk after o_ram_rd_addr, no handshake
    //   word 0..45249 : weight (i_rdata[23:0])   word 45250..45292 : bias (i_rdata[31:0])
    output wire [15:0] o_ram_rd_addr,
    input  wire [31:0] i_rdata,
    // User ports ends
    // Do not modify the ports beyond this line


    // Ports of Axi Slave Bus Interface S00_AXI
    input wire s00_axi_aclk,
    input wire s00_axi_aresetn,
    input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_awaddr,
    input wire [2 : 0] s00_axi_awprot,
    input wire s00_axi_awvalid,
    output wire s00_axi_awready,
    input wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_wdata,
    input wire [(C_S00_AXI_DATA_WIDTH/8)-1 : 0] s00_axi_wstrb,
    input wire s00_axi_wvalid,
    output wire s00_axi_wready,
    output wire [1 : 0] s00_axi_bresp,
    output wire s00_axi_bvalid,
    input wire s00_axi_bready,
    input wire [C_S00_AXI_ADDR_WIDTH-1 : 0] s00_axi_araddr,
    input wire [2 : 0] s00_axi_arprot,
    input wire s00_axi_arvalid,
    output wire s00_axi_arready,
    output wire [C_S00_AXI_DATA_WIDTH-1 : 0] s00_axi_rdata,
    output wire [1 : 0] s00_axi_rresp,
    output wire s00_axi_rvalid,
    input wire s00_axi_rready
);
    wire       cnn_start_valid;
    wire       cnn_start_ready;
    wire       cnn_busy;
    wire       cnn_done_status;
    wire [7:0] cnn_final_result;
    wire       cnn_irq_en;
    wire       cnn_irq_clear;

    // Instantiation of Axi Bus Interface S00_AXI
    AXI4_Lite_interconnect_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
    ) AXI4_Lite_interconnect_v1_0_S00_AXI_inst (
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
        .S_AXI_RREADY(s00_axi_rready),
        .o_cnn_start_valid(cnn_start_valid),
        .i_cnn_start_ready(cnn_start_ready),
        .i_cnn_busy(cnn_busy),
        .o_cnn_irq_en(cnn_irq_en),
        .o_cnn_irq_clear(cnn_irq_clear),
        .i_cnn_final_result(cnn_final_result),
        .i_cnn_done_status(cnn_done_status)
    );

    // Add user logic here
    top_cnn #(
        .IMG_RAM_BASE(32'd0),
        .IMG_WORDS   (13'd4096)
    ) U_TOP_CNN (
        .clk                  (s00_axi_aclk),
        .rst_n                (s00_axi_aresetn),
        .i_start_valid        (cnn_start_valid),
        .o_start_ready        (cnn_start_ready),
        .o_busy               (cnn_busy),
        .i_irq_en             (cnn_irq_en),
        .i_irq_clear          (cnn_irq_clear),
        .o_final_result       (cnn_final_result),
        .o_done_status        (cnn_done_status),
        .o_irq                (o_irq),
        .o_img_rd_addr        (o_img_rd_addr),
        .i_img_rdata          (i_img_rdata),
        .o_ram_rd_addr        (o_ram_rd_addr),
        .i_rdata              (i_rdata),
        // Debug
        .o_layer_idx          (),
        .o_cnn_state          ()
    );
    // User logic ends

endmodule
