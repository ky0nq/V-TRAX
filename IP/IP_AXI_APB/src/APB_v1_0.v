`timescale 1 ns / 1 ps

module APB_v1_0 #(
    parameter integer C_S00_AXI_DATA_WIDTH = 32,
    parameter integer C_S00_AXI_ADDR_WIDTH = 32,
    parameter integer APB_WIDTH            = 16
)(
    // =========================================================
    // AXI4-Lite Slave Interface
    // =========================================================
    input  wire                                  s00_axi_aclk,
    input  wire                                  s00_axi_aresetn,

    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]       s00_axi_awaddr,
    input  wire [2:0]                            s00_axi_awprot,
    input  wire                                  s00_axi_awvalid,
    output wire                                  s00_axi_awready,

    input  wire [C_S00_AXI_DATA_WIDTH-1:0]       s00_axi_wdata,
    input  wire [(C_S00_AXI_DATA_WIDTH/8)-1:0]   s00_axi_wstrb,
    input  wire                                  s00_axi_wvalid,
    output wire                                  s00_axi_wready,

    output wire [1:0]                            s00_axi_bresp,
    output wire                                  s00_axi_bvalid,
    input  wire                                  s00_axi_bready,

    input  wire [C_S00_AXI_ADDR_WIDTH-1:0]       s00_axi_araddr,
    input  wire [2:0]                            s00_axi_arprot,
    input  wire                                  s00_axi_arvalid,
    output wire                                  s00_axi_arready,

    output wire [C_S00_AXI_DATA_WIDTH-1:0]       s00_axi_rdata,
    output wire [1:0]                            s00_axi_rresp,
    output wire                                  s00_axi_rvalid,
    input  wire                                  s00_axi_rready,

    // =========================================================
    // APB Master Interface
    // =========================================================
    output wire [31:0]                           PADDR,

    output wire                                  PSEL_UART,
    //output wire                                  PSEL_SCCB,
    output wire                                  PSEL_GPIO,
    //output wire                                  PSEL_AI,
    output wire                                  PSEL_TIMER,

    output wire                                  PENABLE,
    output wire                                  PWRITE,

    output wire [APB_WIDTH-1:0]                  PWDATA,
    output wire [(APB_WIDTH/8)-1:0]              PSTRB,
    output wire [2:0]                            PPROT,

	input  wire                                 PREADY_UART ,
	input  wire                                 PREADY_GPIO ,
	//input  wire                                 PREADY_AI   ,
	input  wire                                 PREADY_TIMER,
    input  wire                                PSLVERR_UART ,
    input  wire                                PSLVERR_GPIO ,
    //input  wire                                PSLVERR_AI   ,
    input  wire                                PSLVERR_TIMER,
    input  wire [APB_WIDTH-1:0]                 PRDATA_UART ,
    input  wire [APB_WIDTH-1:0]                 PRDATA_GPIO ,
    //input  wire [APB_WIDTH-1:0]                 PRDATA_AI   ,
    input  wire [APB_WIDTH-1:0]                 PRDATA_TIMER
	
	//input  wire                                 PREADY_SCCB ,
    //input  wire [APB_WIDTH-1:0]                 PRDATA_SCCB ,
    //input  wire                                PSLVERR_SCCB ,
);
    //parameter   UART = 3'd1,
    //            SCCB = 3'd2,//Reserved
    //            GPIO = 3'd3,
    //            AI   = 3'd4,
    //            TIMER  = 3'd5;
	
    reg [APB_WIDTH-1:0]				rPRDATA;
    reg								rPREADY;
    reg								rPSLVERR;
	
	
	always @(*)begin
		if(PSEL_UART)begin
			rPRDATA  =	PRDATA_UART;	 
			rPREADY   =	PREADY_UART;	
			rPSLVERR =	PSLVERR_UART; 
		end
		//else if(PSEL_SCCB)begin//Reserved
		//	//rPRDATA  =	PRDATA_SCCB;	 
		//	//rPREADY   =	PREADY_SCCB;	
		//	//rPSLVERR =	PSLVERR_SCCB; 
		//end
		else if(PSEL_GPIO)begin
			rPRDATA  =	PRDATA_GPIO;	 
			rPREADY   =	PREADY_GPIO;	
			rPSLVERR =	PSLVERR_GPIO; 
		end
		//else if(PSEL_AI)begin
		//	rPRDATA  =	PRDATA_AI;	 
		//	rPREADY   =	PREADY_AI;	
		//	rPSLVERR =	PSLVERR_AI; 
		//end
		else if(PSEL_TIMER)begin
			rPRDATA  =	PRDATA_TIMER;	 
			rPREADY   =	PREADY_TIMER;	
			rPSLVERR =	PSLVERR_TIMER; 
		end
		else begin
			rPRDATA  =	0;	 
			rPREADY   =	0;	
			rPSLVERR =	0; 
		end
	end
    
	// =========================================================
    // AXI -> APB Bridge
    // =========================================================
    APB_v1_0_S00_AXI #(
        .C_S_AXI_DATA_WIDTH (C_S00_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH (C_S00_AXI_ADDR_WIDTH),
        .APB_WIDTH          (APB_WIDTH)
    ) APB_v1_0_S00_AXI_inst (

        // =====================================================
        // AXI
        // =====================================================
        .S_AXI_ACLK     (s00_axi_aclk),
        .S_AXI_ARESETN  (s00_axi_aresetn),

        // Write Address
        .S_AXI_AWADDR   (s00_axi_awaddr),
        .S_AXI_AWPROT   (s00_axi_awprot),
        .S_AXI_AWVALID  (s00_axi_awvalid),
        .S_AXI_AWREADY  (s00_axi_awready),

        // Write Data
        .S_AXI_WDATA    (s00_axi_wdata),
        .S_AXI_WSTRB    (s00_axi_wstrb),
        .S_AXI_WVALID   (s00_axi_wvalid),
        .S_AXI_WREADY   (s00_axi_wready),

        // Write Response
        .S_AXI_BRESP    (s00_axi_bresp),
        .S_AXI_BVALID   (s00_axi_bvalid),
        .S_AXI_BREADY   (s00_axi_bready),

        // Read Address
        .S_AXI_ARADDR   (s00_axi_araddr),
        .S_AXI_ARPROT   (s00_axi_arprot),
        .S_AXI_ARVALID  (s00_axi_arvalid),
        .S_AXI_ARREADY  (s00_axi_arready),

        // Read Data
        .S_AXI_RDATA    (s00_axi_rdata),
        .S_AXI_RRESP    (s00_axi_rresp),
        .S_AXI_RVALID   (s00_axi_rvalid),
        .S_AXI_RREADY   (s00_axi_rready),

        // =====================================================
        // APB
        // =====================================================
        .PADDR          (PADDR),
        .PSEL_UART      (PSEL_UART),
        //.PSEL_SCCB      (PSEL_SCCB),
        .PSEL_SCCB      (),
        .PSEL_GPIO      (PSEL_GPIO),
        //.PSEL_AI        (PSEL_AI),
        .PSEL_TIMER       (PSEL_TIMER),
        .PENABLE        (PENABLE),
        .PWRITE         (PWRITE),
        .PWDATA         (PWDATA),
        .PSTRB          (PSTRB),
        .PPROT          (PPROT),
        .PRDATA         (rPRDATA),
        .PREADY         (rPREADY),
        .PSLVERR        (rPSLVERR)
    );

endmodule
