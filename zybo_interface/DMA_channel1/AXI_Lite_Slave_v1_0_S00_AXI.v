`timescale 1 ns / 1 ps

// ============================================================================
//  DMA MM2S Register Map (AXI-Lite Slave)
//
//  0x00 CR        [2]  ABORT       : 1 쓰면 abort_pulse 1클럭 (저장 안 됨, 읽으면 0)
//                 [4]  CYCLIC      : 1 = 프레임 끝나면 자동으로 다음 프레임 (CPU 개입 없음)
//                                    0 으로 바꾸면 현재 프레임까지만 하고 멈춤
//                 [5]  LIVE        : 0 = SA 주소 하나만 반복 (로딩화면 / 필터 결과 고정)
//                                    1 = FB_BASE + newest_idx x BTT (카메라 라이브)
//                                    프레임 경계에서 자동 반영
//                 [6]  IDX_SW      : 0 = newest_idx 를 S2MM 하드웨어 신호에서
//                                    1 = newest_idx 를 SW_IDX 레지스터(0x3C)에서
//                 [12] IOC_IrqEn   : 프레임 완료 인터럽트 enable
//                 [14] Err_IrqEn   : 에러 인터럽트 enable
//  0x04 SR        [0]  busy        (RO)
//                 [1]  idle        (RO)
//                 [4]  read_error  (RO)
//                 [10:8] cur_buf   (RO) 지금 읽고 있는 버퍼 번호 (LIVE 모드일 때 의미)
//                 [12] IOC_Irq     (W1C) 프레임 끝날 때마다 세트
//                 [14] Err_Irq     (W1C) error 0->1 에서 세트
//  0x18 SA        소스 주소 (LIVE=0 일 때 읽을 주소, 로딩화면 = 0x4400_0000)
//  0x1C READ_ERR  에러 난 beat 주소 (RO, 32bit)
//  0x20 DA        (MM2S 미사용, 호환용)
//  0x28 BTT       프레임 바이트 수 (= 버퍼 간격), 쓰면 start_pulse
//  0x30 BURST_CFG [7:0] ARLEN, [9:8] burst type (00 FIXED, 01 INCR)
//  0x34 FB_BASE   DDR 프레임 버퍼 0번 주소
//  0x38 NUM_BUF   [3:0] 프레임 버퍼 개수 (1~8)
//  0x3C SW_IDX    [2:0] IDX_SW=1 일 때 CPU가 쓰는 최신 완료 버퍼 번호
//
//  irq = (IOC_Irq & IOC_IrqEn) | (Err_Irq & Err_IrqEn)
// ============================================================================
module AXI_Lite_Slave_v1_0_S00_AXI #
(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 6
)
(
    // ============ User Ports =============
    output reg  [31:0] SA_reg,
    output reg  [31:0] DA_reg,
    output reg  [31:0] BTT_reg,
    output reg  [31:0] CDMACR_reg,
    output reg  [9:0]  BURST_CFG_reg,
    output reg  [31:0] FB_BASE_reg,
    output reg  [3:0]  NUM_BUF_reg,
    output reg  [2:0]  SW_IDX_reg,
    output reg         start_pulse,
    output reg         abort_pulse,
    output wire        irq,

    input  wire        dma_busy,
    input  wire        frame_done,          // 매 프레임 끝 1클럭 펄스
    input  wire        read_error,
    input  wire [31:0] read_error_addr,
    input  wire [2:0]  cur_buf_idx,
    // =====================================

    input  wire                                S_AXI_ACLK,
    input  wire                                S_AXI_ARESETN,
    input  wire [C_S_AXI_ADDR_WIDTH-1 : 0]     S_AXI_AWADDR,
    input  wire [2 : 0]                        S_AXI_AWPROT,
    input  wire                                S_AXI_AWVALID,
    output wire                                S_AXI_AWREADY,
    input  wire [C_S_AXI_DATA_WIDTH-1 : 0]     S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
    input  wire                                S_AXI_WVALID,
    output wire                                S_AXI_WREADY,
    output wire [1 : 0]                        S_AXI_BRESP,
    output wire                                S_AXI_BVALID,
    input  wire                                S_AXI_BREADY,
    input  wire [C_S_AXI_ADDR_WIDTH-1 : 0]     S_AXI_ARADDR,
    input  wire [2 : 0]                        S_AXI_ARPROT,
    input  wire                                S_AXI_ARVALID,
    output wire                                S_AXI_ARREADY,
    output wire [C_S_AXI_DATA_WIDTH-1 : 0]     S_AXI_RDATA,
    output wire [1 : 0]                        S_AXI_RRESP,
    output wire                                S_AXI_RVALID,
    input  wire                                S_AXI_RREADY
);

    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
    reg                            axi_awready;
    reg                            axi_wready;
    reg [1 : 0]                    axi_bresp;
    reg                            axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
    reg                            axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;
    reg [1 : 0]                    axi_rresp;
    reg                            axi_rvalid;

    // ======== Register Map =========
    localparam ADDR_CDMACR    = 6'h00;
    localparam ADDR_CDMASR    = 6'h04;
    localparam ADDR_SA        = 6'h18;
    localparam ADDR_READ_ERR  = 6'h1C;
    localparam ADDR_DA        = 6'h20;
    localparam ADDR_BTT       = 6'h28;
    localparam ADDR_BURST_CFG = 6'h30;
    localparam ADDR_FB_BASE   = 6'h34;
    localparam ADDR_NUM_BUF   = 6'h38;
    localparam ADDR_SW_IDX    = 6'h3C;

    localparam CR_ABORT     = 2;
    localparam CR_IOC_IRQEN = 12;
    localparam CR_ERR_IRQEN = 14;
    localparam SR_IOC_IRQ   = 12;
    localparam SR_ERR_IRQ   = 14;
    // ===============================

    reg [C_S_AXI_DATA_WIDTH-1:0] reg_data_out;
    reg                          aw_en;
    wire                         slv_reg_rden;
    wire                         slv_reg_wren;

    assign slv_reg_wren = S_AXI_WREADY && S_AXI_WVALID && S_AXI_AWREADY && S_AXI_AWVALID;
    assign slv_reg_rden = S_AXI_ARREADY & S_AXI_ARVALID & ~S_AXI_RVALID;

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    //======================================================
    // AWREADY / AWADDR / WREADY
    //======================================================
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awready <= 1'b0;
            aw_en       <= 1'b1;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                axi_awready <= 1'b1;
                aw_en       <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                aw_en       <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0)
            axi_awaddr <= 0;
        else if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en)
            axi_awaddr <= S_AXI_AWADDR;
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0)
            axi_wready <= 1'b0;
        else if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en)
            axi_wready <= 1'b1;
        else
            axi_wready <= 1'b0;
    end

    //======================================================
    // Register Write
    //======================================================
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            SA_reg        <= 32'd0;
            DA_reg        <= 32'd0;
            BTT_reg       <= 32'd0;
            CDMACR_reg    <= 32'd0;
            BURST_CFG_reg <= 10'b01_0000_1111;   // 기본값: INCR, 16 beat
            FB_BASE_reg   <= 32'd0;
            NUM_BUF_reg   <= 4'd3;
            SW_IDX_reg    <= 3'd0;
            start_pulse   <= 1'b0;
            abort_pulse   <= 1'b0;
        end else begin
            start_pulse <= 1'b0;
            abort_pulse <= 1'b0;
            if (slv_reg_wren) begin
                case (axi_awaddr)
                    ADDR_CDMACR: begin
                        CDMACR_reg           <= S_AXI_WDATA;
                        CDMACR_reg[CR_ABORT] <= 1'b0;                  // self-clear
                        abort_pulse          <= S_AXI_WDATA[CR_ABORT];
                    end
                    ADDR_SA:        SA_reg        <= S_AXI_WDATA;
                    ADDR_DA:        DA_reg        <= S_AXI_WDATA;
                    ADDR_BURST_CFG: BURST_CFG_reg <= S_AXI_WDATA[9:0];
                    ADDR_FB_BASE:   FB_BASE_reg   <= S_AXI_WDATA;
                    ADDR_NUM_BUF:   NUM_BUF_reg   <= S_AXI_WDATA[3:0];
                    ADDR_SW_IDX:    SW_IDX_reg    <= S_AXI_WDATA[2:0];
                    ADDR_BTT: begin
                        BTT_reg     <= S_AXI_WDATA;
                        start_pulse <= 1'b1;
                    end
                    default: ;
                endcase
            end
        end
    end

    //======================================================
    // Interrupt status (W1C) - 세트 우선
    //======================================================
    reg read_error_q;
    reg ioc_irq, err_irq;

    wire error_rise = read_error & ~read_error_q;
    wire sr_wr      = slv_reg_wren && (axi_awaddr == ADDR_CDMASR);

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            read_error_q <= 1'b0;
            ioc_irq      <= 1'b0;
            err_irq      <= 1'b0;
        end else begin
            read_error_q <= read_error;

            if (frame_done)
                ioc_irq <= 1'b1;
            else if (sr_wr && S_AXI_WDATA[SR_IOC_IRQ])
                ioc_irq <= 1'b0;

            if (error_rise)
                err_irq <= 1'b1;
            else if (sr_wr && S_AXI_WDATA[SR_ERR_IRQ])
                err_irq <= 1'b0;
        end
    end

    assign irq = (ioc_irq & CDMACR_reg[CR_IOC_IRQEN]) |
                 (err_irq & CDMACR_reg[CR_ERR_IRQEN]);

    wire [31:0] sr_value = {17'd0,
                            err_irq,        // [14]
                            1'b0,           // [13]
                            ioc_irq,        // [12]
                            1'b0,           // [11]
                            cur_buf_idx,    // [10:8]
                            3'd0,           // [7:5]
                            read_error,     // [4]
                            2'd0,           // [3:2]
                            ~dma_busy,      // [1] idle
                            dma_busy};      // [0] busy

    //======================================================
    // BVALID, BRESP
    //======================================================
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_bvalid <= 1'b0;
            axi_bresp  <= 2'b0;
        end else begin
            if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID) begin
                axi_bvalid <= 1'b1;
                axi_bresp  <= 2'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                axi_bvalid <= 1'b0;
            end
        end
    end

    //======================================================
    // ARREADY, ARADDR
    //======================================================
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_arready <= 1'b0;
            axi_araddr  <= 0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr  <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    //======================================================
    // RVALID, RRESP
    //======================================================
    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_rvalid <= 1'b0;
            axi_rresp  <= 2'b0;
        end else begin
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp  <= 2'b0;
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

    //======================================================
    // Read mux
    //======================================================
    always @(*) begin
        case (axi_araddr)
            ADDR_CDMACR:    reg_data_out = CDMACR_reg;
            ADDR_CDMASR:    reg_data_out = sr_value;
            ADDR_SA:        reg_data_out = SA_reg;
            ADDR_READ_ERR:  reg_data_out = read_error_addr;
            ADDR_DA:        reg_data_out = DA_reg;
            ADDR_BTT:       reg_data_out = BTT_reg;
            ADDR_BURST_CFG: reg_data_out = {22'd0, BURST_CFG_reg};
            ADDR_FB_BASE:   reg_data_out = FB_BASE_reg;
            ADDR_NUM_BUF:   reg_data_out = {28'd0, NUM_BUF_reg};
            ADDR_SW_IDX:    reg_data_out = {29'd0, SW_IDX_reg};
            default:        reg_data_out = 32'd0;
        endcase
    end

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0)
            axi_rdata <= 0;
        else if (slv_reg_rden)
            axi_rdata <= reg_data_out;
    end

endmodule