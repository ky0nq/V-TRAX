`timescale 1ns / 1ps
//
// s2mm_axi_writer (S2MM 쓰기 엔진)
//
// [기존 대비 변경점] 전송 로직(상태 머신, burst, 주소 계산)은 그대로
//   1) base_addr[0:2] 배열 포트 -> base_addr0/1/2 로 분리 (Verilog / IP 패키징 호환)
//   2) 상태 출력 추가
//        busy          : start 이후 1 (순환 모드라 계속 1)
//        frame_done    : 프레임 하나 다 쓸 때마다 1클럭 펄스
//        newest_idx    : 방금 다 쓴 버퍼 번호 (0~2) -> MM2S 의 s2mm_newest_idx 로
//        wr_error      : BRESP 에러(SLVERR/DECERR) 발생 (다음 start 까지 유지)
//        wr_error_addr : 첫 에러 난 burst 의 시작 주소
//
// 동작 : start 후 base0 -> base1 -> base2 -> base0 ... 순서로 프레임을 계속 씀 (순환)
//
module s2mm_axi_writer #(
    parameter ID_WIDTH        = 4,
    parameter ADDR_WIDTH      = 32,
    parameter DATA_WIDTH      = 32,
    parameter FIFO_DEPTH      = 1024,
    parameter BURST_LEN       = 16,
    parameter PIXELS_PER_LINE = 1280,
    parameter FRAME_HEIGHT    = 720,
    parameter PIXEL_BYTES     = 3
)(
    input  wire                              ACLK,
    input  wire                              ARESETN,
    input  wire                              start,
    input  wire [ADDR_WIDTH-1:0]             base_addr0,
    input  wire [ADDR_WIDTH-1:0]             base_addr1,
    input  wire [ADDR_WIDTH-1:0]             base_addr2,

    // 상태 출력 (추가)
    output reg                               busy,
    output reg                               frame_done,
    output reg  [2:0]                        newest_idx,
    output reg                               wr_error,
    output reg  [ADDR_WIDTH-1:0]             wr_error_addr,

    input  wire                              fifo_empty,
    input  wire [$clog2(FIFO_DEPTH+1)-1:0]   fifo_count,
    input  wire [DATA_WIDTH-1:0]             fifo_rd_data,
    output wire                              fifo_rd_en,
    output wire [ID_WIDTH-1:0]               M_AXI_AWID,
    output wire [ADDR_WIDTH-1:0]             M_AXI_AWADDR,
    output wire [3:0]                        M_AXI_AWLEN,
    output wire [2:0]                        M_AXI_AWSIZE,
    output wire [1:0]                        M_AXI_AWBURST,
    output wire [1:0]                        M_AXI_AWLOCK,
    output wire [3:0]                        M_AXI_AWCACHE,
    output wire [2:0]                        M_AXI_AWPROT,
    output wire                              M_AXI_AWVALID,
    input  wire                              M_AXI_AWREADY,
    output wire [ID_WIDTH-1:0]               M_AXI_WID,
    output wire [DATA_WIDTH-1:0]             M_AXI_WDATA,
    output wire [(DATA_WIDTH/8)-1:0]         M_AXI_WSTRB,
    output wire                              M_AXI_WLAST,
    output wire                              M_AXI_WVALID,
    input  wire                              M_AXI_WREADY,
    input  wire [ID_WIDTH-1:0]               M_AXI_BID,
    input  wire [1:0]                        M_AXI_BRESP,
    input  wire                              M_AXI_BVALID,
    output wire                              M_AXI_BREADY
);

    localparam IDLE      = 3'd0;
    localparam WAIT_FIFO = 3'd1;
    localparam AW        = 3'd2;
    localparam PREFETCH  = 3'd3;
    localparam WRITE     = 3'd4;
    localparam BRESP     = 3'd5;

    localparam integer BYTE_WIDTH        = DATA_WIDTH / 8;
    localparam integer BURST_BYTES       = BURST_LEN * BYTE_WIDTH;
    localparam integer LINE_BYTES        = PIXELS_PER_LINE * PIXEL_BYTES;
    localparam integer BURSTS_PER_LINE   = LINE_BYTES / BURST_BYTES;
    localparam integer BEAT_WIDTH        = (BURST_LEN <= 1) ? 1 : $clog2(BURST_LEN);
    localparam integer BURST_COUNT_WIDTH = (BURSTS_PER_LINE <= 1) ? 1 : $clog2(BURSTS_PER_LINE);
    localparam integer LINE_COUNT_WIDTH  = (FRAME_HEIGHT <= 1) ? 1 : $clog2(FRAME_HEIGHT);

    reg [2:0] STATE;
    reg [ADDR_WIDTH-1:0] current_addr;
    reg [BEAT_WIDTH-1:0] beat_count;
    reg [BURST_COUNT_WIDTH-1:0] burst_count;
    reg [LINE_COUNT_WIDTH-1:0] line_count;
    reg [1:0] base_cnt;     // 다음에 쓸 버퍼 번호
    reg [1:0] wr_idx;       // 지금 쓰고 있는 버퍼 번호 (추가)

    // base_cnt 가 가리키는 버퍼 주소 (배열 대신 선택기)
    wire [ADDR_WIDTH-1:0] base_sel = (base_cnt == 2'd0) ? base_addr0 :
                                     (base_cnt == 2'd1) ? base_addr1 :
                                                          base_addr2;

    wire aw_handshake;
    wire w_handshake;
    wire b_handshake;

    assign aw_handshake = M_AXI_AWVALID && M_AXI_AWREADY;
    assign w_handshake  = M_AXI_WVALID && M_AXI_WREADY;
    assign b_handshake  = M_AXI_BVALID && M_AXI_BREADY;

    assign M_AXI_AWID    = {ID_WIDTH{1'b0}};
    assign M_AXI_AWADDR  = current_addr;
    assign M_AXI_AWLEN   = BURST_LEN - 1;
    assign M_AXI_AWSIZE  = $clog2(BYTE_WIDTH);
    assign M_AXI_AWBURST = 2'b01;
    assign M_AXI_AWLOCK  = 2'b00;
    assign M_AXI_AWCACHE = 4'b0011;
    assign M_AXI_AWPROT  = 3'b000;
    assign M_AXI_AWVALID = (STATE == AW);

    assign M_AXI_WID    = {ID_WIDTH{1'b0}};
    assign M_AXI_WDATA  = fifo_rd_data;
    assign M_AXI_WSTRB  = {BYTE_WIDTH{1'b1}};
    assign M_AXI_WVALID = (STATE == WRITE);
    assign M_AXI_WLAST  = (STATE == WRITE) && (beat_count == BURST_LEN-1);

    assign M_AXI_BREADY = (STATE == BRESP);

    assign fifo_rd_en =
        ((STATE == PREFETCH) && !fifo_empty) ||
        ((STATE == WRITE) && w_handshake && (beat_count != BURST_LEN-1));

    always @(posedge ACLK) begin
        if (!ARESETN) begin
            STATE         <= IDLE;
            current_addr  <= 0;
            beat_count    <= 0;
            burst_count   <= 0;
            line_count    <= 0;
            base_cnt      <= 0;
            wr_idx        <= 0;
            busy          <= 1'b0;
            frame_done    <= 1'b0;
            newest_idx    <= 3'd0;
            wr_error      <= 1'b0;
            wr_error_addr <= {ADDR_WIDTH{1'b0}};
        end else begin
            frame_done <= 1'b0;   // 1클럭 펄스

            case (STATE)
                IDLE : begin
                    beat_count <= 0;
                    if (start) begin
                        current_addr  <= base_sel;
                        wr_idx        <= base_cnt;
                        base_cnt      <= (base_cnt == 2'd2) ? 2'd0 : base_cnt + 1'b1;
                        beat_count    <= 0;
                        burst_count   <= 0;
                        line_count    <= 0;
                        busy          <= 1'b1;
                        wr_error      <= 1'b0;
                        wr_error_addr <= {ADDR_WIDTH{1'b0}};
                        STATE         <= WAIT_FIFO;
                    end
                end

                WAIT_FIFO : begin
                    if (fifo_count >= BURST_LEN) STATE <= AW;
                end

                AW : begin
                    if (aw_handshake) begin
                        beat_count <= 0;
                        STATE      <= PREFETCH;
                    end
                end

                PREFETCH : begin
                    if (!fifo_empty) STATE <= WRITE;
                end

                WRITE : begin
                    if (w_handshake) begin
                        if (beat_count == BURST_LEN-1) begin
                            beat_count <= 0;
                            STATE      <= BRESP;
                        end else begin
                            beat_count <= beat_count + 1'b1;
                        end
                    end
                end

                BRESP : begin
                    if (b_handshake) begin
                        // 에러 응답이면 첫 번째만 기록 (current_addr = 방금 burst 시작 주소)
                        if (M_AXI_BRESP[1] && !wr_error) begin
                            wr_error      <= 1'b1;
                            wr_error_addr <= current_addr;
                        end

                        if (burst_count == BURSTS_PER_LINE-1) begin
                            burst_count <= 0;
                            if (line_count == FRAME_HEIGHT-1) begin
                                // ---- 프레임 완료 ----
                                line_count   <= 0;
                                frame_done   <= 1'b1;
                                newest_idx   <= {1'b0, wr_idx};      // 방금 다 쓴 버퍼
                                current_addr <= base_sel;            // 다음 버퍼로
                                wr_idx       <= base_cnt;
                                base_cnt     <= (base_cnt == 2'd2) ? 2'd0 : base_cnt + 1'b1;
                            end else begin
                                line_count   <= line_count + 1'b1;
                                current_addr <= current_addr + BURST_BYTES;
                            end
                        end else begin
                            burst_count  <= burst_count + 1'b1;
                            current_addr <= current_addr + BURST_BYTES;
                        end

                        STATE <= WAIT_FIFO;
                    end
                end

                default : begin
                    STATE <= IDLE;
                end
            endcase
        end
    end

    wire _unused_bid = |M_AXI_BID;

endmodule
