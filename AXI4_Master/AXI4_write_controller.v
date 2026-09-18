//=====================================================================
// axi4_write_controller.v
//
// Write Engine의 Controller — IDLE/DATA 2-state FSM.
//
// ★ 이번 수정 사항 ★
// 1) error_addr 폭을 32bit 고정 -> ADDR_WIDTH로 파라미터화
//    (Read 쪽 read_control과 동일하게 맞춤. 레지스터맵 ERROR(0x1C)에
//     [29:15] read addr / [14:0] write addr 로 합쳐 넣을 때 폭이 맞아야 함)
// 2) region_err(Write Datapath가 목적지 주소를 3슬레이브 중 어디에도
//    못 찾았을 때 세우는 신호) 입력을 받아서 에러 처리에 포함
//=====================================================================
module AXI4_write_controller #(
    parameter ADDR_WIDTH = 15
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---- Register Map I/F ----
    input  wire                    start,
    output reg                     busy,
    output reg                     done,
    output reg                     error,
    output reg  [ADDR_WIDTH-1:0]   error_addr,   // ★ 32 -> ADDR_WIDTH

    // ---- Datapath 상태 입력 ----
    input  wire                    b_hs,
    input  wire                    xfer_done,
    input  wire [1:0]              bresp,
    input  wire [ADDR_WIDTH-1:0]   awaddr,   // 에러 시점 주소 래치용
    input  wire                    region_err,  // ★ 신규: 3슬레이브 중 매칭 안됨

    // ---- Datapath 제어 출력 ----
    output wire                    en,       // state == S_DATA
    output reg                     init      // IDLE -> DATA 진입 pulse
);

    localparam S_IDLE = 1'b0;
    localparam S_DATA = 1'b1;

    reg state;
    assign en = (state == S_DATA);

    //-----------------------------------------------------------
    // state register
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            init  <= 1'b0;
        end else begin
            init <= 1'b0; // 기본 1-cycle pulse
            case (state)
                S_IDLE: if (start) begin
                    state <= S_DATA;
                    init  <= 1'b1;
                end
                S_DATA: if (xfer_done) state <= S_IDLE;
            endcase
        end
    end

    //-----------------------------------------------------------
    // status / error (STATUS 레지스터로 반영)
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            error_addr <= {ADDR_WIDTH{1'b0}};
        end else if (state == S_IDLE) begin
            if (start) begin
                busy  <= 1'b1;
                done  <= 1'b0;
                error <= 1'b0;
            end
        end else begin // S_DATA
            // ★ 변경: BRESP 에러 뿐 아니라 region_err(3슬레이브 매칭 실패)도 에러로 취급
            if (((b_hs && (bresp != 2'b00)) || region_err) && !error) begin
                error      <= 1'b1;
                error_addr <= awaddr;   // 폭이 이제 ADDR_WIDTH로 일치하므로 패딩 불필요
            end
            if (xfer_done) begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end

endmodule