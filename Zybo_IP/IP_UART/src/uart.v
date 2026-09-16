`timescale 1 ns / 1 ps


// UART 최상위 모듈: TX와 RX 모듈을 통합한 단순 래퍼.
// 기본 설정: 100 MHz 클럭, 115200 baud, 8N1 포맷

module uart #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    // TX 인터페이스
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    output wire       tx,
    // RX 인터페이스
    input  wire       rx,
    output wire [7:0] rx_data,
    output wire       rx_valid
);

    uart_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .data_in  (tx_data),
        .valid    (tx_valid),
        .ready    (tx_ready),
        .tx       (tx)
    );

    uart_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_rx (
        .clk      (clk),
        .rst_n    (rst_n),
        .rx       (rx),
        .data_out (rx_data),
        .valid    (rx_valid)
    );

endmodule



// UART 수신기 (8N1: 데이터 8비트, 패리티 없음, 정지비트 1비트)

module uart_rx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,         // 직렬 입력 라인
    output reg  [7:0] data_out,   // 수신된 바이트
    output reg        valid       // 수신 완료 펄스 (1클럭 high)
);

    localparam CLKS_PER_BIT  = CLK_FREQ / BAUD_RATE;
    localparam HALF_BIT      = CLKS_PER_BIT / 2;

    // 수신 FSM 상태
    localparam S_IDLE  = 2'd0; // 대기 (rx falling edge 감지)
    localparam S_START = 2'd1; // 시작비트 중앙까지 대기 후 재확인
    localparam S_DATA  = 2'd2; // 데이터 8비트 샘플링
    localparam S_STOP  = 2'd3; // 정지비트 통과

    reg [1:0]                    state;
    reg [$clog2(CLKS_PER_BIT):0] clk_cnt;
    reg [2:0]                    bit_idx;
    reg [7:0]                    shift_reg;

    // rx를 clk 도메인으로 가져오는 2단 동기화기 (메타스태빌리티 방지)
    reg rx_sync0, rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync0 <= 1'b1;
            rx_sync  <= 1'b1;
        end else begin
            rx_sync0 <= rx;
            rx_sync  <= rx_sync0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 8'h00;
            data_out  <= 8'h00;
            valid     <= 1'b0;
        end else begin
            valid <= 1'b0; // 기본값: 디어서트

            case (state)
                S_IDLE: begin
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    if (rx_sync == 1'b0) // falling edge → 시작비트 후보
                        state <= S_START;
                end

                // 시작비트 중앙 시점까지 대기
                S_START: begin
                    if (clk_cnt == HALF_BIT - 1) begin
                        clk_cnt <= 0;
                        if (rx_sync == 1'b0) // 여전히 low → 유효한 시작비트
                            state <= S_DATA;
                        else
                            state <= S_IDLE; // 노이즈로 판단, 폐기
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // 매 비트 길이마다 비트 중앙에서 샘플링
                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt              <= 0;
                        shift_reg[bit_idx]   <= rx_sync; // LSB부터 채움
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // 정지비트 한 비트 길이 대기 후 데이터 출력
                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt  <= 0;
                        data_out <= shift_reg;
                        valid    <= 1'b1; // 1클럭 펄스
                        state    <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule


// UART 송신기 (8N1: 데이터 8비트, 패리티 없음, 정지비트 1비트)

module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_in,   // 송신할 바이트
    input  wire       valid,     // 전송 시작 펄스 (1클럭 high)
    output reg        ready,     // idle 상태 (다음 데이터 수락 가능)
    output reg        tx         // 직렬 출력 라인
);

    // 한 비트 동안 카운트해야 하는 클럭 수
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // 송신 FSM 상태
    localparam S_IDLE  = 2'd0; // 대기
    localparam S_START = 2'd1; // 시작비트(0) 출력
    localparam S_DATA  = 2'd2; // 데이터 8비트 출력
    localparam S_STOP  = 2'd3; // 정지비트(1) 출력

    reg [1:0]                    state;
    reg [$clog2(CLKS_PER_BIT):0] clk_cnt;   // 비트 길이 카운터
    reg [2:0]                    bit_idx;   // 현재 송신 중인 비트 인덱스
    reg [7:0]                    shift_reg; // 송신 데이터 보관

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 8'h00;
            tx        <= 1'b1; // idle 상태에서 라인은 high
            ready     <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    tx    <= 1'b1;
                    ready <= 1'b1;
                    if (valid) begin
                        // 데이터 캡처 후 시작비트로 진입
                        shift_reg <= data_in;
                        clk_cnt   <= 0;
                        ready     <= 1'b0;
                        state     <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0; // 시작비트
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    tx <= shift_reg[bit_idx]; // LSB부터 차례로 전송
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_STOP: begin
                    tx <= 1'b1; // 정지비트
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
