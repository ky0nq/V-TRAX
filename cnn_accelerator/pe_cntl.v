`timescale 1ns / 1ps
`default_nettype none

// -----------------------------------------------------------------------------
// PE array 제어 유닛
//
// 기준 문서 : 통합 인터페이스 시트 tab gid=457801884 (2026-09-19) 의 PE_CNTL 절.
//
// 명령 하나가 타일 하나를 담당한다. 타일 = 출력위치 3개(행) x 출력채널 3개(열)
// 를 레이어 전체 내적 길이 K(1~4096) 만큼 누산하는 작업이다.
// Weight Buffer 는 WBUF_WORDS(64) word 뿐이라, K 가 길면 chunk 로 나눠 돈다.
//
//   P_IDLE -> P_CLEAR -> P_PREFILL -> P_FEED -+-> P_DRAIN -> P_DONE -> P_IDLE
//                            ^                |
//                            +- P_CHUNK_WAIT <+
//
// chunk 경계에서 지켜야 하는 것
//   - array / skew / 누산기를 지우지 않고 그대로 정지시킨다 (o_step_en = 0)
//   - o_acc_clear, o_tile_clear 는 P_CLEAR 에서만 나간다
//   - o_mac_last 는 "전체 K 의 마지막 MAC" 에만 뜬다. chunk 의 마지막이 아니다.
//     (여기서 잘못 뜨면 미완성 누산값이 Output FIFO 로 flush 된다)
//
// 다음 chunk 는 o_chunk_req_* 로 cnn_cntl 에 요청하고 i_chunk_done_valid 로
// 승인받는다. o_wbuf_free 는 Weight Loader 에게 버퍼를 덮어써도 되는 시점을
// 알려 준다 (읽기 미처리 없음 + Feeder 비어 있음).
// -----------------------------------------------------------------------------
module pe_cntl #(
    parameter integer WBUF_WORDS = 64          // Weight Buffer 깊이 (word)
) (
    // ---- 02절 공통 규약 ----------------------------------------------------
    input  wire        clk,
    input  wire        rst_n,

    // ---- CNN_CNTL 의 타일 명령 --------------------------------------------
    input  wire        i_cmd_valid,
    output wire        o_cmd_ready,
    input  wire [12:0] i_cmd_k_total,      // 전체 내적 길이 K
    input  wire [2:0]  i_cmd_row_mask,
    input  wire [2:0]  i_cmd_col_mask,

    // ---- 타일 도중 weight chunk 요청 --------------------------------------
    output wire        o_chunk_req_valid,
    input  wire        i_chunk_req_ready,
    output wire [12:0] o_chunk_k_base,     // 다음 chunk 의 전역 k 시작
    output wire [6:0]  o_chunk_len,        // min(64, K - k_base)
    input  wire        i_chunk_done_valid,
    output wire        o_chunk_done_ready,

    // ---- Weight Loader ----------------------------------------------------
    output wire        o_wbuf_free,

    // ---- Weight Addr Gen (버퍼 리더) --------------------------------------
    output wire        o_reader_start,
    output wire [6:0]  o_reader_chunk_len,
    output wire        o_reader_issue_en,
    input  wire        i_reader_done,
    input  wire        i_reader_idle,

    // ---- Weight Feeder / PE Data Feeder -----------------------------------
    output wire [2:0]  o_col_mask,
    output wire        o_feeder_en,
    output wire        o_tile_clear,
    input  wire        i_weight_feeder_empty,
    input  wire        i_weight_valid,
    input  wire        i_act_valid,

    // ---- Skew / PE Array --------------------------------------------------
    output wire        o_step_en,
    output wire        o_feed_valid,
    output wire        o_acc_clear,
    output wire [8:0]  o_mac_valid,
    output wire [8:0]  o_mac_last,

    // ---- Output FIFO ------------------------------------------------------
    output wire        o_result_buf_clear,
    input  wire        i_result_space_ready,

    // ---- Result + IRQ -----------------------------------------------------
    input  wire        i_tile_done
);

    localparam [2:0] P_IDLE       = 3'd0;
    localparam [2:0] P_CLEAR      = 3'd1;
    localparam [2:0] P_PREFILL    = 3'd2;
    localparam [2:0] P_FEED       = 3'd3;
    localparam [2:0] P_CHUNK_WAIT = 3'd4;
    localparam [2:0] P_DRAIN      = 3'd5;
    localparam [2:0] P_DONE       = 3'd6;

    localparam [12:0] WBUF_LEN = WBUF_WORDS;

    reg [2:0]  state;

    reg [12:0] k_total;        // 이 타일의 전체 내적 길이
    reg [12:0] k_base;         // 현재 chunk 가 시작하는 전역 k
    reg [6:0]  k_cnt;          // 현재 chunk 안에서 주입한 beat 수
    reg [6:0]  cur_len;        // 현재 chunk 길이

    reg [2:0]  row_mask_q;
    reg [2:0]  col_mask_q;

    reg        req_sent;       // chunk 요청이 수락됨
    reg        reader_done_seen;

    // 3x3 systolic array 는 대각선 지연이 0~4 이다.
    reg [4:0]  valid_pipe;
    reg [4:0]  last_pipe;

    // -------------------------------------------------------------------------
    // chunk 계산
    // -------------------------------------------------------------------------
    wire [12:0] k_remain = k_total - k_base;
    wire [6:0]  next_len = (k_remain > WBUF_LEN) ? WBUF_LEN[6:0] : k_remain[6:0];

    // K=0 인 잘못된 명령은 1 로 clamp
    wire [12:0] first_len_full = (i_cmd_k_total == 13'd0) ? 13'd1 : i_cmd_k_total;
    wire [6:0]  first_len = (first_len_full > WBUF_LEN) ? WBUF_LEN[6:0]
                                                        : first_len_full[6:0];

    wire [12:0] k_next = k_base + {6'd0, cur_len};

    // activation 과 weight 가 같이 준비되고 결과 받을 자리가 있을 때만 주입
    wire inject = (state == P_FEED) && i_act_valid && i_weight_valid &&
                  i_result_space_ready;

    wire beat_chunk_last  = (k_cnt == (cur_len - 7'd1));
    wire beat_global_last = beat_chunk_last && (k_next == k_total);

    wire drain_advance = (state == P_DRAIN) && i_result_space_ready;

    // -------------------------------------------------------------------------
    // 출력
    // -------------------------------------------------------------------------
    assign o_cmd_ready = (state == P_IDLE);

    assign o_chunk_req_valid  = (state == P_CHUNK_WAIT) && !req_sent &&
                                reader_done_seen;
    assign o_chunk_k_base     = k_base;
    assign o_chunk_len        = next_len;
    assign o_chunk_done_ready = (state == P_CHUNK_WAIT) && req_sent;

    // 미처리 읽기가 없고 Feeder 에 미전달 weight 도 없어야 덮어쓰기 허용.
    assign o_wbuf_free = (state == P_IDLE) ||
                         ((state == P_CHUNK_WAIT) && i_reader_idle &&
                          i_weight_feeder_empty);

    assign o_reader_start     = (state == P_PREFILL);   // P_PREFILL 은 1 clk
    assign o_reader_chunk_len = cur_len;
    assign o_reader_issue_en  = (state == P_PREFILL) || (state == P_FEED);

    assign o_col_mask  = col_mask_q;
    assign o_feeder_en = (state == P_PREFILL) || (state == P_FEED);

    // 타일 단위 초기화만. chunk 경계에서는 절대 나가면 안 된다.
    assign o_tile_clear       = (state == P_CLEAR);
    assign o_acc_clear        = (state == P_CLEAR);
    assign o_result_buf_clear = (state == P_CLEAR);

    assign o_step_en    = inject || drain_advance;
    assign o_feed_valid = inject;

    // PE 번호 n = 3*행 + 열. 대각선 d = 행 + 열 인 PE 는 pipe[d] 를 본다.
    // o_step_en 이 0 이면 (chunk 대기 포함) valid/last 가 그대로 유지된다.
    assign o_mac_valid[0] = valid_pipe[0] && row_mask_q[0] && col_mask_q[0];
    assign o_mac_valid[1] = valid_pipe[1] && row_mask_q[0] && col_mask_q[1];
    assign o_mac_valid[2] = valid_pipe[2] && row_mask_q[0] && col_mask_q[2];
    assign o_mac_valid[3] = valid_pipe[1] && row_mask_q[1] && col_mask_q[0];
    assign o_mac_valid[4] = valid_pipe[2] && row_mask_q[1] && col_mask_q[1];
    assign o_mac_valid[5] = valid_pipe[3] && row_mask_q[1] && col_mask_q[2];
    assign o_mac_valid[6] = valid_pipe[2] && row_mask_q[2] && col_mask_q[0];
    assign o_mac_valid[7] = valid_pipe[3] && row_mask_q[2] && col_mask_q[1];
    assign o_mac_valid[8] = valid_pipe[4] && row_mask_q[2] && col_mask_q[2];

    assign o_mac_last[0] = last_pipe[0] && row_mask_q[0] && col_mask_q[0];
    assign o_mac_last[1] = last_pipe[1] && row_mask_q[0] && col_mask_q[1];
    assign o_mac_last[2] = last_pipe[2] && row_mask_q[0] && col_mask_q[2];
    assign o_mac_last[3] = last_pipe[1] && row_mask_q[1] && col_mask_q[0];
    assign o_mac_last[4] = last_pipe[2] && row_mask_q[1] && col_mask_q[1];
    assign o_mac_last[5] = last_pipe[3] && row_mask_q[1] && col_mask_q[2];
    assign o_mac_last[6] = last_pipe[2] && row_mask_q[2] && col_mask_q[0];
    assign o_mac_last[7] = last_pipe[3] && row_mask_q[2] && col_mask_q[1];
    assign o_mac_last[8] = last_pipe[4] && row_mask_q[2] && col_mask_q[2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= P_IDLE;
            k_total          <= 13'd1;
            k_base           <= 13'd0;
            k_cnt            <= 7'd0;
            cur_len          <= 7'd1;
            row_mask_q       <= 3'b000;
            col_mask_q       <= 3'b000;
            req_sent         <= 1'b0;
            reader_done_seen <= 1'b0;
            valid_pipe       <= 5'b00000;
            last_pipe        <= 5'b00000;
        end
        else begin
            // reader_done 은 1 clk 펄스라 플래그로 기억해 둔다 (시트 지시).
            if (i_reader_done)
                reader_done_seen <= 1'b1;

            case (state)
                P_IDLE: begin
                    valid_pipe <= 5'b00000;
                    last_pipe  <= 5'b00000;
                    k_base     <= 13'd0;
                    k_cnt      <= 7'd0;
                    req_sent   <= 1'b0;

                    if (i_cmd_valid) begin
                        k_total    <= first_len_full;
                        cur_len    <= first_len;
                        row_mask_q <= i_cmd_row_mask;
                        col_mask_q <= i_cmd_col_mask;
                        state      <= P_CLEAR;
                    end
                end

                P_CLEAR: begin
                    valid_pipe <= 5'b00000;
                    last_pipe  <= 5'b00000;
                    k_cnt      <= 7'd0;
                    state      <= P_PREFILL;
                end

                // 1 clk. 적재된 chunk 를 버퍼 주소 0 부터 읽도록 리더를 재시작.
                P_PREFILL: begin
                    reader_done_seen <= i_reader_done;
                    state            <= P_FEED;
                end

                P_FEED: begin
                    if (inject) begin
                        valid_pipe <= {valid_pipe[3:0], 1'b1};
                        last_pipe  <= {last_pipe[3:0], beat_global_last};

                        if (beat_global_last) begin
                            state <= P_DRAIN;
                        end
                        else if (beat_chunk_last) begin
                            k_base   <= k_next;
                            k_cnt    <= 7'd0;
                            req_sent <= 1'b0;
                            state    <= P_CHUNK_WAIT;
                        end
                        else begin
                            k_cnt <= k_cnt + 1'b1;
                        end
                    end
                end

                // array / skew / 누산기 전부 정지. o_step_en 이 0 이라 그대로 유지.
                P_CHUNK_WAIT: begin
                    if (o_chunk_req_valid && i_chunk_req_ready)
                        req_sent <= 1'b1;

                    if (req_sent && i_chunk_done_valid) begin
                        cur_len  <= next_len;
                        req_sent <= 1'b0;
                        state    <= P_PREFILL;
                    end
                end

                // 마지막 입력이 PE(2,2) 까지 전파되도록 배열만 계속 진행.
                P_DRAIN: begin
                    if (drain_advance) begin
                        valid_pipe <= {valid_pipe[3:0], 1'b0};
                        last_pipe  <= {last_pipe[3:0], 1'b0};

                        // 이번 edge 가 마지막으로 남은 pipe 단을 소비한다.
                        if (valid_pipe[3:0] == 4'b0000)
                            state <= P_DONE;
                    end
                end

                // Result+IRQ 가 타일 결과 저장 완료를 알려 줄 때까지 대기.
                P_DONE: begin
                    if (i_tile_done)
                        state <= P_IDLE;
                end

                default: state <= P_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
