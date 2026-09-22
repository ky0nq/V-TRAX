`timescale 1ns / 1ps `default_nettype none
`include "cnn_defs.vh"

// =============================================================================
// pe_cntl : 타일 하나의 cycle 단위 제어기
//
//   P_IDLE -> P_CLEAR -> P_PREFILL -> P_FEED -+-> P_DRAIN -> P_WAIT_OUT -> P_IDLE
//                            ^                |
//                            +- P_CHUNK_WAIT <+      (K > 64 일 때만. fc1)
//
//   P_IDLE       타일 명령을 기다린다. 받으면 K 와 mask 를 저장
//   P_CLEAR      1clk. feeder / skew / 누산기 / 결과 수집기를 지운다
//   P_PREFILL    1clk. reader 에 "이번 chunk 를 0 번지부터 읽어라"
//   P_FEED       act 와 weight 가 둘 다 준비된 clk 마다 한 beat 씩 주입 (inject)
//   P_CHUNK_WAIT wgt_buf (64 Word) 의 chunk 를 다 썼다. 배열을 멈춘 채 다음 chunk 를 기다린다
//   P_DRAIN      마지막 입력이 PE(2,2) 까지 가도록 5 step 더 민다
//   P_DONE       결과가 후처리 경로로 다 넘어갔다는 tile_in_done 을 기다린다
//
// 세는 것은 두 개뿐이다 (둘 다 "남은 수" 라서 1 이면 마지막이다)
//   k_left : 이 타일에 남은 beat 수.   K 로 채우고 inject 마다 -1
//   c_left : 이번 chunk 에 남은 beat 수. min(k_left, 64) 로 채우고 inject 마다 -1
//
// Chunk 경계에서 지켜야 하는 것
//   - 배열 / skew / 누산기를 지우지 않고 그대로 멈춘다 (o_step_en = 0)
//   - clear 세 개는 P_CLEAR 에서만 나간다
//   - o_mac_last 는 "타일 전체 K 의 마지막 MAC" 에만 뜬다. chunk 의 마지막이 아니다
//     (여기서 잘못 뜨면 미완성 누산값이 output_fifo 로 나간다)
//
// 진행 규칙
//   inject     = P_FEED && i_act_valid && i_weight_valid && i_result_space_ready
//   o_step_en  = inject || (P_DRAIN && i_result_space_ready)
//   o_feed_valid = inject
//
// MAC 타이밍
//   PE(r,c) 는 PE(0,0) 보다 r + c step 늦게 같은 k 를 곱한다 (대각선 d = r + c, 0 .. 4).
//   valid_pipe / last_pipe 5 단이 그 대각선을 그대로 나타낸다 : PE(r,c) 는 pipe[r+c] 를 본다.
// =============================================================================
module pe_cntl #(
    parameter integer WBUF_WORDS = 64                     // 64 : wgt_buf 의 Word 수 = chunk 최대 길이. K 가 이보다 크면 chunk 로 나눠 돈다 (fc1 의 4096 -> 64 chunk)
) (
    // -------------------------------------------------------------------------
    // 포트 이름은 인터페이스 명세서 "▶ pe_cntl" 표를 따른다 (2026-09-21 개정본).
    //   ->  / <-   이 포트가 이어지는 곳. "묶음모듈.포트 (그 안의 서브모듈.포트)"
    //   [명세 O] 같은 이름이 있다   [명세 ~] 역할은 같고 이름 / 방식이 다르다   [명세 X] 명세에 없다
    // 명세 표에 있지만 이 RTL 이 쓰지 않아 포트로 두지 않은 것 :
    //   i_layer_cfg_valid / o_layer_cfg_ready / i_layer_mode / i_in_w .. i_act_base / i_layer_start,
    //   i_tile_out_y / i_tile_out_x / i_tile_och_base / i_tile_wgt_addr / i_tile_last,
    //   o_tile_done / o_tile_in_done / o_pe_busy / o_pe_idle / o_pe_err
    //   (pe_cntl 은 K 와 mask 만 있으면 돈다. 좌표 · 주소는 cnn_cntl 이 데이터패스에 직접 준다)
    // -------------------------------------------------------------------------
    input wire clk,
    input wire rst_n,

    // ---- cnn_cntl : 타일 명령 (top_cnn_cntl 안에서 직접 이어진다) -------------
    input  wire                  i_tile_valid,           // <- cnn_cntl.o_tile_valid.     fire = valid && ready 에서 K 와 mask 를 저장
    output wire                  o_tile_ready,           // -> cnn_cntl.i_tile_ready      (P_IDLE 일 때 1).  top_cnn_cntl.o_pe_cmd_ready_dbg 로도 나간다
    input  wire [`K_W-1:0]       i_step_total,           // <- cnn_cntl.o_step_total.     타일당 MAC step 수 = K
    input  wire [`KEEP_W-1:0]    i_tile_row_mask,        // <- cnn_cntl.o_tile_row_mask.  유효 PE 행 (출력 위치)
    input  wire [`KEEP_W-1:0]    i_tile_col_mask,        // <- cnn_cntl.o_tile_col_mask.  유효 PE 열 (출력 채널)

    // ---- cnn_cntl : 타일 중간 Chunk 재적재 (K > 64 일 때만) --------------------
    output wire                  o_chunk_req,            // -> cnn_cntl.i_chunk_req.     Level. 앞 chunk 를 다 썼으니 다음 chunk 를 올려 달라. i_chunk_loaded 까지 유지
    input  wire                  i_chunk_loaded,         // <- cnn_cntl.o_chunk_loaded.  다음 chunk 가 wgt_buf 에 다 들어온 뒤 1clk 펄스 -> P_PREFILL

    // ---- wgt_ld_unit -------------------------------------------------------
    output wire                  o_wbuf_free,            // -> wgt_ld_unit.i_buf_free (팀 결정 2026-09-21. 이 저장소에서는 아직 wgt_path.i_buffer_free).  읽는 중인 chunk 를 덮어쓰지 않게 하는 허가

    // ---- wgt_patch_gen -----------------------------------------------------
    output wire                  o_chunk_start,          // [명세 O] -> wgt_path.i_reader_start     (wgt_patch_gen.i_start).      chunk 마다 1clk (P_PREFILL)
    output wire [`CHUNK_W-1:0]   o_chunk_word_count,     // [명세 ~] -> wgt_path.i_reader_chunk_len (wgt_patch_gen.i_chunk_len).  명세는 cnn_cntl 이 주는 것으로 돼 있다. chunk 를 세는 쪽이 pe_cntl 이라 여기서 낸다
    output wire                  o_reader_issue_en,      // [명세 X] -> wgt_path.i_reader_issue_en  (wgt_patch_gen.i_issue_en)
    input  wire                  i_chunk_done,           // [명세 O] <- wgt_path.o_reader_done      (wgt_patch_gen.o_done).       chunk 의 마지막 Word 가 feeder 로 넘어간 1clk 펄스
    input  wire                  i_reader_idle,          // [명세 X] <- wgt_path.o_reader_idle      (wgt_patch_gen.o_idle)

    // ---- act_feeder / wgt_feeder -------------------------------------------
    //   명세도 valid 를 i_act_valid / i_weight_valid 로 구분한다. o_feed_en 은 명세에 두 행
    //   (act_feeder / wgt_feeder) 이지만 값이 같아 한 포트가 두 feeder 로 간다.
    //   명세 pe 구간의 o_askew_clear / o_wskew_clear 는 여기서 o_tile_clear 한 선이다
    //   (pe_core 의 포트가 i_tile_clear 하나다).
    output wire                  o_feed_en,              // [명세 O] -> act_path.i_feeder_en (act_feeder.i_feed_en), wgt_path.i_feeder_en (wgt_feeder.i_feed_en).  PREFILL / FEED 에서 1
    output wire                  o_tile_clear,           // [명세 ~] -> act_path.i_tile_clear (act_feeder.i_clear), wgt_path.i_tile_clear (wgt_feeder.i_clear), pe_core.i_tile_clear (act_skew / wgt_skew .i_clear).  명세 이름 o_askew_clear / o_wskew_clear
    input  wire                  i_weight_feeder_empty,  // [명세 X] <- wgt_path.o_feeder_empty (wgt_feeder.o_empty)
    input  wire                  i_weight_valid,         // [명세 O] <- wgt_path.o_weight_valid (wgt_feeder.o_valid). 
    input  wire                  i_act_valid,            // [명세 O] <- act_path.o_act_valid    (act_feeder.o_valid). 

    // ---- pe_core -----------------------------------------------------------
    output wire                  o_step_en,              // -> pe_core.i_step_en    (act_skew / wgt_skew / pe_array .i_step_en)
    output wire                  o_feed_valid,           // -> pe_core.i_feed_valid (act_skew / wgt_skew .i_feed_valid)
    output wire o_acc_clear,  // -> pe_core.i_acc_clear  (pe_array.i_acc_clear)
    output wire [`PE_N-1:0]      o_mac_valid,            // -> pe_core.i_mac_valid  (pe_array.i_mac_valid)
    output wire [`PE_N-1:0]      o_mac_last,             // -> pe_core.i_mac_last   (pe_array.i_mac_last)

    // ---- out_path ----------------------------------------------------------
    output wire                  o_result_clear,         // -> out_path.i_result_buf_clear   (output_fifo.i_clear)
    input  wire                  i_result_space_ready,   // <- out_path.o_result_space_ready (output_fifo.o_result_space_ready)
    input  wire                  i_tile_in_done          // <- out_path.o_tile_in_done       (pooling_unit.o_tile_in_done).  cnn_cntl 도 같은 선을 받는다
);
    localparam [2:0] P_IDLE = 3'd0;
    localparam [2:0] P_CLEAR = 3'd1;
    localparam [2:0] P_PREFILL = 3'd2;
    localparam [2:0] P_FEED = 3'd3;
    localparam [2:0] P_CHUNK_WAIT = 3'd4;
    localparam [2:0] P_DRAIN = 3'd5;
    localparam [2:0] P_DONE = 3'd6;

    localparam [`K_W-1:0] K_ONE = 1;  // `define K_W    13
    localparam [`CHUNK_W-1:0] C_ONE = 1;  // `define CHUNK_W 7

    // chunk 하나의 길이 = min(남은 beat 수, WBUF_WORDS = 64).  K = 27 -> 27,  K = 4096 -> 64, 64, ... 64
    function [`CHUNK_W-1:0] chunk_len;
        input [`K_W-1:0] remain;
        begin
            chunk_len = (remain > WBUF_WORDS) ? WBUF_WORDS[`CHUNK_W-1:0] : remain[`CHUNK_W-1:0];
        end
    endfunction

    // =========================================================================
    // 레지스터
    // =========================================================================
    reg [2:0] state;
    reg [`K_W-1:0]     k_left;             // 이 타일에 남은 beat 수 (지금 것 포함). 1 이면 지금 beat 가 타일의 마지막
    reg [`CHUNK_W-1:0] c_left;             // 이번 chunk 에 남은 beat 수.            1 이면 지금 beat 가 chunk 의 마지막
    reg [`CHUNK_W-1:0] cur_len;            // 이번 chunk 의 길이. reader 에 주는 값이라 chunk 동안 바뀌지 않는다
    reg [`KEEP_W-1:0]  row_mask_q;         // 유효 PE 행 (출력 위치). 꼬리 타일은 011 / 001
    reg [`KEEP_W-1:0] col_mask_q;  // 유효 PE 열 (출력 채널)
    reg reader_done_seen;  // i_chunk_done   (1clk 펄스) 을 봤다
    reg tile_in_done_seen;  // i_tile_in_done (1clk 펄스) 을 봤다
    reg [4:0]          valid_pipe;         // 대각선 d = r + c 의 PE 가 보는 "이번 step 에 MAC 한다" 토큰
    reg [4:0]          last_pipe;          // 같은 자리의 "이게 타일의 마지막 MAC 이다" 토큰

    // =========================================================================
    // 조합 신호
    // =========================================================================
    wire inject          = (state == P_FEED)  && i_act_valid && i_weight_valid && i_result_space_ready;
    wire drain_advance = (state == P_DRAIN) && i_result_space_ready;
    wire tile_last_beat = (k_left == K_ONE);
    wire chunk_last_beat = (c_left == C_ONE);
    wire feeding = (state == P_PREFILL) || (state == P_FEED);
    wire clearing = (state == P_CLEAR);

    // K = 0 이 들어오면 k_left 가 1 을 지나치지 못해 멈춘다. 최소 1 로 받는다.
    wire [`K_W-1:0] k_init = (i_step_total == {`K_W{1'b0}}) ? K_ONE : i_step_total;

    // =========================================================================
    // 출력
    // =========================================================================
    assign o_tile_ready = (state == P_IDLE);

    // 앞 chunk 를 다 주입했고 (P_CHUNK_WAIT), reader 도 다 넘겼다고 알려 온 뒤에 요청한다.
    // Level 이라 cnn_cntl 이 늦게 봐도 사라지지 않는다. i_chunk_loaded 를 받으면 P_PREFILL 로 가며 내려간다.
    assign o_chunk_req = (state == P_CHUNK_WAIT) && reader_done_seen;

    // wgt_buf 를 덮어써도 되는 구간 : 타일이 없을 때, 또는 chunk 를 기다리며 reader / feeder 가 비었을 때
    assign o_wbuf_free        = (state == P_IDLE) ||
                                ((state == P_CHUNK_WAIT) && i_reader_idle && i_weight_feeder_empty);

    assign o_chunk_start = (state == P_PREFILL);  // P_PREFILL 은 1clk
    assign o_chunk_word_count = cur_len;
    assign o_reader_issue_en = feeding;
    assign o_feed_en = feeding;

    assign o_tile_clear = clearing;
    assign o_acc_clear = clearing;
    assign o_result_clear = clearing;

    assign o_step_en = inject || drain_advance;
    assign o_feed_valid = inject;

    // PE n = 3 * r + c.  PE(r,c) 는 pipe[r + c] 를 보고, 자기 행 / 열이 유효할 때만 MAC 한다.
    genvar gr, gc;
    generate
        for (gr = 0; gr < `KEEP_W; gr = gr + 1) begin : g_row
            for (gc = 0; gc < `KEEP_W; gc = gc + 1) begin : g_col
                assign o_mac_valid[`KEEP_W*gr + gc] = valid_pipe[gr + gc] && row_mask_q[gr] && col_mask_q[gc];
                assign o_mac_last [`KEEP_W*gr + gc] = last_pipe [gr + gc] && row_mask_q[gr] && col_mask_q[gc];
            end
        end
    endgenerate

    // =========================================================================
    // FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= P_IDLE;
            k_left            <= K_ONE;
            c_left            <= C_ONE;
            cur_len           <= C_ONE;
            row_mask_q        <= {`KEEP_W{1'b0}};
            col_mask_q        <= {`KEEP_W{1'b0}};
            reader_done_seen  <= 1'b0;
            tile_in_done_seen <= 1'b0;
            valid_pipe        <= 5'b00000;
            last_pipe         <= 5'b00000;
        end else begin
            // 두 완료 신호는 1clk 펄스라 플래그로 기억한다.
            //   i_chunk_done   : chunk 의 마지막 Word 가 feeder 로 넘어간 clk. 그 Word 를 주입하기 전에 온다.
            //   i_tile_in_done : 유효 lane 이 하나뿐인 꼬리 타일은 출력이 한 beat 라 아직 P_DRAIN 인 동안
            //                    지나간다. P_WAIT_OUT 에서 level 만 보면 놓치고 영원히 멈춘다.
            if (i_chunk_done) reader_done_seen <= 1'b1;
            if (i_tile_in_done) tile_in_done_seen <= 1'b1;

            case (state)
                P_IDLE: begin
                    tile_in_done_seen <= 1'b0;
                    if (i_tile_valid) begin                 // o_tile_ready = 1 이므로 이 clk 에 성립
                        k_left     <= k_init;
                        c_left     <= chunk_len(k_init);
                        cur_len    <= chunk_len(k_init);
                        row_mask_q <= i_tile_row_mask;
                        col_mask_q <= i_tile_col_mask;
                        state      <= P_CLEAR;
                    end
                end

                P_CLEAR: begin
                    valid_pipe <= 5'b00000;
                    last_pipe  <= 5'b00000;
                    state      <= P_PREFILL;
                end

                P_PREFILL: begin
                    reader_done_seen <= i_chunk_done;       // 새 chunk 가 시작되니 플래그를 내린다
                    state <= P_FEED;
                end

                P_FEED: begin
                    if (inject) begin
                        valid_pipe <= {valid_pipe[3:0], 1'b1};
                        last_pipe  <= {last_pipe[3:0], tile_last_beat};
                        k_left     <= k_left - 1'b1;
                        c_left     <= c_left - 1'b1;

                        if (tile_last_beat) state <= P_DRAIN;
                        else if (chunk_last_beat)
                            state <= P_CHUNK_WAIT;   // k_left 에는 다음 chunk 부터의 남은 수가 들어간다
                    end
                end

                // step = 0 으로 배열을 그대로 두고, 다음 chunk 가 올라오기만 기다린다.
                P_CHUNK_WAIT: begin
                    if (i_chunk_loaded) begin
                        c_left  <= chunk_len(k_left);
                        cur_len <= chunk_len(k_left);
                        state   <= P_PREFILL;
                    end
                end

                // 주입은 끝났다. valid 토큰이 pipe 를 다 빠져나갈 때까지 배열만 민다.
                P_DRAIN: begin
                    if (drain_advance) begin
                        valid_pipe <= {valid_pipe[3:0], 1'b0};
                        last_pipe  <= {last_pipe[3:0], 1'b0};
                        if (valid_pipe[3:0] == 4'b0000) state <= P_DONE;
                    end
                end

                // 배열은 정지. 후처리 경로가 이 타일의 결과를 다 받았다는 것만 기다린다.
                P_DONE: begin
                    if (i_tile_in_done || tile_in_done_seen) state <= P_IDLE;
                end

                default: state <= P_IDLE;
            endcase
        end
    end

    // =========================================================================
    // 디버그 : 파형에서 state_name 의 Radix 를 ASCII 로 바꾸면 상태가 글자로 보인다 (합성에서는 빠진다)
    // =========================================================================
    // synthesis translate_off
    reg [8*12-1:0] state_name;
    always @* begin
        case (state)
            P_IDLE:       state_name = "P_IDLE";
            P_CLEAR:      state_name = "P_CLEAR";
            P_PREFILL:    state_name = "P_PREFILL";
            P_FEED:       state_name = "P_FEED";
            P_CHUNK_WAIT: state_name = "P_CHUNK_WAIT";
            P_DRAIN:      state_name = "P_DRAIN";
            P_DONE:       state_name = "P_DONE";
            default:      state_name = "P_???";
        endcase
    end
    // synthesis translate_on

endmodule
// -----------------------------------------------------------------------------

`default_nettype wire
