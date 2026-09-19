`timescale 1ns / 1ps
`default_nettype none

// -----------------------------------------------------------------------------
// CNN 가속기 제어 유닛 통합 top
//
// 기준 문서 : 통합 인터페이스 시트 tab gid=457801884 (2026-09-19).
// 포트는 연결 대상 블록별로 묶어 놨다. 각 블록 담당자가 자기 묶음만 보면 된다.
//
// cnn_cntl 은 레이어 / 타일 / Pool 스케줄러, pe_cntl 은 그 아래 cycle 단위
// 제어기다. 둘 사이에 오가는 건 두 가지다.
//   - 타일 명령       : o_pe_cmd_valid / o_cmd_ready + K, row/col mask
//   - 타일 도중 chunk : pe_cntl 이 요청 -> cnn_cntl 이 적재 -> 완료 통보
// 타일 완료는 Result+IRQ 의 tile_done 을 양쪽이 같이 본다.
//
// AXI 레지스터맵은 일부러 여기를 통과시키지 않는다. image/weight/param 주소는
// DMA 와 외부 RAM 로더가, irq_en/irq_clear 는 Result+IRQ 가 직접 받는 신호라서
// 여기로 끌어왔다 그대로 내보내면 feed-through 배선만 늘어난다.
// 이 블록이 실제로 쓰는 CSR 신호는 start / done 뿐이다.
// -----------------------------------------------------------------------------
module top_cnn_cntl #(
    parameter integer NUM_LAYERS = 5,
    parameter integer WBUF_WORDS = 64,

    // fc 64 폭 때문에 시트 원래 폭에서 넓힌 값. cnn_cntl 의 "폭 확장" 주석 참고.
    parameter integer W_AW     = 18,   // weight word 주소 : 91,962 word
    parameter integer PARAM_AW = 8,    // param 레코드 주소 : 151 개
    parameter integer OCG_W    = 5,    // oc_group : ceil(64/3) = 22 그룹
    parameter integer OCB_W    = 6,    // out_ch_base : 최대 63
    parameter integer CH_W     = 7     // out_c : 최대 64
) (
    // ---- 02절 공통 규약 ----------------------------------------------------
    input  wire        clk,
    input  wire        rst_n,

    // ---- CSR / CPU --------------------------------------------------------
    input  wire        i_start_valid,
    output wire        o_start_ready,
    output wire        o_busy,
    input  wire        i_done_status,

    // ---- Feature Writer ---------------------------------------------------
    input  wire        i_frame_ready,
    output wire        o_frame_consume,
    output wire        o_writer_mode,
    output wire [13:0] o_dst_base,
    output wire [CH_W-1:0] o_out_c,

    // ---- Patch Gen / FC Feeder : 레이어 descriptor ------------------------
    output wire        o_layer_cfg_valid,
    output wire [13:0] o_src_base,
    output wire [6:0]  o_in_w,
    output wire [6:0]  o_in_h,
    output wire [5:0]  o_in_c,
    output wire [6:0]  o_out_w,
    output wire [6:0]  o_out_h,
    output wire [1:0]  o_stride,
    output wire        o_pad_en,
    output wire [7:0]  o_input_zp,
    output wire [12:0] o_k_total,
    output wire        o_path_sel,
    output wire [1:0]  o_read_sel,

    // ---- 타일 설정 : 피더 / Output FIFO / Processing / Result -------------
    output wire        o_pg_tile_start,
    output wire        o_fc_tile_start,
    output wire [11:0] o_pos_base,
    output wire [2:0]  o_row_mask,
    output wire [2:0]  o_col_mask,
    output wire        o_tile_cfg_valid,
    output wire [OCB_W-1:0] o_out_ch_base,
    output wire        o_tile_last,
    output wire        o_is_final_layer,
    output wire        o_relu_en,
    output wire [PARAM_AW-1:0] o_param_base, // 외부 RAM param_addr base
    input  wire        i_params_ready,

    // ---- Weight Loader ----------------------------------------------------
    output wire        o_wload_valid,
    input  wire        i_wload_ready,
    output wire [W_AW-1:0] o_weight_base, // 외부 RAM weight_addr base
    output wire [OCG_W-1:0] o_oc_group,
    output wire [12:0] o_load_k_base,
    output wire [6:0]  o_load_chunk_len,
    input  wire        i_wload_done_valid,
    output wire        o_wload_done_ready,
    output wire        o_wbuf_free,

    // ---- Weight Addr Gen (버퍼 리더) --------------------------------------
    output wire        o_reader_start,
    output wire [6:0]  o_reader_chunk_len,
    output wire        o_reader_issue_en,
    input  wire        i_reader_done,
    input  wire        i_reader_idle,

    // ---- Weight Feeder / PE Data Feeder -----------------------------------
    output wire [2:0]  o_pe_col_mask,   // pe_cntl 이 타이밍 맞춰 중계한 col mask
    output wire        o_feeder_en,
    output wire        o_tile_clear,
    input  wire        i_weight_feeder_empty,
    input  wire        i_weight_valid,
    input  wire        i_act_valid,

    // ---- Activation Skew / Weight Skew / PE Array -------------------------
    output wire        o_step_en,
    output wire        o_feed_valid,
    output wire        o_acc_clear,
    output wire [8:0]  o_mac_valid,
    output wire [8:0]  o_mac_last,

    // ---- Output FIFO ------------------------------------------------------
    output wire        o_result_buf_clear,
    input  wire        i_result_space_ready,

    // ---- Pool Feeder / MaxPooling -----------------------------------------
    output wire        o_pool_cfg_valid,
    output wire        o_pool_start,
    output wire [13:0] o_pool_src_base,
    output wire [13:0] o_pool_dst_base,
    output wire [6:0]  o_pool_in_w,
    output wire [6:0]  o_pool_in_h,
    output wire [CH_W-1:0] o_pool_c,
    output wire        o_pool_en,
    input  wire        i_pool_done,

    // ---- Result + IRQ -----------------------------------------------------
    input  wire        i_tile_done,

    // ---- 상태 / 디버그 ----------------------------------------------------
    output wire        o_pe_cmd_ready,
    output wire [2:0]  o_layer_idx
);

    // cnn_cntl <-> pe_cntl 내부 연결
    wire        pe_cmd_valid;
    wire        pe_cmd_ready;
    wire [2:0]  row_mask_int;
    wire [2:0]  col_mask_int;
    wire [12:0] k_total_int;

    wire        chunk_req_valid;
    wire        chunk_req_ready;
    wire [12:0] chunk_k_base;
    wire [6:0]  chunk_len;
    wire        chunk_done_valid;
    wire        chunk_done_ready;

    assign o_row_mask     = row_mask_int;
    assign o_col_mask     = col_mask_int;
    assign o_k_total      = k_total_int;
    assign o_pe_cmd_ready = pe_cmd_ready;

    cnn_cntl #(
        .NUM_LAYERS (NUM_LAYERS),
        .WBUF_WORDS (WBUF_WORDS),
        .W_AW       (W_AW),
        .PARAM_AW   (PARAM_AW),
        .OCG_W      (OCG_W),
        .OCB_W      (OCB_W),
        .CH_W       (CH_W)
    ) U_CNN_CNTL (
        .clk              (clk),
        .rst_n            (rst_n),

        .i_start_valid      (i_start_valid),
        .o_start_ready      (o_start_ready),
        .o_busy             (o_busy),
        .i_done_status      (i_done_status),

        .i_frame_ready      (i_frame_ready),
        .o_frame_consume    (o_frame_consume),
        .o_writer_mode      (o_writer_mode),
        .o_dst_base         (o_dst_base),
        .o_out_c            (o_out_c),

        .o_layer_cfg_valid  (o_layer_cfg_valid),
        .o_src_base         (o_src_base),
        .o_in_w             (o_in_w),
        .o_in_h             (o_in_h),
        .o_in_c             (o_in_c),
        .o_out_w            (o_out_w),
        .o_out_h            (o_out_h),
        .o_stride           (o_stride),
        .o_pad_en           (o_pad_en),
        .o_input_zp         (o_input_zp),
        .o_k_total          (k_total_int),
        .o_path_sel         (o_path_sel),
        .o_read_sel         (o_read_sel),

        .o_pg_tile_start    (o_pg_tile_start),
        .o_fc_tile_start    (o_fc_tile_start),
        .o_pos_base         (o_pos_base),
        .o_row_mask         (row_mask_int),
        .o_col_mask         (col_mask_int),
        .o_tile_cfg_valid   (o_tile_cfg_valid),
        .o_out_ch_base      (o_out_ch_base),
        .o_tile_last        (o_tile_last),
        .o_is_final_layer   (o_is_final_layer),
        .o_relu_en          (o_relu_en),
        .o_param_base       (o_param_base),
        .i_params_ready     (i_params_ready),

        .o_pe_cmd_valid     (pe_cmd_valid),
        .i_pe_cmd_ready     (pe_cmd_ready),

        .i_chunk_req_valid  (chunk_req_valid),
        .o_chunk_req_ready  (chunk_req_ready),
        .i_chunk_k_base     (chunk_k_base),
        .i_chunk_len        (chunk_len),
        .o_chunk_done_valid (chunk_done_valid),
        .i_chunk_done_ready (chunk_done_ready),

        .o_wload_valid      (o_wload_valid),
        .i_wload_ready      (i_wload_ready),
        .o_weight_base      (o_weight_base),
        .o_oc_group         (o_oc_group),
        .o_load_k_base      (o_load_k_base),
        .o_load_chunk_len   (o_load_chunk_len),
        .i_wload_done_valid (i_wload_done_valid),
        .o_wload_done_ready (o_wload_done_ready),

        .o_pool_cfg_valid   (o_pool_cfg_valid),
        .o_pool_start       (o_pool_start),
        .o_pool_src_base    (o_pool_src_base),
        .o_pool_dst_base    (o_pool_dst_base),
        .o_pool_in_w        (o_pool_in_w),
        .o_pool_in_h        (o_pool_in_h),
        .o_pool_c           (o_pool_c),
        .o_pool_en          (o_pool_en),
        .i_pool_done        (i_pool_done),

        .i_tile_done        (i_tile_done),
        .o_layer_idx        (o_layer_idx)
    );

    pe_cntl #(
        .WBUF_WORDS (WBUF_WORDS)
    ) U_PE_CNTL (
        .clk                 (clk),
        .rst_n               (rst_n),

        .i_cmd_valid           (pe_cmd_valid),
        .o_cmd_ready           (pe_cmd_ready),
        .i_cmd_k_total         (k_total_int),
        .i_cmd_row_mask        (row_mask_int),
        .i_cmd_col_mask        (col_mask_int),

        .o_chunk_req_valid     (chunk_req_valid),
        .i_chunk_req_ready     (chunk_req_ready),
        .o_chunk_k_base        (chunk_k_base),
        .o_chunk_len           (chunk_len),
        .i_chunk_done_valid    (chunk_done_valid),
        .o_chunk_done_ready    (chunk_done_ready),

        .o_wbuf_free           (o_wbuf_free),

        .o_reader_start        (o_reader_start),
        .o_reader_chunk_len    (o_reader_chunk_len),
        .o_reader_issue_en     (o_reader_issue_en),
        .i_reader_done         (i_reader_done),
        .i_reader_idle         (i_reader_idle),

        .o_col_mask            (o_pe_col_mask),
        .o_feeder_en           (o_feeder_en),
        .o_tile_clear          (o_tile_clear),
        .i_weight_feeder_empty (i_weight_feeder_empty),
        .i_weight_valid        (i_weight_valid),
        .i_act_valid           (i_act_valid),

        .o_step_en             (o_step_en),
        .o_feed_valid          (o_feed_valid),
        .o_acc_clear           (o_acc_clear),
        .o_mac_valid           (o_mac_valid),
        .o_mac_last            (o_mac_last),

        .o_result_buf_clear    (o_result_buf_clear),
        .i_result_space_ready  (i_result_space_ready),

        .i_tile_done           (i_tile_done)
    );

endmodule

`default_nettype wire
