`timescale 1ns / 1ps `default_nettype none
`include "cnn_defs.vh"

// =============================================================================
// top_cnn_cntl : cnn_cntl + pe_cntl
// cnn_cntl 은 레이어 / 타일 스케줄러, pe_cntl 은 그 아래 cycle 단위 제어기다.
// -----------------------------------------------------------------------------
// cnn_cntl <-> pe_cntl 사이에 오가는 건 아래 세 묶음이 전부다.
//
// [1] 타일 명령  (cnn_cntl 이 타일 하나를 발행)
//
//       cnn_cntl                                 pe_cntl
//       o_tile_valid         -------------->  i_tile_valid
//       i_tile_ready         <--------------  o_tile_ready
//       o_step_total    (13) -------------->  i_step_total       (= K)
//       o_tile_row_mask ( 3) -------------->  i_tile_row_mask
//       o_tile_col_mask ( 3) -------------->  i_tile_col_mask
//
//     fire = valid && ready 인 에지에서 pe_cntl 이 K 와 mask 를 저장한다.
//
// [2] 타일 중간 Chunk 재적재  (pe_cntl 이 요청, cnn_cntl 이 응답)
//
//       pe_cntl                                  cnn_cntl
//       o_chunk_req    ---------------------->  i_chunk_req        (Level : 다음 chunk 를 올려 달라)
//       i_chunk_loaded <----------------------  o_chunk_loaded     (1clk  : 다 올렸다)
//
//     몇 번째 chunk 인지 (k_base) 와 길이는 cnn_cntl 이 스스로 센다. 두 모듈이 같은 WBUF_WORDS 를
//     쓰므로 pe_cntl 이 reader 에 주는 길이와 cnn_cntl 이 적재한 길이는 항상 같다 (TB 가 검사한다).
//
//     cnn_cntl 은 C_PE_WAIT 에서 이 요청을 받아 wgt_ld_unit 명령으로 바꾼다.
//     그 동안 타일 / 레이어 index 는 바뀌지 않는다.
//
// [3] 공유 완료 신호
//
//       pooling_unit.o_tile_in_done ---+--> cnn_cntl.i_tile_in_done
//                                      +--> pe_cntl.i_tile_in_done
//
//     둘 다 1clk 펄스를 seen 플래그로 기억한다. 유효 lane 이 하나뿐인 꼬리
//     타일은 출력이 한 beat 라 pe_cntl 이 아직 P_DRAIN 인 동안 펄스가 지나갈
//     수 있기 때문이다.

module top_cnn_cntl #(
    parameter integer NUM_LAYERS = 4,
    parameter integer WBUF_WORDS = 64,

    parameter [`ACT_AW-1:0] REGION_A_BASE = 14'd0,
    parameter [`ACT_AW-1:0] REGION_B_BASE = 14'd8192,
    parameter [        7:0] INPUT_ZP      = 8'd0,

    parameter [   `DIM_W-1:0] L0_IN_W       = 7'd64,
    parameter [   `DIM_W-1:0] L0_IN_H       = 7'd64,
    parameter [    `CH_W-1:0] L0_IN_C       = 6'd3,
    parameter [   `DIM_W-1:0] L0_OUT_W      = 7'd64,
    parameter [   `DIM_W-1:0] L0_OUT_H      = 7'd64,
    parameter [    `CH_W-1:0] L0_OUT_C      = 6'd6,
    parameter [          1:0] L0_STRIDE     = 2'd1,
    parameter                 L0_PAD_EN     = 1'b1,
    parameter [     `K_W-1:0] L0_K          = 13'd27,
    parameter [     `K_W-1:0] L0_OUT_PIX    = 13'd4096,
    parameter [    `W_AW-1:0] L0_W_BASE     = 16'd0,
    parameter [`PARAM_AW-1:0] L0_PARAM_BASE = 7'd0,
    parameter                 L0_RELU       = 1'b1,
    parameter                 L0_POOL_EN    = 1'b1,

    parameter [   `DIM_W-1:0] L1_IN_W       = 7'd32,
    parameter [   `DIM_W-1:0] L1_IN_H       = 7'd32,
    parameter [    `CH_W-1:0] L1_IN_C       = 6'd6,
    parameter [   `DIM_W-1:0] L1_OUT_W      = 7'd32,
    parameter [   `DIM_W-1:0] L1_OUT_H      = 7'd32,
    parameter [    `CH_W-1:0] L1_OUT_C      = 6'd4,
    parameter [          1:0] L1_STRIDE     = 2'd1,
    parameter                 L1_PAD_EN     = 1'b1,
    parameter [     `K_W-1:0] L1_K          = 13'd54,
    parameter [     `K_W-1:0] L1_OUT_PIX    = 13'd1024,
    parameter [    `W_AW-1:0] L1_W_BASE     = 16'd54,
    parameter [`PARAM_AW-1:0] L1_PARAM_BASE = 7'd6,
    parameter                 L1_RELU       = 1'b1,
    parameter                 L1_POOL_EN    = 1'b0,

    parameter [   `DIM_W-1:0] L2_IN_W       = 7'd32,
    parameter [   `DIM_W-1:0] L2_IN_H       = 7'd32,
    parameter [    `CH_W-1:0] L2_IN_C       = 6'd4,
    parameter [     `K_W-1:0] L2_K          = 13'd4096,
    parameter [    `CH_W-1:0] L2_OUT_C      = 6'd32,
    parameter [    `W_AW-1:0] L2_W_BASE     = 16'd162,
    parameter [`PARAM_AW-1:0] L2_PARAM_BASE = 7'd10,
    parameter                 L2_RELU       = 1'b1,

    parameter [   `DIM_W-1:0] L3_IN_W       = 7'd1,
    parameter [   `DIM_W-1:0] L3_IN_H       = 7'd1,
    parameter [    `CH_W-1:0] L3_IN_C       = 6'd32,
    parameter [     `K_W-1:0] L3_K          = 13'd32,
    parameter [    `CH_W-1:0] L3_OUT_C      = 6'd1,
    parameter [    `W_AW-1:0] L3_W_BASE     = 16'd45218,
    parameter [`PARAM_AW-1:0] L3_PARAM_BASE = 7'd42,
    parameter                 L3_RELU       = 1'b0,

    // 레이어별 Requant M / S (cnn_cntl 로 그대로 내려간다)
    parameter [31:0] L0_QM = 32'h4000_0000,
    parameter [ 5:0] L0_QS = 6'd30,
    parameter [31:0] L1_QM = 32'h4000_0000,
    parameter [ 5:0] L1_QS = 6'd30,
    parameter [31:0] L2_QM = 32'h4000_0000,
    parameter [ 5:0] L2_QS = 6'd30,
    parameter [31:0] L3_QM = 32'h4000_0000,
    parameter [ 5:0] L3_QS = 6'd30
) (
    input wire clk,
    input wire rst_n,

    // ---- AXI4-Lite CSR (외부 SoC) ------------------------------------------
    input  wire                  i_start_valid,          // [C] <- cnn_accel_top.i_start_valid  (가속기 밖 AXI4-Lite CSR 의 CTRL.cnn_start)
    output wire                  o_start_ready,          // [C] -> cnn_accel_top.o_start_ready  (CSR. IDLE 이고 done_status=0 일 때만 1)
    output wire o_busy,  // [C] -> cnn_accel_top.o_busy         (CSR BUSY)

    // ---- act_ld_unit -------------------------------------------------------
    output wire                  o_img_ld_start,         // [C] -> act_path.i_img_ld_start      (act_ld_unit.i_img_ld_start)
    input  wire                  i_ld_done,              // [C] <- act_ld_unit.o_ld_done.  초기 이미지 적재 완료 1clk 펄스
    output wire                  o_writer_mode,          // [C] -> act_path.i_writer_mode       (act_ld_unit.i_mode : 0=이미지 적재, 1=결과 쓰기)

    // ---- act_patch_gen / fc_gen : 레이어 descriptor ------------------------
    output wire                  o_layer_cfg_valid,      // [C] -> act_path.i_layer_cfg_valid   (Conv 면 act_patch_gen, FC 면 fc_gen 의 i_layer_cfg_valid. path_sel 로 갈린다)
    output wire [`ACT_AW-1:0]    o_src_base,             // [C] -> act_path.i_src_base          (act_patch_gen / fc_gen .i_src_base)
    output wire [`DIM_W-1:0]     o_in_w,                 // [C] -> act_path.i_in_w              (act_patch_gen / fc_gen .i_in_w)
    output wire [`DIM_W-1:0]     o_in_h,                 // [C] -> act_path.i_in_h              (act_patch_gen / fc_gen .i_in_h)
    output wire [`CH_W-1:0]      o_in_c,                 // [C] -> act_path.i_in_c              (act_patch_gen / fc_gen .i_in_c)
    output wire [`DIM_W-1:0]     o_out_w,                // [C] -> act_path.i_out_w             (act_patch_gen.i_out_w : 블록 격자 폭 = out_w/2)
    output wire [`DIM_W-1:0]     o_out_h,                // [C] -> act_path.i_out_h             (act_patch_gen.i_out_h)
    output wire [1:0]            o_stride,               // [C] -> act_path.i_stride            (act_patch_gen.i_stride)
    output wire                  o_pad_en,               // [C] -> act_path.i_pad_en            (act_patch_gen.i_pad_en)
    output wire [7:0]            o_input_zp,             // [C] -> act_path.i_input_zp          (act_patch_gen.i_input_zp : padding 에 채우는 값)
    output wire [`K_W-1:0]       o_k_total,              // [C] -> act_path.i_k_total (act_patch_gen / fc_gen), wgt_path.i_k_total (wgt_ld_unit : 주소 = base + og*K + kb).  안에서는 pe_cntl.i_step_total 로도 간다 (cnn_cntl.o_step_total)
    output wire                  o_path_sel,             // [C] -> act_path.i_path_sel (act_mux.i_sel + input_buf 읽기 포트 선택), out_path.i_is_fc (output_fifo.i_is_fc).  0=Conv, 1=FC

    // ---- 타일 설정 ----------------------------------------------------------
    output wire                  o_pg_tile_start,        // [C] -> act_path.i_pg_tile_start     (act_patch_gen.i_tile_start)
    output wire                  o_fc_tile_start,        // [C] -> act_path.i_fc_tile_start     (fc_gen.i_tile_start)
    output wire [`POS_W-1:0]     o_pos_base,             // [C] -> act_path.i_pos_base (act_patch_gen.i_pos_base), out_path.i_patch_base (output_fifo.i_patch_base).  패치 순번 (Z 순서)
    output wire [`KEEP_W-1:0]    o_row_mask,             // [C] -> act_path.i_row_mask (act_patch_gen.i_row_mask), out_path.i_row_mask (output_fifo).  안에서는 pe_cntl.i_tile_row_mask 로도 간다 (cnn_cntl.o_tile_row_mask)
    output wire [`KEEP_W-1:0]    o_col_mask,             // [C] -> out_path.i_col_mask          (output_fifo / post_process .i_col_mask).  안에서는 pe_cntl.i_tile_col_mask 로도 간다 (cnn_cntl.o_tile_col_mask)
    output wire                  o_tile_cfg_valid,       // [C] -> out_path.i_tile_cfg_valid    (output_fifo / post_process .i_cfg_valid)
    output wire [`OCB_W-1:0]     o_out_ch_base,          // [C] -> out_path.i_out_ch_base       (output_fifo / post_process .i_out_ch_base)
    output wire                  o_tile_last,            // [C] -> out_path.i_tile_last         (output_fifo.i_tile_last -> meta.layer_end)
    output wire                  o_relu_en,              // [C] -> out_path.i_relu_en           (post_process.i_relu_en)
    output wire [`PARAM_AW-1:0]  o_param_base,           // [C] -> out_path.i_param_base        (post_process.i_param_base : param_buf 에서 base+ocb+lane 을 캐시)
    input  wire                  i_params_ready,         // [C] <- out_path.o_params_ready      (post_process.o_params_ready)

    // ---- 레이어 출력 경로 ---------------------------------------------------
    output wire                  o_output_cfg_valid,     // [C] -> act_path.i_output_cfg_valid (act_ld_unit.i_layer_cfg_valid), out_path.i_output_cfg_valid (output_fifo / pooling_unit / result_buf 의 레이어 cfg)
    output wire [`ACT_AW-1:0]    o_dst_base,             // [C] -> act_path.i_dst_base          (act_ld_unit.i_dst_base : 결과를 쓸 영역)
    output wire [`CH_W-1:0]      o_out_c,                // [C] -> act_path.i_out_c             (act_ld_unit.i_out_c : 쓰기 주소의 ceil(out_c/3))
    output wire [`DIM_W-1:0]     o_pool_in_w,            // [C] -> out_path.i_pool_in_w         (pooling_unit.i_in_w, output_fifo.i_conv_w : 패치 순번 -> pos 변환의 W)
    output wire [`DIM_W-1:0]     o_pool_in_h,            // [C] -> out_path.i_pool_in_h         (pooling_unit.i_in_h)
    output wire [`CH_W-1:0]      o_pool_c,               // [C] -> out_path.i_pool_c            (pooling_unit.i_channels)
    output wire                  o_pool_en,              // [C] -> out_path.i_pool_en           (pooling_unit.i_pool_en)
    output wire                  o_is_final_layer,       // [C] -> out_path.i_is_final_layer    (result_buf.i_is_final_layer)

    // ---- wgt_ld_unit -------------------------------------------------------
    output wire                  o_wload_start,          // [C] -> wgt_ld_unit.i_ld_start.  o_ld_ready 를 본 clk 의 1clk 펄스
    input  wire                  i_wload_ready,          // [C] <- wgt_ld_unit.o_ld_ready  (= IDLE && i_buf_free)
    output wire [`W_AW-1:0]      o_mem_base,             // [C] -> wgt_ld_unit.i_mem_base.  현재 Chunk 의 시작 Word 주소 (cnn_cntl 이 최종 계산)
    output wire [`CHUNK_W-1:0]   o_load_chunk_len,       // [C] -> wgt_path.i_chunk_len         (wgt_ld_unit.i_chunk_len)
    input  wire                  i_wload_done,           // [C] <- wgt_ld_unit.o_ld_done.  Chunk 적재 완료 1clk 펄스
    output wire                  o_wbuf_free,            // [P] -> wgt_path.i_buffer_free       (wgt_ld_unit.i_buffer_free : 읽는 중인 chunk 를 덮어쓰지 않게)

    // ---- wgt_patch_gen -----------------------------------------------------
    output wire                  o_reader_start,         // [P] -> wgt_path.i_reader_start      (wgt_patch_gen.i_start)
    output wire [`CHUNK_W-1:0]   o_reader_chunk_len,     // [P] -> wgt_path.i_reader_chunk_len  (wgt_patch_gen.i_chunk_len)
    output wire                  o_reader_issue_en,      // [P] -> wgt_path.i_reader_issue_en   (wgt_patch_gen.i_issue_en)
    input  wire                  i_reader_done,          // [P] <- wgt_path.o_reader_done       (wgt_patch_gen.o_done)
    input  wire                  i_reader_idle,          // [P] <- wgt_path.o_reader_idle       (wgt_patch_gen.o_idle)

    // ---- Feeder ------------------------------------------------------------
    output wire [`KEEP_W-1:0]    o_pe_col_mask,          // [C] -> wgt_path.i_col_mask.  명세 : cnn_cntl.o_col_mask -> wgt_patch_gen.i_col_mask.  이 저장소의 wgt_path 는 같은 값을 wgt_feeder 가 받는다
    output wire                  o_feeder_en,            // [P] -> act_path.i_feeder_en (act_feeder.i_feed_en), wgt_path.i_feeder_en (wgt_feeder.i_feed_en)
    output wire                  o_tile_clear,           // [P] -> act_path.i_tile_clear (act_feeder.i_clear), wgt_path.i_tile_clear (wgt_feeder.i_clear), pe_core.i_tile_clear (act_skew / wgt_skew .i_clear)
    input  wire                  i_weight_feeder_empty,  // [P] <- wgt_path.o_feeder_empty      (wgt_feeder.o_empty)
    input  wire                  i_weight_valid,         // [P] <- wgt_path.o_weight_valid      (wgt_feeder.o_valid).  같은 선이 pe_core.i_wgt_valid 로도 간다
    input  wire                  i_act_valid,            // [P] <- act_path.o_act_valid         (act_feeder.o_valid).  같은 선이 pe_core.i_act_valid 로도 간다

    // ---- pe_core -----------------------------------------------------------
    output wire                  o_step_en,              // [P] -> pe_core.i_step_en            (act_skew / wgt_skew / pe_array .i_step_en)
    output wire                  o_feed_valid,           // [P] -> pe_core.i_feed_valid         (act_skew / wgt_skew .i_feed_valid)
    output wire                  o_acc_clear,            // [P] -> pe_core.i_acc_clear          (pe_array.i_acc_clear)
    output wire [`PE_N-1:0]      o_mac_valid,            // [P] -> pe_core.i_mac_valid          (pe_array.i_mac_valid)
    output wire [`PE_N-1:0]      o_mac_last,             // [P] -> pe_core.i_mac_last           (pe_array.i_mac_last)

    // ---- output_fifo -------------------------------------------------------
    output wire                  o_result_buf_clear,     // [P] -> out_path.i_result_buf_clear  (output_fifo.i_clear)
    input  wire                  i_result_space_ready,   // [P] <- out_path.o_result_space_ready (output_fifo.o_result_space_ready)

    // ---- 공유 RAM (Parameter 읽기) ------------------------------------------
    output wire                  o_ram_owner,            // [C] -> cnn_accel_top.o_ram_owner        (가속기 밖 RAM 중재. 0=weight, 1=param)
    input  wire                  i_ram_idle,             // [C] <- cnn_accel_top.i_ram_idle         (가속기 밖 RAM 중재)
    output wire                  o_param_mem_req_valid,  // [C] -> cnn_accel_top.o_param_req_valid  (가속기 밖 Param RAM)
    output wire [`PARAM_AW-1:0]  o_param_mem_addr,       // [C] -> cnn_accel_top.o_param_addr       (논리 레코드 번호)
    input  wire                  i_param_mem_req_ready,  // [C] <- cnn_accel_top.i_param_req_ready
    input  wire [31:0]           i_param_mem_data,       // [C] <- Param RAM.  signed INT32 Bias
    input  wire                  i_param_mem_rsp_valid,  // [C] <- cnn_accel_top.i_param_rsp_valid
    output wire                  o_param_mem_rsp_ready,  // [C] -> cnn_accel_top.o_param_rsp_ready

    // ---- param_buf ---------------------------------------------------------
    output wire                  o_param_wr_en,          // [C] -> out_path.i_param_wr_en       (param_buf.i_wr_en)
    output wire [`PARAM_AW-1:0]  o_param_wr_addr,        // [C] -> out_path.i_param_wr_addr     (param_buf.i_wr_addr)
    output wire [31:0]           o_param_wr_data,        // [C] -> out_path.i_param_wr_data (param_buf).  signed INT32 Bias (명세 32bit)
    output wire                  o_param_load_start,     // [C] -> out_path.i_param_load_start (param_buf).  Bias 적재 시작 1clk
    input  wire                  i_param_load_done,      // [C] <- out_path.o_param_load_done  (param_buf).  43 번째 Bias 저장 직후 1clk
    output wire [31:0]           o_quant_multiplier,     // [C] -> out_path.i_quant_multiplier.  현재 레이어 공통 M
    output wire [5:0]            o_quant_shift,          // [C] -> out_path.i_quant_shift.       현재 레이어 공통 S (0 ~ 30)

    // ---- 완료 --------------------------------------------------------------
    input  wire                  i_tile_in_done,         // [C+P] <- out_path.o_tile_in_done    (pooling_unit.o_tile_in_done)
    input  wire                  i_layer_done,           // [C] <- out_path.o_layer_done        (result_buf.o_layer_done)
    input  wire                  i_done_status,          // [C] <- out_path.o_done_status       (result_buf.o_done_status).  같은 선이 cnn_accel_top.o_done_status (CSR) 로도 나간다

    // ---- 디버그 ------------------------------------------------------------
    output wire [2:0] o_layer_idx,  // [C] -> cnn_accel_top.o_layer_idx
    output wire [3:0] o_cnn_state,  // [C] -> cnn_accel_top.o_cnn_state
    output wire                  o_pe_cmd_ready_dbg      // [P] -> cnn_accel_top 에서는 미연결. tb_top_cnn_cntl 이 받아서 pe_cmd fire 를 본다
);

    // ===== [1] 타일 명령 =====
    wire pe_cmd_valid, pe_cmd_ready;
    wire [`K_W-1:0] pe_step_total;  // = o_k_total
    wire [`KEEP_W-1:0] pe_row_mask, pe_col_mask;  // = o_row_mask / o_col_mask

    // -------------------------------------------------------------------------
    // 이름 다리
    //   cnn_cntl / pe_cntl 의 포트는 인터페이스 명세서 이름이고, 이 묶음 모듈의 바깥
    //   포트는 데이터패스 (act_path / wgt_path / pe_core / out_path, cnn_accel_top) 가
    //   쓰는 이름 그대로다. 다른 모듈은 고치지 않으므로 이름은 여기서만 바뀐다.
    //
    //     안 (명세 이름)                     바깥 (그대로)
    //     o_ld_start                  ->  o_img_ld_start
    //     o_write_grant               ->  o_writer_mode
    //     o_in_zp                     ->  o_input_zp
    //     o_fc_mode, o_is_fc          ->  o_path_sel              (같은 값)
    //     o_bias_base                 ->  o_param_base
    //     o_conv_w / o_conv_h         ->  o_pool_in_w / o_pool_in_h
    //     o_out_channels              ->  o_out_c, o_pool_c       (같은 값)
    //     o_layer_cfg_valid           ->  o_layer_cfg_valid, o_output_cfg_valid   (같은 펄스)
    //     o_pos_base, o_patch_base    ->  o_pos_base              (같은 값)
    //     o_chunk_word_count (cnn)    ->  o_load_chunk_len
    //     o_chunk_start / o_chunk_word_count / i_chunk_done (pe)
    //                                 ->  o_reader_start / o_reader_chunk_len / i_reader_done
    //     o_feed_en                   ->  o_feeder_en
    //     o_result_clear              ->  o_result_buf_clear
    // -------------------------------------------------------------------------

    assign o_pe_col_mask = o_col_mask;
    assign o_output_cfg_valid = o_layer_cfg_valid;  // 레이어 cfg 펄스 하나가 act_path 와 out_path 양쪽으로 간다
    assign o_pool_c           = o_out_c;            // 출력 채널 수 하나가 act_ld_unit 과 pooling_unit 양쪽으로 간다

    wire chunk_req, chunk_loaded;
    assign o_pe_cmd_ready_dbg = pe_cmd_ready;

    cnn_cntl #(
        .NUM_LAYERS(NUM_LAYERS),
        .WBUF_WORDS(WBUF_WORDS),
        .REGION_A_BASE(REGION_A_BASE),
        .REGION_B_BASE(REGION_B_BASE),
        .INPUT_ZP(INPUT_ZP),
        .L0_IN_W(L0_IN_W),
        .L0_IN_H(L0_IN_H),
        .L0_IN_C(L0_IN_C),
        .L0_OUT_W(L0_OUT_W),
        .L0_OUT_H(L0_OUT_H),
        .L0_OUT_C(L0_OUT_C),
        .L0_STRIDE(L0_STRIDE),
        .L0_PAD_EN(L0_PAD_EN),
        .L0_K(L0_K),
        .L0_OUT_PIX(L0_OUT_PIX),
        .L0_W_BASE(L0_W_BASE),
        .L0_PARAM_BASE(L0_PARAM_BASE),
        .L0_RELU(L0_RELU),
        .L0_POOL_EN(L0_POOL_EN),
        .L1_IN_W(L1_IN_W),
        .L1_IN_H(L1_IN_H),
        .L1_IN_C(L1_IN_C),
        .L1_OUT_W(L1_OUT_W),
        .L1_OUT_H(L1_OUT_H),
        .L1_OUT_C(L1_OUT_C),
        .L1_STRIDE(L1_STRIDE),
        .L1_PAD_EN(L1_PAD_EN),
        .L1_K(L1_K),
        .L1_OUT_PIX(L1_OUT_PIX),
        .L1_W_BASE(L1_W_BASE),
        .L1_PARAM_BASE(L1_PARAM_BASE),
        .L1_RELU(L1_RELU),
        .L1_POOL_EN(L1_POOL_EN),
        .L2_IN_W(L2_IN_W),
        .L2_IN_H(L2_IN_H),
        .L2_IN_C(L2_IN_C),
        .L2_K(L2_K),
        .L2_OUT_C(L2_OUT_C),
        .L2_W_BASE(L2_W_BASE),
        .L2_PARAM_BASE(L2_PARAM_BASE),
        .L2_RELU(L2_RELU),
        .L3_IN_W(L3_IN_W),
        .L3_IN_H(L3_IN_H),
        .L3_IN_C(L3_IN_C),
        .L3_K(L3_K),
        .L3_OUT_C(L3_OUT_C),
        .L3_W_BASE(L3_W_BASE),
        .L3_PARAM_BASE(L3_PARAM_BASE),
        .L3_RELU(L3_RELU),
        .L0_QM(L0_QM),
        .L0_QS(L0_QS),
        .L1_QM(L1_QM),
        .L1_QS(L1_QS),
        .L2_QM(L2_QM),
        .L2_QS(L2_QS),
        .L3_QM(L3_QM),
        .L3_QS(L3_QS)
    ) U_CNN_CNTL (
        .clk(clk),
        .rst_n(rst_n),
        .i_start_valid(i_start_valid),
        .o_start_ready(o_start_ready),
        .o_busy(o_busy),
        .o_ld_start(o_img_ld_start),
        .i_ld_done(i_ld_done),
        .o_write_grant(o_writer_mode),
        .o_layer_cfg_valid(o_layer_cfg_valid),
        .o_src_base(o_src_base),
        .o_in_w(o_in_w),
        .o_in_h(o_in_h),
        .o_in_c(o_in_c),
        .o_out_w(o_out_w),
        .o_out_h(o_out_h),
        .o_stride(o_stride),
        .o_pad_en(o_pad_en),
        .o_in_zp(o_input_zp),
        .o_k_total(o_k_total),
        .o_fc_mode(o_path_sel),
        .o_pg_tile_start(o_pg_tile_start),
        .o_fc_tile_start(o_fc_tile_start),
        .o_pos_base(o_pos_base),
        .o_row_mask(o_row_mask),
        .o_col_mask(o_col_mask),
        .o_tile_cfg_valid(o_tile_cfg_valid),
        .o_out_ch_base(o_out_ch_base),
        .o_tile_last(o_tile_last),
        .o_relu_en(o_relu_en),
        .o_bias_base(o_param_base),
        .i_params_ready(i_params_ready),
        .o_dst_base(o_dst_base),
        .o_out_channels(o_out_c),
        .o_conv_w(o_pool_in_w),
        .o_conv_h(o_pool_in_h),
        .o_pool_en(o_pool_en),
        .o_is_final_layer(o_is_final_layer),
        .o_ram_base(),                      // 명세의 act_ld_unit.i_ram_base 로 갈 값. 이 저장소의 act_ld_unit 에는 아직 입력이 없다
        .o_ld_word_count(),                 // 명세의 act_ld_unit.i_ld_word_count 로 갈 값. 위와 같음 (IMG_WORDS 파라미터로 고정돼 있다)
        .o_fc_in_count(),                   // 명세의 fc_gen.i_fc_in_count 로 갈 값. 지금은 o_k_total 이 fc_gen 으로 간다
        .o_is_fc(),                         // o_fc_mode 와 같은 값. 바깥에는 o_path_sel 하나로 나간다
        .o_patch_base(),                    // o_pos_base 와 같은 값. 바깥에는 o_pos_base 하나로 나간다
        .o_tile_valid(pe_cmd_valid),
        .i_tile_ready(pe_cmd_ready),
        .o_step_total(pe_step_total),
        .o_tile_row_mask(pe_row_mask),
        .o_tile_col_mask(pe_col_mask),
        .i_chunk_req(chunk_req),
        .o_chunk_loaded(chunk_loaded),
        .o_wload_start(o_wload_start),
        .i_wload_ready(i_wload_ready),
        .o_mem_base(o_mem_base),
        .o_chunk_word_count(o_load_chunk_len),
        .i_wload_done(i_wload_done),
        .o_ram_owner(o_ram_owner),
        .i_ram_idle(i_ram_idle),
        .o_param_mem_req_valid(o_param_mem_req_valid),
        .o_param_mem_addr(o_param_mem_addr),
        .i_param_mem_req_ready(i_param_mem_req_ready),
        .i_param_mem_data(i_param_mem_data),
        .i_param_mem_rsp_valid(i_param_mem_rsp_valid),
        .o_param_mem_rsp_ready(o_param_mem_rsp_ready),
        .o_param_wr_en(o_param_wr_en),
        .o_param_wr_addr(o_param_wr_addr),
        .o_param_wr_data(o_param_wr_data),
        .o_quant_multiplier(o_quant_multiplier),
        .o_quant_shift(o_quant_shift),
        .o_param_load_start(o_param_load_start),
        .i_param_load_done(i_param_load_done),
        .i_tile_in_done(i_tile_in_done),
        .i_layer_done(i_layer_done),
        .i_done_status(i_done_status),
        .o_layer_idx(o_layer_idx),
        .o_state(o_cnn_state)
    );

    pe_cntl #(
        .WBUF_WORDS(WBUF_WORDS)
    ) U_PE_CNTL (
        .clk(clk),
        .rst_n(rst_n),
        .i_tile_valid(pe_cmd_valid),
        .o_tile_ready(pe_cmd_ready),
        .i_step_total(pe_step_total),
        .i_tile_row_mask(pe_row_mask),
        .i_tile_col_mask(pe_col_mask),
        .o_chunk_req(chunk_req),
        .i_chunk_loaded(chunk_loaded),
        .o_wbuf_free(o_wbuf_free),
        .o_chunk_start(o_reader_start),
        .o_chunk_word_count(o_reader_chunk_len),
        .o_reader_issue_en(o_reader_issue_en),
        .i_chunk_done(i_reader_done),
        .i_reader_idle(i_reader_idle),
        .o_feed_en(o_feeder_en),
        .o_tile_clear(o_tile_clear),
        .i_weight_feeder_empty(i_weight_feeder_empty),
        .i_weight_valid(i_weight_valid),
        .i_act_valid(i_act_valid),
        .o_step_en(o_step_en),
        .o_feed_valid(o_feed_valid),
        .o_acc_clear(o_acc_clear),
        .o_mac_valid(o_mac_valid),
        .o_mac_last(o_mac_last),
        .o_result_clear(o_result_buf_clear),
        .i_result_space_ready(i_result_space_ready),
        .i_tile_in_done(i_tile_in_done)
    );

    // =========================================================================
    // 디버그 : 두 제어기의 state 를 이 계층에서 한 번에 본다 (합성에서는 빠진다)
    //
    //   파형에 top_cnn_cntl 만 펼쳐도 아래 다섯 개가 보인다. Radix 를 ASCII 로 바꾼다.
    //     cnn_state_name   cnn_cntl 메인 FSM        (= U_CNN_CNTL.state_name)
    //     wsvc_name        cnn_cntl weight 적재 FSM  (= U_CNN_CNTL.wsvc_name)
    //     tile_name        "L1 p12 oc1"             (= U_CNN_CNTL.tile_name)
    //     pe_state_name    pe_cntl FSM              (= U_PE_CNTL.state_name)
    //     ctrl_state_name  "C_PE_WAIT / P_FEED" 처럼 두 FSM 을 한 줄에 붙인 것.
    //                      타일 하나가 어느 단계에서 얼마나 머무는지 이 줄 하나로 읽힌다.
    //
    //   문자열은 오른쪽 정렬(앞이 NUL)이라 그대로 이어 붙이면 가운데에 빈 칸이 생긴다.
    //   ljust12 가 앞의 NUL 을 밀어내고 뒤를 공백으로 채운다.
    // =========================================================================
    // synthesis translate_off
    function [8*12-1:0] ljust12;
        input [8*12-1:0] s;
        integer i;
        begin
            ljust12 = s;
            for (i = 0; i < 12; i = i + 1)
            if (ljust12[8*12-1-:8] == 8'h00) ljust12 = {ljust12[8*11-1:0], " "};
        end
    endfunction

    wire [8*12-1:0] cnn_state_name = U_CNN_CNTL.state_name;
    wire [ 8*8-1:0] wsvc_name = U_CNN_CNTL.wsvc_name;
    wire [8*16-1:0] tile_name = U_CNN_CNTL.tile_name;
    wire [8*12-1:0] pe_state_name = U_PE_CNTL.state_name;

    reg  [8*27-1:0] ctrl_state_name;
    always @*
        ctrl_state_name = {
            ljust12(cnn_state_name), " / ", ljust12(pe_state_name)
        };
    // synthesis translate_on

endmodule

`default_nettype wire
