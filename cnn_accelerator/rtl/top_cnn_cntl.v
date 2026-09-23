`timescale 1ns / 1ps
`include "cnn_defs.vh"

module cnn_cntl #(
    parameter integer NUM_LAYERS = 4,                       // 4  : Conv0, Conv1, fc1, fc2
    parameter integer WBUF_WORDS = 64,                      // 64 : wgt_buf 의 Word 수 = chunk 최대 길이. K 가 이보다 크면 나눠 돈다 (fc1 만 해당)

    // input_buf 는 물리 버퍼 하나 (24bit x 16384). 주소로 두 영역을 나눠, 레이어마다 읽을 영역과 쓸 영역을 맞바꾼다 (region_sel).
    parameter [`ACT_AW-1:0] REGION_A_BASE = 14'd0,          // 0    : 영역 A = 0 .. 8191.     이미지, Conv1 출력
    parameter [`ACT_AW-1:0] REGION_B_BASE = 14'd8192,       // 8192 : 영역 B = 8192 .. 16383. Conv0+Pool 출력, fc1 출력

    // 초기 이미지가 놓인 RAM 의 시작 Word 주소. act_ld_unit.i_ram_base 로 그대로 나간다.
    parameter [31:0] IMG_RAM_BASE = 32'd0,                  // 0

    // ---- Layer 0 : Conv0 3x3 3->6, S=1, P=1, ReLU, 출력단 MaxPool ----------
    parameter [`DIM_W-1:0]      L0_IN_W       = 7'd64,        // 64   : 입력 폭
    parameter [`DIM_W-1:0]      L0_IN_H       = 7'd64,        // 64   : 입력 높이
    parameter [`CH_W-1:0]       L0_IN_C       = 6'd3,         // 3    : 입력 채널 (RGB)
    parameter [`DIM_W-1:0]      L0_OUT_W      = 7'd64,        // 64   : 출력 폭 (S=1, P=1 이라 입력과 같다. Pool 이전 크기)
    parameter [`DIM_W-1:0]      L0_OUT_H      = 7'd64,        // 64   : 출력 높이
    parameter [`CH_W-1:0]       L0_OUT_C      = 6'd6,         // 6    : 출력 채널 -> oc_group 2개
    parameter [1:0]             L0_STRIDE     = 2'd1,         // 1
    parameter                   L0_PAD_EN     = 1'b1,         // 1    : zero padding 사용
    parameter [`K_W-1:0]        L0_K          = 13'd27,       // 27   = 3 x 3 x 3 (창 x 입력 채널). 타일당 beat 수
    parameter [`K_W-1:0]        L0_OUT_PIX    = 13'd4096,     // 4096 = 64 x 64. 위치 타일 = ceil(4096/3) = 1366
    parameter [`W_AW-1:0]       L0_W_BASE     = 16'd0,        // 0    : Weight RAM 시작 Word. 이 레이어는 2 x 27 = 54 Word
    parameter [`PARAM_AW-1:0]   L0_PARAM_BASE = 7'd0,         // 0    : Param 레코드 시작. 6 레코드 (0..5)
    parameter                   L0_RELU       = 1'b1,         // 1
    parameter                   L0_POOL_EN    = 1'b1,         // 1    : 출력 경로에서 2x2 MaxPool -> 32x32x6 저장

    // ---- Layer 1 : Conv1 3x3 6->4, S=1, P=1, ReLU --------------------------
    parameter [`DIM_W-1:0]      L1_IN_W         = 7'd32,        // 32
    parameter [`DIM_W-1:0]      L1_IN_H         = 7'd32,        // 32
    parameter [`CH_W-1:0]       L1_IN_C         = 6'd6,         // 6    : 픽셀당 2 Word
    parameter [`DIM_W-1:0]      L1_OUT_W        = 7'd32,        // 32
    parameter [`DIM_W-1:0]      L1_OUT_H        = 7'd32,        // 32
    parameter [`CH_W-1:0]       L1_OUT_C        = 6'd4,         // 4    : oc_group 2개 (둘째는 채널 1개, col_mask 001)
    parameter [1:0]             L1_STRIDE       = 2'd1,         // 1
    parameter                   L1_PAD_EN       = 1'b1,         // 1
    parameter [`K_W-1:0]        L1_K            = 13'd54,       // 54   = 3 x 3 x 6
    parameter [`K_W-1:0]        L1_OUT_PIX      = 13'd1024,     // 1024 = 32 x 32. 위치 타일 = ceil(1024/3) = 342
    parameter [`W_AW-1:0]       L1_W_BASE       = 16'd54,       // 54   = 0 + 54. 이 레이어는 2 x 54 = 108 Word
    parameter [`PARAM_AW-1:0]   L1_PARAM_BASE   = 7'd6,         // 6    = 0 + 6. 4 레코드 (6..9)
    parameter                   L1_RELU         = 1'b1,         // 1
    parameter                   L1_POOL_EN      = 1'b0,         // 0    : Pool 없음

    // ---- Layer 2 : fc1  4096 -> 32, ReLU -----------------------------------
    //   FC 도 in_c 가 필요하다. fc_gen 이 저장 순서(HWC)대로 Word 를 읽으면서
    //   픽셀 끝의 패딩 lane 을 건너뛰어야 하기 때문이다 (C=4 : 둘째 Word 는 lane0 만).
    parameter [`DIM_W-1:0]      L2_IN_W         = 7'd32,                // 32
    parameter [`DIM_W-1:0]      L2_IN_H         = 7'd32,                // 32
    parameter [`CH_W-1:0]       L2_IN_C         = 6'd4,                 // 4
    parameter [`K_W-1:0]        L2_K            = 13'd4096,             // 4096 = 32 x 32 x 4. 64 Word 씩 64 chunk
    parameter [`CH_W-1:0]       L2_OUT_C        = 6'd32,                // 32   : oc_group 11개 (마지막은 2개, col_mask 011)
    parameter [`W_AW-1:0]       L2_W_BASE       = 16'd162,              // 162  = 54 + 108. 이 레이어는 11 x 4096 = 45,056 Word
    parameter [`PARAM_AW-1:0]   L2_PARAM_BASE   = 7'd10,                // 10   = 6 + 4. 32 레코드 (10..41)
    parameter                   L2_RELU         = 1'b1,                 // 1

    // ---- Layer 3 : fc2  32 -> 1, 최종 회귀 ---------------------------------
    parameter [`DIM_W-1:0]      L3_IN_W         = 7'd1,                 // 1
    parameter [`DIM_W-1:0]      L3_IN_H         = 7'd1,                 // 1
    parameter [`CH_W-1:0]       L3_IN_C         = 6'd32,                // 32   : fc1 출력 (11 Word)
    parameter [`K_W-1:0]        L3_K            = 13'd32,               // 32
    parameter [`CH_W-1:0]       L3_OUT_C        = 6'd1,                 // 1    : oc_group 1개 (col_mask 001)
    parameter [`W_AW-1:0]       L3_W_BASE       = 16'd45218,            // 45218 = 162 + 45,056. 이 레이어는 32 Word -> 총 45,250
    parameter [`PARAM_AW-1:0]   L3_PARAM_BASE   = 7'd42,                // 42   = 10 + 32. 1 레코드 -> 총 43
    parameter                   L3_RELU         = 1'b0,                 // 0    : 최종 레이어는 ReLU 없음

    // ---- per layer Requant M / S  ----------------------------------------
    //   q = sat_int8( round( (acc + bias) * M / 2^S ) ).  레이어의 모든 출력 채널에 공통.
    //   S 의 사용 범위는 0 ~ 30. 기본값 M = 2^30, S = 30 은 "그대로 통과" (x * 2^30 / 2^30).
    //   실제 값은 SW export 에서 받아 인스턴스에서 덮어쓴다. 최종 FC (L3) 는 명세상 M / S 를 쓰지 않는다.
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

    // ---- AXI4-Lite CSR / RAM --------------------------------------
    input  wire                     i_start_valid,          // <- top_cnn_cntl.i_start_valid <- cnn_accel_top.i_start_valid (CSR CTRL.cnn_start)
    output wire                     o_start_ready,          // -> cnn_accel_top.o_start_ready (CSR).  IDLE 이고 done_status=0 일 때만 1
    output wire                     o_busy,                 // -> cnn_accel_top.o_busy (CSR BUSY)
    output wire                     o_ram_owner,            // -> cnn_accel_top.o_ram_owner (밖의 RAM 중재).  0=weight, 1=param
    input wire                      i_ram_idle,             // <- cnn_accel_top.i_ram_idle
    output wire                     o_param_mem_req_valid,  // -> cnn_accel_top.o_param_req_valid (밖의 Param RAM)
    output wire [`PARAM_AW-1:0]     o_param_mem_addr,       // -> cnn_accel_top.o_param_addr.  논리 레코드 번호 0 .. 42
    input wire                      i_param_mem_req_ready,  // <- cnn_accel_top.i_param_req_ready
    input  wire [31:0]              i_param_mem_data,       // <- Param RAM.  signed INT32 Bias 1개 (팀 결정 2026-09-21 : 96bit 레코드는 쓰지 않는다)
    input wire                      i_param_mem_rsp_valid,  // <- cnn_accel_top.i_param_rsp_valid
    output wire                     o_param_mem_rsp_ready,  // -> cnn_accel_top.o_param_rsp_ready

    // ---- pe_cntl : tile cmd -------------
    output wire                  o_tile_valid,           // -> pe_cntl.i_tile_valid.      fire = valid && ready
    input  wire                  i_tile_ready,           // <- pe_cntl.o_tile_ready       (pe_cntl 이 P_IDLE 이고, 앞 타일이 끝난 뒤 out_path.o_result_space_ready=1 을 본 뒤 1)
    output wire [`K_W-1:0]       o_step_total,           // -> pe_cntl.i_step_total.      타일당 MAC step 수 = K (Conv 9*in_c, FC in_len).  o_k_total 과 같은 값
    output wire [`KEEP_W-1:0]    o_tile_row_mask,        // -> pe_cntl.i_tile_row_mask.   o_row_mask 와 같은 값
    output wire [`KEEP_W-1:0]    o_tile_col_mask,        // -> pe_cntl.i_tile_col_mask.   o_col_mask 와 같은 값

    // ---- pe_cntl : Chunk 재적재 (K > 64 인 fc1 만. 추론당 693 회) --------------
    //   pe_cntl 은 "다음 chunk 를 올려 달라" 만 말한다. 몇 번째 chunk 인지 (k_base) 와 길이는
    //   weight 주소를 가진 cnn_cntl 이 스스로 센다 : k_base += 직전 길이, 길이 = min(K - k_base, 64).
    input  wire                  i_chunk_req,            // <- pe_cntl.o_chunk_req.     Level. 앞 chunk 를 다 썼다. o_chunk_loaded 가 갈 때까지 유지된다
    output wire                  o_chunk_loaded,         // -> pe_cntl.i_chunk_loaded.  다음 chunk 가 wgt_buf 에 다 들어온 뒤 1clk 펄스

    // ---- act_ld_unit -------------------------------------------------------
    //   명세 cnn_cntl 표 (R543 ~ R546) : i_ld_done / o_ld_start / o_ram_base(32) / o_ld_word_count(13)
    //   o_ram_base, o_ld_word_count 는 RTL 에 없다. act_ld_unit 이 논리 인덱스 0 .. IMG_WORDS-1 (= 4095) 을
    //   고정으로 읽고, 물리 base 는 가속기 밖 RAM 접속부가 붙인다.
    output reg                   o_ld_start,             // -> act_path.i_img_ld_start  (act_ld_unit.i_img_ld_start).  1clk 펄스 (R544)
    output wire [31:0]           o_ram_base,             // -> act_ld_unit.i_ram_base.       입력 영상의 RAM 시작 Word 주소 = IMG_RAM_BASE (상수 0). i_ld_start 에서 래치
    output wire [`K_W-1:0]       o_ld_word_count,        // -> act_ld_unit.i_ld_word_count.  적재할 24bit Word 수 = L0_IN_W * L0_IN_H * ceil(L0_IN_C / 3) = 64 * 64 * 1 = 4096 (상수)
    input  wire                  i_ld_done,              // <- act_ld_unit.o_ld_done.  마지막 Word 저장 뒤 1clk 펄스 (팀 결정 2026-09-21 : frame_ready + consume 은 쓰지 않는다). 이 저장소의 act_ld_unit 은 아직 level 방식이라 top_cnn_cntl 이 펄스로 바꿔 준다
    output reg                   o_write_grant,          // -> act_path.i_writer_mode   (act_ld_unit.i_mode).  1 이면 결과가 input_buf 쓰기 포트를 쓴다 (0 = 이미지 적재).  명세 (R600) 의 연결 대상은 out_path (result_buf)

    // ---- act_patch_gen / fc_gen / MUX : 레이어 descriptor -------------------
    output reg                   o_layer_cfg_valid,      // -> act_path.i_layer_cfg_valid (act_patch_gen / fc_gen), act_path.i_output_cfg_valid (act_ld_unit), out_path.i_output_cfg_valid (output_fifo / pooling_unit / result_buf).  1clk 펄스
    output wire [`ACT_AW-1:0]    o_src_base,             // -> act_path.i_src_base  (act_patch_gen / fc_gen .i_src_base)
    output wire [`DIM_W-1:0]     o_in_w,                 // -> act_path.i_in_w      (act_patch_gen / fc_gen .i_in_w)
    output wire [`DIM_W-1:0]     o_in_h,                 // -> act_path.i_in_h      (act_patch_gen / fc_gen .i_in_h)
    output wire [`CH_W-1:0]      o_in_c,                 // -> act_path.i_in_c      (act_patch_gen / fc_gen .i_in_c)
    output wire [1:0]            o_stride,               // -> act_path.i_stride    (act_patch_gen.i_stride)
    output wire                  o_pad_en,               // -> act_path.i_pad_en    (act_patch_gen.i_pad_en)
    output wire [`K_W-1:0]       o_k_total,              // -> act_path.i_k_total (act_patch_gen / fc_gen : 명세의 o_fc_in_count 도 이 값), wgt_path.i_k_total (wgt_ld_unit : 주소 = base + og*K + kb)
    output wire [`K_W-1:0]       o_fc_in_count,          // -> fc_gen.i_fc_in_count.  FC 입력 Activation 개수. o_k_total 과 같은 값 (fc1 4096 / fc2 32). 명세가 포트를 따로 둔다
    output wire                  o_fc_mode,              // -> act_path.i_path_sel  (act_mux.i_sel + input_buf 읽기 포트 선택).  0=Conv, 1=FC

    // ---- act_patch_gen / fc_gen : 타일 -------------------------------------
    //   명세는 둘 다 o_tile_start 인데 한 모듈에 같은 이름의 포트를 둘 수 없어 접두어를 붙였다.
    output reg                   o_pg_tile_start,        // -> act_path.i_pg_tile_start (act_patch_gen.i_tile_start). 
    output reg                   o_fc_tile_start,        // -> act_path.i_fc_tile_start (fc_gen.i_tile_start).        
    output wire [`POS_W-1:0]     o_pos_base,             // -> act_path.i_pos_base  (act_patch_gen.i_pos_base).  첫 패치 순번 (2x2 블록 Z 순서)
    output wire [`KEEP_W-1:0]    o_row_mask,             // -> act_path.i_row_mask (act_patch_gen), out_path.i_row_mask (output_fifo)

    // ---- wgt_ld_unit / wgt_patch_gen ---------------------------------------
    //   팀 결정 (2026-09-21)
    //     - o_ld_ready 를 확인한 뒤 시작을 1clk 펄스로 준다.  o_ld_ready = IDLE && i_buf_free (pe_cntl.o_wbuf_free)
    //     - Chunk 시작 주소는 cnn_cntl 이 끝까지 계산해 i_mem_base 하나로 준다 (oc_group / k_base 포트 없음)
    //     - 적재 완료는 o_ld_done 1clk 펄스 (done_ready 없음)
    //   act_ld_unit 쪽 o_ld_start / i_ld_done 과 이름이 겹쳐서 여기는 wload 접두어를 붙였다.
    output wire                  o_wload_start,          // -> wgt_ld_unit.i_ld_start.   i_wload_ready=1 인 clk 에만 뜨는 1clk 펄스. 주소 · 길이는 같은 clk 에 유효
    input  wire                  i_wload_ready,          // <- wgt_ld_unit.o_ld_ready  (= IDLE && i_buf_free).  이 저장소의 wgt_ld_unit 에서는 아직 o_load_ready
    output wire [`W_AW-1:0]      o_mem_base,             // -> wgt_ld_unit.i_mem_base.   현재 Chunk 의 시작 Word 주소 = Lx_W_BASE + oc_group * K + k_base.  논리 주소라 16bit (전체 45,250 Word)
    output wire [`CHUNK_W-1:0]   o_chunk_word_count,     // -> wgt_ld_unit.i_chunk_word_count.  적재할 Word 수 1 .. 64.  wgt_patch_gen 쪽 길이는 pe_cntl.o_chunk_word_count 가 낸다
    input  wire                  i_wload_done,           // <- wgt_ld_unit.o_ld_done.    마지막 Word 가 wgt_buf 에 저장된 뒤 1clk 펄스

    // ---- out_path ----------------------------------------------------------
    output wire                  o_tile_cfg_valid,       // -> out_path.i_tile_cfg_valid (output_fifo / post_process .i_cfg_valid).  i_tile_ready=1 인 clk 에만 뜨는 1clk 펄스 (요구사항 4. i_tile_ready 에 out_path.o_result_space_ready 가 들어 있다)
    output wire                  o_is_fc,                // -> out_path.i_is_fc          (output_fifo.i_is_fc).  o_fc_mode 와 같은 값
    output wire                  o_is_final_layer,       // -> out_path.i_is_final_layer (result_buf.i_is_final_layer)
    output wire                  o_pool_en,              // -> out_path.i_pool_en        (pooling_unit.i_pool_en)
    output wire [`DIM_W-1:0]     o_conv_w,               // -> out_path.i_pool_in_w      (pooling_unit.i_in_w, output_fifo.i_conv_w).  Pool 이전 출력 폭
    output wire [`DIM_W-1:0]     o_conv_h,               // -> out_path.i_conv_h         (pooling_unit.i_in_h).  Pool 이전 출력 높이 (명세 R544. 2026-09-22 다시 넣음)
    output wire [`CH_W-1:0]      o_out_channels,         // -> out_path.i_pool_c (pooling_unit.i_channels), act_path.i_out_c (act_ld_unit.i_out_c : 쓰기 주소의 ceil(C/3))
    output wire [`ACT_AW-1:0]    o_dst_base,             // -> act_path.i_dst_base       (act_ld_unit.i_dst_base : 결과를 쓸 영역)
    output wire                  o_relu_en,              // -> out_path.i_relu_en        (post_process.i_relu_en)
    output wire [31:0]           o_quant_multiplier,     // -> out_path.i_quant_multiplier.  현재 레이어의 공통 M (Lx_QM). out_path 가 i_layer_cfg_valid 에서 저장
    output wire [5:0]            o_quant_shift,          // -> out_path.i_quant_shift.       현재 레이어의 공통 S (Lx_QS, 0 ~ 30)
    output wire [`PARAM_AW-1:0]  o_bias_base,            // -> out_path.i_param_base     (post_process.i_param_base).  현재 레이어 첫 레코드 0 / 6 / 10 / 42
    output wire [`POS_W-1:0]     o_patch_base,           // -> out_path.i_patch_base     (output_fifo.i_patch_base).   o_pos_base 와 같은 값
    output wire [`OCB_W-1:0]     o_out_ch_base,          // -> out_path.i_out_ch_base    (output_fifo / post_process .i_out_ch_base)
    output wire [`KEEP_W-1:0]    o_col_mask,             // -> out_path.i_col_mask (output_fifo / post_process), wgt_patch_gen.i_col_mask (명세대로. 받는 쪽이 Chunk 시작에 래치). oc_group 은 타일이 끝난 뒤에만 바뀌므로 타일 동안 고정이다
    output wire                  o_tile_last,            // -> out_path.i_tile_last      (output_fifo.i_tile_last -> meta.layer_end)
    output reg                   o_param_load_start,     // -> out_path.i_param_load_start (param_buf).  추론 시작 후 Bias 적재 시작 1clk 펄스. param_buf 가 적재 카운터를 0 으로. 쓰기는 다음 clk 부터
    input  wire                  i_param_load_done,      // <- out_path.o_param_load_done  (param_buf).  43 번째 Bias 를 저장한 에지 직후 1clk 펄스. 이걸 받고 C_PARAM_LOAD 를 빠져나간다
    input  wire                  i_params_ready,         // <- out_path.o_params_ready   (post_process.o_params_ready)
    output wire                  o_param_wr_en,          // -> out_path.i_param_wr_en    (param_buf.i_wr_en)
    output wire [`PARAM_AW-1:0]  o_param_wr_addr,        // -> out_path.i_param_wr_addr  (param_buf.i_wr_addr)
    output wire [31:0]           o_param_wr_data,        // -> out_path.i_param_wr_data  (param_buf.i_wr_data).  signed INT32 Bias. 43 개를 주소 0 .. 42 순서로 한 번씩
    input  wire                  i_tile_in_done,         // <- out_path.o_tile_in_done   (pooling_unit.o_tile_in_done).  pe_cntl 도 같은 선을 받는다
    input  wire                  i_layer_done,           // <- out_path.o_layer_done     (result_buf.o_layer_done)
    input  wire                  i_done_status,          // <- out_path.o_done_status    (result_buf.o_done_status)

    // ---- 디버그 ------------------------------------------------------------
    output wire [2:0]            o_layer_idx,  // [명세 X] -> cnn_accel_top.o_layer_idx
    output wire [3:0]            o_state       // [명세 X] -> cnn_accel_top.o_cnn_state
);

    // =========================================================================
    // FSM
    // =========================================================================
    // 인코딩 순서 = 진행 순서
    localparam [3:0] C_IDLE = 4'd0;
    localparam [3:0] C_START = 4'd1;
    localparam [3:0] C_PARAM_LOAD = 4'd2;   // 추론당 1회, 타일 루프 밖
    localparam [3:0] C_SET = 4'd3;          // 레이어당 1회
    localparam [3:0] C_W_LOAD = 4'd4;       // 여기부터 타일당 1회
    localparam [3:0] C_TILE_CFG = 4'd5;
    localparam [3:0] C_PE_TILE = 4'd6;
    localparam [3:0] C_PE_WAIT = 4'd7;
    localparam [3:0] C_NXT_TILE = 4'd8;
    localparam [3:0] C_LAYER_END = 4'd9;
    localparam [3:0] C_DONE = 4'd10;

    reg [3:0] state;
    reg [2:0] layer_idx;
    reg       region_sel; // input_buf 의 두 주소 영역 중 어느 쪽을 읽나. 0 : A 읽고 B 쓰기, 1 : B 읽고 A 쓰기. 레이어가 끝날 때마다 반전

    // 타일 좌표. 두 카운터가 "어느 출력 9개를 계산하나" 를 정한다.
    reg [`POS_W-1:0] pos_group;      // `define POS_W  12 = 출력 위치 0 ~ 4095. 위치 3개 묶음 번호 (Conv0 0~1365 / Conv1 0~341 / FC 0)
    reg [`OCG_W-1:0] oc_group;       // `define OCG_W   4 = 채널 그룹 0 ~ 15.   채널 3개 묶음 번호 (Conv 0~1 / fc1 0~10 / fc2 0)
    reg [`W_AW-1:0]  og_off_q;       // oc_group * K 를 oc_group 과 같은 에지에 누적한 값 (곱셈기 대신. 타이밍 2026-09-23)

    reg tile_started;
    reg tcfg_sent;
    reg first_chunk_rdy;
    reg tile_in_done_seen;
    reg layer_done_seen;
    reg ld_done_seen;               // i_ld_done 은 1clk 펄스라 기억해 둔다

    assign o_busy        = (state != C_IDLE);
    assign o_layer_idx   = layer_idx;
    assign o_state       = state;
    assign o_start_ready = (state == C_IDLE) && !i_done_status;

    // =========================================================================
    // Layer Setting by config
    // =========================================================================
    reg [`DIM_W-1:0] ly_in_w, ly_in_h, ly_out_w, ly_out_h;
    reg [`CH_W-1:0] ly_in_c, ly_out_c;
    reg [1:0] ly_stride;
    reg       ly_pad_en;
    reg [`K_W-1:0] ly_k, ly_out_pix;
    reg [    `W_AW-1:0] ly_w_base;
    reg [`PARAM_AW-1:0] ly_param_base;
    reg ly_relu, ly_pool_en, ly_is_fc;
    reg [31:0] ly_qm;
    reg [ 5:0] ly_qs;  // 현재 레이어의 Requant M / S

    always @* begin
        ly_in_w = 7'd1; ly_in_h = 7'd1; ly_in_c = 6'd1;
        ly_out_w = 7'd1; ly_out_h = 7'd1; ly_out_c = 6'd1;
        ly_stride = 2'd1; ly_pad_en = 1'b0; ly_k = 13'd1;
        ly_out_pix = 13'd1;
        ly_w_base = {`W_AW{1'b0}}; ly_param_base = {`PARAM_AW{1'b0}};
        ly_relu = 1'b0; ly_pool_en = 1'b0;
        ly_is_fc = 1'b1;
        ly_qm = 32'h4000_0000; ly_qs = 6'd30;

        case (layer_idx)  // layer setting
            3'd0: begin                                    // Conv0 : L0_* = 64x64x3 -> 64x64x6, K 27, out_pix 4096, w_base 0, param_base 0, ReLU, Pool
                ly_in_w = L0_IN_W; ly_in_h = L0_IN_H; ly_in_c = L0_IN_C;
                ly_out_w = L0_OUT_W; ly_out_h = L0_OUT_H; ly_out_c = L0_OUT_C;
                ly_stride = L0_STRIDE; ly_pad_en = L0_PAD_EN; ly_k = L0_K;
                ly_out_pix = L0_OUT_PIX;
                ly_w_base = L0_W_BASE; ly_param_base = L0_PARAM_BASE;
                ly_relu = L0_RELU; ly_pool_en = L0_POOL_EN;
                ly_is_fc = 1'b0;
                ly_qm = L0_QM; ly_qs = L0_QS;
            end
            3'd1: begin                                    // Conv1 : L1_* = 32x32x6 -> 32x32x4, K 54, out_pix 1024, w_base 54, param_base 6, ReLU
                ly_in_w = L1_IN_W; ly_in_h = L1_IN_H; ly_in_c = L1_IN_C;
                ly_out_w = L1_OUT_W; ly_out_h = L1_OUT_H; ly_out_c = L1_OUT_C;
                ly_stride = L1_STRIDE; ly_pad_en = L1_PAD_EN;
                ly_k = L1_K;
                ly_out_pix = L1_OUT_PIX;
                ly_w_base = L1_W_BASE; ly_param_base = L1_PARAM_BASE;
                ly_relu = L1_RELU; ly_pool_en = L1_POOL_EN;
                ly_is_fc = 1'b0;
                ly_qm = L1_QM; ly_qs = L1_QS;
            end
            3'd2: begin                                    // fc1   : L2_* = 32x32x4 -> 32, K 4096, out_pix 1 (고정), w_base 162, param_base 10, ReLU
                ly_in_w = L2_IN_W; ly_in_h = L2_IN_H; ly_in_c = L2_IN_C;
                ly_out_w = 7'd1; ly_out_h = 7'd1; ly_out_c = L2_OUT_C;
                ly_k = L2_K;
                ly_out_pix = 13'd1;
                ly_w_base = L2_W_BASE; ly_param_base = L2_PARAM_BASE;
                ly_relu = L2_RELU;
                ly_is_fc = 1'b1;
                ly_qm = L2_QM; ly_qs = L2_QS;
            end
            default: begin                                 // fc2   : L3_* = 32 -> 1, K 32, out_pix 1 (고정), w_base 45218, param_base 42, ReLU 없음
                ly_in_w = L3_IN_W; ly_in_h = L3_IN_H; ly_in_c = L3_IN_C;
                ly_out_w = 7'd1; ly_out_h = 7'd1; ly_out_c = L3_OUT_C;
                ly_k = L3_K;
                ly_out_pix = 13'd1;
                ly_w_base = L3_W_BASE; ly_param_base = L3_PARAM_BASE;
                ly_relu = L3_RELU;
                ly_is_fc = 1'b1;
                ly_qm = L3_QM; ly_qs = L3_QS;
            end
        endcase
    end

    // 아래 ly_* 는 위 case 에서 layer_idx 로 고른 L0_* .. L3_* parameter 값이다.
    // 괄호 안은 L0 / L1 / L2 / L3 (Conv0 / Conv1 / fc1 / fc2) 의 실제 값.
    assign o_in_w               = ly_in_w;                              // Lx_IN_W       (64 / 32 / 32 / 1)
    assign o_in_h               = ly_in_h;                              // Lx_IN_H       (64 / 32 / 32 / 1)
    assign o_in_c               = ly_in_c;                              // Lx_IN_C       (3 / 6 / 4 / 32)
    assign o_out_channels       = ly_out_c;                             // Lx_OUT_C      (6 / 4 / 32 / 1)
    assign o_stride             = ly_stride;                            // Lx_STRIDE     (1 / 1 / 1 / 1)     FC 는 기본값 1
    assign o_pad_en             = ly_pad_en;                            // Lx_PAD_EN     (1 / 1 / 0 / 0)     FC 는 기본값 0
    assign o_k_total            = ly_k;                                 // Lx_K          (27 / 54 / 4096 / 32)
    assign o_relu_en            = ly_relu;                              // Lx_RELU       (1 / 1 / 1 / 0)
    assign o_pool_en            = ly_pool_en;                           // Lx_POOL_EN    (1 / 0 / 0 / 0)     Conv0 만
    assign o_quant_multiplier   = ly_qm;                                // Lx_QM : 레이어 공통 M
    assign o_quant_shift        = ly_qs;                                // Lx_QS : 레이어 공통 S (0 ~ 30)
    assign o_bias_base          = ly_param_base;                        // Lx_PARAM_BASE (0 / 6 / 10 / 42)
    assign o_fc_mode            = ly_is_fc;                             //               (0 / 0 / 1 / 1)
    assign o_is_fc              = ly_is_fc;                             // out_path 로 가는 같은 값 (명세는 포트를 따로 둔다)
    assign o_is_final_layer     = (layer_idx == (NUM_LAYERS - 1));      // NUM_LAYERS = 4 -> layer_idx == 3 (fc2) 일 때만 1

    // pooling_unit 에 주는 것은 "후처리 결과" 의 크기, 즉 Pool 이전 크기다
    assign o_conv_w = ly_out_w;  // Lx_OUT_W      (64 / 32 / 1 / 1)
    assign o_conv_h = ly_out_h;  // Lx_OUT_H      (64 / 32 / 1 / 1)

    // pe_cntl 로 가는 타일 명령 payload. 데이터패스로 가는 것과 값이 같고, 명세가 포트를 따로 둔다.
    // 명세 act_ld_unit / fc_gen 표의 입력에 맞춘 포트. 값은 상수이거나 이미 나가는 값과 같다.
    localparam [`K_W-1:0] IMG_WORDS = L0_IN_W * L0_IN_H * ((L0_IN_C + 2) / 3);      // 64 * 64 * ceil(3/3) = 4096
    assign o_ram_base               = IMG_RAM_BASE;                                 // = 0
    assign o_ld_word_count          = IMG_WORDS;                                    // = 4096
    assign o_fc_in_count            = ly_k;                                         // Lx_K (27 / 54 / 4096 / 32). only fc layer
    assign o_step_total             = ly_k;                                         // Lx_K (27 / 54 / 4096 / 32) = 타일당 MAC step 수
    assign o_tile_row_mask          = o_row_mask;
    assign o_tile_col_mask          = o_col_mask;
    assign o_patch_base             = o_pos_base;                                   // out_path 로 가는 같은 패치 순번

    // swap address region
    //   L0 : region_sel=0 -> read A(0)    / write B(8192)
    //   L1 : region_sel=1 -> read B(8192) / write A(0)
    //   L2 : region_sel=0 -> read A(0)    / write B(8192)
    //   L3 : region_sel=1 -> read B(8192) / 결과 레지스터 (메모리에 안 씀)
    assign o_src_base = region_sel ? REGION_B_BASE : REGION_A_BASE;   // REGION_A_BASE = 0, REGION_B_BASE = 8192
    assign o_dst_base = region_sel ? REGION_A_BASE : REGION_B_BASE;
    // input image -> REGION_A

    // =========================================================================
    // Tile Decode
    // =========================================================================
    function [2:0] lane_mask;
        input [`K_W-1:0] total;
        input [`K_W-1:0] base;
        reg [`K_W-1:0] remain;
        begin
            if (total <= base) lane_mask = 3'b000;
            else begin
                remain = total - base;
                if (remain >= 3) lane_mask = 3'b111;
                else if (remain == 2) lane_mask = 3'b011;
                else lane_mask = 3'b001;
            end
        end
    endfunction

    wire [`K_W-1:0] pos_base_k = {{(`K_W - `POS_W) {1'b0}}, pos_group} * 13'd3;
    wire [`K_W-1:0] oc_base_k = {{(`K_W - `OCG_W) {1'b0}}, oc_group} * 13'd3;
    wire [`K_W-1:0] out_c_k = {{(`K_W - `CH_W) {1'b0}}, ly_out_c};

    // o_pos_base = 이 타일 첫 행의 패치 순번 (2x2 블록 안 Z 순서). raster 위치가 아니다.
    // act_patch_gen.i_pos_base 와 out_path.i_patch_base 로 같은 값이 간다.
    assign o_pos_base    = pos_base_k[`POS_W-1:0];
    assign o_out_ch_base = oc_base_k[`OCB_W-1:0];
    // mask 는 레지스터로 낸다 (타이밍 2026-09-23 : pos_group -> lane_mask -> output_fifo first_row 가 14 단).
    //   pos_group / oc_group 은 C_NXT_TILE 에지에 바뀌고 tile_cfg / 타일 명령 / tile_start 는 그보다
    //   2 clk 이상 뒤라 한 clk 늦은 값이 항상 맞다.
    reg [`KEEP_W-1:0] row_mask_r, col_mask_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_mask_r <= {`KEEP_W{1'b0}};
            col_mask_r <= {`KEEP_W{1'b0}};
        end else begin
            row_mask_r <= lane_mask(ly_out_pix, pos_base_k);
            col_mask_r <= lane_mask(out_c_k,    oc_base_k);
        end
    end
    assign o_row_mask    = row_mask_r;
    assign o_col_mask    = col_mask_r;

    wire pos_last = ((pos_base_k + 13'd3) >= ly_out_pix);
    wire oc_last = ((oc_base_k + 13'd3) >= out_c_k);
    assign o_tile_last = pos_last && oc_last;

    // =========================================================================
    // Weight 적재 서비스
    //   첫 Chunk(C_W_LOAD)와 타일 중간 Chunk(C_PE_WAIT)가 같은 Loader 를 쓴다.
    //   (명세 548행 "cnn_cntl 보조 제어 L_IDLE / L_WAIT / L_REPLY")
    // =========================================================================
    localparam [1:0] W_IDLE = 2'd0;
    localparam [1:0] W_ISSUE = 2'd1;
    localparam [1:0] W_WAIT = 2'd2;
    localparam [1:0] W_REPLY = 2'd3;

    reg [1:0] wsvc;
    reg [`K_W-1:0] svc_kbase;
    reg [`CHUNK_W-1:0] svc_len;

    // 같은 레이어 · 같은 oc_group 이고 K 가 한 Chunk 에 들어가면 buffer 를
    // 그대로 재사용한다. oc_group 이 안쪽 loop 라, 그룹이 하나뿐인 레이어에서만
    // 걸린다. 그룹이 둘 이상인 Conv 는 타일마다 그룹이 바뀌어 매번 다시 적재한다.
    reg wtag_valid;
    reg [2:0] wtag_layer;
    reg [`OCG_W-1:0] wtag_oc;
    wire wload_skip = wtag_valid && (wtag_layer == layer_idx) &&
                      (wtag_oc == oc_group) && (ly_k <= WBUF_WORDS);
    // WBUF_WORDS = 64 : K 가 64 이하 (Conv0 27 / Conv1 54 / fc2 32) 일 때만 재사용 가능. fc1 (4096) 은 항상 다시 적재

    // chunk 하나의 길이 = min(남은 k 수, WBUF_WORDS = 64)
    function [`CHUNK_W-1:0] chunk_len;
        input [`K_W-1:0] remain;
        begin
            chunk_len = (remain > WBUF_WORDS) ? WBUF_WORDS[`CHUNK_W-1:0] : remain[`CHUNK_W-1:0];
        end
    endfunction

    wire [`CHUNK_W-1:0] first_len = chunk_len(
        ly_k
    );  // 타일의 첫 chunk : 27 / 54 / 64 / 32

    // 타일 도중의 다음 chunk. 직전 chunk 바로 뒤에서 시작한다 (fc1 : 64, 128, ... 4032).
    wire [`K_W-1:0]     nx_kbase = svc_kbase + {{(`K_W-`CHUNK_W){1'b0}}, svc_len};
    wire [`CHUNK_W-1:0] nx_len = chunk_len(ly_k - nx_kbase);

    // 요청을 받아들이는 clk : PE 실행 중이고 적재 서비스가 비어 있을 때
    wire chunk_accept = (state == C_PE_WAIT) && (wsvc == W_IDLE) && i_chunk_req;

    // Chunk 시작 주소를 여기서 끝까지 계산한다 (팀 결정 : wgt_ld_unit 은 i_mem_base 만 받는다).
    //   mem_base = Lx_W_BASE + oc_group * K + k_base
    //   Conv1 og=1 첫 chunk : 54 + 1*54 + 0 = 108       fc1 og=3 세 번째 chunk : 162 + 3*4096 + 128 = 12,578
    //   곱은 최대 10 * 4096 = 40,960, 합은 최대 45,249 라 `W_AW = 16bit 에 들어간다.
    wire [`W_AW-1:0] og_ext = {{(`W_AW - `OCG_W) {1'b0}}, oc_group};
    wire [`W_AW-1:0] k_ext = {{(`W_AW - `K_W) {1'b0}}, ly_k};
    wire [`W_AW-1:0] kb_ext = {{(`W_AW - `K_W) {1'b0}}, svc_kbase};
    wire [`W_AW-1:0] og_off = og_off_q;        // oc_group * K (og_off_q 는 oc_group 갱신 에지에 같이 갱신)
    assign o_mem_base         = ly_w_base + og_off + kb_ext;

    // ready 를 본 clk 에만 1clk. W_ISSUE 는 ready 가 올 때까지 기다리는 상태다.
    assign o_wload_start      = (wsvc == W_ISSUE) && i_wload_ready;
    assign o_chunk_word_count = svc_len;
    assign o_chunk_loaded     = (wsvc == W_REPLY);  // W_REPLY 는 1clk

    // =========================================================================
    // Parameter RAM -> param_buf 일괄 적재 (C_PARAM_LOAD, 추론당 1회)
    //
    //   레코드 0 .. PARAM_TOTAL-1 을 주소 그대로 param_buf 에 옮긴다.
    //   레코드는 레이어 순서로 빈틈없이 쌓여 있다고 본다 (L*_PARAM_BASE 가
    //   앞 레이어의 base + out_c). 그래서 마지막 레이어의 base + out_c 가
    //   전체 개수다.
    //   요청은 한 번에 하나만 띄운다 (!pl_busy). 43 레코드면 100clk 대라
    //   42 만 clk 짜리 추론에서 문제가 되지 않는다.
    // =========================================================================
    localparam [`PARAM_AW:0] PARAM_TOTAL =
          (NUM_LAYERS <= 1) ? (L0_PARAM_BASE + L0_OUT_C) :
          (NUM_LAYERS == 2) ? (L1_PARAM_BASE + L1_OUT_C) :
          (NUM_LAYERS == 3) ? (L2_PARAM_BASE + L2_OUT_C) :
                              (L3_PARAM_BASE + L3_OUT_C);   // NUM_LAYERS=4 -> 42 + 1 = 43 레코드 (= 6 + 4 + 32 + 1)

    reg [`PARAM_AW:0] pl_idx;  // 0 .. PARAM_TOTAL (1bit 여유)
    reg pl_busy;
    reg pl_done;
    reg [`PARAM_AW-1:0] pl_addr_q;

    wire pl_more = (pl_idx < PARAM_TOTAL);   // PARAM_TOTAL = 43 : 아직 요청하지 않은 레코드가 남았다

    assign o_ram_owner = (state == C_PARAM_LOAD);

    assign o_param_mem_req_valid = (state == C_PARAM_LOAD) && pl_more && !pl_busy;
    assign o_param_mem_addr = pl_idx[`PARAM_AW-1:0];
    assign o_param_mem_rsp_ready = pl_busy;

    wire pmem_req_fire = o_param_mem_req_valid && i_param_mem_req_ready;
    wire pmem_rsp_fire = i_param_mem_rsp_valid && o_param_mem_rsp_ready;

    assign o_param_wr_en = pmem_rsp_fire;
    assign o_param_wr_addr = pl_addr_q;
    assign o_param_wr_data = i_param_mem_data;  // 32bit Bias 를 그대로 넘긴다

    // =========================================================================
    assign o_tile_valid = (state == C_PE_TILE) && tile_started;

    // tile_cfg : C_TILE_CFG 에 들어와 아직 안 낸 상태에서 i_tile_ready=1 인 첫 clk 에 1clk 뜬다.
    // i_tile_ready 는 pe_cntl 이 P_IDLE 이고 앞 타일 뒤에 out_path.o_result_space_ready=1 을 본 뒤에만
    // 1 이라, 수집기가 비어 있을 때만 나간다 (요구사항 4). 같은 에지에 tcfg_sent 가 1 이 되므로 두 번 뜨지 않는다.
    assign o_tile_cfg_valid = (state == C_TILE_CFG) && !tcfg_sent && i_tile_ready;

    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= C_IDLE;
            layer_idx <= 3'd0;
            region_sel <= 1'b0;
            pos_group         <= {`POS_W{1'b0}};   // `define POS_W  12 → 12'd0 : 출력 위치 0 부터
            oc_group          <= {`OCG_W{1'b0}};   // `define OCG_W   4 →  4'd0 : 출력 채널 0 부터
            og_off_q          <= {`W_AW{1'b0}};
            tile_started <= 1'b0;
            tcfg_sent <= 1'b0;
            first_chunk_rdy <= 1'b0;
            tile_in_done_seen <= 1'b0;
            layer_done_seen <= 1'b0;
            o_ld_start <= 1'b0;
            o_param_load_start <= 1'b0;
            ld_done_seen <= 1'b0;
            o_write_grant <= 1'b0;
            o_layer_cfg_valid <= 1'b0;
            o_pg_tile_start <= 1'b0;
            o_fc_tile_start <= 1'b0;
            wsvc <= W_IDLE;
            svc_kbase         <= {`K_W{1'b0}};     // `define K_W    13 → 13'd0 : K 1 ~ 4096 을 담는 폭. 적재할 chunk 의 시작 k
            svc_len           <= {`CHUNK_W{1'b0}}; // `define CHUNK_W 7 →  7'd0 : chunk_len 1 ~ 64 (wgt_buf 가 64 word 라 64 를 담아야 해서 7bit)
            wtag_valid <= 1'b0;
            wtag_layer <= 3'd0;
            wtag_oc           <= {`OCG_W{1'b0}};   // `define OCG_W   4 →  4'd0 : wgt_buf 에 지금 들어 있는 oc_group 태그
            pl_idx            <= {(`PARAM_AW+1){1'b0}}; // `define PARAM_AW 7 이지만 +1 = 8bit → 8'd0 : 주소가 아니라 개수(0~PARAM_TOTAL=43)를 세므로 1bit 더
            pl_busy <= 1'b0;
            pl_done <= 1'b0;
            pl_addr_q         <= {`PARAM_AW{1'b0}}; // `define PARAM_AW  7 →  7'd0 : param_buf 쓰기 주소 0 ~ 127
        end else begin
            o_ld_start    <= 1'b0;
            o_param_load_start <= 1'b0;
            o_layer_cfg_valid <= 1'b0;
            o_pg_tile_start   <= 1'b0;
            o_fc_tile_start   <= 1'b0;

            if (i_tile_in_done) tile_in_done_seen <= 1'b1;
            if (i_layer_done)   layer_done_seen <= 1'b1;
            if (i_ld_done)      ld_done_seen <= 1'b1;

            // =================================================================
            // Weight Load
            // =================================================================
            case (wsvc)
                W_IDLE: begin
                    if (chunk_accept) begin
                        svc_kbase <= nx_kbase;
                        svc_len   <= nx_len;
                        wsvc      <= W_ISSUE;
                    end
                end
                W_ISSUE: if (i_wload_ready) wsvc <= W_WAIT;
                W_WAIT:
                if (i_wload_done) begin                     // 1clk 펄스. 적재 중에는 항상 이 상태라 놓치지 않는다
                    if (state == C_PE_WAIT) begin           // 타일 도중의 chunk. 첫 chunk 는 C_W_LOAD 에서 온다
                        wtag_valid <= 1'b0;                 // buffer 에 후속 Chunk 가 들어감
                        wsvc <= W_REPLY;
                    end else begin
                        first_chunk_rdy <= 1'b1;
                        wtag_valid      <= 1'b1;
                        wtag_layer      <= layer_idx;
                        wtag_oc         <= oc_group;
                        wsvc            <= W_IDLE;
                    end
                end
                default: wsvc <= W_IDLE;                    // W_REPLY : o_chunk_loaded 1clk
            endcase

            // =================================================================
            // Parameter Load
            // =================================================================
            if (pmem_req_fire) begin
                pl_busy   <= 1'b1;
                pl_addr_q <= o_param_mem_addr;
                pl_idx    <= pl_idx + 1'b1;
            end
            if (pmem_rsp_fire) begin
                pl_busy <= 1'b0;
            end
            // 적재 완료는 param_buf 가 판단한다 (팀 결정 2026-09-21). 1clk 펄스라 기억해 둔다.
            if (i_param_load_done) pl_done <= 1'b1;

            // =================================================================
            // Main FSM
            // =================================================================
            case (state)
                C_IDLE: begin
                    o_write_grant <= 1'b0;
                    wtag_valid    <= 1'b0;
                    ld_done_seen  <= 1'b0;
                    if (i_start_valid && o_start_ready) begin
                        o_ld_start <= 1'b1;                             // Image RAM -> input_buf
                        state      <= C_START;
                    end
                end

                C_START: begin // Image Load State
                    if (i_ld_done || ld_done_seen) begin
                        ld_done_seen        <= 1'b0;
                        o_write_grant       <= 1'b1;
                        layer_idx           <= 3'd0;
                        region_sel          <= 1'b0;
                        pl_idx              <= {(`PARAM_AW+1){1'b0}};   // PARAM_AW 7 => Param 레코드 주소 0 ~ 127
                        pl_busy             <= 1'b0;
                        pl_done             <= 1'b0;
                        o_param_load_start  <= 1'b1;                    // C_PARAM_LOAD 첫 clk 에 보인다. param_buf 가 카운터를 0 으로
                        state               <= C_PARAM_LOAD;
                    end
                end

                C_PARAM_LOAD: begin // one load per inference - RAM owner = param (1)
                    if (pl_done && i_ram_idle) state <= C_SET;
                end

                C_SET: begin // layer descriptor + output path setting 
                    o_layer_cfg_valid <= 1'b1;
                    // layer change -> tile zero setting
                    pos_group          <= {`POS_W{1'b0}};  // POS_W  12 → 12'd0 : 출력 위치 0,1,2 (pos_base = 0)
                    oc_group           <= {`OCG_W{1'b0}};  // OCG_W   4 →  4'd0 : 출력 채널 0,1,2 (out_ch_base = 0)
                    og_off_q           <= {`W_AW{1'b0}};
                    tile_started <= 1'b0;
                    tcfg_sent <= 1'b0;
                    first_chunk_rdy <= 1'b0;
                    tile_in_done_seen <= 1'b0;
                    layer_done_seen <= 1'b0;
                    state <= C_W_LOAD;
                end

                C_W_LOAD: begin // this tile's first weight chunk - RAM owner = weight (0)
                    if (wload_skip) begin
                        first_chunk_rdy <= 1'b1;
                    end else if (!first_chunk_rdy && (wsvc == W_IDLE)) begin
                        svc_kbase     <= {`K_W{1'b0}};     // K_W 13 → 13'd0 : tile's first chunk => k_base = 0
                        svc_len       <= first_len;        // = min(K, 64). Conv0 27 / Conv1 54 / fc1 64 / fc2 32
                        wsvc <= W_ISSUE;
                    end

                    // Parameters already loaded initially; no owner change. (i_ram_idle)
                    // Keep this. We must wait for the previous chunk's reads to fully drain before opening the next tile. &#10;
                    // This ensures the invariant that the shared RAM is completely idle during `tile_cfg`.
                    if (first_chunk_rdy && i_ram_idle && (wsvc == W_IDLE)) begin
                        state <= C_TILE_CFG;
                    end
                end

                // tile_cfg 1clk (i_tile_ready=1 인 clk 에만, 요구사항 4. 펄스는 위 assign 이 낸다)
                // cfg 다음 clk 부터 i_params_ready=1 이면 진행한다. out_path 는 cfg 에지에 상태를 바꾸므로
                // 그 다음 clk 의 값은 이 타일의 것이다 (bias 캐시 hit 이면 0 으로 내려가지 않고 바로 1).
                // 전에는 0 을 한 번 본 뒤 1 을 기다렸다 (핸드셰이크 정리 2026-09-23, 명세 R562 갱신 필요)
                C_TILE_CFG: begin // param_buf -> post_process
                    if (!tcfg_sent) begin
                        if (i_tile_ready) tcfg_sent <= 1'b1;
                    end else if (i_params_ready) begin
                        tile_started <= 1'b0;
                        state        <= C_PE_TILE;
                    end
                end

                // act_path - tile_start only one => pe_cmd 
                C_PE_TILE: begin
                    if (!tile_started) begin
                        if (ly_is_fc) o_fc_tile_start <= 1'b1;
                        else o_pg_tile_start <= 1'b1;
                        tile_started      <= 1'b1;
                        tile_in_done_seen <= 1'b0;
                    end else if (i_tile_ready) begin
                        state <= C_PE_WAIT;
                    end
                end

                // PE execute. 여기서는 탈출 조건만 본다.
                // 타일 도중의 next Chunk (fc1 : 타일당 63 회) 는 이 always 블록 위쪽의
                // Weight 적재 서비스 case (wsvc) 가 처리한다 : pe_cntl 의 i_chunk_req 를
                // chunk_accept 로 받아 nx_kbase / nx_len 을 올리고, 끝나면 o_chunk_loaded 를 낸다.
                // 다음 타일 조건 (요구사항 14) : tile_in_done + i_tile_ready (= pe_cntl IDLE && 수집기 비어 있음)
                C_PE_WAIT: begin
                    if ((i_tile_in_done || tile_in_done_seen) && i_tile_ready)
                        state <= C_NXT_TILE;
                end

                // oc_group = inner, pos_group = outer.
                // 같은 위치(patch_base 유지)에서 채널 그룹을 다 돈 뒤 다음 위치(+3)로 간다.
                C_NXT_TILE: begin
                    tile_started      <= 1'b0;
                    tcfg_sent         <= 1'b0;
                    first_chunk_rdy   <= 1'b0;
                    tile_in_done_seen <= 1'b0;

                    if (!oc_last) begin
                        oc_group  <= oc_group + 1'b1;   // same patch 3, next output channel 3. 
                        og_off_q  <= og_off_q + k_ext;
                        state <= C_W_LOAD;
                    end else if (!pos_last) begin
                        oc_group  <= {`OCG_W{1'b0}};    // OCG_W 4 → 4'd0 : next position -> again channel 0
                        og_off_q  <= {`W_AW{1'b0}};
                        pos_group <= pos_group + 1'b1;  // patch_base += 3
                        state <= C_W_LOAD;
                    end else begin
                        state <= C_LAYER_END;
                    end
                end

                // Pool/Bypass result & store
                C_LAYER_END: begin
                    if (i_layer_done || layer_done_seen) begin
                        layer_done_seen <= 1'b0;
                        if (layer_idx == (NUM_LAYERS - 1)) begin   // NUM_LAYERS = 4 -> layer_idx == 3 (fc2) = done
                            state <= C_DONE;
                        end else begin
                            region_sel <= ~region_sel;
                            layer_idx  <= layer_idx + 1'b1;
                            state      <= C_SET;
                        end
                    end
                end

                default: begin  // C_DONE
                    o_write_grant <= 1'b0;
                    state         <= C_IDLE;
                end
            endcase
        end
    end

    // -----------------------------------------------------------
    // debugging
    // ----------------------------------------------------------- 
    // synthesis translate_off
    reg [8*12-1:0] state_name;
    reg [ 8*8-1:0] wsvc_name;
    reg [ 8*5-1:0] layer_name;
    reg [8*16-1:0] tile_name;
    always @* begin
        case (layer_idx)
            3'd0:    layer_name = "Conv0";
            3'd1:    layer_name = "Conv1";
            3'd2:    layer_name = "fc1";
            3'd3:    layer_name = "fc2";
            default: layer_name = "L?";
        endcase
        $sformat(tile_name, "L%0d p%0d oc%0d", layer_idx, pos_group, oc_group);
        case (state)
            C_IDLE:       state_name = "C_IDLE";
            C_START:      state_name = "C_START";
            C_PARAM_LOAD: state_name = "C_PARAM_LOAD";
            C_SET:        state_name = "C_SET";
            C_W_LOAD:     state_name = "C_W_LOAD";
            C_TILE_CFG:   state_name = "C_TILE_CFG";
            C_PE_TILE:    state_name = "C_PE_TILE";
            C_PE_WAIT:    state_name = "C_PE_WAIT";
            C_NXT_TILE:   state_name = "C_NXT_TILE";
            C_LAYER_END:  state_name = "C_LAYER_END";
            C_DONE:       state_name = "C_DONE";
            default:      state_name = "C_???";
        endcase
        case (wsvc)
            W_IDLE:  wsvc_name = "W_IDLE";
            W_ISSUE: wsvc_name = "W_ISSUE";
            W_WAIT:  wsvc_name = "W_WAIT";
            default: wsvc_name = "W_REPLY";
        endcase
    end
    // synthesis translate_on
    // -----------------------------------------------------------
endmodule
// -----------------------------------------------------------------------------

module pe_cntl #(
    parameter integer WBUF_WORDS = 64                     // 64 : wgt_buf 의 Word 수 = chunk 최대 길이. K 가 이보다 크면 chunk 로 나눠 돈다 (fc1 의 4096 -> 64 chunk)
) (
    input wire clk,
    input wire rst_n,

    // ---- cnn_cntl : 타일 명령 (top_cnn_cntl 안에서 직접 이어진다) -------------
    input  wire                  i_tile_valid,           // <- cnn_cntl.o_tile_valid.     fire = valid && ready 에서 K 와 mask 를 저장
    output wire                  o_tile_ready,           // -> cnn_cntl.i_tile_ready      (P_IDLE 이고 앞 타일 뒤 i_result_space_ready=1 을 본 뒤 1. 명세 : 이전 타일 저장 완료까지 0).  top_cnn_cntl.o_pe_cmd_ready_dbg 로도 나간다
    input  wire [`K_W-1:0]       i_step_total,           // <- cnn_cntl.o_step_total.     타일당 MAC step 수 = K
    input  wire [`KEEP_W-1:0]    i_tile_row_mask,        // <- cnn_cntl.o_tile_row_mask.  유효 PE 행 (출력 위치)
    input  wire [`KEEP_W-1:0]    i_tile_col_mask,        // <- cnn_cntl.o_tile_col_mask.  유효 PE 열 (출력 채널)

    // ---- cnn_cntl : 타일 중간 Chunk 재적재 (K > 64 일 때만) --------------------
    //   명세 (2026-09-22 시트 R579~584) 는 아직 6 신호 버전이다 : o_chunk_req_valid / i_chunk_req_ready /
    //   o_chunk_k_base / o_chunk_len / i_chunk_done_valid / o_chunk_done_ready. 09-22 오전 결정으로
    //   아래 2 신호로 줄였고 k_base · len 은 cnn_cntl 이 센다. 시트 쪽을 고쳐야 한다.
    output wire                  o_chunk_req,            // [명세 ~] -> cnn_cntl.i_chunk_req.     Level. 앞 chunk 를 다 썼으니 다음 chunk 를 올려 달라. i_chunk_loaded 까지 유지.  명세 이름 o_chunk_req_valid
    input  wire                  i_chunk_loaded,         // [명세 ~] <- cnn_cntl.o_chunk_loaded.  다음 chunk 가 wgt_buf 에 다 들어온 뒤 1clk 펄스 -> P_PREFILL.  명세 이름 i_chunk_done_valid

    // ---- wgt_ld_unit -------------------------------------------------------
    output wire                  o_wbuf_free,            // -> wgt_ld_unit.i_buf_free (팀 결정 2026-09-21. 이 저장소에서는 아직 wgt_path.i_buffer_free).  읽는 중인 chunk 를 덮어쓰지 않게 하는 허가

    // ---- wgt_patch_gen -----------------------------------------------------
    output wire                  o_chunk_start,          // -> wgt_path.i_reader_start     (wgt_patch_gen.i_start).      chunk 마다 1clk (P_PREFILL)
    output wire [`CHUNK_W-1:0]   o_chunk_word_count,     // -> wgt_path.i_reader_chunk_len (wgt_patch_gen.i_chunk_len).  시트 R596 : pe_cntl 이 낸다 (cur_len). cnn_cntl 의 같은 이름 포트는 wgt_ld_unit 용
    input  wire                  i_chunk_done,           // <- wgt_path.o_reader_done      (wgt_patch_gen.o_done).       chunk 의 마지막 Word 가 feeder 로 넘어간 1clk 펄스

    // ---- act_feeder / wgt_feeder -------------------------------------------
    //   명세도 valid 를 i_act_valid / i_weight_valid 로 구분한다. o_feed_en 은 명세에 두 행
    //   (act_feeder / wgt_feeder) 이지만 값이 같아 한 포트가 두 feeder 로 간다.
    //   o_tile_clear 는 pe_cntl 표 (시트 R599) 에 이 이름으로 있다. skew 표 (R254 / R255) 만 아직
    //   pe_cntl.o_askew_clear / o_wskew_clear 라고 적혀 있는데 여기서는 한 선이다 (pe_core 포트가 i_tile_clear 하나).
    output wire                  o_feed_en,              // -> act_path.i_feeder_en (act_feeder.i_feed_en), wgt_path.i_feeder_en (wgt_feeder.i_feed_en).  PREFILL / FEED 에서 1
    output wire                  o_tile_clear,           // -> act_path.i_tile_clear (act_feeder.i_clear), wgt_path.i_tile_clear (wgt_feeder.i_clear), pe_core.i_tile_clear (act_skew / wgt_skew .i_clear).  시트 R599. skew 표의 o_askew_clear / o_wskew_clear 도 이 선
    input  wire                  i_weight_valid,         // <- wgt_path.o_weight_valid (wgt_feeder.o_valid). 
    input  wire                  i_act_valid,            // <- act_path.o_act_valid    (act_feeder.o_valid). 

    // ---- pe_core -----------------------------------------------------------
    output wire                  o_step_en,              // -> pe_core.i_step_en    (act_skew / wgt_skew / pe_array .i_step_en)
    output wire                  o_feed_valid,           // -> pe_core.i_feed_valid (act_skew / wgt_skew .i_feed_valid)
    output wire                  o_acc_clear,            // -> pe_core.i_acc_clear  (pe_array.i_acc_clear)
    output wire [`PE_N-1:0]      o_mac_valid,            // -> pe_core.i_mac_valid  (pe_array.i_mac_valid)
    output wire [`PE_N-1:0]      o_mac_last,             // -> pe_core.i_mac_last   (pe_array.i_mac_last)

    // ---- out_path ----------------------------------------------------------
    input  wire                  i_result_space_ready,   // <- out_path.o_result_space_ready (output_fifo.o_result_space_ready).  수집기가 새 타일을 받을 수 있다. o_tile_ready 에만 들어간다 (space_seen, 아래 assign 주석).  타일 단위 의미로 팀 확정 (2026-09-22). 시트 R588 / R597 / R598 / R604 를 이 의미로 고쳐야 한다
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
    reg [2:0]           state;
    reg [`K_W-1:0]      k_left;             // 이 타일에 남은 beat 수 (지금 것 포함). 1 이면 지금 beat 가 타일의 마지막
    reg [`CHUNK_W-1:0]  c_left;             // 이번 chunk 에 남은 beat 수.            1 이면 지금 beat 가 chunk 의 마지막
    reg [`CHUNK_W-1:0]  cur_len;            // 이번 chunk 의 길이. reader 에 주는 값이라 chunk 동안 바뀌지 않는다
    reg [`KEEP_W-1:0]   row_mask_q;         // 유효 PE 행 (출력 위치). 꼬리 타일은 011 / 001
    reg [`KEEP_W-1:0]   col_mask_q;         // 유효 PE 열 (출력 채널)
    reg                 reader_done_seen;   // i_chunk_done   (1clk 펄스) 을 봤다
    reg                 tile_in_done_seen;  // i_tile_in_done (1clk 펄스) 을 봤다
    reg                 space_seen;         // P_IDLE 에서 i_result_space_ready=1 을 봤다. 타일 명령 fire 에 지운다
    reg [4:0]           valid_pipe;         // 대각선 d = r + c 의 PE 가 보는 "이번 step 에 MAC 한다" 토큰
    reg [4:0]           last_pipe;          // 같은 자리의 "이게 타일의 마지막 MAC 이다" 토큰

    wire inject             = (state == P_FEED)  && i_act_valid && i_weight_valid; // pe array 1 step data push possible
    wire drain_advance      = (state == P_DRAIN);
    wire tile_last_beat     = (k_left == K_ONE);
    wire chunk_last_beat    = (c_left == C_ONE);
    wire feeding            = (state == P_PREFILL) || (state == P_FEED);
    wire clearing           = (state == P_CLEAR);

    // K = 0 이 들어오면 k_left 가 1 을 지나치지 못해 멈춘다. 최소 1 로 받는다.
    wire [`K_W-1:0] k_init = (i_step_total == {`K_W{1'b0}}) ? K_ONE : i_step_total;

    // 새 타일 수락 가능 (명세 o_cmd_ready : "P_IDLE 에서 1. 이전 타일 저장 완료까지 0")
    //   = P_IDLE 이고, 앞 타일이 끝난 뒤 수집기가 빈 것 (i_result_space_ready=1) 을 한 번이라도 봤다.
    //   space_seen 이 필요한 이유 : 타일 명령은 tile_cfg 뒤에 오는데, 요구사항 문서대로면 수집기는
    //   tile_cfg 를 받은 순간부터 다시 0 이다. 지금 값만 보면 이 타일의 명령을 영영 못 받는다.
    //   out_path.o_result_space_ready 는 여기에만 쓴다. cnn_cntl 은 이 선 하나로 tile_cfg 발행
    //   (요구사항 4) 과 다음 타일 진입 (요구사항 14) 을 막는다. inject / DRAIN 에는 걸지 않는다 :
    //   타일 활성 중 0 인 신호라 step 단위로 걸면 첫 inject 에서 멈춘다.
    assign o_tile_ready = (state == P_IDLE) && (i_result_space_ready || space_seen);

    // 앞 chunk 를 다 주입했고 (P_CHUNK_WAIT), reader 도 다 넘겼다고 알려 온 뒤에 요청한다.
    // Level 이라 cnn_cntl 이 늦게 봐도 사라지지 않는다. i_chunk_loaded 를 받으면 P_PREFILL 로 가며 내려간다.
    assign o_chunk_req = (state == P_CHUNK_WAIT) && reader_done_seen;

    // wgt_buf 덮어쓰기 허가 : 타일이 없거나, chunk 대기 중이고 reader 가 마지막 Word 를 넘겼다 (reader_done_seen).
    //   P_CHUNK_WAIT 는 이 chunk 의 마지막 beat 를 주입한 뒤에 들어오고, reader 는 chunk 길이만큼만 읽으므로
    //   reader_done_seen=1 이면 reader 에 남은 읽기도 없고 feeder 에 남은 Word 도 없다.
    //   (예전의 i_reader_idle / i_weight_feeder_empty / o_reader_issue_en 은 2026-09-22 팀 결정으로 없앴다.
    //    wgt_patch_gen · wgt_feeder 표에 받는 / 내는 포트가 없는 신호였다.)
    assign o_wbuf_free        = (state == P_IDLE) ||
                                ((state == P_CHUNK_WAIT) && reader_done_seen);

    assign o_chunk_start        = (state == P_PREFILL);  // P_PREFILL 은 1clk
    assign o_chunk_word_count   = cur_len;
    assign o_feed_en            = feeding;

    assign o_tile_clear         = clearing;
    assign o_acc_clear          = clearing;

    assign o_step_en            = inject || drain_advance;
    assign o_feed_valid         = inject;

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
            space_seen        <= 1'b0;
            valid_pipe        <= 5'b00000;
            last_pipe         <= 5'b00000;
        end else begin

            //   i_chunk_done   : chunk 의 마지막 Word 가 feeder 로 넘어간 clk. 그 Word 를 주입하기 전에 온다.
            //   i_tile_in_done : 유효 lane 이 하나뿐인 꼬리 타일은 출력이 한 beat 라 아직 P_DRAIN 인 동안
            //                    지나간다. P_WAIT_OUT 에서 level 만 보면 놓치고 영원히 멈춘다.

            // 1 pulse signal
            if (i_chunk_done)   reader_done_seen <= 1'b1;
            if (i_tile_in_done) tile_in_done_seen <= 1'b1;

            case (state)
                P_IDLE: begin
                    tile_in_done_seen <= 1'b0;
                    if (i_result_space_ready) space_seen <= 1'b1;
                    if (i_tile_valid && o_tile_ready) begin                 // o_tile_ready = 1 -> now clk 
                        space_seen <= 1'b0;                 // 이 타일이 끝난 뒤 수집기가 비는 것을 다시 봐야 한다
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
                    reader_done_seen <= i_chunk_done;       // new chunk -> signal LOW set
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

                // insert done but, waiting all data outs.
                P_DRAIN: begin
                    if (drain_advance) begin
                        valid_pipe <= {valid_pipe[3:0], 1'b0};
                        last_pipe  <= {last_pipe[3:0], 1'b0};
                        if (valid_pipe[3:0] == 4'b0000) state <= P_DONE;
                    end
                end

                // pe_core stop -> out_path signal wait
                P_DONE: begin
                    if (i_tile_in_done || tile_in_done_seen) state <= P_IDLE;
                end

                default: state <= P_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // debugging
    // ----------------------------------------------------------- 
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
            default:      state_name = " ";
        endcase
    end
    // synthesis translate_on
    // -----------------------------------------------------------
endmodule
// -----------------------------------------------------------------------------

// =============================================================================
// top_cnn_cntl : cnn_cntl + pe_cntl
// cnn_cntl : Layer / Tile Discriptor , pe_cntl - inner cycle control
// [1] Tile cmd  
//       cnn_cntl                                 pe_cntl
//       o_tile_valid         -------------->  i_tile_valid
//       i_tile_ready         <--------------  o_tile_ready
//       o_step_total    (13) -------------->  i_step_total       (= K)
//       o_tile_row_mask ( 3) -------------->  i_tile_row_mask
//       o_tile_col_mask ( 3) -------------->  i_tile_col_mask
//
//     fire = valid && ready 인 에지에서 pe_cntl 이 K 와 mask 를 저장한다.
//
// [2] 타일 중간 Chunk reload  (pe_cntl : request , cnn_cntl : response)
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
// [3] Done signal
//       pooling_unit.o_tile_in_done ---+--> cnn_cntl.i_tile_in_done
//                                      +--> pe_cntl.i_tile_in_done
//       1 clk pulse -> seen : storing state

module top_cnn_cntl #(
    parameter integer NUM_LAYERS = 4,
    parameter integer WBUF_WORDS = 64,

    parameter [`ACT_AW-1:0] REGION_A_BASE = 14'd0,
    parameter [`ACT_AW-1:0] REGION_B_BASE = 14'd8192,

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

    // ---- AXI4-Lite CSR (SoC) ------------------------------------------
    input  wire                  i_start_valid,          // [C] <- cnn_accel_top.i_start_valid  (가속기 밖 AXI4-Lite CSR 의 CTRL.cnn_start)
    output wire                  o_start_ready,          // [C] -> cnn_accel_top.o_start_ready  (CSR. IDLE 이고 done_status=0 일 때만 1)
    output wire                  o_busy,                 // [C] -> cnn_accel_top.o_busy         (CSR BUSY)

    // ---- act_ld_unit -------------------------------------------------------
    output wire                  o_img_ld_start,         // [C] -> act_path.i_img_ld_start      (act_ld_unit.i_img_ld_start)
    input  wire                  i_ld_done,              // [C] <- act_ld_unit.o_ld_done.  초기 이미지 적재 완료 1clk 펄스
    output wire                  o_writer_mode,          // [C] -> act_path.i_writer_mode       (act_ld_unit.i_mode : 0=이미지 적재, 1=결과 쓰기)

    // ---- act_patch_gen / fc_gen : Layer descriptor ------------------------
    output wire                  o_layer_cfg_valid,      // [C] -> act_path.i_layer_cfg_valid   (Conv 면 act_patch_gen, FC 면 fc_gen 의 i_layer_cfg_valid. path_sel 로 갈린다)
    output wire [`ACT_AW-1:0]    o_src_base,             // [C] -> act_path.i_src_base          (act_patch_gen / fc_gen .i_src_base)
    output wire [`DIM_W-1:0]     o_in_w,                 // [C] -> act_path.i_in_w              (act_patch_gen / fc_gen .i_in_w)
    output wire [`DIM_W-1:0]     o_in_h,                 // [C] -> act_path.i_in_h              (act_patch_gen / fc_gen .i_in_h)
    output wire [`CH_W-1:0]      o_in_c,                 // [C] -> act_path.i_in_c              (act_patch_gen / fc_gen .i_in_c)
    output wire [1:0]            o_stride,               // [C] -> act_path.i_stride            (act_patch_gen.i_stride)
    output wire                  o_pad_en,               // [C] -> act_path.i_pad_en            (act_patch_gen.i_pad_en)
    output wire [`K_W-1:0]       o_k_total,              // [C] -> act_path.i_k_total (act_patch_gen / fc_gen), wgt_path.i_k_total (wgt_ld_unit : 주소 = base + og*K + kb).  안에서는 pe_cntl.i_step_total 로도 간다 (cnn_cntl.o_step_total)
    output wire                  o_path_sel,             // [C] -> act_path.i_path_sel (act_mux.i_sel + input_buf 읽기 포트 선택), out_path.i_is_fc (output_fifo.i_is_fc).  0=Conv, 1=FC

    // ---- Tile Setting ----------------------------------------------------------
    output wire                  o_pg_tile_start,        // [C] -> act_path.i_pg_tile_start     (act_patch_gen.i_tile_start)
    output wire                  o_fc_tile_start,        // [C] -> act_path.i_fc_tile_start     (fc_gen.i_tile_start)
    output wire [13:0]           o_pos_base,             // [C] -> act_path.i_pos_base (act_patch_gen R116, 14b), out_path.i_patch_base (output_fifo R320, 12b : 하위 12b).  cnn_cntl 의 12b 값을 0 확장
    output wire [`KEEP_W-1:0]    o_row_mask,             // [C] -> act_path.i_row_mask (act_patch_gen.i_row_mask), out_path.i_row_mask (output_fifo).  안에서는 pe_cntl.i_tile_row_mask 로도 간다 (cnn_cntl.o_tile_row_mask)
    output wire [`KEEP_W-1:0]    o_col_mask,             // [C] -> out_path.i_col_mask          (output_fifo / post_process .i_col_mask).  안에서는 pe_cntl.i_tile_col_mask 로도 간다 (cnn_cntl.o_tile_col_mask)
    output wire                  o_tile_cfg_valid,       // [C] -> out_path.i_tile_cfg_valid    (output_fifo / post_process .i_cfg_valid)
    output wire [`OCB_W-1:0]     o_out_ch_base,          // [C] -> out_path.i_out_ch_base       (output_fifo / post_process .i_out_ch_base)
    output wire                  o_tile_last,            // [C] -> out_path.i_tile_last         (output_fifo.i_tile_last -> meta.layer_end)
    output wire                  o_relu_en,              // [C] -> out_path.i_relu_en           (post_process.i_relu_en)
    output wire [5:0]            o_param_base,           // [C] -> out_path.i_bias_base (post_process R319, 6b).  cnn_cntl 의 7b 값 (0 / 6 / 10 / 42) 의 하위 6b
    input  wire                  i_params_ready,         // [C] <- out_path.o_params_ready      (post_process.o_params_ready)

    // ---- Layer output path ---------------------------------------------------
    output wire                  o_output_cfg_valid,     // [C] -> act_path.i_output_cfg_valid (act_ld_unit.i_layer_cfg_valid), out_path.i_output_cfg_valid (output_fifo / pooling_unit / result_buf 의 레이어 cfg)
    output wire [`ACT_AW-1:0]    o_dst_base,             // [C] -> act_path.i_dst_base          (act_ld_unit.i_dst_base : 결과를 쓸 영역)
    output wire [`CH_W-1:0]      o_out_c,                // [C] -> act_path.i_out_c             (act_ld_unit.i_out_c : 쓰기 주소의 ceil(out_c/3))
    output wire [`DIM_W-1:0]     o_pool_in_w,            // [C] -> out_path.i_pool_in_w         (pooling_unit.i_in_w, output_fifo.i_conv_w : 패치 순번 -> pos 변환의 W)
    output wire [`DIM_W-1:0]     o_pool_in_h,            // [C] -> out_path.i_conv_h            (pooling_unit.i_in_h)
    output wire [`CH_W-1:0]      o_pool_c,               // [C] -> out_path.i_pool_c            (pooling_unit.i_channels)
    output wire                  o_pool_en,              // [C] -> out_path.i_pool_en           (pooling_unit.i_pool_en)
    output wire                  o_is_final_layer,       // [C] -> out_path.i_is_final_layer    (result_buf.i_is_final_layer)

    // ---- wgt_ld_unit -------------------------------------------------------
    output wire                  o_wload_start,          // [C] -> wgt_ld_unit.i_ld_start.  o_ld_ready 를 본 clk 의 1clk 펄스
    input  wire                  i_wload_ready,          // [C] <- wgt_ld_unit.o_ld_ready  (= IDLE && i_buf_free)
    output wire [31:0]           o_mem_base,             // [C] -> wgt_ld_unit.i_mem_base (R185, 32b).  cnn_cntl 의 16b 논리 Word 주소 (0 ~ 45249) 를 0 확장
    output wire [`CHUNK_W-1:0]   o_load_chunk_len,       // [C] -> wgt_path.i_chunk_len         (wgt_ld_unit.i_chunk_len)
    input  wire                  i_wload_done,           // [C] <- wgt_ld_unit.o_ld_done.  Chunk 적재 완료 1clk 펄스
    output wire                  o_wbuf_free,            // [P] -> wgt_path.i_buffer_free       (wgt_ld_unit.i_buffer_free : 읽는 중인 chunk 를 덮어쓰지 않게)

    // ---- wgt_patch_gen -----------------------------------------------------
    output wire                  o_reader_start,         // [P] -> wgt_path.i_reader_start      (wgt_patch_gen.i_start)
    output wire [`CHUNK_W-1:0]   o_reader_chunk_len,     // [P] -> wgt_path.i_reader_chunk_len  (wgt_patch_gen.i_chunk_len)
    input  wire                  i_reader_done,          // [P] <- wgt_path.o_reader_done       (wgt_patch_gen.o_done)

    // ---- Feeder ------------------------------------------------------------
    output wire [`KEEP_W-1:0]    o_pe_col_mask,          // [C] -> wgt_path.i_col_mask.  명세 : cnn_cntl.o_col_mask -> wgt_patch_gen.i_col_mask.  이 저장소의 wgt_path 는 같은 값을 wgt_feeder 가 받는다
    output wire                  o_feeder_en,            // [P] -> act_path.i_feeder_en (act_feeder.i_feed_en), wgt_path.i_feeder_en (wgt_feeder.i_feed_en)
    output wire                  o_tile_clear,           // [P] -> act_path.i_tile_clear (act_feeder.i_clear), wgt_path.i_tile_clear (wgt_feeder.i_clear), pe_core.i_tile_clear (act_skew / wgt_skew .i_clear)
    input  wire                  i_weight_valid,         // [P] <- wgt_path.o_weight_valid      (wgt_feeder.o_valid).  같은 선이 pe_core.i_wgt_valid 로도 간다
    input  wire                  i_act_valid,            // [P] <- act_path.o_act_valid         (act_feeder.o_valid).  같은 선이 pe_core.i_act_valid 로도 간다

    // ---- pe_core -----------------------------------------------------------
    output wire                  o_step_en,              // [P] -> pe_core.i_step_en            (act_skew / wgt_skew / pe_array .i_step_en)
    output wire                  o_feed_valid,           // [P] -> pe_core.i_feed_valid         (act_skew / wgt_skew .i_feed_valid)
    output wire                  o_acc_clear,            // [P] -> pe_core.i_acc_clear          (pe_array.i_acc_clear)
    output wire [`PE_N-1:0]      o_mac_valid,            // [P] -> pe_core.i_mac_valid          (pe_array.i_mac_valid)
    output wire [`PE_N-1:0]      o_mac_last,             // [P] -> pe_core.i_mac_last           (pe_array.i_mac_last)

    // ---- output_fifo -------------------------------------------------------
    input  wire                  i_result_space_ready,   // [P] <- out_path.o_result_space_ready (output_fifo.o_result_space_ready).  pe_cntl.o_tile_ready 에 들어간다 (요구사항 4 · 14)

    // ---- RAM (Read Parameter) ---------------------------------------------
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
    output wire [5:0]            o_param_wr_addr,        // [C] -> out_path.i_param_wr_addr (param_buf R330, 6b).  cnn_cntl 의 7b 값 (0 ~ 42) 의 하위 6b
    output wire [31:0]           o_param_wr_data,        // [C] -> out_path.i_param_wr_data (param_buf).  signed INT32 Bias (명세 32bit)
    output wire                  o_param_load_start,     // [C] -> out_path.i_param_load_start (param_buf).  Bias 적재 시작 1clk
    input  wire                  i_param_load_done,      // [C] <- out_path.o_param_load_done  (param_buf).  43 번째 Bias 저장 직후 1clk
    output wire [31:0]           o_quant_multiplier,     // [C] -> out_path.i_quant_multiplier.  현재 레이어 공통 M
    output wire [5:0]            o_quant_shift,          // [C] -> out_path.i_quant_shift.       현재 레이어 공통 S (0 ~ 30)

    // ---- Done signal --------------------------------------------------------------
    input  wire                  i_tile_in_done,         // [C+P] <- out_path.o_tile_in_done    (pooling_unit.o_tile_in_done)
    input  wire                  i_layer_done,           // [C] <- out_path.o_layer_done        (result_buf.o_layer_done)
    input  wire                  i_done_status,          // [C] <- out_path.o_done_status       (result_buf.o_done_status).  같은 선이 cnn_accel_top.o_done_status (CSR) 로도 나간다

    // ---- Debugging ------------------------------------------------------------
    output wire [2:0]            o_layer_idx,            // [C] -> cnn_accel_top.o_layer_idx
    output wire [3:0]            o_cnn_state,            // [C] -> cnn_accel_top.o_cnn_state
    output wire                  o_pe_cmd_ready_dbg      // [P] -> cnn_accel_top 에서는 미연결. tb_top_cnn_cntl 이 받아서 pe_cmd fire 를 본다
);

    // ===== [1] Tile command =====
    wire pe_cmd_valid, pe_cmd_ready;
    wire [`K_W-1:0] pe_step_total;                  // = o_k_total
    wire [`KEEP_W-1:0] pe_row_mask, pe_col_mask;    // = o_row_mask / o_col_mask

    // -------------------------------------------------------------------------
    // 이름 다리
    //   cnn_cntl / pe_cntl 의 포트는 인터페이스 명세서 이름이고, 이 묶음 모듈의 바깥
    //   포트는 데이터패스 (act_path / wgt_path / pe_core / out_path, cnn_accel_top) 가
    //   쓰는 이름 그대로다. 다른 모듈은 고치지 않으므로 이름은 여기서만 바뀐다.
    //
    //     안 (명세 이름)                     바깥 (그대로)
    //     o_ld_start                  ->  o_img_ld_start
    //     o_write_grant               ->  o_writer_mode
    //     o_fc_mode, o_is_fc          ->  o_path_sel              (같은 값)
    //     o_bias_base                 ->  o_param_base
    //     o_conv_w, o_conv_h          ->  o_pool_in_w, o_pool_in_h  (o_result_clear 는 2026-09-22 팀 결정으로 없앴다. o_conv_h 는 같은 날 다시 넣었다)
    //     o_out_channels              ->  o_out_c, o_pool_c       (같은 값)
    //     o_layer_cfg_valid           ->  o_layer_cfg_valid, o_output_cfg_valid   (같은 펄스)
    //     o_pos_base, o_patch_base    ->  o_pos_base              (같은 값)
    //     o_chunk_word_count (cnn)    ->  o_load_chunk_len
    //     o_chunk_start / o_chunk_word_count / i_chunk_done (pe)  ->  o_reader_start / o_reader_chunk_len / i_reader_done
    //     o_feed_en                   ->  o_feeder_en
    // -------------------------------------------------------------------------

    // 폭 다리 : cnn_cntl 은 명세 cnn_cntl 표 폭 (12 / 16 / 7 / 7), 바깥 포트는 받는 쪽 표 폭 (14 / 32 / 6 / 6)
    wire [`POS_W-1:0]    pos_base_12;
    wire [`W_AW-1:0]     mem_base_16;
    wire [`PARAM_AW-1:0] param_base_7, param_wr_addr_7;
    assign o_pos_base      = {2'b00, pos_base_12};
    assign o_mem_base      = {16'd0, mem_base_16};
    assign o_param_base    = param_base_7[5:0];
    assign o_param_wr_addr = param_wr_addr_7[5:0];

    assign o_pe_col_mask        = o_col_mask;
    assign o_output_cfg_valid   = o_layer_cfg_valid;  // 레이어 cfg 펄스 하나가 act_path 와 out_path 양쪽으로 간다
    assign o_pool_c             = o_out_c;            // 출력 채널 수 하나가 act_ld_unit 과 pooling_unit 양쪽으로 간다

    wire chunk_req, chunk_loaded;
    assign o_pe_cmd_ready_dbg   = pe_cmd_ready;

    cnn_cntl #(
        .NUM_LAYERS(NUM_LAYERS),
        .WBUF_WORDS(WBUF_WORDS),
        .REGION_A_BASE(REGION_A_BASE),
        .REGION_B_BASE(REGION_B_BASE),
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
        .o_stride(o_stride),
        .o_pad_en(o_pad_en),
        .o_k_total(o_k_total),
        .o_fc_mode(o_path_sel),
        .o_pg_tile_start(o_pg_tile_start),
        .o_fc_tile_start(o_fc_tile_start),
        .o_pos_base(pos_base_12),
        .o_row_mask(o_row_mask),
        .o_col_mask(o_col_mask),
        .o_tile_cfg_valid(o_tile_cfg_valid),
        .o_out_ch_base(o_out_ch_base),
        .o_tile_last(o_tile_last),
        .o_relu_en(o_relu_en),
        .o_bias_base(param_base_7),
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
        .o_mem_base(mem_base_16),
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
        .o_param_wr_addr(param_wr_addr_7),
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
        .i_chunk_done(i_reader_done),
        .o_feed_en(o_feeder_en),
        .o_tile_clear(o_tile_clear),
        .i_weight_valid(i_weight_valid),
        .i_act_valid(i_act_valid),
        .o_step_en(o_step_en),
        .o_feed_valid(o_feed_valid),
        .o_acc_clear(o_acc_clear),
        .o_mac_valid(o_mac_valid),
        .o_mac_last(o_mac_last),
        .i_result_space_ready(i_result_space_ready),
        .i_tile_in_done(i_tile_in_done)
    );

    // -----------------------------------------------------------
    // debugging
    // ----------------------------------------------------------- 
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
    // -----------------------------------------------------------
endmodule
