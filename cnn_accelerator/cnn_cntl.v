`timescale 1ns / 1ps
`default_nettype none

// -----------------------------------------------------------------------------
// CNN 레이어 / 타일 / Pool 제어 유닛
//
// 기준 문서 : 통합 인터페이스 시트 tab gid=457801884 (2026-09-19) 의 CNN_CNTL 절.
//   이 탭은 구 v1.2 탭(gid=1732006064)을 대체한다. 달라진 점은
//     - MaxPool 이 별도 pass 로 분리됨
//     - Weight Loader / Feature Writer 가 valid-ready 핸드셰이크로 바뀜
//     - FC 의 chunk 루프가 PE 타일 "안"으로 들어감 (pe_cntl 이 요청, 여기서 적재)
//
// 작업 구조
//   layer -> oc_group -> pos_group 의 3중 루프. 타일 하나는
//   출력위치 3개(PE 행) x 출력채널 3개(PE 열) 를 레이어 전체 K 만큼 누산한다.
//   K 가 Weight Buffer(64 word) 보다 길면 pe_cntl 이 타일 도중에 다음 chunk 를
//   요청(i_chunk_req_*)하고, 이 블록이 누산기를 건드리지 않고 적재해 준다.
//
//   POOL_AFTER 로 표시된 레이어는 모든 타일 저장이 끝난 뒤, 방금 쓴 bank 를
//   읽어서 MaxPool pass 를 따로 한 번 돌린다.
//
// 메인 FSM (Notion "Control Unit 설계"와 동일한 이름)
//   C_IDLE -> C_START -> C_SET -> C_W_LOAD -> C_PARAM_LOAD
//          -> C_PE_TILE -> C_PE_WAIT -> C_NXT_TILE
//          -> (C_W_LOAD | C_POOL | C_LAYER_END) -> C_DONE
// C_W_LOAD 와 C_PARAM_LOAD 를 분리해 단일 포트 RAM에서도 weight read와
// parameter read가 겹치지 않는다. 저장원은 ROM이 아니라 외부 RAM이다.
//
// 레이어 구성 (2026-09-19 확정)
//   시트 04절 참조 모델의 5 레이어 구조를 쓰되, FC 폭은 팀 블록 다이어그램대로
//   fc1 4096->64 / fc2 64->64 / fc3 64->1 이다. 시트 04절 본문의 32 는 구버전.
//
//   !! 이 64 폭 때문에 시트에 적힌 포트 폭 5 개가 부족해져서 넓혔다.
//      아래 "폭 확장" 주석 참고. 시트도 같이 고쳐야 한다.
//
//   Notion "CNN Layer 구조 선택_rev3" 의 Candidate ID 32 로 바뀔 가능성이 있어
//   파라미터 목록 뒤에 프리셋을 주석으로 남겨 두었다. RTL 수정 없이 전환 가능.
// -----------------------------------------------------------------------------
module cnn_cntl #(
    parameter integer NUM_LAYERS = 5,

    // Weight Buffer 깊이 (24bit word 단위)
    parameter integer WBUF_WORDS = 64,

    // ---- 폭 확장 : 시트 원래 폭으로는 fc 64 폭을 표현할 수 없어 넓힌 값 -------
    //   W_AW     16 -> 18 : weight word 총 91,962 개 > 65,536
    //   PARAM_AW  7 ->  8 : param 레코드 총 151 개 > 128
    //   OCG_W     4 ->  5 : ceil(64/3) = 22 그룹 > 15  (조용히 틀리는 항목)
    //   OCB_W     5 ->  6 : oc_base 최대 63 > 31
    //   CH_W      6 ->  7 : out_c 64 > 63
    parameter integer W_AW     = 18,
    parameter integer PARAM_AW = 8,
    parameter integer OCG_W    = 5,
    parameter integer OCB_W    = 6,
    parameter integer CH_W     = 7,

    // Input Buffer 는 24bit x 16384 word 이고 8192 word bank 두 개로 나눠 쓴다.
    // Conv0 의 raw 출력 64*64*ceil(6/3) = 8192 word 가 bank 하나에 딱 들어간다.
    parameter [13:0] BANK_A_BASE = 14'd0,
    parameter [13:0] BANK_B_BASE = 14'd8192,

    // padding 에 쓸 실수 0 에 해당하는 signed INT8 값. 대칭 양자화면 0.
    parameter [7:0]  INPUT_ZP    = 8'd0,

    // ---- Layer 0 : Conv0 3x3 3->6, S=1, P=1, ReLU, 뒤에 MaxPool 2x2 S=2 ----
    parameter [6:0]  L0_IN_W = 7'd64,  parameter [6:0]  L0_IN_H = 7'd64,
    parameter [5:0]  L0_IN_C = 6'd3,   parameter [6:0]  L0_OUT_W = 7'd64,
    parameter [6:0]  L0_OUT_H = 7'd64, parameter [CH_W-1:0] L0_OUT_C = 7'd6,
    parameter [1:0]  L0_STRIDE = 2'd1, parameter        L0_PAD_EN = 1'b1,
    parameter [12:0] L0_K      = 13'd27,            // 3*3*3
    parameter [12:0] L0_OUT_PIX = 13'd4096,         // 64*64
    parameter [W_AW-1:0] L0_W_BASE = 18'd0,         //  2 grp *   27 =     54
    parameter [PARAM_AW-1:0] L0_PARAM_BASE = 8'd0,  // 채널 0..5
    parameter        L0_RELU = 1'b1,   parameter        L0_POOL_AFTER = 1'b1,

    // ---- Layer 1 : Conv1 3x3 6->16, S=2, P=1, ReLU ------------------------
    parameter [6:0]  L1_IN_W = 7'd32,  parameter [6:0]  L1_IN_H = 7'd32,
    parameter [5:0]  L1_IN_C = 6'd6,   parameter [6:0]  L1_OUT_W = 7'd16,
    parameter [6:0]  L1_OUT_H = 7'd16, parameter [CH_W-1:0] L1_OUT_C = 7'd16,
    parameter [1:0]  L1_STRIDE = 2'd2, parameter        L1_PAD_EN = 1'b1,
    parameter [12:0] L1_K      = 13'd54,            // 3*3*6
    parameter [12:0] L1_OUT_PIX = 13'd256,          // 16*16
    parameter [W_AW-1:0] L1_W_BASE = 18'd54,        //  6 grp *   54 =    324
    parameter [PARAM_AW-1:0] L1_PARAM_BASE = 8'd6,  // 채널 6..21
    parameter        L1_RELU = 1'b1,   parameter        L1_POOL_AFTER = 1'b0,

    // ---- Layer 2 : fc1  4096 -> 64, ReLU ----------------------------------
    parameter [12:0] L2_K      = 13'd4096,          // flatten 16*16*16
    parameter [CH_W-1:0] L2_OUT_C = 7'd64,
    parameter [W_AW-1:0] L2_W_BASE = 18'd378,       // 22 grp * 4096 = 90,112
    parameter [PARAM_AW-1:0] L2_PARAM_BASE = 8'd22, // 채널 22..85
    parameter        L2_RELU = 1'b1,

    // ---- Layer 3 : fc2  64 -> 64, ReLU ------------------------------------
    parameter [12:0] L3_K      = 13'd64,
    parameter [CH_W-1:0] L3_OUT_C = 7'd64,
    parameter [W_AW-1:0] L3_W_BASE = 18'd90490,     // 22 grp *   64 =  1,408
    parameter [PARAM_AW-1:0] L3_PARAM_BASE = 8'd86, // 채널 86..149
    parameter        L3_RELU = 1'b1,

    // ---- Layer 4 : fc3  64 -> 1, 최종 회귀 출력 ----------------------------
    // 블록 다이어그램에는 fc3 뒤에 ReLU 가 그려져 있지만 steering angle 은
    // -90~+90 의 부호 있는 값이고, Notion rev3 도 최종 FC 는 requantization 없이
    // INT32 누산값을 유지한 뒤 angle scale 을 적용한다고 되어 있어서 기본은 끔.
    // 팀에서 켜기로 하면 L4_RELU 만 바꾸면 된다.
    parameter [12:0] L4_K      = 13'd64,
    parameter [CH_W-1:0] L4_OUT_C = 7'd1,
    parameter [W_AW-1:0] L4_W_BASE = 18'd91898,     //  1 grp *   64 =     64
    parameter [PARAM_AW-1:0] L4_PARAM_BASE = 8'd150,// 채널 150
    parameter        L4_RELU   = 1'b0
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
    output reg         o_frame_consume,
    output wire        o_writer_mode,        // 0 = 초기 frame, 1 = 추론 결과
    output wire [13:0] o_dst_base,
    output wire [CH_W-1:0] o_out_c,

    // ---- 레이어 descriptor : Patch Gen / FC Feeder ------------------------
    output reg         o_layer_cfg_valid,
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
    output wire        o_path_sel,           // 0 = Patch Gen, 1 = FC Feeder
    output wire [1:0]  o_read_sel,           // 0 = PG, 1 = FC, 2 = Pool Feeder

    // ---- 타일 설정 : 피더 / Output FIFO / Output Processing / Result ------
    output reg         o_pg_tile_start,
    output reg         o_fc_tile_start,
    output wire [11:0] o_pos_base,
    output wire [2:0]  o_row_mask,
    output wire [2:0]  o_col_mask,
    output reg         o_tile_cfg_valid,
    output wire [OCB_W-1:0] o_out_ch_base,
    output wire        o_tile_last,
    output wire        o_is_final_layer,
    output wire        o_relu_en,
    output wire [PARAM_AW-1:0] o_param_base, // RAM param_addr의 layer base
    input  wire        i_params_ready,

    // ---- PE_CNTL 타일 명령 -------------------------------------------------
    output wire        o_pe_cmd_valid,
    input  wire        i_pe_cmd_ready,

    // ---- PE_CNTL 의 타일 도중 weight chunk 요청 ----------------------------
    input  wire        i_chunk_req_valid,
    output wire        o_chunk_req_ready,
    input  wire [12:0] i_chunk_k_base,
    input  wire [6:0]  i_chunk_len,
    output wire        o_chunk_done_valid,
    input  wire        i_chunk_done_ready,

    // ---- Weight Loader ----------------------------------------------------
    output wire        o_wload_valid,
    input  wire        i_wload_ready,
    output wire [W_AW-1:0] o_weight_base, // RAM weight_addr의 layer base
    output wire [OCG_W-1:0] o_oc_group,
    output wire [12:0] o_load_k_base,
    output wire [6:0]  o_load_chunk_len,
    input  wire        i_wload_done_valid,
    output wire        o_wload_done_ready,

    // ---- MaxPool pass : Pool Feeder / MaxPooling / Writer / Result --------
    output reg         o_pool_cfg_valid,
    output reg         o_pool_start,
    output wire [13:0] o_pool_src_base,
    output wire [13:0] o_pool_dst_base,
    output wire [6:0]  o_pool_in_w,
    output wire [6:0]  o_pool_in_h,
    output wire [CH_W-1:0] o_pool_c,
    output wire        o_pool_en,
    input  wire        i_pool_done,

    // ---- Result + IRQ -----------------------------------------------------
    input  wire        i_tile_done,

    // ---- 디버그 -----------------------------------------------------------
    output wire [2:0]  o_layer_idx
);

    // -------------------------------------------------------------------------
    // Candidate ID 32 프리셋 (Notion "CNN Layer 구조 선택_rev3").
    // 나중에 이 구조로 바뀌면 인스턴스에서 파라미터만 덮어쓰면 된다.
    //   NUM_LAYERS    = 4
    //   L1_OUT_W/H    = 32 / 32,  L1_OUT_C = 4,  L1_STRIDE = 1
    //   L1_OUT_PIX    = 1024,     L1_W_BASE = 54       ( 2 grp *   54 =  108)
    //   L2_K = 4096,  L2_OUT_C = 32, L2_W_BASE = 162   (11 grp * 4096 =45056)
    //   L2_PARAM_BASE = 10
    //   L3_K = 32,    L3_OUT_C = 1,  L3_W_BASE = 45218 ( 1 grp *   32 =   32)
    //   L3_PARAM_BASE = 42,       L3_RELU = 0          (L3 가 최종 레이어)
    // 이 구성이면 weight word 총 45,250 개 / param 43 개라서 시트 원래 폭
    // (W_AW=16, PARAM_AW=7, OCG_W=4, OCB_W=5, CH_W=6) 으로도 충분하다.
    // -------------------------------------------------------------------------

    localparam [3:0] C_IDLE       = 4'd0;
    localparam [3:0] C_START      = 4'd1;
    localparam [3:0] C_SET        = 4'd2;
    localparam [3:0] C_W_LOAD     = 4'd3;
    localparam [3:0] C_PARAM_LOAD = 4'd4;
    localparam [3:0] C_PE_TILE    = 4'd5;
    localparam [3:0] C_PE_WAIT    = 4'd6;
    localparam [3:0] C_NXT_TILE   = 4'd7;
    localparam [3:0] C_POOL       = 4'd8;
    localparam [3:0] C_LAYER_END  = 4'd9;
    localparam [3:0] C_DONE       = 4'd10;

    // weight 적재 서비스 sub-FSM. 타일의 첫 chunk 와 pe_cntl 이 도중에 요청하는
    // chunk 를 같은 경로로 처리한다.
    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_LOAD = 2'd1;
    localparam [1:0] S_WAIT = 2'd2;
    localparam [1:0] S_ACK  = 2'd3;

    localparam [12:0] WBUF_LEN = WBUF_WORDS;

    reg [3:0]  state;
    reg [2:0]  layer_idx;
    reg        bank_sel;          // 0 : A 읽고 B 쓰기, 1 : B 읽고 A 쓰기

    // 타일 카운터. oc_group 이 출력채널 3개, pos_group 이 출력위치 3개를 고른다.
    reg [OCG_W-1:0] oc_group;     // fc1/fc2 는 22 그룹
    reg [10:0] pos_group;         // Conv0 는 1366 그룹

    reg        tile_started;
    reg        first_chunk_rdy;
    reg        param_load_started;
    reg        pool_cfg_done;
    reg        pool_started;

    reg [1:0]  svc;
    reg [12:0] svc_k_base;
    reg [6:0]  svc_len;
    reg        svc_needs_ack;     // 1 이면 pe_cntl 이 요청한 chunk

    // Weight Buffer 잔류 태그.
    //   K 가 chunk 하나에 다 들어가는 레이어는 타일이 끝나도 그 oc_group 의
    //   chunk 0 이 버퍼에 그대로 남는다. 같은 그룹의 다음 타일은 재적재를
    //   건너뛸 수 있다. (시트에 없는 최적화 - Conv0 에서 그룹당 1365 회 절약)
    //   타일 도중 chunk 를 받으면 버퍼가 덮여서 태그를 버린다.
    reg        wbuf_tag_valid;
    reg [2:0]  wbuf_layer;
    reg [OCG_W-1:0] wbuf_oc;

    // 레이어 descriptor. layer_idx 로부터 조합논리로 뽑는다.
    reg [6:0]  ly_in_w, ly_in_h, ly_out_w, ly_out_h;
    reg [5:0]  ly_in_c;
    reg [CH_W-1:0] ly_out_c;
    reg [1:0]  ly_stride;
    reg        ly_pad_en;
    reg [12:0] ly_k;
    reg [12:0] ly_out_pix;
    reg [W_AW-1:0] ly_w_base;
    reg [PARAM_AW-1:0] ly_param_base;
    reg        ly_relu;
    reg        ly_pool_after;
    reg        ly_is_fc;

    assign o_busy        = (state != C_IDLE);
    assign o_start_ready = (state == C_IDLE) && !i_done_status;
    assign o_layer_idx   = layer_idx;
    assign o_writer_mode = !((state == C_IDLE) || (state == C_START));

    // -------------------------------------------------------------------------
    // lane mask : 이 그룹의 3 lane 중 몇 개가 실제로 유효한지.
    // 출력채널(열) 방향과 출력위치(행) 방향에 같은 함수를 쓴다.
    // -------------------------------------------------------------------------
    function [2:0] lane_mask;
        input [12:0] total;
        input [12:0] base;
        reg   [12:0] remain;
        begin
            if (total <= base) begin
                lane_mask = 3'b000;
            end
            else begin
                remain = total - base;
                if (remain >= 13'd3)
                    lane_mask = 3'b111;
                else if (remain == 13'd2)
                    lane_mask = 3'b011;
                else
                    lane_mask = 3'b001;
            end
        end
    endfunction

    // -------------------------------------------------------------------------
    // 레이어 마이크로코드
    // -------------------------------------------------------------------------
    always @* begin
        ly_in_w = 7'd0;  ly_in_h = 7'd0;  ly_in_c  = 6'd0;
        ly_out_w = 7'd0; ly_out_h = 7'd0; ly_out_c = {CH_W{1'b0}};
        ly_stride = 2'd0; ly_pad_en = 1'b0;
        ly_k = 13'd1;  ly_out_pix = 13'd1;
        ly_w_base = {W_AW{1'b0}}; ly_param_base = {PARAM_AW{1'b0}};
        ly_relu = 1'b0; ly_pool_after = 1'b0; ly_is_fc = 1'b1;

        case (layer_idx)
            3'd0: begin                                   // Conv0
                ly_in_w  = L0_IN_W;  ly_in_h  = L0_IN_H;  ly_in_c  = L0_IN_C;
                ly_out_w = L0_OUT_W; ly_out_h = L0_OUT_H; ly_out_c = L0_OUT_C;
                ly_stride = L0_STRIDE; ly_pad_en = L0_PAD_EN;
                ly_k = L0_K; ly_out_pix = L0_OUT_PIX;
                ly_w_base = L0_W_BASE; ly_param_base = L0_PARAM_BASE;
                ly_relu = L0_RELU; ly_pool_after = L0_POOL_AFTER;
                ly_is_fc = 1'b0;
            end

            3'd1: begin                                   // Conv1
                ly_in_w  = L1_IN_W;  ly_in_h  = L1_IN_H;  ly_in_c  = L1_IN_C;
                ly_out_w = L1_OUT_W; ly_out_h = L1_OUT_H; ly_out_c = L1_OUT_C;
                ly_stride = L1_STRIDE; ly_pad_en = L1_PAD_EN;
                ly_k = L1_K; ly_out_pix = L1_OUT_PIX;
                ly_w_base = L1_W_BASE; ly_param_base = L1_PARAM_BASE;
                ly_relu = L1_RELU; ly_pool_after = L1_POOL_AFTER;
                ly_is_fc = 1'b0;
            end

            3'd2: begin                                   // fc1 4096 -> 64
                ly_out_c = L2_OUT_C; ly_k = L2_K; ly_out_pix = 13'd1;
                ly_w_base = L2_W_BASE; ly_param_base = L2_PARAM_BASE;
                ly_relu = L2_RELU; ly_is_fc = 1'b1;
            end

            3'd3: begin                                   // fc2 64 -> 64
                ly_out_c = L3_OUT_C; ly_k = L3_K; ly_out_pix = 13'd1;
                ly_w_base = L3_W_BASE; ly_param_base = L3_PARAM_BASE;
                ly_relu = L3_RELU; ly_is_fc = 1'b1;
            end

            default: begin                                // fc3 64 -> 1
                ly_out_c = L4_OUT_C; ly_k = L4_K; ly_out_pix = 13'd1;
                ly_w_base = L4_W_BASE; ly_param_base = L4_PARAM_BASE;
                ly_relu = L4_RELU; ly_is_fc = 1'b1;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // descriptor 출력
    // -------------------------------------------------------------------------
    assign o_in_w     = ly_in_w;
    assign o_in_h     = ly_in_h;
    assign o_in_c     = ly_in_c;
    assign o_out_w    = ly_out_w;
    assign o_out_h    = ly_out_h;
    assign o_out_c    = ly_out_c;
    assign o_stride   = ly_stride;
    assign o_pad_en   = ly_pad_en;
    assign o_input_zp = INPUT_ZP;
    assign o_k_total  = ly_k;
    assign o_relu_en  = ly_relu;
    assign o_param_base = ly_param_base;
    assign o_path_sel = ly_is_fc;
    assign o_is_final_layer = (layer_idx == (NUM_LAYERS - 1));

    assign o_read_sel = (state == C_POOL) ? 2'd2 : (ly_is_fc ? 2'd1 : 2'd0);

    // ping-pong. 읽기는 직전 job 이 쓴 bank 에서 한다.
    assign o_src_base = bank_sel ? BANK_B_BASE : BANK_A_BASE;
    assign o_dst_base = bank_sel ? BANK_A_BASE : BANK_B_BASE;

    // Pool pass 는 C_LAYER_END 에서 bank 를 이미 뒤집은 뒤에 돌기 때문에
    // 방금 레이어가 쓴 bank 를 읽고 반대쪽에 쓴다.
    assign o_pool_src_base = bank_sel ? BANK_B_BASE : BANK_A_BASE;
    assign o_pool_dst_base = bank_sel ? BANK_A_BASE : BANK_B_BASE;
    assign o_pool_in_w = ly_out_w;
    assign o_pool_in_h = ly_out_h;
    assign o_pool_c    = ly_out_c;
    assign o_pool_en   = ly_pool_after;

    // -------------------------------------------------------------------------
    // 타일 디코드. 3 을 곱하는 건 shift + add 로 합성된다.
    // -------------------------------------------------------------------------
    wire [13:0] pos_base_ext = 14'd3 * {3'd0, pos_group};
    wire [9:0]  oc_base_ext  = 10'd3 * {{(10-OCG_W){1'b0}}, oc_group};

    wire [12:0] out_c_13   = {{(13-CH_W){1'b0}}, ly_out_c};
    wire [12:0] oc_base_13 = {3'd0, oc_base_ext};

    assign o_pos_base    = pos_base_ext[11:0];
    assign o_out_ch_base = oc_base_ext[OCB_W-1:0];
    assign o_row_mask    = lane_mask(ly_out_pix, pos_base_ext[12:0]);
    assign o_col_mask    = lane_mask(out_c_13,   oc_base_13);

    wire pos_last = ((pos_base_ext + 14'd3) >= {1'b0, ly_out_pix});
    wire oc_last  = ((oc_base_ext  + 10'd3) >= out_c_13[9:0]);

    assign o_tile_last = pos_last && oc_last;

    // 타일의 첫 chunk 길이는 min(WBUF_WORDS, K)
    wire [6:0] first_len = (ly_k > WBUF_LEN) ? WBUF_LEN[6:0] : ly_k[6:0];

    // 버퍼에 같은 그룹의 chunk 0 이 아직 남아 있으면 재적재를 건너뛴다.
    wire wload_skip = wbuf_tag_valid && (wbuf_layer == layer_idx) &&
                      (wbuf_oc == oc_group);

    // -------------------------------------------------------------------------
    // Weight Loader / chunk 서비스 핸드셰이크
    // -------------------------------------------------------------------------
    assign o_wload_valid      = (svc == S_LOAD);
    assign o_wload_done_ready = (svc == S_WAIT);
    assign o_chunk_done_valid = (svc == S_ACK);
    assign o_chunk_req_ready  = (state == C_PE_WAIT) && (svc == S_IDLE);

    assign o_weight_base    = ly_w_base;
    assign o_oc_group       = oc_group;
    assign o_load_k_base    = svc_k_base;
    assign o_load_chunk_len = svc_len;

    // C_W_LOAD -> C_PARAM_LOAD 의 단일 포트 RAM 순서를 모두 통과한 뒤에만
    // PE 명령을 제시한다. C_PE_TILE 진입 자체가 두 적재의 완료를 뜻한다.
    assign o_pe_cmd_valid = (state == C_PE_TILE) && tile_started;

    // -------------------------------------------------------------------------
    // weight 적재 서비스 sub-FSM.
    // 메인 FSM 과 따로 돌아서, C_PE_WAIT 중에 들어온 chunk 요청을 타일을 깨지
    // 않고 처리할 수 있다.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            svc            <= S_IDLE;
            svc_k_base     <= 13'd0;
            svc_len        <= 7'd0;
            svc_needs_ack  <= 1'b0;
            wbuf_tag_valid <= 1'b0;
            wbuf_layer     <= 3'd0;
            wbuf_oc        <= {OCG_W{1'b0}};
        end
        else begin
            case (svc)
                S_IDLE: begin
                    // 타일의 첫 chunk
                    if ((state == C_W_LOAD) && !first_chunk_rdy && !wload_skip) begin
                        svc_k_base    <= 13'd0;
                        svc_len       <= first_len;
                        svc_needs_ack <= 1'b0;
                        svc           <= S_LOAD;
                    end
                    // pe_cntl 이 타일 도중에 요청한 chunk
                    else if ((state == C_PE_WAIT) && i_chunk_req_valid) begin
                        svc_k_base    <= i_chunk_k_base;
                        svc_len       <= i_chunk_len;
                        svc_needs_ack <= 1'b1;
                        svc           <= S_LOAD;
                    end
                end

                S_LOAD: begin
                    if (i_wload_ready)
                        svc <= S_WAIT;
                end

                S_WAIT: begin
                    if (i_wload_done_valid) begin
                        if (svc_needs_ack) begin
                            // 도중 chunk 는 chunk 0 을 덮어쓴다.
                            wbuf_tag_valid <= 1'b0;
                            svc            <= S_ACK;
                        end
                        else begin
                            // 이 타일이 chunk 를 더 안 쓰는 경우에만 태그를 건다.
                            wbuf_tag_valid <= (ly_k <= WBUF_LEN);
                            wbuf_layer     <= layer_idx;
                            wbuf_oc        <= oc_group;
                            svc            <= S_IDLE;
                        end
                    end
                end

                default: begin        // S_ACK
                    if (i_chunk_done_ready)
                        svc <= S_IDLE;
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 메인 FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= C_IDLE;
            layer_idx         <= 3'd0;
            bank_sel          <= 1'b0;
            oc_group          <= {OCG_W{1'b0}};
            pos_group         <= 11'd0;
            tile_started      <= 1'b0;
            first_chunk_rdy   <= 1'b0;
            param_load_started <= 1'b0;
            pool_cfg_done     <= 1'b0;
            pool_started      <= 1'b0;
            o_frame_consume   <= 1'b0;
            o_layer_cfg_valid <= 1'b0;
            o_tile_cfg_valid  <= 1'b0;
            o_pg_tile_start   <= 1'b0;
            o_fc_tile_start   <= 1'b0;
            o_pool_cfg_valid  <= 1'b0;
            o_pool_start      <= 1'b0;
        end
        else begin
            // 펄스 출력은 기본 0
            o_frame_consume   <= 1'b0;
            o_layer_cfg_valid <= 1'b0;
            o_tile_cfg_valid  <= 1'b0;
            o_pg_tile_start   <= 1'b0;
            o_fc_tile_start   <= 1'b0;
            o_pool_cfg_valid  <= 1'b0;
            o_pool_start      <= 1'b0;

            // 서비스 FSM 이 첫 chunk 를 올려 주면 여기서 플래그를 세운다.
            if ((state == C_W_LOAD) && !first_chunk_rdy) begin
                if (wload_skip)
                    first_chunk_rdy <= 1'b1;
                else if ((svc == S_WAIT) && i_wload_done_valid && !svc_needs_ack)
                    first_chunk_rdy <= 1'b1;
            end

            case (state)
                C_IDLE: begin
                    layer_idx       <= 3'd0;
                    bank_sel        <= 1'b0;
                    oc_group        <= {OCG_W{1'b0}};
                    pos_group       <= 11'd0;
                    tile_started    <= 1'b0;
                    first_chunk_rdy <= 1'b0;
                    param_load_started <= 1'b0;
                    pool_cfg_done   <= 1'b0;
                    pool_started    <= 1'b0;

                    if (o_start_ready && i_start_valid)
                        state <= C_START;
                end

                // 입력 frame 이 저장됐는지 확인하고 사용 선언.
                C_START: begin
                    if (i_frame_ready) begin
                        o_frame_consume <= 1'b1;
                        state           <= C_SET;
                    end
                end

                C_SET: begin
                    o_layer_cfg_valid <= 1'b1;
                    oc_group          <= {OCG_W{1'b0}};
                    pos_group         <= 11'd0;
                    tile_started      <= 1'b0;
                    first_chunk_rdy   <= 1'b0;
                    param_load_started <= 1'b0;
                    state             <= C_W_LOAD;
                end

                // 외부 RAM 이 단일 포트여도 충돌하지 않도록 타일의 첫 weight
                // chunk 를 먼저 적재한다. 같은 oc_group 의 재사용 가능한 chunk 는
                // wload_skip 으로 건너뛴다.
                C_W_LOAD: begin
                    if (first_chunk_rdy) begin
                        param_load_started <= 1'b0;
                        state              <= C_PARAM_LOAD;
                    end
                end

                // Weight 적재가 끝난 뒤에만 tile cfg 를 내보내 Param Buffer 적재를
                // 시작한다. i_params_ready 는 해당 요청의 완료 응답이다.
                C_PARAM_LOAD: begin
                    if (!param_load_started) begin
                        o_tile_cfg_valid   <= 1'b1;
                        param_load_started <= 1'b1;
                    end
                    else if (i_params_ready) begin
                        tile_started <= 1'b0;
                        state        <= C_PE_TILE;
                    end
                end

                // weight/parameter 적재 완료 뒤 activation 피더와 PE 를 시작한다.
                C_PE_TILE: begin
                    if (!tile_started) begin
                        if (ly_is_fc)
                            o_fc_tile_start <= 1'b1;
                        else
                            o_pg_tile_start <= 1'b1;
                        tile_started <= 1'b1;
                    end

                    if (o_pe_cmd_valid && i_pe_cmd_ready)
                        state <= C_PE_WAIT;
                end

                // pe_cntl 의 chunk 요청은 위 서비스 FSM 이 처리한다.
                C_PE_WAIT: begin
                    if (i_tile_done)
                        state <= C_NXT_TILE;
                end

                // pos_group -> oc_group 순으로 진행.
                // pos_group 만 움직일 때는 weight 를 그대로 재사용한다.
                C_NXT_TILE: begin
                    tile_started    <= 1'b0;
                    first_chunk_rdy <= 1'b0;
                    param_load_started <= 1'b0;

                    if (!pos_last) begin
                        pos_group <= pos_group + 1'b1;
                        state     <= C_W_LOAD;
                    end
                    else if (!oc_last) begin
                        pos_group <= 11'd0;
                        oc_group  <= oc_group + 1'b1;
                        state     <= C_W_LOAD;
                    end
                    else begin
                        // 방금 쓴 bank 를 다음 job 의 입력으로 넘긴다.
                        bank_sel <= ~bank_sel;
                        state    <= ly_pool_after ? C_POOL : C_LAYER_END;
                    end
                end

                // 방금 레이어가 쓴 bank 를 읽는 별도 MaxPool pass.
                C_POOL: begin
                    if (!pool_cfg_done) begin
                        o_pool_cfg_valid <= 1'b1;
                        pool_cfg_done    <= 1'b1;
                    end
                    else if (!pool_started) begin
                        o_pool_start <= 1'b1;
                        pool_started <= 1'b1;
                    end
                    else if (i_pool_done) begin
                        pool_cfg_done <= 1'b0;
                        pool_started  <= 1'b0;
                        bank_sel      <= ~bank_sel;
                        state         <= C_LAYER_END;
                    end
                end

                C_LAYER_END: begin
                    if (layer_idx == (NUM_LAYERS - 1)) begin
                        state <= C_DONE;
                    end
                    else begin
                        layer_idx <= layer_idx + 1'b1;
                        state     <= C_SET;
                    end
                end

                // Result+IRQ 가 최종값을 저장하고 done_status 를 올린다.
                C_DONE: begin
                    if (i_done_status)
                        state <= C_IDLE;
                end

                default: state <= C_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
