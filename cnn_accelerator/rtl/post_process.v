`timescale 1ns / 1ps

module post_process (
    input wire clk,
    input wire rst_n,

    // Tile / Layer Configuration
    input wire i_cfg_valid,
    output wire o_cfg_ready,

    input wire [2:0] i_col_mask,
    input wire [4:0] i_out_ch_base,

    input wire [5:0] i_bias_base,
    input wire       i_relu_en,
    input wire       i_is_final_layer,

    input wire signed [31:0] i_quant_multiplier,
    input wire        [ 5:0] i_quant_shift,

    input wire signed [31:0] i_angle_multiplier,
    input wire        [ 5:0] i_angle_shift,

    // Parameter Buffer Interface
    input  wire i_params_loaded,
    output reg  o_params_ready,

    // Bias Read Request
    output reg        o_bias_req_valid,
    input  wire       i_bias_req_ready,
    output reg  [5:0] o_bias_addr,

    // Bias Read Response
    input  wire signed [31:0] i_bias_data,
    input  wire               i_bias_rsp_valid,
    output reg                o_bias_rsp_ready,

    // Input Stream from Output FIFO
    input  wire [95:0] i_data,
    input  wire        i_valid,
    output reg         o_ready,

    input wire [ 2:0] i_keep,
    input wire [18:0] i_meta,

    // Normal Output Stream to Pooling Unit
    // 3 lanes x INT8 = 24-bit
    output reg  [23:0] o_data,
    output reg         o_valid,
    input  wire        i_ready,

    output reg [ 2:0] o_keep,
    output reg [18:0] o_meta,

    // Final Layer Output to Result Buffer
    // INT32 scalar path
    output reg signed [7:0] o_final_data,
    output reg              o_final_valid,
    input  wire             i_final_ready
);
    // =========================================================
    // Internal Configuration Registers
    // =========================================================

    // 현재 Tile에서 유효한 출력 channel lane을 저장
    // 001 : lane0
    // 011 : lane0~1
    // 111 : lane0~2
    reg [2:0] col_mask_reg;

    // 현재 Tile의 첫 번째 output channel 번호 저장
    // Bias 주소 계산 시 사용
    reg [4:0] out_ch_base_reg;

    // 현재 Layer의 Bias 시작 주소 저장
    // 실제 Bias 주소 = bias_base + out_ch_base + lane
    reg [5:0] bias_base_reg;

    // 현재 Layer에서 ReLU 적용 여부 저장
    reg relu_en_reg;

    // 현재 Tile이 Final Layer인지 저장
    // Normal path와 Final path를 분기하기 위해 사용
    reg is_final_layer_reg;

    // 중간 Layer requantization에 사용하는 multiplier 저장
    reg signed [31:0] quant_multiplier_reg;

    // 중간 Layer requantization에 사용하는 right shift 값 저장
    reg        [ 5:0] quant_shift_reg;

    // Final Layer의 INT32 결과를 signed INT8 각도로
    // 변환할 때 사용하는 multiplier 저장
    reg signed [31:0] angle_multiplier_reg;

    // Final Layer 각도 변환 시 사용하는 right shift 값 저장
    reg        [ 5:0] angle_shift_reg;

    // 새로운 Tile configuration을 받아도 되는 상태인지 표시
    reg cfg_change_allowed;


    // =========================================================
    // Bias Cache
    // =========================================================

    // 현재 output channel tile에서 사용하는 Bias를 저장
    // Bias는 Tile 설정 시 param_buf에서 한 번 읽고,
    // 해당 Tile의 여러 row 처리 동안 반복 사용
    reg signed [31:0] bias_cache0;
    reg signed [31:0] bias_cache1;
    reg signed [31:0] bias_cache2;

    // ---- Bias 읽기 (핸드셰이크 정리 2026-09-23) ----
    //   요청은 lane 0..n-1 을 연속으로 내고 (req_idx), 응답은 오는 대로 받는다 (rsp_idx, ready 항상 1).
    //   전에는 lane 마다 요청 -> 응답 대기 를 반복해서 타일마다 7 clk 이 들었다.
    reg [1:0] bias_req_idx;      // 다음에 요청할 lane
    reg [1:0] bias_rsp_idx;      // 다음에 받을 lane
    reg [1:0] bias_n_lanes;      // 이 타일에 필요한 lane 수 (col_mask 001/011/111 -> 1/2/3)

    // ---- Bias 캐시 2 entry ----
    //   같은 (bias_base, out_ch_base, col_mask) 타일이 다시 오면 param_buf 를 읽지 않는다.
    //   Conv0 / Conv1 은 채널 그룹 2 개가 번갈아 오므로 첫 두 타일 뒤로는 전부 hit (타일당 -7 clk).
    //   param_buf 가 다시 적재되면 (i_params_loaded=0) 비운다.
    reg        [ 1:0] bc_valid;
    reg        [13:0] bc_key0, bc_key1;           // {bias_base[5:0], out_ch_base[4:0], col_mask[2:0]}
    reg signed [31:0] bc0_b0, bc0_b1, bc0_b2;
    reg signed [31:0] bc1_b0, bc1_b1, bc1_b2;
    reg               bc_next;                    // 다음에 채울 entry
    wire       [13:0] cfg_key  = {i_bias_base, i_out_ch_base, i_col_mask};
    wire              cfg_hit0 = bc_valid[0] && (bc_key0 == cfg_key);
    wire              cfg_hit1 = bc_valid[1] && (bc_key1 == cfg_key);
    wire              cfg_hit  = cfg_hit0 || cfg_hit1;


    // =========================================================
    // Input Beat Buffer
    // =========================================================

    // output_fifo에서 받은 96-bit beat 전체를 저장
    // 한 beat를 저장한 뒤 lane0~2를 순차적으로 처리하기 위해 필요
    reg [95:0] data_reg;

    // 저장된 beat에서 실제 유효한 lane 정보
    // data_reg와 동일한 handshake에서 함께 저장
    reg [2:0] keep_reg;

    // position / channel / tile_end / layer_end 정보를
    // 처리 완료 후 downstream으로 그대로 전달하기 위해 저장
    reg [18:0] meta_reg;


    // =========================================================
    // Input Lane Split
    // =========================================================

    // 96-bit 입력 beat를 3개의 signed INT32 lane으로 해석
    // 별도 32-bit reg 3개를 두지 않고 data_reg에서 직접 분리
    wire signed [31:0] lane0_data;
    wire signed [31:0] lane1_data;
    wire signed [31:0] lane2_data;

    assign lane0_data = $signed(data_reg[31:0]);
    assign lane1_data = $signed(data_reg[63:32]);
    assign lane2_data = $signed(data_reg[95:64]);


    // =========================================================
    // Lane Processing Control
    // =========================================================

    // 현재 처리 중인 lane 번호
    // Requant 연산기 하나를 lane0~2가 공유하기 위해 사용
    reg [1:0] lane_idx;

    // lane_idx에 따라 선택된 현재 PE 결과
    // lane0_data / lane1_data / lane2_data 중 하나를 선택
    reg signed [31:0] current_lane_data;

    // 현재 lane에 대응하는 Bias
    // bias_cache0~2 중 하나를 선택
    reg signed [31:0] current_bias;

    // 현재 lane이 실제 유효한 lane인지 확인
    // keep_reg[lane_idx]를 사용하여 invalid lane 계산을 건너뜀
    reg current_lane_valid;

    // =========================================================
    // Current Lane Selector
    // =========================================================
    always @(*) begin
        // 기본값
        // 잘못된 lane_idx가 들어와도 latch가 생기지 않도록 설정
        current_lane_data  = 32'd0;
        current_bias       = 32'd0;
        current_lane_valid = 1'b0;

        case (lane_idx)

            2'd0: begin
                current_lane_data  = lane0_data;
                current_bias       = bias_cache0;
                current_lane_valid = keep_reg[0];
            end

            2'd1: begin
                current_lane_data  = lane1_data;
                current_bias       = bias_cache1;
                current_lane_valid = keep_reg[1];
            end

            2'd2: begin
                current_lane_data  = lane2_data;
                current_bias       = bias_cache2;
                current_lane_valid = keep_reg[2];
            end

            default: begin
                current_lane_data  = 32'd0;
                current_bias       = 32'd0;
                current_lane_valid = 1'b0;
            end
        endcase
    end

    // =========================================================
    // Requant 파이프라인 (타이밍 : 2026-09-23)
    //   한 클럭에 있던 bias 덧셈 -> 32x32 곱 -> 반올림 shift -> 포화 를 4 단으로 나눴다.
    //   PROCESS 에서 lane 을 한 클럭에 하나씩 넣고 (s0), 3 클럭 뒤 (s3) 결과가 result_lane 에 써진다.
    //   lane 순서 / keep / meta 처리는 그대로. 한 beat 에 (lane 수 + 3) 클럭이 든다.
    // =========================================================

    // ---- s0 : bias 덧셈 + INT32 포화 (조합, PROCESS 의 현재 lane) ----
    reg signed [32:0] bias_sum_ext;   // 33-bit 로 더해서 overflow 를 본다
    reg signed [31:0] acc32_reg;      // Golden 의 A32

    always @(*) begin
        bias_sum_ext = {current_lane_data[31], current_lane_data} + {current_bias[31], current_bias};
    end

    always @(*) begin
        case (bias_sum_ext[32:31])
            2'b01:   acc32_reg = 32'h7FFF_FFFF;   // 양의 overflow -> INT32 최댓값
            2'b10:   acc32_reg = 32'h8000_0000;   // 음의 overflow -> INT32 최솟값
            default: acc32_reg = bias_sum_ext[31:0];
        endcase
    end

    // 현재 레이어의 M / S.  Final FC2 는 각도 변환용 M / S
    reg signed [31:0] current_multiplier;
    reg        [ 5:0] current_shift;

    always @(*) begin
        if (is_final_layer_reg) begin
            current_multiplier = angle_multiplier_reg;
            current_shift      = angle_shift_reg;
        end
        else begin
            current_multiplier = quant_multiplier_reg;
            current_shift      = quant_shift_reg;
        end
    end

    // ---- 파이프라인 레지스터 ----
    //   s1 : acc32, M (DSP 입력 레지스터)   s2 : 64-bit 곱   s3 : 곱 + 반올림 오프셋
    //   s3 에서 shift + 포화 해서 result_lane / final_result_reg 에 쓴다
    reg               s1_valid, s2_valid, s3_valid;   // 그 단에 유효 lane 이 있다
    reg        [ 1:0] s1_lane,  s2_lane,  s3_lane;    // 그 lane 번호
    reg               s1_last,  s2_last,  s3_last;    // 이 beat 의 마지막 lane 이다
    reg signed [31:0] s1_acc32;
    reg signed [31:0] s1_mult;
    reg signed [63:0] s2_prod;
    reg signed [63:0] s3_sum;

    // 반올림 : round-to-nearest, ties away from zero
    //   P >= 0 : (P + 2^(S-1)) >> S
    //   P <  0 : (P + 2^(S-1) - 1) >>> S      (= -((|P| + 2^(S-1)) >> S) 와 같다)
    //   S = 0  : P 그대로
    //   |P| <= 2^62 (INT32 x INT32) 이고 S <= 30 이라 64-bit 에서 넘치지 않는다
    wire        [63:0] round_half = (current_shift == 6'd0) ? 64'd0 : (64'd1 << (current_shift - 6'd1));
    wire signed [63:0] round_off  = (current_shift == 6'd0) ? 64'sd0 :
                                    (s2_prod[63] ? ($signed(round_half) - 64'sd1) : $signed(round_half));

    // s3 : 산술 우측 shift 뒤 값 (부호 포함)
    wire signed [63:0] rounded_result = s3_sum >>> current_shift;

    // 일반 레이어 : ReLU + signed INT8 포화
    reg signed [7:0] normal_int8_result;

    always @(*) begin
        if (relu_en_reg && rounded_result[63])        normal_int8_result = 8'd0;    // ReLU
        else if (rounded_result > $signed(64'd127))   normal_int8_result = 8'h7F;   // 최댓값 초과
        else if (rounded_result < -$signed(64'd128))  normal_int8_result = 8'h80;   // 최솟값 미만
        else                                          normal_int8_result = rounded_result[7:0];
    end

    // Final FC2 : ReLU 없이 signed INT8 포화 (1 도 단위 각도)
    reg signed [7:0] final_int8_result;

    always @(*) begin
        if (rounded_result > $signed(64'd127))        final_int8_result = 8'h7F;
        else if (rounded_result < -$signed(64'd128))  final_int8_result = 8'h80;
        else                                          final_int8_result = rounded_result[7:0];
    end

    // Final 결과를 handshake 완료까지 유지
    reg signed [7:0] final_result_reg;

    // =========================================================
    // Normal INT8 Output Lane Registers
    // =========================================================

    // 각 lane의 post-process 완료 INT8 결과 저장
    // 3개 lane 처리가 모두 끝난 뒤 24-bit o_data로 묶어서 출력
    reg signed [7:0] result_lane0;
    reg signed [7:0] result_lane1;
    reg signed [7:0] result_lane2;

    // 이 beat 의 마지막 lane 을 파이프라인에 넣었다 (넣은 뒤 결과가 나올 때까지 기다린다)
    reg issue_done;

    // =========================================================
    // FSM State
    // =========================================================
    reg [2:0] state;
    reg [2:0] next_state;

    // FSM state
    localparam IDLE       = 3'd0; // Tile configuration 대기
    localparam BIAS_REQ   = 3'd1; // 필요한 Bias read request 발생
    localparam BIAS_WAIT  = 3'd2; // Bias response 대기
    localparam READY      = 3'd3; // Bias 준비 완료, FIFO 입력 대기
    localparam PROCESS    = 3'd4; // lane0~2 순차 post-process
    localparam OUT_WAIT   = 3'd5; // Normal INT8 output 수락 대기
    localparam FINAL_WAIT = 3'd6; // Final output 수락 대기

    // =========================================================
    // Handshake Wires
    // =========================================================

    // output_fifo → post_process 실제 입력 수락
    wire input_fire;

    // post_process → pooling_unit 실제 출력 전달
    wire output_fire;

    // post_process → result_buf Final 결과 실제 전달
    wire final_fire;

    // param_buf Bias read request 실제 수락
    wire bias_req_fire;

    // param_buf Bias read response 실제 수락
    wire bias_rsp_fire;

    // Tile configuration을 실제로 수락하는 조건
    // IDLE 또는 READY 상태에서 params가 모두 적재된 경우에만 수락
    wire cfg_fire;

    // Tile configuration의 channel mask 유효성 확인
    // Normal Layer : 001 / 011 / 111
    // Final FC2    : 001만 허용
    wire cfg_mask_valid;

    assign cfg_mask_valid = i_is_final_layer ? (i_col_mask == 3'b001) : 
            ((i_col_mask == 3'b001) ||
             (i_col_mask == 3'b011) ||
             (i_col_mask == 3'b111));

    // 현재 Tile에서 사용하게 될 마지막 Bias 주소
    wire [6:0] cfg_bias_last_addr;

    // Bias address가 전체 Bias parameter 범위 0~42 안인지 확인
    wire cfg_bias_addr_valid;

    // Final FC2 configuration 유효성 확인
    // FC2 : output channel 1개, bias addr 42, ReLU 미사용
    wire cfg_final_valid;

    assign cfg_final_valid = !i_is_final_layer || (
            (i_out_ch_base == 5'd0) &&
            (i_bias_base   == 6'd42) &&
            (i_relu_en     == 1'b0));

    assign cfg_bias_last_addr = {1'b0, i_bias_base} + {2'b00, i_out_ch_base} + 
        ((i_col_mask == 3'b111) ? 7'd2 : (i_col_mask == 3'b011) ? 7'd1 : 7'd0);

    assign cfg_bias_addr_valid = (cfg_bias_last_addr <= 7'd42);

    // 입력 beat의 keep가 현재 Tile 설정과 일치하는지 확인
    // 정상 keep는 001 / 011 / 111만 허용
    wire input_keep_valid;

    // Final FC2 입력 형식 확인
    // Final Layer는 lane0 하나만 사용하며
    // 마지막 Tile이자 마지막 Layer 결과여야 함
    wire final_input_valid;

    // 입력 beat의 metadata가 현재 Tile 설정과 일치하는지 확인
    wire input_meta_valid;

    assign input_meta_valid = (i_meta[16:12] == out_ch_base_reg) && (!i_meta[18] || i_meta[17]);

    assign final_input_valid = !is_final_layer_reg || ((i_keep    == 3'b001) && (i_meta[17] == 1'b1) && (i_meta[18] == 1'b1));
    assign input_keep_valid = ((i_keep == 3'b001) || (i_keep == 3'b011) || (i_keep == 3'b111)) && (i_keep == col_mask_reg);
    
    
    assign cfg_fire = i_cfg_valid && o_cfg_ready;
    assign o_cfg_ready =
        i_params_loaded &&
        cfg_mask_valid &&
        cfg_bias_addr_valid &&
        cfg_final_valid &&
        cfg_change_allowed &&
        ((state == IDLE) || (state == READY));

    assign input_fire    = i_valid          && o_ready;
    assign output_fire   = o_valid          && i_ready;
    assign final_fire    = o_final_valid    && i_final_ready;
    assign bias_req_fire = o_bias_req_valid && i_bias_req_ready;
    assign bias_rsp_fire = i_bias_rsp_valid && o_bias_rsp_ready;

    // 이번 응답이 이 타일의 마지막 lane 이다
    wire bias_rsp_last = bias_rsp_fire && (bias_rsp_idx == bias_n_lanes - 2'd1);

    // =========================================================
    // Lane 넣기 / 완료 판정
    // =========================================================

    // 이 beat 에서 마지막으로 넣을 lane (keep 은 001 / 011 / 111 로 lane0 부터 연속)
    wire [1:0] last_lane_idx = keep_reg[2] ? 2'd2 : (keep_reg[1] ? 2'd1 : 2'd0);

    // s0 에 마지막 lane 을 넣는 클럭
    wire       issue_last    = (state == PROCESS) && !issue_done && (lane_idx == last_lane_idx);

    // 마지막 lane 의 결과가 result_lane 에 써지는 클럭. 다음 클럭에 OUT_WAIT / FINAL_WAIT
    wire       process_done  = (state == PROCESS) && s3_last;

    // =========================================================
    // FSM State Register
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    // =========================================================
    // FSM Next State Logic
    // =========================================================
    always @(*) begin
        next_state = state;

        case (state)
            // -------------------------------------------------
            // Tile configuration 대기
            // -------------------------------------------------
            IDLE: begin
                if (cfg_fire)
                    next_state = cfg_hit ? READY : BIAS_REQ;   // 캐시 hit 이면 바로 READY
            end


            // -------------------------------------------------
            // param_buf에 현재 lane Bias 요청
            // -------------------------------------------------
            BIAS_REQ: begin
                // lane 0..n-1 요청을 연속으로 내고, 마지막 응답이 오면 READY
                if (bias_rsp_last)
                    next_state = READY;
            end


            // -------------------------------------------------
            // Bias 응답 대기
            // -------------------------------------------------
            BIAS_WAIT: begin
                // 쓰지 않는다 (연속 읽기로 바꾸면서 BIAS_REQ 에 합쳤다)
                next_state = READY;
            end


            // -------------------------------------------------
            // output_fifo 입력 대기
            // -------------------------------------------------
            READY: begin
                if (cfg_fire) begin
                    next_state = cfg_hit ? READY : BIAS_REQ;
                end
                else if (input_fire) begin
                    next_state = PROCESS;
                end
            end


            // -------------------------------------------------
            // 현재 beat의 lane 순차 처리
            // -------------------------------------------------
            PROCESS: begin
                if (process_done) begin
                    if (is_final_layer_reg)
                        next_state = FINAL_WAIT;
                    else
                        next_state = OUT_WAIT;
                end
            end


            // -------------------------------------------------
            // Normal output이 pooling_unit에 수락되기를 대기
            // -------------------------------------------------
            OUT_WAIT: begin
                if (output_fire)
                    next_state = READY;
            end


            // -------------------------------------------------
            // Final angle이 result_buf에 수락되기를 대기
            // -------------------------------------------------
            FINAL_WAIT: begin
                if (final_fire)
                    next_state = READY;
            end


            default: begin
                next_state = IDLE;
            end

        endcase
    end


    // =========================================================
    // FSM Output / Control Logic
    // =========================================================
    always @(*) begin
        // -----------------------------------------------------
        // 기본값
        // -----------------------------------------------------
        o_params_ready   = 1'b0;

        o_bias_req_valid = 1'b0;
        o_bias_addr      = 6'd0;
        o_bias_rsp_ready = 1'b0;

        o_ready          = 1'b0;

        // 데이터 출력은 state 로 게이팅하지 않고 레지스터를 그대로 낸다 (타이밍 2026-09-23 :
        // state -> o_meta -> pooling 위치 계산 -> o_ready 가 한 경로였다). 유효 여부는 valid 만 본다
        o_data           = {result_lane2, result_lane1, result_lane0};
        o_valid          = 1'b0;
        o_keep           = keep_reg;
        o_meta           = meta_reg;
        o_final_data     = final_result_reg;
        o_final_valid    = 1'b0;


        case (state)

            IDLE: begin
                // cfg 대기
            end

            BIAS_REQ: begin
                // 남은 lane 이 있으면 요청, 응답은 항상 받는다
                o_bias_req_valid = (bias_req_idx < bias_n_lanes);
                o_bias_addr      = bias_base_reg + out_ch_base_reg + bias_req_idx;
                o_bias_rsp_ready = 1'b1;
            end

            BIAS_WAIT: begin
            end

            READY: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;
                // cfg 수락 cycle은 입력 금지
                // valid beat가 들어온 경우 keep도 정상이어야 수락
                o_ready = !cfg_change_allowed && !cfg_fire && (!i_valid || (input_keep_valid && input_meta_valid && final_input_valid));
            end

            PROCESS: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;

                // lane 연산 중이므로 새로운 입력은 받지 않음
            end

            OUT_WAIT: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;
                // 처리 완료된 3개의 INT8 lane을 24-bit로 출력
                o_data  = {result_lane2, result_lane1, result_lane0};
                // 현재 beat에서 실제 유효했던 lane 정보 전달
                o_keep  = keep_reg;
                // position / channel / tile_end / layer_end 전달
                o_meta  = meta_reg;
                // pooling_unit에 출력 유효 표시
                o_valid = 1'b1;
            end

            FINAL_WAIT: begin
                // 현재 Tile의 Bias cache 유효
                o_params_ready = 1'b1;

                // Final FC2 결과는 signed INT8 각도값
                o_final_data  = final_result_reg;
                o_final_valid = 1'b1;
            end

            default: begin
            end

        endcase
    end

    // =========================================================
    // Configuration Register Update
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_mask_reg         <= 3'b000;
            out_ch_base_reg      <= 5'd0;
            bias_base_reg        <= 6'd0;
            relu_en_reg          <= 1'b0;
            is_final_layer_reg   <= 1'b0;

            quant_multiplier_reg <= 32'd0;
            quant_shift_reg      <= 6'd0;

            angle_multiplier_reg <= 32'd0;
            angle_shift_reg      <= 6'd0;
        end
        else if (cfg_fire) begin
            col_mask_reg         <= i_col_mask;
            out_ch_base_reg      <= i_out_ch_base;
            bias_base_reg        <= i_bias_base;
            relu_en_reg          <= i_relu_en;
            is_final_layer_reg   <= i_is_final_layer;

            quant_multiplier_reg <= i_quant_multiplier;
            quant_shift_reg      <= i_quant_shift;

            angle_multiplier_reg <= i_angle_multiplier;
            angle_shift_reg      <= i_angle_shift;
        end
    end

    // =========================================================
    // Tile Configuration Lock
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // reset 후 첫 Tile configuration 허용
            cfg_change_allowed <= 1'b1;
        end
        else if (cfg_fire) begin
            // 현재 Tile configuration을 받았으므로
            // Tile이 끝날 때까지 새로운 cfg 금지
            cfg_change_allowed <= 1'b0;
        end
        else if (output_fire && meta_reg[17]) begin
            // Normal path의 마지막 row가 실제 수락되면
            // 다음 Tile configuration 허용
            cfg_change_allowed <= 1'b1;
        end
        else if (final_fire) begin
            // Final FC2 결과가 실제 수락되면
            // 현재 Tile 종료
            cfg_change_allowed <= 1'b1;
        end
    end

    // =========================================================
    // Bias 읽기 / 캐시 갱신
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bias_cache0   <= 32'd0;
            bias_cache1   <= 32'd0;
            bias_cache2   <= 32'd0;
            bias_req_idx  <= 2'd0;
            bias_rsp_idx  <= 2'd0;
            bias_n_lanes  <= 2'd1;
            bc_valid      <= 2'b00;
            bc_key0       <= 14'd0;  bc_key1 <= 14'd0;
            bc0_b0 <= 32'd0; bc0_b1 <= 32'd0; bc0_b2 <= 32'd0;
            bc1_b0 <= 32'd0; bc1_b1 <= 32'd0; bc1_b2 <= 32'd0;
            bc_next       <= 1'b0;
        end
        else begin
            // param_buf 가 다시 적재되면 (새 추론) 캐시를 비운다
            if (!i_params_loaded) bc_valid <= 2'b00;

            if (cfg_fire) begin
                bias_req_idx <= 2'd0;
                bias_rsp_idx <= 2'd0;
                bias_n_lanes <= i_col_mask[2] ? 2'd3 : (i_col_mask[1] ? 2'd2 : 2'd1);
                if (cfg_hit0) begin
                    bias_cache0 <= bc0_b0;  bias_cache1 <= bc0_b1;  bias_cache2 <= bc0_b2;
                end
                else if (cfg_hit1) begin
                    bias_cache0 <= bc1_b0;  bias_cache1 <= bc1_b1;  bias_cache2 <= bc1_b2;
                end
                else begin
                    bias_cache0 <= 32'd0;   bias_cache1 <= 32'd0;   bias_cache2 <= 32'd0;
                end
            end
            else begin
                if (bias_req_fire) bias_req_idx <= bias_req_idx + 2'd1;
                if (bias_rsp_fire) begin
                    bias_rsp_idx <= bias_rsp_idx + 2'd1;
                    case (bias_rsp_idx)
                        2'd0:    bias_cache0 <= i_bias_data;
                        2'd1:    bias_cache1 <= i_bias_data;
                        default: bias_cache2 <= i_bias_data;
                    endcase
                end
                // 마지막 응답 : 이 타일의 bias 세 개를 캐시에 넣는다
                if (bias_rsp_last && i_params_loaded) begin
                    if (!bc_next) begin
                        bc_key0 <= {bias_base_reg, out_ch_base_reg, col_mask_reg};
                        bc0_b0  <= (bias_rsp_idx == 2'd0) ? i_bias_data : bias_cache0;
                        bc0_b1  <= (bias_rsp_idx == 2'd1) ? i_bias_data : bias_cache1;
                        bc0_b2  <= (bias_rsp_idx == 2'd2) ? i_bias_data : bias_cache2;
                        bc_valid[0] <= 1'b1;
                    end
                    else begin
                        bc_key1 <= {bias_base_reg, out_ch_base_reg, col_mask_reg};
                        bc1_b0  <= (bias_rsp_idx == 2'd0) ? i_bias_data : bias_cache0;
                        bc1_b1  <= (bias_rsp_idx == 2'd1) ? i_bias_data : bias_cache1;
                        bc1_b2  <= (bias_rsp_idx == 2'd2) ? i_bias_data : bias_cache2;
                        bc_valid[1] <= 1'b1;
                    end
                    bc_next <= ~bc_next;
                end
            end
        end
    end


    // =========================================================
    // Input Beat Buffer Update
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_reg <= 96'd0;
            keep_reg <= 3'b000;
            meta_reg <= 19'd0;
        end
        else if (input_fire) begin
            // output_fifo에서 실제 handshake가 성립한 경우에만
            // 현재 96-bit PE result beat와 부가 정보를 저장
            data_reg <= i_data;
            keep_reg <= i_keep;
            meta_reg <= i_meta;
        end
    end

    // =========================================================
    // Lane 파이프라인 진행
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane_idx         <= 2'd0;
            issue_done       <= 1'b0;
            s1_valid <= 1'b0;  s2_valid <= 1'b0;  s3_valid <= 1'b0;
            s1_lane  <= 2'd0;  s2_lane  <= 2'd0;  s3_lane  <= 2'd0;
            s1_last  <= 1'b0;  s2_last  <= 1'b0;  s3_last  <= 1'b0;
            s1_acc32 <= 32'd0; s1_mult  <= 32'd0;
            s2_prod  <= 64'd0; s3_sum   <= 64'd0;
            result_lane0     <= 8'd0;
            result_lane1     <= 8'd0;
            result_lane2     <= 8'd0;
            final_result_reg <= 8'd0;
        end
        else begin
            // ---- 파이프라인은 매 클럭 전진한다 ----
            // s0 -> s1 : PROCESS 에서 현재 lane 을 넣는다 (무효 lane 은 valid=0 으로 지나간다)
            s1_valid <= (state == PROCESS) && !issue_done && current_lane_valid;
            s1_lane  <= lane_idx;
            s1_last  <= issue_last;
            s1_acc32 <= acc32_reg;
            s1_mult  <= current_multiplier;

            // s1 -> s2 : signed 32 x 32 곱 (DSP)
            s2_valid <= s1_valid;
            s2_lane  <= s1_lane;
            s2_last  <= s1_last;
            s2_prod  <= s1_acc32 * s1_mult;

            // s2 -> s3 : 반올림 오프셋 덧셈
            s3_valid <= s2_valid;
            s3_lane  <= s2_lane;
            s3_last  <= s2_last;
            s3_sum   <= s2_prod + round_off;

            // ---- s3 : shift + 포화 결과 저장 ----
            if (s3_valid) begin
                if (!is_final_layer_reg) begin
                    case (s3_lane)
                        2'd0:    result_lane0 <= normal_int8_result;
                        2'd1:    result_lane1 <= normal_int8_result;
                        2'd2:    result_lane2 <= normal_int8_result;
                        default: begin end
                    endcase
                end
                else if (s3_lane == 2'd0) begin
                    // Final FC2 는 lane0 하나
                    final_result_reg <= final_int8_result;
                end
            end

            // ---- lane 넣기 ----
            if (input_fire) begin
                // 새 beat : lane0 부터. 무효 lane 자리에 이전 결과가 남지 않도록 지운다
                lane_idx         <= 2'd0;
                issue_done       <= 1'b0;
                result_lane0     <= 8'd0;
                result_lane1     <= 8'd0;
                result_lane2     <= 8'd0;
                final_result_reg <= 8'd0;
            end
            else if ((state == PROCESS) && !issue_done) begin
                if (issue_last) issue_done <= 1'b1;
                else            lane_idx   <= lane_idx + 2'd1;
            end
        end
    end

endmodule
