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

    // 각 lane의 Bias가 정상적으로 cache에 저장되었는지 표시
    // 모든 필요한 Bias가 준비되면 o_params_ready를 1로 만들 때 사용
    reg [2:0] bias_loaded_mask;

    // 현재 어떤 lane의 Bias를 요청 중인지 표시
    // 0 : lane0, 1 : lane1, 2 : lane2
    reg [1:0] bias_lane_idx;

    // Bias read request가 수락된 뒤,
    // 응답이 어느 lane에 해당하는지 기억하기 위해 사용
    // param_buf 응답이 request와 다른 cycle에 들어올 수 있기 때문
    reg [1:0] bias_rsp_lane_reg;


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
    // Arithmetic Intermediate Registers
    // =========================================================

    // PE INT32 결과 + Bias INT32의 덧셈 결과
    // signed INT32 + signed INT32는 overflow 확인을 위해
    // 33-bit 임시값으로 계산
    reg signed [32:0] bias_sum_ext;

    // Bias 적용 후 실제 Golden의 A32 값
    // overflow/wrap을 그대로 허용하지 않고 범위 확인 후 사용
    reg signed [31:0] acc32_reg;

    // A32 × quant_multiplier 결과
    // signed 32-bit × signed 32-bit이므로 64-bit 필요
    reg signed [63:0] mult_result;

    // round-to-nearest, ties away from zero를 수행하기 위해
    // 곱셈 결과의 절댓값을 저장
    reg [63:0] mult_abs;

    // shift 시 더할 rounding offset
    // S > 0일 때 2^(S-1)
    reg [63:0] round_offset;

    // abs(P) + rounding offset 계산 시 carry 손실을 막기 위한
    // 65-bit 확장 임시값
    reg [64:0] round_mag_ext;

    // 현재 Layer에서 실제 사용할 multiplier
    reg signed [31:0] current_multiplier;

    // 현재 Layer에서 실제 사용할 right shift 값
    reg        [ 5:0] current_shift;

    // rounding 및 shift 완료 후 부호까지 복원한 값
    // 최종 ReLU / saturation 전에 사용
    reg signed [63:0] rounded_result;

    // 일반 Layer에서 현재 lane의 최종 INT8 결과
    reg signed [7:0] normal_int8_result;
    
    // Final Layer의 최종 signed INT8 각도 결과
    reg signed [7:0] final_int8_result;

    // Final Layer 결과를 handshake 완료까지 유지
    reg signed [7:0] final_result_reg;

    // =========================================================
    // Bias Addition
    // =========================================================
    always @(*) begin
        // signed INT32 PE result와 signed INT32 Bias를
        // 각각 33-bit로 sign extension 후 덧셈
        // 32-bit 덧셈에서 발생할 수 있는 overflow를
        // 조용히 wrap시키지 않기 위해 33-bit로 계산
        bias_sum_ext = {current_lane_data[31], current_lane_data} + {current_bias[31], current_bias};
    end

    // =========================================================
    // INT32 Range Check
    // =========================================================
    always @(*) begin
        // 33-bit 결과의 상위 2-bit를 확인하여
        // signed INT32 범위를 벗어났는지 판단
        case (bias_sum_ext[32:31])

            // 01 : positive overflow
            // signed INT32 최댓값으로 saturation
            2'b01: begin
                acc32_reg = 32'h7FFF_FFFF;
            end

            // 10 : negative overflow
            // signed INT32 최솟값으로 saturation
            2'b10: begin
                acc32_reg = 32'h8000_0000;
            end

            // 00 또는 11이면 signed INT32 범위 내
            default: begin
                acc32_reg = bias_sum_ext[31:0];
            end

        endcase
    end

    // =========================================================
    // Requant Parameter Select
    // =========================================================
    always @(*) begin
        if (is_final_layer_reg) begin
            // Final FC2는 각도 변환 전용 M/S 사용
            current_multiplier = angle_multiplier_reg;
            current_shift      = angle_shift_reg;
        end
        else begin
            // Conv1 / Conv2 / FC1은 일반 requant M/S 사용
            current_multiplier = quant_multiplier_reg;
            current_shift      = quant_shift_reg;
        end
    end

    // =========================================================
    // Signed 32x32 Multiply
    // =========================================================
    always @(*) begin
        // signed INT32 × signed INT32
        // 결과는 signed 64-bit
        mult_result = $signed(acc32_reg) * $signed(current_multiplier);
    end

    // =========================================================
    // Round-to-Nearest, Ties Away from Zero
    // =========================================================
    always @(*) begin
        // 기본값
        mult_abs       = 64'd0;
        round_offset   = 64'd0;
        round_mag_ext  = 65'd0;
        rounded_result = 64'd0;

        // shift가 0이면 rounding 없이 원본 그대로 사용
        if (current_shift == 6'd0) begin
            rounded_result = mult_result;
        end
        else begin
            // mult_result의 절댓값 계산
            // 음수일 경우 2's complement로 magnitude 생성
            if (mult_result[63]) begin
                mult_abs = (~mult_result) + 64'd1;
            end
            else begin
                mult_abs = mult_result;
            end

            // rounding offset = 2^(S-1)
            round_offset = 64'd1 << (current_shift - 6'd1);

            // abs(P) + rounding offset
            // carry 손실 방지를 위해 65-bit에서 계산
            round_mag_ext = {1'b0, mult_abs} + {1'b0, round_offset};

            // magnitude를 right shift한 뒤
            // 원래 mult_result의 부호 복원
            if (mult_result[63]) begin
                rounded_result = -$signed(round_mag_ext >> current_shift);
            end
            else begin
                rounded_result = $signed(round_mag_ext >> current_shift);
            end
        end
    end

    // =========================================================
    // Normal Layer ReLU / INT8 Saturation
    // =========================================================
    always @(*) begin
        // 기본값
        normal_int8_result = 8'd0;

        // ReLU가 활성화된 Layer에서 음수이면 0
        if (relu_en_reg && rounded_result[63]) begin
            normal_int8_result = 8'd0;
        end

        // signed INT8 최댓값 초과
        else if (rounded_result > $signed(64'd127)) begin
            normal_int8_result = 8'h7F;
        end

        // signed INT8 최솟값 미만
        else if (rounded_result < -$signed(64'd128)) begin
            normal_int8_result = 8'h80;
        end

        // signed INT8 범위 안이면 하위 8-bit 사용
        else begin
            normal_int8_result = rounded_result[7:0];
        end
    end

    // =========================================================
    // Final Layer INT8 Saturation
    // =========================================================
    always @(*) begin
        // 기본값
        final_int8_result = 8'd0;

        // Final Layer는 ReLU를 적용하지 않음
        // signed INT8 최댓값 초과
        if (rounded_result > $signed(64'd127)) begin
            final_int8_result = 8'h7F;
        end

        // signed INT8 최솟값 미만
        else if (rounded_result < -$signed(64'd128)) begin
            final_int8_result = 8'h80;
        end

        // signed INT8 범위 내
        else begin
            final_int8_result = rounded_result[7:0];
        end
    end

    // =========================================================
    // Normal INT8 Output Lane Registers
    // =========================================================

    // 각 lane의 post-process 완료 INT8 결과 저장
    // 3개 lane 처리가 모두 끝난 뒤 24-bit o_data로 묶어서 출력
    reg signed [7:0] result_lane0;
    reg signed [7:0] result_lane1;
    reg signed [7:0] result_lane2;

    // 각 lane의 계산이 완료됐는지 표시
    // 모든 유효 lane 계산 완료 여부를 판단할 때 사용
    reg [2:0] result_done_mask;

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

    // 현재 Bias response까지 반영했을 때
    // 어떤 lane의 Bias가 준비되는지 계산
    wire [2:0] bias_loaded_next;

    assign bias_loaded_next = bias_loaded_mask | (3'b001 << bias_rsp_lane_reg);

    // =========================================================
    // Lane Process Completion
    // =========================================================

    // 현재 lane에 대응하는 bit
    wire [2:0] current_lane_bit;

    // 현재 lane 처리까지 반영한 완료 mask
    wire [2:0] result_done_next;

    // 현재 cycle의 lane 처리로
    // beat의 모든 유효 lane 처리가 완료되는지 표시
    wire process_done;

    assign current_lane_bit = (3'b001 << lane_idx);
    assign result_done_next = result_done_mask | (current_lane_valid ? current_lane_bit : 3'b000);
    assign process_done = (state == PROCESS) && (keep_reg != 3'b000) && ((result_done_next & keep_reg) == keep_reg);



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
                  next_state = BIAS_REQ;
            end


            // -------------------------------------------------
            // param_buf에 현재 lane Bias 요청
            // -------------------------------------------------
            BIAS_REQ: begin
                if (bias_req_fire)
                    next_state = BIAS_WAIT;
            end


            // -------------------------------------------------
            // Bias 응답 대기
            // -------------------------------------------------
            BIAS_WAIT: begin
                if (bias_rsp_fire) begin
                    // 이번 response까지 포함했을 때
                    // 현재 Tile에 필요한 모든 Bias가 준비되었는지 확인
                    if ((bias_loaded_next & col_mask_reg) == col_mask_reg)
                        next_state = READY;
                    else
                        next_state = BIAS_REQ;
                end
            end


            // -------------------------------------------------
            // output_fifo 입력 대기
            // -------------------------------------------------
            READY: begin
                if (cfg_fire) begin
                    next_state = BIAS_REQ;
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

        // Normal output 기본값
        o_data           = 24'd0;
        o_valid          = 1'b0;
        o_keep           = 3'b000;
        o_meta           = 19'd0;

        // Final output은 다음 단계에서 연결
        o_final_data     = 8'd0;
        o_final_valid    = 1'b0;


        case (state)

            IDLE: begin
                // cfg 대기
            end

            BIAS_REQ: begin
                // Bias request 발생
                o_bias_req_valid = 1'b1;
                // 현재 bias lane에 해당하는 주소 출력
                o_bias_addr = bias_base_reg + out_ch_base_reg + bias_lane_idx;
            end

            BIAS_WAIT: begin
                // Bias response를 받을 준비
                o_bias_rsp_ready = 1'b1;
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
    // Bias Cache / Control Register Update
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // reset 시 Bias cache 초기화
            bias_cache0       <= 32'd0;
            bias_cache1       <= 32'd0;
            bias_cache2       <= 32'd0;

            // 아직 어떤 Bias도 준비되지 않은 상태
            bias_loaded_mask  <= 3'b000;

            // 유효 lane은 항상 낮은 번호부터 연속되므로
            // 새로운 Tile은 항상 lane0 Bias부터 요청
            bias_lane_idx     <= 2'd0;

            // 아직 수락된 Bias request가 없으므로 초기화
            bias_rsp_lane_reg <= 2'd0;
        end
        else if (cfg_fire) begin
            // 새로운 Tile 설정이 들어오면
            // 이전 Tile의 Bias cache 상태를 무효화
            bias_cache0       <= 32'd0;
            bias_cache1       <= 32'd0;
            bias_cache2       <= 32'd0;

            bias_loaded_mask  <= 3'b000;

            // 새 Tile의 첫 Bias는 lane0부터 요청
            bias_lane_idx     <= 2'd0;

            bias_rsp_lane_reg <= 2'd0;
        end
        else if (bias_req_fire) begin
            // 현재 Bias request가 param_buf에 실제 수락되었으므로,
            // 이후 들어올 Bias response가 어느 lane의 요청인지 기억
            bias_rsp_lane_reg <= bias_lane_idx;
        end
        else if (bias_rsp_fire) begin
            // 응답이 어느 lane에 대한 것인지 확인하여
            // 해당 Bias cache에 저장
            case (bias_rsp_lane_reg)

                2'd0: begin
                    bias_cache0 <= i_bias_data;
                end

                2'd1: begin
                    bias_cache1 <= i_bias_data;
                end

                2'd2: begin
                    bias_cache2 <= i_bias_data;
                end

                default: begin
                    // 정상 동작에서는 발생하지 않음
                end

            endcase

            // 현재 response까지 포함하여
            // Bias 준비 상태 갱신
            bias_loaded_mask <= bias_loaded_next;

            // 아직 필요한 Bias가 더 남아 있다면
            // 다음 연속 lane을 요청하도록 index 증가
            if ((bias_loaded_next & col_mask_reg) != col_mask_reg) begin
                bias_lane_idx <= bias_rsp_lane_reg + 2'd1;
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
    // Lane Processing Register Update
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 처리할 lane은 항상 lane0부터 시작
            lane_idx         <= 2'd0;

            // 아직 처리 완료된 lane 없음
            result_done_mask <= 3'b000;

            // 이전 beat의 결과가 남지 않도록 초기화
            result_lane0     <= 8'd0;
            result_lane1     <= 8'd0;
            result_lane2     <= 8'd0;
            final_result_reg <= 8'd0;
        end
        else if (input_fire) begin
            // 새로운 beat를 수락하면 lane 처리 상태 초기화
            lane_idx         <= 2'd0;
            result_done_mask <= 3'b000;

            // invalid lane의 출력에도 이전 결과가 남지 않도록 초기화
            result_lane0     <= 8'd0;
            result_lane1     <= 8'd0;
            result_lane2     <= 8'd0;
            final_result_reg <= 8'd0;
        end else if (state == PROCESS) begin
            // 현재 lane이 유효한 경우 처리 완료 bit 기록
            if (current_lane_valid) begin

                // Normal Layer
                if (!is_final_layer_reg) begin
                    case (lane_idx)

                        2'd0: begin
                            result_lane0 <= normal_int8_result;
                        end

                        2'd1: begin
                            result_lane1 <= normal_int8_result;
                        end

                        2'd2: begin
                            result_lane2 <= normal_int8_result;
                        end

                        default: begin
                            // 정상 동작에서는 발생하지 않음
                        end

                    endcase
                end

                // Final FC2
                // FC2는 output channel이 1개이므로 lane0만 사용
                else begin
                    if (lane_idx == 2'd0) begin
                        final_result_reg <= final_int8_result;
                    end
                end

                // 현재 lane 처리 완료 기록
                result_done_mask <= result_done_next;
            end
            // 현재 lane 처리로 전체 beat가 끝나지 않았다면
            // 다음 lane으로 이동
            if (!process_done) begin
                lane_idx <= lane_idx + 2'd1;
            end
        end
    end

endmodule
