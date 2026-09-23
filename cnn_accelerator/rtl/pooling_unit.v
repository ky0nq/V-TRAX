`timescale 1ns / 1ps

module pooling_unit (
    input wire clk,
    input wire rst_n,

    // =========================================================
    // Layer Configuration
    // =========================================================
    input wire       i_cfg_valid,
    input wire       i_pool_en,
    input wire [6:0] i_in_w,
    input wire [6:0] i_in_h,
    input wire [5:0] i_channels,

    // =========================================================
    // Input Stream from post_process
    // =========================================================
    input  wire [23:0] i_data,
    input  wire        i_valid,
    output wire        o_ready,

    input wire [ 2:0] i_keep,
    input wire [18:0] i_meta,

    // =========================================================
    // Output Stream to result_buf
    // =========================================================
    output wire [23:0] o_data,
    output wire        o_valid,
    input  wire        i_ready,

    output wire [ 2:0] o_keep,
    output wire [18:0] o_meta,

    // =========================================================
    // Tile Input Completion
    // =========================================================
    output reg o_tile_in_done
);

    // =========================================================
    // Layer Configuration Registers
    // =========================================================
    reg       pool_en_reg;
    reg [6:0] in_w_reg;
    reg [6:0] in_h_reg;
    reg [5:0] channels_reg;
    reg [2:0] in_w_sh_reg;     // log2(in_w).  in_w 는 2 의 거듭제곱 (64 / 32 / 16)
    reg [6:0] in_w_mask_reg;   // in_w - 1

    // 2 의 거듭제곱 (1 ~ 64) 의 log2. 그 밖의 값은 최상위 1 의 자리 (타이밍 : 나눗셈 대신 shift 용)
    function [2:0] f_log2_7;
        input [6:0] v;
        begin
            casez (v)
                7'b1??????: f_log2_7 = 3'd6;
                7'b01?????: f_log2_7 = 3'd5;
                7'b001????: f_log2_7 = 3'd4;
                7'b0001???: f_log2_7 = 3'd3;
                7'b00001??: f_log2_7 = 3'd2;
                7'b000001?: f_log2_7 = 3'd1;
                default:    f_log2_7 = 3'd0;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pool_en_reg   <= 1'b0;
            in_w_reg      <= 7'd0;
            in_h_reg      <= 7'd0;
            channels_reg  <= 6'd0;
            in_w_sh_reg   <= 3'd0;
            in_w_mask_reg <= 7'd0;
        end else if (i_cfg_valid) begin
            pool_en_reg   <= i_pool_en;
            in_w_reg      <= i_in_w;
            in_h_reg      <= i_in_h;
            channels_reg  <= i_channels;
            in_w_sh_reg   <= f_log2_7(i_in_w);
            in_w_mask_reg <= i_in_w - 7'd1;
        end
    end

    // =========================================================
    // Input Metadata Decode
    // =========================================================

    // i_meta[11:0]  : input spatial position
    // i_meta[16:12] : output channel base
    // i_meta[17]    : tile_end
    // i_meta[18]    : layer_end
    wire [11:0] input_pos;
    wire [ 4:0] input_out_ch_base;
    wire        input_tile_end;
    wire        input_layer_end;

    assign input_pos         = i_meta[11:0];
    assign input_out_ch_base = i_meta[16:12];
    assign input_tile_end    = i_meta[17];
    assign input_layer_end   = i_meta[18];

    // 채널 그룹 선택. 1792 줄 always 블록보다 앞에 있어야 해서 여기로 올렸다 (사용 전 선언).
    // Current MaxPool layer has 6 output channels.
    //
    // group 0 : channel 0 ~ 2, out_ch_base = 0
    // group 1 : channel 3 ~ 5, out_ch_base = 3
    wire pool_group_sel;

    assign pool_group_sel = (input_out_ch_base == 5'd3);

    // =========================================================
    // Position Decode
    // =========================================================

    // pos = y * in_w + x
    wire [11:0] input_y_full;
    wire [11:0] input_x_full;

    // 나눗셈 대신 shift / mask (in_w 는 2 의 거듭제곱). 조합 나눗셈은 100 MHz 에서 -20 ns 였다 (2026-09-23)
    assign input_y_full = input_pos >> in_w_sh_reg;
    assign input_x_full = input_pos & {5'd0, in_w_mask_reg};

    wire [6:0] input_y;
    wire [6:0] input_x;

    assign input_y = input_y_full[6:0];
    assign input_x = input_x_full[6:0];

    // =========================================================
    // 2x2 Pool Window Position
    // =========================================================

    // q = 2 * (y % 2) + (x % 2)
    //
    // q=0 : top-left
    // q=1 : top-right
    // q=2 : bottom-left
    // q=3 : bottom-right
    wire [1:0] pool_q;

    assign pool_q = {input_y[0], input_x[0]};

    // Pool output position
    // pool_pos = (y/2) * (in_w/2) + (x/2)
    wire [11:0] pool_position;

    // (y/2) * (in_w/2) + (x/2)  =  (y/2) << (log2(in_w) - 1)  |  (x/2)
    wire [11:0] pool_y2 = {6'd0, input_y[6:1]};
    wire [11:0] pool_x2 = {6'd0, input_x[6:1]};
    assign pool_position = (in_w_sh_reg == 3'd0) ? 12'd0 : ((pool_y2 << (in_w_sh_reg - 3'd1)) | pool_x2);

    // 동일 channel group에서 하나의 2x2 window를 구분하는 ID.
    // 현재 구조에서는 pool output position과 동일하다.
    wire [11:0] pool_window_id;

    assign pool_window_id = pool_position;

    // =========================================================
    // Input / Output Handshake
    // =========================================================

    // result_buf로 전달할 출력 1개를 보관하는 register
    reg  [23:0] out_data_reg;
    reg         out_valid_reg;
    reg  [ 2:0] out_keep_reg;
    reg  [18:0] out_meta_reg;

    wire        input_fire;
    wire        output_fire;
    wire        pool_state_ready;

    // Output register가 비어 있거나,
    // 현재 output이 같은 cycle에 downstream으로 전달되면
    // 새로운 input을 받을 수 있다.
    assign o_ready =
        rst_n &&
        !i_cfg_valid &&
        (!out_valid_reg || i_ready) &&
        (!i_valid || pool_state_ready);

    assign input_fire = i_valid && o_ready;
    assign output_fire = out_valid_reg && i_ready;

    // Output stream
    assign o_data = out_data_reg;
    assign o_valid = out_valid_reg;
    assign o_keep = out_keep_reg;
    assign o_meta = out_meta_reg;

    // =========================================================
    // Input Lane Split
    // =========================================================

    wire signed [7:0] input_lane0;
    wire signed [7:0] input_lane1;
    wire signed [7:0] input_lane2;

    assign input_lane0 = $signed(i_data[7:0]);
    assign input_lane1 = $signed(i_data[15:8]);
    assign input_lane2 = $signed(i_data[23:16]);

    // =========================================================
    // Pool Channel Group Select
    // =========================================================



    // =========================================================
    // Group 0 Pool State : Channel 0 ~ 2
    // =========================================================

    // 현재 처리 중인 2x2 window가 존재하는지 표시
    reg               group0_window_valid;

    // 현재 그룹이 처리 중인 window ID
    reg        [11:0] group0_window_id;

    // 다음에 받아야 하는 q
    // 0 -> 1 -> 2 -> 3
    reg        [ 1:0] group0_next_q;

    // 각 channel lane의 현재 Max 값
    reg signed [ 7:0] group0_max0;
    reg signed [ 7:0] group0_max1;
    reg signed [ 7:0] group0_max2;
    reg [2:0] group0_keep_reg;

    // =========================================================
    // Group 1 Pool State : Channel 3 ~ 5
    // =========================================================

    reg               group1_window_valid;
    reg        [11:0] group1_window_id;
    reg        [ 1:0] group1_next_q;

    reg signed [ 7:0] group1_max0;
    reg signed [ 7:0] group1_max1;
    reg signed [ 7:0] group1_max2;
    reg [2:0] group1_keep_reg;

    // =========================================================
    // Final Max for q=3
    // =========================================================

    wire signed [7:0] group0_final_max0;
    wire signed [7:0] group0_final_max1;
    wire signed [7:0] group0_final_max2;

    wire signed [7:0] group1_final_max0;
    wire signed [7:0] group1_final_max1;
    wire signed [7:0] group1_final_max2;

    assign group0_final_max0 =
        (group0_keep_reg[0] && (input_lane0 > group0_max0)) ?
        input_lane0 : group0_max0;

    assign group0_final_max1 =
        (group0_keep_reg[1] && (input_lane1 > group0_max1)) ?
        input_lane1 : group0_max1;

    assign group0_final_max2 =
        (group0_keep_reg[2] && (input_lane2 > group0_max2)) ?
        input_lane2 : group0_max2;

    assign group1_final_max0 =
        (group1_keep_reg[0] && (input_lane0 > group1_max0)) ?
        input_lane0 : group1_max0;

    assign group1_final_max1 =
        (group1_keep_reg[1] && (input_lane1 > group1_max1)) ?
        input_lane1 : group1_max1;

    assign group1_final_max2 =
        (group1_keep_reg[2] && (input_lane2 > group1_max2)) ?
        input_lane2 : group1_max2;

    // =========================================================
    // Last Pool Output Detection
    // =========================================================

    wire [2:0] input_valid_lane_count;
    wire [6:0] input_group_end_ch;

    wire pool_last_window;
    wire pool_last_group;
    wire pool_output_layer_end;

    assign input_valid_lane_count =
        {2'd0, i_keep[0]} +
        {2'd0, i_keep[1]} +
        {2'd0, i_keep[2]};

    // exclusive end channel
    // ex) base=3, keep=111 -> 3+3=6
    assign input_group_end_ch =
        {2'd0, input_out_ch_base} +
        {4'd0, input_valid_lane_count};

    assign pool_last_window =
        (in_w_reg != 7'd0) &&
        (in_h_reg != 7'd0) &&
        (input_x == (in_w_reg - 7'd1)) &&
        (input_y == (in_h_reg - 7'd1));

    assign pool_last_group =
        (input_group_end_ch >= {1'b0, channels_reg});

    assign pool_output_layer_end =
        input_layer_end &&
        pool_last_window &&
        pool_last_group;

    // =========================================================
    // Input Validity Check
    // =========================================================
    wire input_keep_valid;
    wire input_position_valid;
    wire pool_group_valid;
    wire pool_cfg_valid;
    wire pool_layer_end_valid;
    // =========================================================
    // Pool Configuration Validity
    // =========================================================

    // 2x2 MaxPool 조건
    // - width / height는 최소 2
    // - width / height는 짝수
    // - channel 수는 1 이상, 최대 6
    assign pool_cfg_valid =
        (in_w_reg >= 7'd2) &&
        (in_h_reg >= 7'd2) &&
        (in_w_reg[0] == 1'b0) &&
        (in_h_reg[0] == 1'b0) &&
        ((in_w_reg & (in_w_reg - 7'd1)) == 7'd0) &&   // in_w 는 2 의 거듭제곱 (위치를 shift 로 계산)
        (channels_reg >= 6'd1) &&
        (channels_reg <= 6'd6);

    // Pool 경로에서 layer_end가 들어온 경우에는
    // 반드시 마지막 2x2 window의 q=3,
    // 그리고 마지막 channel group이어야 한다.
    assign pool_layer_end_valid =
        !input_layer_end ||
        (
            (pool_q == 2'd3) &&
            pool_last_window &&
            pool_last_group
        );

    assign input_position_valid =
        (in_w_reg != 7'd0) &&
        (in_h_reg != 7'd0) &&
        (input_y_full < {5'd0, in_h_reg}) &&
        (input_x_full < {5'd0, in_w_reg});

    assign input_keep_valid =
        (i_keep == 3'b001) ||
        (i_keep == 3'b011) ||
        (i_keep == 3'b111);

    // 현재 MaxPool Layer는 6채널이므로
    // channel group base는 0 또는 3만 허용
    assign pool_group_valid =
        (input_out_ch_base == 5'd0) ||
        (input_out_ch_base == 5'd3);

    // =========================================================
    // Pool Input Sequence Check
    // =========================================================

    // Bypass에서는 항상 입력 가능.
    //
    // MaxPool에서는 각 channel group마다
    // q=0 -> q=1 -> q=2 -> q=3 순서를 강제한다.
    //
    // q=0 : 해당 group에 진행 중인 window가 없어야 함
    // q=1~3 : 동일 window_id이고 next_q와 일치해야 함
    assign pool_state_ready =
        !pool_en_reg ?
        (
            input_keep_valid
        )
        :
        (
            input_keep_valid &&
            input_position_valid &&
            pool_group_valid &&
            pool_cfg_valid &&
            pool_layer_end_valid &&
            (
                !pool_group_sel ?
                (
                    (pool_q == 2'd0) ?
                        !group0_window_valid
                    :
                        (
                            group0_window_valid &&
                            (group0_window_id == pool_window_id) &&
                            (group0_next_q == pool_q) &&
                            (group0_keep_reg == i_keep)
                        )
                )
                :
                (
                    (pool_q == 2'd0) ?
                        !group1_window_valid
                    :
                        (
                            group1_window_valid &&
                            (group1_window_id == pool_window_id) &&
                            (group1_next_q == pool_q) &&
                            (group1_keep_reg == i_keep)
                        )
                )
            )
        );
        
    // (아래 always 는 group*_final_max / *_keep_reg / pool_output_layer_end 를 쓰므로 그 선언들 뒤로 옮겼다 : 사용 전 선언)
    // =========================================================
    // Output Register / Tile Input Done
    // =========================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_data_reg      <= 24'd0;
            out_valid_reg     <= 1'b0;
            out_keep_reg      <= 3'b000;
            out_meta_reg      <= 19'd0;

            o_tile_in_done    <= 1'b0;
        end
        else begin
            // pulse 기본값
            o_tile_in_done <= 1'b0;

            // 새 Layer 시작 시 pending output 제거
            if (i_cfg_valid) begin
                out_valid_reg <= 1'b0;
            end
            else begin

                // 현재 output이 downstream에 수락되면 제거
                if (output_fire) begin
                    out_valid_reg <= 1'b0;
                end

                // -------------------------------------------------
                // 새로운 input 수락
                // -------------------------------------------------
                if (input_fire) begin

                    // =============================================
                    // Bypass
                    // =============================================
                    if (!pool_en_reg) begin
                        out_data_reg  <= i_data;
                        out_keep_reg  <= i_keep;
                        out_meta_reg  <= i_meta;
                        out_valid_reg <= 1'b1;
                    end

                    // =============================================
                    // 2x2 MaxPool
                    // q=3에서만 output 생성
                    // =============================================
                    else if (pool_q == 2'd3) begin

                        if (!pool_group_sel) begin
                            out_data_reg <= {
                                group0_final_max2,
                                group0_final_max1,
                                group0_final_max0
                            };
                        
                            out_keep_reg <= group0_keep_reg;
                        end
                        else begin
                            out_data_reg <= {
                                group1_final_max2,
                                group1_final_max1,
                                group1_final_max0
                            };
                        
                            out_keep_reg <= group1_keep_reg;
                        end

                        // Pool output metadata
                        //
                        // [18] layer_end
                        // [17] tile_end = 항상 0
                        // [16:12] out_ch_base
                        // [11:0] pooled position
                        out_meta_reg <= {
                            pool_output_layer_end,
                            1'b0,
                            input_out_ch_base,
                            pool_position
                        };

                        out_valid_reg <= 1'b1;
                    end

                    // Tile의 마지막 input beat를 수락한 순간
                    if (input_tile_end) begin
                        o_tile_in_done <= 1'b1;
                    end
                end
            end
        end
    end
    

    // =========================================================
    // Pool State Update
    // =========================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            group0_window_valid <= 1'b0;
            group0_window_id    <= 12'd0;
            group0_next_q       <= 2'd0;

            group0_max0         <= 8'd0;
            group0_max1         <= 8'd0;
            group0_max2         <= 8'd0;

            group1_window_valid <= 1'b0;
            group1_window_id    <= 12'd0;
            group1_next_q       <= 2'd0;

            group1_max0         <= 8'd0;
            group1_max1         <= 8'd0;
            group1_max2         <= 8'd0;
            group0_keep_reg     <= 3'b000;
            group1_keep_reg     <= 3'b000;
        end
        else if (i_cfg_valid) begin
            // Layer 변경 시에만 Pool 상태 clear
            group0_window_valid <= 1'b0;
            group0_window_id    <= 12'd0;
            group0_next_q       <= 2'd0;

            group0_max0         <= 8'd0;
            group0_max1         <= 8'd0;
            group0_max2         <= 8'd0;

            group1_window_valid <= 1'b0;
            group1_window_id    <= 12'd0;
            group1_next_q       <= 2'd0;

            group1_max0         <= 8'd0;
            group1_max1         <= 8'd0;
            group1_max2         <= 8'd0;
            group0_keep_reg     <= 3'b000;
            group1_keep_reg     <= 3'b000;
        end
        else if (input_fire && pool_en_reg) begin

            // =================================================
            // Group 0 : Channel 0 ~ 2
            // =================================================
            if (!pool_group_sel) begin
                case (pool_q)

                    // q=0 : 실제 첫 입력값으로 Max 초기화
                    2'd0: begin
                        group0_window_valid <= 1'b1;
                        group0_window_id    <= pool_window_id;
                        group0_next_q       <= 2'd1;

                        group0_keep_reg <= i_keep;

                        group0_max0 <= input_lane0;
                        group0_max1 <= input_lane1;
                        group0_max2 <= input_lane2;
                    end

                    // q=1
                    2'd1: begin
                        if (group0_keep_reg[0] && (input_lane0 > group0_max0))
                            group0_max0 <= input_lane0;

                        if (group0_keep_reg[1] && (input_lane1 > group0_max1))
                            group0_max1 <= input_lane1;

                        if (group0_keep_reg[2] && (input_lane2 > group0_max2))
                            group0_max2 <= input_lane2;

                        group0_next_q <= 2'd2;
                    end

                    // q=2
                    2'd2: begin
                        if (group0_keep_reg[0] && (input_lane0 > group0_max0))
                            group0_max0 <= input_lane0;

                        if (group0_keep_reg[1] && (input_lane1 > group0_max1))
                            group0_max1 <= input_lane1;

                        if (group0_keep_reg[2] && (input_lane2 > group0_max2))
                            group0_max2 <= input_lane2;

                        group0_next_q <= 2'd3;
                    end

                    // q=3 :
                    // 마지막 비교 결과는 다음 단계에서
                    // output register에 직접 넣는다.
                    2'd3: begin
                        group0_window_valid <= 1'b0;
                        group0_next_q       <= 2'd0;
                    end

                    default: begin
                    end
                endcase
            end

            // =================================================
            // Group 1 : Channel 3 ~ 5
            // =================================================
            else begin
                case (pool_q)

                    2'd0: begin
                        group1_window_valid <= 1'b1;
                        group1_window_id    <= pool_window_id;
                        group1_next_q       <= 2'd1;

                        group1_keep_reg <= i_keep;

                        group1_max0 <= input_lane0;
                        group1_max1 <= input_lane1;
                        group1_max2 <= input_lane2;
                    end

                    2'd1: begin
                        if (group1_keep_reg[0] && (input_lane0 > group1_max0))
                            group1_max0 <= input_lane0;

                        if (group1_keep_reg[1] && (input_lane1 > group1_max1))
                            group1_max1 <= input_lane1;

                        if (group1_keep_reg[2] && (input_lane2 > group1_max2))
                            group1_max2 <= input_lane2;

                        group1_next_q <= 2'd2;
                    end

                    2'd2: begin
                        if (group1_keep_reg[0] && (input_lane0 > group1_max0))
                            group1_max0 <= input_lane0;

                        if (group1_keep_reg[1] && (input_lane1 > group1_max1))
                            group1_max1 <= input_lane1;

                        if (group1_keep_reg[2] && (input_lane2 > group1_max2))
                            group1_max2 <= input_lane2;

                        group1_next_q <= 2'd3;
                    end

                    2'd3: begin
                        group1_window_valid <= 1'b0;
                        group1_next_q       <= 2'd0;
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

endmodule
