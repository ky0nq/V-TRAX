`timescale 1ns / 1ps

// ============================================================================
// act_path : Activation 경로 Top
//   RAM -> act_ld_unit -> input_buf -+-> act_patch_gen -+
//                                    |                  +-> MUX -> act_feeder -> PE
//                                    +-> fc_gen --------+
//
// ============================================================================
module act_path (
    input wire clk,
    input wire rst_n,

    // ---- RAM -> input_buf load ----
    input  wire        i_ld_start,
    input  wire [31:0] i_ram_base,
    input  wire [12:0] i_ld_word_count,
    output wire        o_ram_rd_en,
    output wire [11:0] o_ram_rd_addr,
    input  wire [23:0] i_ram_rdata,
    input  wire        i_ram_valid,
    output wire        o_ld_done,

    // ---- cnn_cntl ----
    input wire        i_fc_mode,
    input wire        i_tile_start,
    input wire [13:0] i_src_base,
    input wire [13:0] i_pos_base,    // patch 순번 (2x2 window 순서)
    input wire [ 5:0] i_in_c,
    input wire [ 6:0] i_in_h,
    input wire [ 6:0] i_in_w,
    input wire [ 1:0] i_stride,
    input wire        i_pad_en,
    input wire [12:0] i_k_total,
    input wire [ 2:0] i_row_mask,
    input wire [12:0] i_fc_in_len,
    input wire        i_step_en,
    input wire        i_clear,

    // ---- PE array ----
    output wire [23:0] o_data,
    output wire [ 2:0] o_keep,
    output wire        o_valid,
    input  wire        i_ready,

    // ---- result_buf ----
    input wire        i_result_wr_en,
    input wire [13:0] i_result_wr_addr,
    input wire [23:0] i_result_wr_data,
    input wire [ 2:0] i_result_wr_be


);

    // load -> input_buf
    wire        ibuf_we;
    wire [13:0] ibuf_waddr;
    wire [23:0] ibuf_wdata;
    wire [ 2:0] ibuf_wbe;

    // input_buf read arbitration
    wire pg_rd_en, fc_rd_en;
    wire [13:0] pg_rd_addr, fc_rd_addr;
    wire [23:0] ibuf_rdata;
    wire        ibuf_rvalid;

    wire        ibuf_rd_en = i_fc_mode ? fc_rd_en : pg_rd_en;
    wire [13:0] ibuf_rd_addr = i_fc_mode ? fc_rd_addr : pg_rd_addr;

    // generator -> MUX -> feeder
    wire [23:0] pg_data, fc_data, mux_data;
    wire [2:0] pg_keep, fc_keep, mux_keep;
    wire pg_valid, fc_valid, mux_valid;
    wire pg_ready, fc_ready, mux_ready;

    act_ld_unit ACT_LD_UNIT (
        .clk            (clk),
        .rst_n          (rst_n),
        .i_ld_start     (i_ld_start),
        .i_ram_base     (i_ram_base),
        .i_ld_word_count(i_ld_word_count),
        .o_ram_rd_en    (o_ram_rd_en),
        .o_ram_rd_addr  (o_ram_rd_addr),
        .i_ram_rdata    (i_ram_rdata),
        .i_ram_valid    (i_ram_valid),
        .o_ibuf_we      (ibuf_we),
        .o_ibuf_waddr   (ibuf_waddr),
        .o_ibuf_wdata   (ibuf_wdata),
        .o_ibuf_wbe     (ibuf_wbe),
        .o_ld_done      (o_ld_done)
    );

    input_buf INPUT_BUF (
        .clk       (clk),
        .rst_n     (rst_n),
        // 출력단 생길경우 교체
        // .i_wr_en   (ibuf_we | i_result_wr_en),
        // .i_wr_addr (i_result_wr_en ? i_result_wr_addr : ibuf_waddr),
        // .i_wr_data (i_result_wr_en ? i_result_wr_data : ibuf_wdata),
        // .i_wr_be   (i_result_wr_en ? i_result_wr_be   : ibuf_wbe),
        .i_wr_en   (ibuf_we),
        .i_wr_addr (ibuf_waddr),
        .i_wr_data (ibuf_wdata),
        .i_wr_be   (ibuf_wbe),
        //
        .i_rd_en   (ibuf_rd_en),
        .i_rd_addr (ibuf_rd_addr),
        .o_rd_data (ibuf_rdata),
        .o_rd_valid(ibuf_rvalid)
    );

    act_patch_gen ACT_PATCH_GEN (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_tile_start(i_tile_start && !i_fc_mode),
        .i_src_base  (i_src_base),
        .i_pos_base  (i_pos_base),
        .i_in_c      (i_in_c),
        .i_in_h      (i_in_h),
        .i_in_w      (i_in_w),
        .i_stride    (i_stride),
        .i_pad_en    (i_pad_en),
        .i_k_total   (i_k_total),
        .i_row_mask  (i_row_mask),
        .o_rd_en     (pg_rd_en),
        .o_rd_addr   (pg_rd_addr),
        .i_rd_data   (ibuf_rdata),
        .i_rd_valid  (ibuf_rvalid && !i_fc_mode),
        .o_data      (pg_data),
        .o_keep      (pg_keep),
        .o_valid     (pg_valid),
        .i_ready     (pg_ready)
    );

    fc_gen FC_GEN (
        .clk         (clk),
        .rst_n       (rst_n),
        .i_tile_start(i_tile_start && i_fc_mode),
        .i_src_base  (i_src_base),
        .i_in_len    (i_fc_in_len),
        .o_rd_en     (fc_rd_en),
        .o_rd_addr   (fc_rd_addr),
        .i_rd_data   (ibuf_rdata),
        .i_rd_valid  (ibuf_rvalid && i_fc_mode),
        .o_data      (fc_data),
        .o_keep      (fc_keep),
        .o_valid     (fc_valid),
        .i_ready     (fc_ready)
    );

    MUX MUX (
        .i_fc_mode (i_fc_mode),
        .i_pg_data (pg_data),
        .i_pg_keep (pg_keep),
        .i_pg_valid(pg_valid),
        .o_pg_ready(pg_ready),
        .i_fc_data (fc_data),
        .i_fc_keep (fc_keep),
        .i_fc_valid(fc_valid),
        .o_fc_ready(fc_ready),
        .o_data    (mux_data),
        .o_keep    (mux_keep),
        .o_valid   (mux_valid),
        .i_ready   (mux_ready)
    );

    act_feeder ACT_FEEDER (
        .clk      (clk),
        .rst_n    (rst_n),
        .i_data   (mux_data),
        .i_keep   (mux_keep),
        .i_valid  (mux_valid),
        .o_ready  (mux_ready),
        .i_step_en(i_step_en),
        .i_clear  (i_clear),
        .o_data   (o_data),
        .o_keep   (o_keep),
        .o_valid  (o_valid),
        .i_ready  (i_ready)
    );

endmodule

module act_ld_unit (
    input wire clk,
    input wire rst_n,

    input wire        i_ld_start,
    input wire [31:0] i_ram_base,
    input wire [12:0] i_ld_word_count,

    output wire        o_ram_rd_en,
    output wire [11:0] o_ram_rd_addr,
    input  wire [23:0] i_ram_rdata,
    input  wire        i_ram_valid,

    output wire        o_ibuf_we,
    output wire [13:0] o_ibuf_waddr,
    output wire [23:0] o_ibuf_wdata,
    output wire [ 2:0] o_ibuf_wbe,

    output reg o_ld_done
);
    localparam S_IDLE = 1'd0, S_WAIT = 1'b1;

    reg [1:0] state;
    reg [31:0] ram_base;
    reg [12:0] word_count;
    reg [12:0] word_index;

    wire start_rd = (state == S_IDLE) && i_ld_start && (i_ld_word_count != 13'd0);
    wire c = (state == S_WAIT) && i_ram_valid;
    wire last_word = (word_index == word_count - 13'd1);


    // IDLE: 첫 word 요청
    // WAIT: 현재 응답을 받으면서 다음 word 요청
    wire [31:0] ram_addr_full = (state == S_IDLE) ? i_ram_base : ram_base 
                + {19'd0, word_index} + 32'd1;

    // 유효 명령 조건: ram_base + word_count <= 4096
    assign o_ram_rd_en   = rst_n && (start_rd || (accept_rsp && !last_word));
    assign o_ram_rd_addr = ram_addr_full[11:0];

    // 현재 응답은 현재 word_index에 저장
    assign o_ibuf_we    = rst_n && accept_rsp;
    assign o_ibuf_waddr = {1'b0, word_index};
    assign o_ibuf_wdata = i_ram_rdata;
    assign o_ibuf_wbe   = 3'b111;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            ram_base   <= 32'd0;
            word_count <= 13'd0;
            word_index <= 13'd0;
            o_ld_done  <= 1'b0;
        end else begin
            o_ld_done <= 0;

            case (state)
                S_IDLE: begin
                    if (i_ld_start) begin
                        ram_base   <= i_ram_base;
                        word_count <= i_ld_word_count;
                        word_index <= 13'd0;

                        if (i_ld_word_count == 0) o_ld_done <= 1;
                        else state <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    if (accept_rsp) begin
                        if (last_word) begin
                            o_ld_done <= 1'b1;
                            state     <= S_IDLE;
                        end else begin
                            word_index <= word_index + 1;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

module input_buf (
    input wire        clk,
    input wire        rst_n,
    input wire        i_wr_en,
    input wire [13:0] i_wr_addr,
    input wire [23:0] i_wr_data,
    input wire [ 2:0] i_wr_be,

    input wire        i_rd_en,
    input wire [13:0] i_rd_addr,

    output reg [23:0] o_rd_data,
    output reg        o_rd_valid
);
    reg [23:0] mem[0:16383];

    always @(posedge clk) begin
        if (!rst_n) begin
            // 메모리 내용은 유지하고 응답 valid만 초기화
            o_rd_valid <= 1'b0;
        end else begin
            o_rd_valid <= i_rd_en;
            // byte enable == 1, INT8 lane only
            if (i_wr_en) begin
                if (i_wr_be[0]) mem[i_wr_addr][7:0] <= i_wr_data[7:0];
                if (i_wr_be[1]) mem[i_wr_addr][15:8] <= i_wr_data[15:8];
                if (i_wr_be[2]) mem[i_wr_addr][23:16] <= i_wr_data[23:16];
            end

            if (i_rd_en) o_rd_data <= mem[i_rd_addr];
        end
    end
endmodule


// ============================================================================
// act_patch_gen 
// ============================================================================
module act_patch_gen (
    input wire clk,
    input wire rst_n,

    input wire        i_tile_start,
    input wire [13:0] i_src_base,
    input wire [13:0] i_pos_base,
    input wire [ 5:0] i_in_c,
    input wire [ 6:0] i_in_h,
    input wire [ 6:0] i_in_w,
    input wire [ 1:0] i_stride,
    input wire        i_pad_en,
    input wire [12:0] i_k_total,
    input wire [ 2:0] i_row_mask,

    output reg         o_rd_en,
    output reg  [13:0] o_rd_addr,
    input  wire [23:0] i_rd_data,
    input  wire        i_rd_valid,

    output wire [23:0] o_data,
    output wire [ 2:0] o_keep,
    output wire        o_valid,
    input  wire        i_ready
);

    localparam [2:0] S_IDLE     = 3'd0,
                     S_SELECT   = 3'd1,
                     S_LOC      = 3'd2,
                     S_CAP_INIT = 3'd3,
                     S_CAP_REQ  = 3'd4,
                     S_CAP_WAIT = 3'd5,
                     S_SEND     = 3'd6;

    reg [2:0] state;

    // ---------------- tile / layer 설정 ----------------
    reg [13:0] src_base;
    reg [5:0] in_c;
    reg [6:0] in_h;
    reg [6:0] in_w;
    reg pad_en;
    reg [12:0] k_total;
    reg [2:0] row_mask;
    reg words2;  // 1: pixel당 2 word (Cin=6), 0: 1 word (Cin=3)
    reg [5:0] wpr;  // 한 행의 window 개수 = out_w / 2
    reg [13:0] base_patch;
    reg [1:0] patch_lane;

    wire signed [15:0] in_w_s16 = {9'd0, in_w};
    wire signed [15:0] in_w2 = {8'd0, in_w, 1'b0};  // 2*in_w
    wire signed [15:0] row0_L = pad_en ? -(in_w_s16 + 16'sd1) : 16'sd0;

    // ---------------- window cache (2 bank x 32 word) ----------------
    reg [23:0] window_mem[0:63];
    reg [1:0] win_valid;
    reg [11:0] win_tag0, win_tag1;

    // 현재 lane의 patch -> window / quadrant
    wire [13:0] cur_patch = base_patch + {12'd0, patch_lane};
    wire [11:0] cur_win = cur_patch[13:2];
    wire [ 1:0] cur_q = cur_patch[1:0];
    wire        cur_bank = cur_win[0];
    wire [11:0] cur_tag = cur_bank ? win_tag1 : win_tag0;
    wire        cur_hit = win_valid[cur_bank] && (cur_tag == cur_win);

    // lane별 bank / window 내 시작 pixel 오프셋(qy*4+qx)
    reg         lane_bank                                             [0:2];
    reg  [ 2:0] lane_qoff                                             [0:2];

    // ---------------- window 위치 추적 (나눗셈 대체) ----------------
    reg         trk_valid;
    reg  [11:0] trk_num;
    reg [5:0] trk_wx, trk_wy;
    reg signed [15:0] trk_rowL;  // window 행 시작 pixel의 선형 index
    reg [11:0] loc_rem;

    wire signed [8:0] org_x = $signed(
        {2'b00, trk_wx, 1'b0}
    ) - (pad_en ? 9'sd1 : 9'sd0);
    wire signed [8:0] org_y = $signed(
        {2'b00, trk_wy, 1'b0}
    ) - (pad_en ? 9'sd1 : 9'sd0);
    wire signed [15:0] org_L = trk_rowL + $signed({9'd0, trk_wx, 1'b0});

    // ---------------- capture counter ----------------
    reg signed [8:0] cap_x, cap_y, cap_x0;
    reg signed [15:0] cap_lin, cap_row_lin;
    reg [1:0] cap_cx, cap_cy;
    reg cap_grp;
    // 4×4 window 안의 저장 word 주소
    // Cin=3: pixel당 1 word, Cin=6: pixel당 2 word
    wire [4:0] cap_word = words2 ? {cap_cy, cap_cx, cap_grp} : {1'b0, cap_cy, cap_cx};

    wire signed [9:0] in_w_s10 = {3'b000, in_w};
    wire signed [9:0] in_h_s10 = {3'b000, in_h};
    wire cap_in_range = (cap_x >= 0) && (cap_x < in_w_s10) &&
                        (cap_y >= 0) && (cap_y < in_h_s10);
    wire cap_last = (cap_cx == 2'd3) && (cap_cy == 2'd3) && (cap_grp == words2);

    wire [15:0] cap_lin_sh = words2 ? {cap_lin[14:0], 1'b0} : cap_lin;
    wire [15:0] cap_addr_full = {2'b00, src_base} + cap_lin_sh + {15'd0, cap_grp};

    // ---------------- SEND counter  ----------------
    reg [12:0] send_k;
    reg [1:0] send_kx, send_ky;
    reg  [1:0] send_byte;
    reg        send_grp;
    // group 0: C0~C2, group 1: C3~C5
    wire [5:0] send_ch = (send_grp ? 6'd3 : 6'd0) + {4'd0, send_byte};

    assign o_valid = rst_n && (state == S_SEND) && (row_mask != 3'b000);
    assign o_keep  = row_mask;

    // lane별 window_mem 직접 읽기
    reg [23:0] lane_data;

    integer lane;
    reg [3:0]  pix;
    reg [5:0]  widx;
    reg [23:0] selected_word;

    always @(*) begin
        for (lane = 0; lane < 3; lane = lane + 1) begin
            // 현재 패치의 시작점 + kernel 위치
            pix = {1'b0, lane_qoff[lane]} + {send_ky, 2'b00} + {2'b00, send_kx};

            // bank와 픽셀 내부 word 선택
            widx = words2 ? {lane_bank[lane], pix, send_grp} : {lane_bank[lane], 1'b0, pix};

            selected_word = window_mem[widx];

            // 활성 lane에 현재 입력 채널의 8bit 값 출력
            lane_data[8*lane +: 8] = (state == S_SEND && row_mask[lane]) ? 
                                    selected_word[8*send_byte +: 8] : 8'd0;
        end
    end
    assign o_data = lane_data;

    // ---------------- capture 진행 (REQ/WAIT 공용) ----------------
    task cap_advance;
        begin
            if (cap_last) begin
                if (cur_bank) win_tag1 <= cur_win;
                else win_tag0 <= cur_win;
                win_valid[cur_bank] <= 1'b1;
                state <= S_SELECT;
            end else begin
                state <= S_CAP_REQ;
                if (words2 && !cap_grp) begin
                    cap_grp <= 1'b1;
                end else begin
                    cap_grp <= 1'b0;
                    if (cap_cx == 2'd3) begin
                        cap_cx      <= 2'd0;
                        cap_cy      <= cap_cy + 2'd1;
                        cap_x       <= cap_x0;
                        cap_y       <= cap_y + 9'sd1;
                        cap_lin     <= cap_row_lin + in_w_s16;
                        cap_row_lin <= cap_row_lin + in_w_s16;
                    end else begin
                        cap_cx  <= cap_cx + 2'd1;
                        cap_x   <= cap_x + 9'sd1;
                        cap_lin <= cap_lin + 16'sd1;
                    end
                end
            end
        end
    endtask

    wire [6:0] out_w_in = i_pad_en ? i_in_w : (i_in_w - 7'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            src_base   <= 14'd0;
            in_c       <= 6'd3;
            in_h       <= 7'd0;
            in_w       <= 7'd0;
            pad_en     <= 1'b0;
            k_total    <= 13'd0;
            row_mask   <= 3'b000;
            words2     <= 1'b0;
            wpr        <= 6'd1;
            base_patch <= 14'd0;
            patch_lane <= 2'd0;
            win_valid  <= 2'b00;
            win_tag0   <= 12'd0;
            win_tag1   <= 12'd0;
            trk_valid  <= 1'b0;
            trk_num    <= 12'd0;
            trk_wx     <= 6'd0;
            trk_wy     <= 6'd0;
            trk_rowL   <= 16'sd0;
            loc_rem    <= 12'd0;
            o_rd_en    <= 1'b0;
            o_rd_addr  <= 14'd0;
            send_k     <= 13'd0;
            send_kx    <= 2'd0;
            send_ky    <= 2'd0;
            send_byte  <= 2'd0;
            send_grp   <= 1'b0;
        end else begin
            o_rd_en <= 1'b0;

            case (state)
                // -----------------------------------------------------
                S_IDLE: begin
                    if (i_tile_start) begin
                        base_patch <= i_pos_base;
                        patch_lane <= 2'd0;
                        row_mask <= i_row_mask;

                        src_base <= i_src_base;
                        in_c <= i_in_c;
                        in_h <= i_in_h;
                        in_w <= i_in_w;
                        pad_en <= i_pad_en;
                        k_total <= i_k_total;
                        words2 <= (i_in_c > 6'd3);
                        wpr <= (out_w_in[6:1] == 6'd0) ? 6'd1 : out_w_in[6:1];

                        // 새 순회 / 설정 변경이면 cache와 위치 추적 무효화
                        if ((i_pos_base == 14'd0) ||
                            (i_src_base != src_base) ||
                            (i_in_c     != in_c)     ||
                            (i_in_h     != in_h)     ||
                            (i_in_w     != in_w)     ||
                            (i_pad_en   != pad_en)) begin
                            win_valid <= 2'b00;
                            trk_valid <= 1'b0;
                        end

                        state <= S_SELECT;
                    end
                end

                // -----------------------------------------------------
                // lane 0,1,2의 patch가 속한 window가 cache에 있는지 확인
                // -----------------------------------------------------
                S_SELECT: begin
                    if (patch_lane == 2'd3) begin
                        send_k    <= 13'd0;
                        send_kx   <= 2'd0;
                        send_ky   <= 2'd0;
                        send_byte <= 2'd0;
                        send_grp  <= 1'b0;
                        state     <= (row_mask == 3'b000) ? S_IDLE : S_SEND;
                    end else if (!row_mask[patch_lane]) begin
                        patch_lane <= patch_lane + 2'd1;
                    end else if (cur_hit) begin
                        lane_bank[patch_lane] <= cur_bank;
                        lane_qoff[patch_lane] <= {cur_q[1], 1'b0, cur_q[0]};
                        patch_lane            <= patch_lane + 2'd1;
                    end else begin
                        // miss: 이 bank를 새 window로 교체
                        win_valid[cur_bank] <= 1'b0;

                        if (trk_valid && (cur_win == trk_num + 12'd1)) begin
                            // 직전 window의 다음 window -> 위치 1칸 전진
                            trk_num <= cur_win;
                            if ({1'b0, trk_wx} + 7'd1 >= {1'b0, wpr}) begin
                                trk_wx   <= 6'd0;
                                trk_wy   <= trk_wy + 6'd1;
                                trk_rowL <= trk_rowL + in_w2;
                            end else begin
                                trk_wx <= trk_wx + 6'd1;
                            end
                            state <= S_CAP_INIT;
                        end else begin
                            // 임의 위치: 뺄셈 반복으로 (wx, wy) 계산
                            trk_valid <= 1'b0;
                            loc_rem   <= cur_win;
                            trk_wy    <= 6'd0;
                            trk_rowL  <= row0_L;
                            state     <= S_LOC;
                        end
                    end
                end

                S_LOC: begin
                    if (loc_rem >= {6'd0, wpr}) begin
                        loc_rem  <= loc_rem - {6'd0, wpr};
                        trk_wy   <= trk_wy + 6'd1;
                        trk_rowL <= trk_rowL + in_w2;
                    end else begin
                        trk_wx    <= loc_rem[5:0];
                        trk_num   <= cur_win;
                        trk_valid <= 1'b1;
                        state     <= S_CAP_INIT;
                    end
                end

                // -----------------------------------------------------
                S_CAP_INIT: begin
                    cap_x       <= org_x;
                    cap_x0      <= org_x;
                    cap_y       <= org_y;
                    cap_lin     <= org_L;
                    cap_row_lin <= org_L;
                    cap_cx      <= 2'd0;
                    cap_cy      <= 2'd0;
                    cap_grp     <= 1'b0;
                    state       <= S_CAP_REQ;
                end

                // Read 요청을 register로 내보냄 (범위 밖이면 0 저장)
                S_CAP_REQ: begin
                    if (cap_in_range) begin
                        o_rd_en   <= 1'b1;
                        o_rd_addr <= cap_addr_full[13:0];
                        state     <= S_CAP_WAIT;
                    end else begin
                        window_mem[{cur_bank, cap_word}] <= 24'd0;
                        cap_advance;
                    end
                end

                S_CAP_WAIT: begin
                    if (i_rd_valid) begin
                        window_mem[{cur_bank, cap_word}] <= i_rd_data;
                        cap_advance;
                    end
                end

                // -----------------------------------------------------
                S_SEND: begin
                    if (o_valid && i_ready) begin
                        if (send_k == k_total - 13'd1) begin
                            state <= S_IDLE;
                        end else begin
                            send_k <= send_k + 13'd1;
                            if (send_ch == in_c - 6'd1) begin
                                send_byte <= 2'd0;
                                send_grp  <= 1'b0;
                                if (send_kx == 2'd2) begin
                                    send_kx <= 2'd0;
                                    send_ky <= send_ky + 2'd1;
                                end else begin
                                    send_kx <= send_kx + 2'd1;
                                end
                            end else begin
                                if (send_byte == 2'd2) begin
                                    send_byte <= 2'd0;
                                    send_grp  <= 1'b1;
                                end else begin
                                    send_byte <= send_byte + 2'd1;
                                end
                            end
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule

module fc_gen (
    input wire clk,
    input wire rst_n,

    input wire        i_tile_start,
    input wire [13:0] i_src_base,
    input wire [12:0] i_in_len,

    output reg         o_rd_en,
    output reg  [13:0] o_rd_addr,
    input  wire [23:0] i_rd_data,
    input  wire        i_rd_valid,

    output reg  [23:0] o_data,
    output wire [ 2:0] o_keep,
    output wire        o_valid,
    input  wire        i_ready
);
    // Conv 출력 32x32x4 -> FC0 입력 4096개
    localparam integer FC0_CHANNELS = 4;
    localparam integer FC0_INPUT_COUNT = 32 * 32 * FC0_CHANNELS;

    localparam [1:0] S_IDLE = 2'd0, S_GET = 2'd1, S_WAIT = 2'd2, S_SEND = 2'd3;

    reg [ 1:0] state;

    reg [13:0] base_addr;
    reg [12:0] input_count;
    reg [12:0] k;

    reg        cached_valid;
    reg [13:0] cached_addr;
    reg [23:0] cached_word;

    reg [13:0] wanted_addr;

    reg [12:0] word_offset;
    reg [ 1:0] byte_lane;

    // 필요한 word가 현재 캐시에 있으면 메모리 읽기 생략
    wire cache_hit = cached_valid && (cached_addr == wanted_addr);

    assign o_valid = rst_n && (state == S_SEND);
    assign o_keep  = 3'b001;

    always @(*) begin
        if (input_count == FC0_INPUT_COUNT) begin
            // 픽셀당 2 word: C0,C1,C2 / C3
            word_offset = {1'b0, k[12:2], (k[1:0] == 2'd3)};
            byte_lane   = (k[1:0] == 2'd3) ? 2'd0 : k[1:0];
        end else begin
            // 이후 FC: word당 3개
            word_offset = k / 13'd3;
            byte_lane   = k % 13'd3;
        end

        wanted_addr = base_addr + word_offset;

        o_rd_en = rst_n && (state == S_GET) && !cache_hit;
        o_rd_addr = wanted_addr;

        o_data = 24'd0;
        if (state == S_SEND) o_data[7:0] = cached_word[8*byte_lane+:8];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            base_addr    <= 14'd0;
            input_count  <= 13'd0;
            k            <= 13'd0;
            cached_valid <= 1'b0;
            cached_addr  <= 14'd0;
            cached_word  <= 24'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (i_tile_start && i_in_len != 0) begin
                        base_addr    <= i_src_base;
                        input_count  <= i_in_len;
                        k            <= 13'd0;
                        cached_valid <= 1'b0;
                        state        <= S_GET;
                    end
                end

                S_GET: begin
                    state <= cache_hit ? S_SEND : S_WAIT;
                end

                S_WAIT: begin
                    if (i_rd_valid) begin
                        cached_word  <= i_rd_data;
                        cached_addr  <= wanted_addr;
                        cached_valid <= 1'b1;
                        state        <= S_SEND;
                    end
                end

                S_SEND: begin
                    if (o_valid && i_ready) begin
                        if (k == input_count - 1'b1) begin
                            state <= S_IDLE;
                        end else begin
                            k     <= k + 1'b1;
                            state <= S_GET;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

module MUX (
    input wire i_fc_mode,

    input  wire [23:0] i_pg_data,
    input  wire [ 2:0] i_pg_keep,
    input  wire        i_pg_valid,
    output wire        o_pg_ready,

    input  wire [23:0] i_fc_data,
    input  wire [ 2:0] i_fc_keep,
    input  wire        i_fc_valid,
    output wire        o_fc_ready,

    output wire [23:0] o_data,
    output wire [ 2:0] o_keep,
    output wire        o_valid,
    input  wire        i_ready
);
    assign o_data = i_fc_mode ? i_fc_data : i_pg_data;
    assign o_keep = i_fc_mode ? i_fc_keep : i_pg_keep;
    assign o_valid = i_fc_mode ? i_fc_valid : i_pg_valid;

    assign o_pg_ready = !i_fc_mode && i_ready;
    assign o_fc_ready = i_fc_mode && i_ready;
endmodule

module act_feeder (
    input wire clk,
    input wire rst_n,

    input  wire [23:0] i_data,
    input  wire [ 2:0] i_keep,
    input  wire        i_valid,
    output wire        o_ready,
    input  wire        i_step_en,
    input  wire        i_clear,

    output reg  [23:0] o_data,
    output reg  [ 2:0] o_keep,
    output reg         o_valid,
    input  wire        i_ready
);
    assign o_ready = rst_n && !i_clear && i_step_en && (!o_valid || i_ready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_data  <= 24'd0;
            o_keep  <= 3'b000;
            o_valid <= 1'b0;
        end else if (i_clear) begin
            o_data  <= 24'd0;
            o_keep  <= 3'b000;
            o_valid <= 1'b0;
        end else begin
            if (o_ready) begin
                o_valid <= i_valid;
                if (i_valid) begin
                    o_data <= i_data;
                    o_keep <= i_keep;
                end
            end else if (o_valid && i_ready) begin
                o_valid <= 1'b0;
            end
        end
    end
endmodule




