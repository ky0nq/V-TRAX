//=====================================================================
// axi4_write_datapath.v
//
// AXI4 DMA Write Engine의 Datapath.
//
// ★ 이번 수정 사항 ★
// 1) 3슬레이브(ROM/DDR3L/RAM) 주소 디코더 추가 (Read와 동일 패턴)
// 2) awid에 실리는 slave_sel도 2bit로 3값 표현 가능하게 반영
// 3) 잘못된 목적지 주소(region_err) 방어 추가
//=====================================================================
module AXI4_write_datapath #(
    parameter ADDR_WIDTH  = 15,
    parameter DATA_WIDTH  = 32,
    parameter LEN_WIDTH   = 32,
    parameter BURST_WIDTH = 8,

    // ===== 신규: 3슬레이브 주소맵 (Interconnect / Read Datapath와 동일 값이어야 함) =====
    parameter [ADDR_WIDTH-1:0] ROM_BASE    = 15'h0000,
    parameter [ADDR_WIDTH-1:0] ROM_SIZE    = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_BASE  = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_SIZE  = 15'h4000,
    parameter [ADDR_WIDTH-1:0] RAM_BASE    = 15'h4400,
    parameter [ADDR_WIDTH-1:0] RAM_SIZE    = 15'h0400
) (
    input wire clk,
    input wire rst_n,

    // ---- Controller -> Datapath ----
    input wire en,   // state == S_DATA
    input wire init, // IDLE -> DATA 진입 1cycle pulse

    // ---- Register Map 입력 ----
    input wire [ ADDR_WIDTH-1:0] dst_addr,
    input wire [  LEN_WIDTH-1:0] length,    // 총 전송 바이트 수
    input wire [BURST_WIDTH+1:0] burst_cfg, // ★ 폭 수정: [7:0]이 아니라 [9:8]도 있어야 함
    // burst_cfg[9:8] : Burst Type (00 FIXED / 01 INCR / 10 WRAP)
    // burst_cfg[7:0] : 최대 Burst 설정값 (AWLEN encoding)

    // ---- Datapath -> Controller (status) ----
    output wire b_hs,
    output wire xfer_done,

    // ---- 신규: region 에러도 Controller한테 보고 ----
    output reg  region_err,

    // ---- FIFO I/F (consumer) ----
    output wire                  fifo_rd_en,
    input  wire [DATA_WIDTH-1:0] fifo_rd_data,
    input  wire                  fifo_empty,

    // ---- AXI4 AW/W/B 채널 ----
    output reg  [            3:0] awid,
    output reg  [ ADDR_WIDTH-1:0] awaddr,
    output reg  [BURST_WIDTH-1:0] awlen,
    output wire [            2:0] awsize,
    output wire [            1:0] awburst,
    output reg                    awvalid,
    input  wire                   awready,

    output reg  [  DATA_WIDTH-1:0] wdata,
    output reg  [DATA_WIDTH/8-1:0] wstrb,
    output reg                     wlast,
    output reg                     wvalid,
    input  wire                    wready,

    input  wire [3:0] bid,
    input wire [1:0] bresp,
    input wire bvalid,
    output wire bready
);
    localparam MASTER_ID = 1'b1;  // ID[3] : DMA = 1
    localparam integer BYTES_PER_BEAT = DATA_WIDTH / 8;  // 4
    localparam integer ADDR_LSB = $clog2(BYTES_PER_BEAT);  // 2

    //-----------------------------------------------------------
    // DMA Configuration Latch
    //-----------------------------------------------------------
    reg [ADDR_WIDTH-1:0]  cfg_dst_addr;
    reg [LEN_WIDTH-1:0]   cfg_length;
    reg [BURST_WIDTH+1:0] cfg_burst_cfg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_dst_addr  <= {ADDR_WIDTH{1'b0}};
            cfg_length    <= {LEN_WIDTH{1'b0}};
            cfg_burst_cfg <= {(BURST_WIDTH+2){1'b0}};
        end
        else if (init) begin
            cfg_dst_addr  <= dst_addr;
            cfg_length    <= length;
            cfg_burst_cfg <= burst_cfg;
        end
    end

    //-----------------------------------------------------------
    // Burst Configuration Decode
    //-----------------------------------------------------------
    localparam [1:0] BURST_FIXED = 2'b00;
    localparam [1:0] BURST_INCR  = 2'b01;
    localparam [1:0] BURST_WRAP  = 2'b10;

    wire [1:0] burst_type = cfg_burst_cfg[BURST_WIDTH+1:BURST_WIDTH];

    wire [BURST_WIDTH-1:0] burst_len_cfg = cfg_burst_cfg[BURST_WIDTH-1:0];

    assign awsize  = 3'b010;
    assign awburst = burst_type;
    assign bready  = 1'b1;

    reg [ADDR_WIDTH-1:0] cur_addr;
    reg [LEN_WIDTH-1:0] req_byte_cnt;
    reg [LEN_WIDTH-1:0] total_byte_cnt;

    reg [1:0] num_busy;
    reg [1:0] aw_outstanding;

    reg [BURST_WIDTH-1:0] w_beat_cnt;
    reg [LEN_WIDTH-1:0] aw_payload_bytes;

    reg [LEN_WIDTH-1:0] burst_byte_cnt;
    reg [LEN_WIDTH-1:0] w_valid_bytes;

    reg [BURST_WIDTH-1:0] desc_awlen         [0:1];
    reg [LEN_WIDTH-1:0]   desc_payload_bytes [0:1];
    reg [ADDR_LSB-1:0]    desc_start_offset  [0:1];

    reg desc_wr_ptr;
    reg desc_rd_ptr;
    reg [1:0] desc_count;

    wire aw_hs = awvalid && awready;
    assign b_hs = bvalid && bready;
    wire w_hs = wvalid && wready;

    //======================================================================
    // ★ 신규 — 3슬레이브(ROM/DDR3L/RAM) 주소 디코더 (Read와 동일 패턴)
    //   2'b01 = ROM, 2'b10 = DDR3L, 2'b11 = RAM, 2'b00 = 매칭 안됨(에러)
    //======================================================================
    function [1:0] decode_slave;
        input [ADDR_WIDTH-1:0] addr;
        begin
            if      (addr >= ROM_BASE   && addr < ROM_BASE   + ROM_SIZE)   decode_slave = 2'b01;
            else if (addr >= DDR3L_BASE && addr < DDR3L_BASE + DDR3L_SIZE) decode_slave = 2'b10;
            else if (addr >= RAM_BASE   && addr < RAM_BASE   + RAM_SIZE)   decode_slave = 2'b11;
            else                                                           decode_slave = 2'b00;
        end
    endfunction

    // 지금 선택된 슬레이브 영역의 끝 주소 (region boundary 계산용)
    function [ADDR_WIDTH-1:0] region_end_addr;
        input [ADDR_WIDTH-1:0] addr;
        begin
            case (decode_slave(addr))
                2'b01:   region_end_addr = ROM_BASE   + ROM_SIZE;
                2'b10:   region_end_addr = DDR3L_BASE + DDR3L_SIZE;
                2'b11:   region_end_addr = RAM_BASE   + RAM_SIZE;
                default: region_end_addr = addr;
            endcase
        end
    endfunction

    //-----------------------------------------------------------
    // AW Back-to-Back Issue Context
    //-----------------------------------------------------------
    wire [ADDR_WIDTH-1:0] issue_addr =
        aw_hs
        ? (cur_addr + aw_payload_bytes[ADDR_WIDTH-1:0])
        : cur_addr;

    wire [LEN_WIDTH-1:0] issue_req_byte_cnt =
        aw_hs
        ? (req_byte_cnt + aw_payload_bytes)
        : req_byte_cnt;

    // ★ 변경: issue_addr[14] 단일비트 대신 decode_slave 함수로 3분기
    wire [1:0] issue_slave_sel = decode_slave(issue_addr);

    // ★ 신규: 목적지 주소가 어느 슬레이브에도 안 걸리는 경우
    wire issue_region_err = (issue_slave_sel == 2'b00);

    //-----------------------------------------------------------
    // Projected NUM / Outstanding State
    //-----------------------------------------------------------
    reg [1:0] projected_num_busy;

    always @(*) begin
        projected_num_busy = num_busy;
        if (b_hs)
            projected_num_busy[bid[0]] = 1'b0;
        if (aw_hs)
            projected_num_busy[awid[0]] = 1'b1;
    end

    wire projected_num_available =
        (projected_num_busy != 2'b11);

    wire issue_selected_num =
        (!projected_num_busy[0]) ? 1'b0 :
        (!projected_num_busy[1]) ? 1'b1 :
                                  1'b0;

    reg [1:0] projected_aw_outstanding;

    always @(*) begin
        projected_aw_outstanding = aw_outstanding;
        case ({aw_hs, b_hs})
            2'b10:
                projected_aw_outstanding =
                    aw_outstanding + 2'd1;
            2'b01:
                projected_aw_outstanding =
                    aw_outstanding - 2'd1;
            default:
                projected_aw_outstanding =
                    aw_outstanding;
        endcase
    end

    //-----------------------------------------------------------
    // FIFO Read Pipeline / Data Alignment Buffer
    //-----------------------------------------------------------
    localparam integer ALIGN_BUF_WORDS = 3;
    localparam integer ALIGN_BUF_WIDTH = DATA_WIDTH * ALIGN_BUF_WORDS;
    localparam integer ALIGN_BUF_BYTES = BYTES_PER_BEAT * ALIGN_BUF_WORDS;
    localparam integer ALIGN_CNT_W = $clog2(ALIGN_BUF_BYTES + 1);

    reg fifo_read_pending;
    reg [LEN_WIDTH-1:0] fifo_word_req_cnt;
    reg [ALIGN_BUF_WIDTH-1:0] align_buf;
    reg [ALIGN_CNT_W-1:0] align_byte_count;

    //-----------------------------------------------------------
    // INCR Burst Calculator
    //-----------------------------------------------------------
    wire [LEN_WIDTH-1:0] incr_remaining_bytes = (cfg_length > issue_req_byte_cnt) ? (cfg_length - issue_req_byte_cnt) : {LEN_WIDTH{1'b0}};

    wire [ADDR_LSB-1:0] incr_start_offset = issue_addr[ADDR_LSB-1:0];

    wire [12:0] incr_bytes_to_4k_raw = 13'd4096 - {1'b0, issue_addr[11:0]};

    wire [LEN_WIDTH-1:0] incr_bytes_to_4k = {{(LEN_WIDTH - 13) {1'b0}}, incr_bytes_to_4k_raw};

    // ★ 신규: 4KB 경계뿐 아니라, 목적지 슬레이브 영역 경계도 안 넘게 제한
    wire [LEN_WIDTH-1:0] incr_bytes_to_region =
        region_end_addr(issue_addr) - issue_addr;

    wire [BURST_WIDTH:0] incr_max_burst_beats = {1'b0, burst_len_cfg} + 1'b1;

    wire [LEN_WIDTH-1:0] incr_max_burst_span_bytes = incr_max_burst_beats * BYTES_PER_BEAT;

    wire [LEN_WIDTH-1:0] incr_max_burst_payload_bytes = incr_max_burst_span_bytes - incr_start_offset;

    // ★ 변경: remaining / 4KB / region경계 세 값 중 최소값
    wire [LEN_WIDTH-1:0] incr_burst_limit_1 = (incr_remaining_bytes <= incr_bytes_to_4k) ? incr_remaining_bytes : incr_bytes_to_4k;
    wire [LEN_WIDTH-1:0] incr_burst_limit_2 = (incr_burst_limit_1 <= incr_bytes_to_region) ? incr_burst_limit_1 : incr_bytes_to_region;

    wire [LEN_WIDTH-1:0] incr_actual_burst_bytes = (incr_burst_limit_2 <= incr_max_burst_payload_bytes) ? incr_burst_limit_2 : incr_max_burst_payload_bytes;

    wire [LEN_WIDTH:0] incr_beat_calc_value = {1'b0, incr_actual_burst_bytes} + incr_start_offset + (BYTES_PER_BEAT - 1);

    wire [LEN_WIDTH:0] incr_actual_burst_beats_calc = incr_beat_calc_value >> ADDR_LSB;

    wire [BURST_WIDTH:0] incr_actual_burst_beats = incr_actual_burst_beats_calc[BURST_WIDTH:0];

    wire [BURST_WIDTH:0] incr_actual_awlen_ext = incr_actual_burst_beats - 1'b1;

    wire [BURST_WIDTH-1:0] incr_actual_awlen = incr_actual_awlen_ext[BURST_WIDTH-1:0];

    //-----------------------------------------------------------
    // Burst Mode Selector
    //-----------------------------------------------------------
    reg [LEN_WIDTH-1:0]   actual_burst_bytes;
    reg [BURST_WIDTH-1:0] actual_awlen;
    reg                   burst_calc_valid;

    always @(*) begin
        actual_burst_bytes = {LEN_WIDTH{1'b0}};
        actual_awlen        = {BURST_WIDTH{1'b0}};
        burst_calc_valid    = 1'b0;

        case (burst_type)
            BURST_FIXED: begin
                // TODO: FIXED Burst Calculator 구현 예정
            end

            BURST_INCR: begin
                actual_burst_bytes = incr_actual_burst_bytes;
                actual_awlen        = incr_actual_awlen;
                burst_calc_valid    = 1'b1;
            end

            BURST_WRAP: begin
                // TODO: WRAP Burst Calculator 구현 예정
            end

            default: begin
                // 지원하지 않는 Burst Type
            end
        endcase
    end

    wire issue_req_pending = (issue_req_byte_cnt < cfg_length);

    //-----------------------------------------------------------
    // Descriptor Queue 상태
    //-----------------------------------------------------------
    wire desc_empty = (desc_count == 2'd0);
    wire desc_full  = (desc_count == 2'd2);
    wire desc_push = aw_hs;
    wire desc_pop  = w_hs && wlast && !desc_empty;

    reg [1:0] projected_desc_count;

    always @(*) begin
        projected_desc_count = desc_count;
        case ({desc_push, desc_pop})
            2'b10:
                projected_desc_count =
                    desc_count + 2'd1;
            2'b01:
                projected_desc_count =
                    desc_count - 2'd1;
            default:
                projected_desc_count =
                    desc_count;
        endcase
    end

    wire projected_desc_space =
        (projected_desc_count < 2'd2);

    wire [BURST_WIDTH-1:0] current_desc_awlen;
    wire [LEN_WIDTH-1:0]   current_desc_payload_bytes;
    wire [ADDR_LSB-1:0]    current_desc_start_offset;

    assign current_desc_awlen = desc_awlen[desc_rd_ptr];
    assign current_desc_payload_bytes = desc_payload_bytes[desc_rd_ptr];
    assign current_desc_start_offset = desc_start_offset[desc_rd_ptr];

    wire [LEN_WIDTH:0] fifo_total_words_calc = ({1'b0, cfg_length} + (BYTES_PER_BEAT - 1)) >> ADDR_LSB;
    wire [LEN_WIDTH-1:0] fifo_total_words = fifo_total_words_calc[LEN_WIDTH-1:0];
    wire fifo_words_remaining = (fifo_word_req_cnt < fifo_total_words);
    wire fifo_return = fifo_read_pending;

    wire w_slot_available = !wvalid || w_hs;
    wire move_to_next_desc = w_hs && wlast;
    wire next_desc_queued = (desc_count >= 2'd2);
    wire next_desc_bypass =
        (desc_count == 2'd1) && desc_push;

    wire load_desc_available =
        move_to_next_desc
        ? (next_desc_queued || next_desc_bypass)
        : !desc_empty;

    wire [BURST_WIDTH-1:0] load_desc_awlen =
        move_to_next_desc
        ? (next_desc_queued ? desc_awlen[~desc_rd_ptr] : awlen)
        : current_desc_awlen;

    wire [LEN_WIDTH-1:0] load_desc_payload_bytes =
        move_to_next_desc
        ? (next_desc_queued ? desc_payload_bytes[~desc_rd_ptr]
                            : aw_payload_bytes)
        : current_desc_payload_bytes;

    wire [ADDR_LSB-1:0] load_desc_start_offset =
        move_to_next_desc
        ? (next_desc_queued ? desc_start_offset[~desc_rd_ptr]
                            : awaddr[ADDR_LSB-1:0])
        : current_desc_start_offset;

    wire [BURST_WIDTH-1:0] load_w_beat_cnt =
        move_to_next_desc
        ? {BURST_WIDTH{1'b0}}
        : (w_hs ? (w_beat_cnt + 1'b1) : w_beat_cnt);

    wire [LEN_WIDTH-1:0] load_burst_byte_cnt =
        move_to_next_desc
        ? {LEN_WIDTH{1'b0}}
        : (w_hs ? (burst_byte_cnt + w_valid_bytes)
                : burst_byte_cnt);

    wire load_first_beat =
        (load_w_beat_cnt == {BURST_WIDTH{1'b0}});

    wire load_last_beat =
        (load_w_beat_cnt == load_desc_awlen);

    wire [LEN_WIDTH-1:0] load_start_offset_ext =
        {{(LEN_WIDTH - ADDR_LSB) {1'b0}}, load_desc_start_offset};

    wire [LEN_WIDTH-1:0] load_beat_start_lane =
        load_first_beat
        ? load_start_offset_ext
        : {LEN_WIDTH{1'b0}};

    wire [LEN_WIDTH-1:0] load_beat_capacity_bytes =
        BYTES_PER_BEAT - load_beat_start_lane;

    wire [LEN_WIDTH-1:0] load_burst_bytes_remaining =
        (load_desc_payload_bytes > load_burst_byte_cnt)
        ? (load_desc_payload_bytes - load_burst_byte_cnt)
        : {LEN_WIDTH{1'b0}};

    wire [LEN_WIDTH-1:0] load_valid_bytes =
        (load_burst_bytes_remaining <= load_beat_capacity_bytes)
        ? load_burst_bytes_remaining
        : load_beat_capacity_bytes;

    wire [ALIGN_CNT_W-1:0] align_consume_bytes =
        w_hs
        ? w_valid_bytes[ALIGN_CNT_W-1:0]
        : {ALIGN_CNT_W{1'b0}};

    wire [ALIGN_BUF_WIDTH-1:0] align_buf_after_consume =
        w_hs
        ? (align_buf >> (w_valid_bytes * 8))
        : align_buf;

    wire [ALIGN_CNT_W-1:0] align_count_after_consume =
        align_byte_count - align_consume_bytes;

    wire [ALIGN_BUF_WIDTH-1:0] fifo_return_data_ext =
        {{(ALIGN_BUF_WIDTH - DATA_WIDTH){1'b0}}, fifo_rd_data};

    wire [ALIGN_BUF_WIDTH-1:0] align_buf_after_update =
        fifo_return
        ? (align_buf_after_consume |
           (fifo_return_data_ext << (align_count_after_consume * 8)))
        : align_buf_after_consume;

    wire [ALIGN_CNT_W-1:0] align_count_after_update =
        align_count_after_consume +
        (fifo_return ? BYTES_PER_BEAT : 0);

    wire fifo_credit_available =
        (align_count_after_update <=
         (ALIGN_BUF_BYTES - BYTES_PER_BEAT));

    wire fifo_read_req =
        en &&
        !init &&
        burst_calc_valid &&
        !fifo_empty &&
        fifo_words_remaining &&
        fifo_credit_available;

    assign fifo_rd_en = fifo_read_req;

    wire [DATA_WIDTH-1:0] load_payload_word =
        align_buf_after_update[DATA_WIDTH-1:0];

    wire [DATA_WIDTH-1:0] load_aligned_wdata =
        load_payload_word << (load_beat_start_lane * 8);

    wire [LEN_WIDTH-1:0] align_count_after_update_ext =
        {{(LEN_WIDTH - ALIGN_CNT_W){1'b0}}, align_count_after_update};

    wire load_data_ready =
        (load_valid_bytes != {LEN_WIDTH{1'b0}}) &&
        (align_count_after_update_ext >= load_valid_bytes);

    wire w_load_req =
        en &&
        w_slot_available &&
        load_desc_available &&
        load_data_ready;

    //-----------------------------------------------------------
    // AW Register Load Control
    //-----------------------------------------------------------
    wire aw_slot_available = !awvalid || aw_hs;

    // ★ 변경: issue_region_err(목적지 주소가 어느 슬레이브에도 안 걸림)일 때 AW 발행 자체를 막음
    wire aw_load_req =
        en &&
        !init &&
        aw_slot_available &&
        issue_req_pending &&
        (projected_aw_outstanding < 2'd2) &&
        projected_num_available &&
        projected_desc_space &&
        burst_calc_valid &&
        !issue_region_err;

    //-----------------------------------------------------------
    // DMA Write Transfer 완료 판단
    //-----------------------------------------------------------
    wire payload_done = (total_byte_cnt >= cfg_length);
    wire last_b_hs = b_hs && (aw_outstanding == 2'd1);
    assign xfer_done = payload_done && last_b_hs;

    //-----------------------------------------------------------
    // ★ 신규 — region_err 래치 (Controller한테 에러로 보고)
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            region_err <= 1'b0;
        end
        else if (init) begin
            region_err <= 1'b0;
        end
        else if (issue_region_err && !region_err) begin
            region_err <= 1'b1;
        end
    end

    //-----------------------------------------------------------
    // cur_addr / req_byte_cnt / NUM / aw_outstanding
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            num_busy       <= 2'b00;
            aw_outstanding <= 2'd0;
        end else if (init) begin
            cur_addr       <= dst_addr;
            req_byte_cnt   <= {LEN_WIDTH{1'b0}};
            num_busy       <= 2'b00;
            aw_outstanding <= 2'd0;
        end else begin
            if (aw_hs) begin
                cur_addr     <= cur_addr + aw_payload_bytes[ADDR_WIDTH-1:0];
                req_byte_cnt <= req_byte_cnt + aw_payload_bytes;
            end
            if (b_hs) num_busy[bid[0]] <= 1'b0;
            if (aw_hs) num_busy[awid[0]] <= 1'b1;
            case ({
                aw_hs, b_hs
            })
                2'b10:   aw_outstanding <= aw_outstanding + 2'd1;
                2'b01:   aw_outstanding <= aw_outstanding - 2'd1;
                default: aw_outstanding <= aw_outstanding;
            endcase
        end
    end

    //-----------------------------------------------------------
    // Burst Descriptor Queue
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            desc_wr_ptr <= 1'b0;
            desc_rd_ptr <= 1'b0;
            desc_count  <= 2'd0;

            desc_awlen[0]         <= {BURST_WIDTH{1'b0}};
            desc_awlen[1]         <= {BURST_WIDTH{1'b0}};

            desc_payload_bytes[0] <= {LEN_WIDTH{1'b0}};
            desc_payload_bytes[1] <= {LEN_WIDTH{1'b0}};

            desc_start_offset[0]  <= {ADDR_LSB{1'b0}};
            desc_start_offset[1]  <= {ADDR_LSB{1'b0}};
        end
        else if (init) begin
            desc_wr_ptr <= 1'b0;
            desc_rd_ptr <= 1'b0;
            desc_count  <= 2'd0;

            desc_awlen[0]         <= {BURST_WIDTH{1'b0}};
            desc_awlen[1]         <= {BURST_WIDTH{1'b0}};

            desc_payload_bytes[0] <= {LEN_WIDTH{1'b0}};
            desc_payload_bytes[1] <= {LEN_WIDTH{1'b0}};

            desc_start_offset[0]  <= {ADDR_LSB{1'b0}};
            desc_start_offset[1]  <= {ADDR_LSB{1'b0}};
        end
        else begin
            if (desc_push) begin
                desc_awlen[desc_wr_ptr] <= awlen;
                desc_payload_bytes[desc_wr_ptr]
                    <= aw_payload_bytes;
                desc_start_offset[desc_wr_ptr]
                    <= awaddr[ADDR_LSB-1:0];
                desc_wr_ptr <= ~desc_wr_ptr;
            end

            if (desc_pop) begin
                desc_rd_ptr <= ~desc_rd_ptr;
            end

            case ({desc_push, desc_pop})
                2'b10:
                    desc_count <= desc_count + 2'd1;
                2'b01:
                    desc_count <= desc_count - 2'd1;
                default:
                    desc_count <= desc_count;
            endcase
        end
    end

    //-----------------------------------------------------------
    // Synchronous FIFO Read Pipeline
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_read_pending <= 1'b0;
            fifo_word_req_cnt <= {LEN_WIDTH{1'b0}};
        end
        else if (init) begin
            fifo_read_pending <= 1'b0;
            fifo_word_req_cnt <= {LEN_WIDTH{1'b0}};
        end
        else begin
            fifo_read_pending <= fifo_read_req;
            if (fifo_read_req) begin
                fifo_word_req_cnt <= fifo_word_req_cnt + 1'b1;
            end
        end
    end

    //-----------------------------------------------------------
    // Data Alignment Buffer
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            align_buf        <= {ALIGN_BUF_WIDTH{1'b0}};
            align_byte_count <= {ALIGN_CNT_W{1'b0}};
        end
        else if (init) begin
            align_buf        <= {ALIGN_BUF_WIDTH{1'b0}};
            align_byte_count <= {ALIGN_CNT_W{1'b0}};
        end
        else begin
            align_buf        <= align_buf_after_update;
            align_byte_count <= align_count_after_update;
        end
    end

    //-----------------------------------------------------------
    // Next WSTRB Generator
    //-----------------------------------------------------------
    integer strb_idx;
    reg [DATA_WIDTH/8-1:0] load_wstrb;

    always @(*) begin
        load_wstrb = {(DATA_WIDTH / 8) {1'b0}};
        for (
            strb_idx = 0;
            strb_idx < BYTES_PER_BEAT;
            strb_idx = strb_idx + 1
        ) begin
            if ((strb_idx >= load_beat_start_lane) &&
                (strb_idx <
                 (load_beat_start_lane + load_valid_bytes))) begin
                load_wstrb[strb_idx] = 1'b1;
            end
        end
    end

    //-----------------------------------------------------------
    // W Beat / Payload Byte Counter
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            burst_byte_cnt <= {LEN_WIDTH{1'b0}};
            w_beat_cnt     <= {BURST_WIDTH{1'b0}};
        end
        else if (init) begin
            total_byte_cnt <= {LEN_WIDTH{1'b0}};
            burst_byte_cnt <= {LEN_WIDTH{1'b0}};
            w_beat_cnt     <= {BURST_WIDTH{1'b0}};
        end
        else if (w_hs) begin
            total_byte_cnt <=
                total_byte_cnt + w_valid_bytes;

            if (wlast) begin
                burst_byte_cnt <= {LEN_WIDTH{1'b0}};
                w_beat_cnt     <= {BURST_WIDTH{1'b0}};
            end
            else begin
                burst_byte_cnt <=
                    burst_byte_cnt + w_valid_bytes;
                w_beat_cnt <=
                    w_beat_cnt + 1'b1;
            end
        end
    end

    //-----------------------------------------------------------
    // AW 채널 구동
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            awvalid          <= 1'b0;
            awaddr           <= {ADDR_WIDTH{1'b0}};
            awlen            <= {BURST_WIDTH{1'b0}};
            awid             <= 4'd0;
            aw_payload_bytes <= {LEN_WIDTH{1'b0}};
        end else if (init) begin
            awvalid          <= 1'b0;
            awaddr           <= {ADDR_WIDTH{1'b0}};
            awlen            <= {BURST_WIDTH{1'b0}};
            awid             <= 4'd0;
            aw_payload_bytes <= {LEN_WIDTH{1'b0}};
        end else begin
            if (aw_load_req) begin
                awaddr <= issue_addr;
                awlen <= actual_awlen;
                // ★ 변경: issue_slave_sel이 이제 3값(01/10/11) 표현 가능
                awid <= {MASTER_ID, issue_slave_sel, issue_selected_num};
                aw_payload_bytes <= actual_burst_bytes;
                awvalid <= 1'b1;
            end
            else if (aw_hs) begin
                awvalid <= 1'b0;
            end
        end
    end

    //-----------------------------------------------------------
    // W Channel
    //-----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wvalid        <= 1'b0;
            wdata         <= {DATA_WIDTH{1'b0}};
            wstrb         <= {(DATA_WIDTH / 8) {1'b0}};
            wlast         <= 1'b0;
            w_valid_bytes <= {LEN_WIDTH{1'b0}};
        end
        else if (init) begin
            wvalid        <= 1'b0;
            wdata         <= {DATA_WIDTH{1'b0}};
            wstrb         <= {(DATA_WIDTH / 8) {1'b0}};
            wlast         <= 1'b0;
            w_valid_bytes <= {LEN_WIDTH{1'b0}};
        end
        else begin
            if (w_load_req) begin
                wdata <= load_aligned_wdata;
                wstrb <= load_wstrb;
                wlast <= load_last_beat;
                w_valid_bytes <= load_valid_bytes;
                wvalid <= 1'b1;
            end
            else if (w_hs) begin
                wvalid        <= 1'b0;
                wlast         <= 1'b0;
                wstrb         <= {(DATA_WIDTH / 8) {1'b0}};
                w_valid_bytes <= {LEN_WIDTH{1'b0}};
            end
        end
    end

endmodule