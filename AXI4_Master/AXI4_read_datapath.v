`timescale 1ns / 1ps

module AXI4_read_datapath #(
    parameter ADDR_WIDTH   = 15,
    parameter DATA_WIDTH   = 32,
    parameter LEN_WIDTH    = 32,
    parameter BURST_WIDTH  = 8,

    // 예시값 — 실제 인터페이스 합의 기준 값으로 교체할 것
    parameter [ADDR_WIDTH-1:0] ROM_BASE    = 15'h0000,
    parameter [ADDR_WIDTH-1:0] ROM_SIZE    = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_BASE  = 15'h0400,
    parameter [ADDR_WIDTH-1:0] DDR3L_SIZE  = 15'h4000,
    parameter [ADDR_WIDTH-1:0] RAM_BASE    = 15'h4400,
    parameter [ADDR_WIDTH-1:0] RAM_SIZE    = 15'h0400,

    // Burst size cap = 16B = 4 beats, matching a MicroBlaze cache line
    // (C_DCACHE_LINE_LEN = 4 words). Keeping the DMA burst equal to the
    // line size means a later cache-fill path needs no burst re-sizing.
    parameter MAX_BURST_BYTES = 16
)(
    input                            clk,
    input                            rst_n,

    // Control Unit -> Datapath
    input                            en,          // state == S_DATA
    input                            init,        // IDLE -> DATA transition = 1 pulse
    input                            abort,       // Force Shutdown

    // Register Map
    input      [ADDR_WIDTH-1:0]      src_addr,
    input      [LEN_WIDTH-1:0]       length,      // total transfer byte
    input      [BURST_WIDTH+1:0]     burst_cfg,   // AXI ARLEN Maximum value (in Register Setting value) [7:0] & burst type [9:8]

    // Datapath -> Control Unit
    output                           r_hs,
    output                           xfer_done,   // transfer done or abort assertion
    output     [ADDR_WIDTH-1:0]      err_addr,    // address of the beat that failed
    output reg                       err_valid,   // RRESP error latched
    output reg                       cfg_err,     // illegal / unserviceable configuration

    // FIFO : 32-bit, data only.
    output reg                       fifo_wr_en,
    output reg [DATA_WIDTH-1:0]      fifo_wr_data,
    input                            fifo_full,

    // AXI4 AR / R channel
    output reg [3:0]                 arid,
    output reg [ADDR_WIDTH-1:0]      araddr,
    output reg [BURST_WIDTH-1:0]     arlen,
    output     [2:0]                 arsize,
    output     [1:0]                 arburst,
    output reg                       arvalid,
    input                            arready,

    input      [DATA_WIDTH-1:0]      rdata,
    input                            rvalid,
    input                            rlast,
    input      [3:0]                 rid,
    input      [1:0]                 rresp,
    output                           rready
);

    localparam [0:0] MASTER_ID      = 1'b1;                     // ID[3] : DMA = 1
    localparam BYTES_PER_BEAT       = DATA_WIDTH/8;             // 1 beat (byte)
    localparam ADDR_LSB             = $clog2(BYTES_PER_BEAT);   // byte offset
    localparam PAGE_BYTES           = 4096;                     // AXI4-Full Maximum burst size = 4KB boundary
    localparam PAGE_LSB             = $clog2(PAGE_BYTES);       // 12
    localparam MAX_BURST_BEATS      = MAX_BURST_BYTES / BYTES_PER_BEAT;  // 16/4 = 4 beats
    localparam [2:0] ARSIZE_VAL     = ADDR_LSB;

    assign arsize = ARSIZE_VAL;

    wire [1:0]            burst_type_cfg;
    wire [BURST_WIDTH-1:0] burst_len_cfg;

    assign burst_type_cfg = burst_cfg[BURST_WIDTH+1:BURST_WIDTH];
    assign burst_len_cfg  = burst_cfg[BURST_WIDTH-1:0];
    assign arburst        = burst_type_cfg;

    //======================================================================
    //   Alignment policy : SRC_ADDR and LENGTH must be 4-byte aligned
    //======================================================================
    wire align_err_c;
    assign align_err_c = (src_addr[ADDR_LSB-1:0] != {ADDR_LSB{1'b0}}) ||
                         (length[ADDR_LSB-1:0]   != {ADDR_LSB{1'b0}});

    wire [LEN_WIDTH-1:0] total_beats_c;
    assign total_beats_c = length >> ADDR_LSB;   // exact : LENGTH is a multiple of 4

    reg                 align_err_q;
    reg [LEN_WIDTH-1:0] total_beats_q;           // latched at init


    // Transfer State
    reg [ADDR_WIDTH-1:0] cur_addr;        // Next AR address (always beat-aligned)
    reg [LEN_WIDTH-1:0]  req_beat_cnt;    // requested beats

    reg [1:0]             num_busy;
    reg [ADDR_WIDTH-1:0]  slot_addr [0:1];  // base address of the burst in each slot
    reg [BURST_WIDTH-1:0] slot_beat [0:1];  // beats already received in each slot

    wire next_num;
    assign next_num = num_busy[0] ? 1'b1 : 1'b0;

    wire ar_hs;
    assign ar_hs = arvalid && arready;    // AR channel handshake

    assign r_hs  = rvalid && rready;      // R channel handshake (raw, for Control Unit)
    assign rready = !fifo_full;

    wire r_mine, r_beat, r_burst_end;
    assign r_mine      = (rid[3] == MASTER_ID); // RID[3] == 1, DMA response
    assign r_beat      = r_hs && r_mine;        // Valid Data Beat
    assign r_burst_end = r_beat && rlast;       // -> Beat's Burst Last

    //======================================================================
    //   신규 — 3슬레이브(ROM/DDR3L/RAM) 주소 디코더
    //   2'b01 = ROM, 2'b10 = DDR3L, 2'b11 = RAM, 2'b00 = 매칭 안됨(에러)
    //   Interconnect 쪽 decode_slave 함수와 동일한 로직/동일한 코드값
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

    // 신규 — 지금 선택된 슬레이브 영역의 "끝 주소"(exclusive)를 구함
    function [ADDR_WIDTH-1:0] region_end_addr;
        input [ADDR_WIDTH-1:0] addr;
        begin
            case (decode_slave(addr))
                2'b01:   region_end_addr = ROM_BASE   + ROM_SIZE;
                2'b10:   region_end_addr = DDR3L_BASE + DDR3L_SIZE;
                2'b11:   region_end_addr = RAM_BASE   + RAM_SIZE;
                default: region_end_addr = addr; // 매칭 안됨 -> 이후 region_err로 걸러짐
            endcase
        end
    endfunction

    wire [1:0] slave_sel;
    assign slave_sel = decode_slave(cur_addr);      // 기존: cur_addr[REGION_LSB] ? ... 대체됨

    // 주소가 어느 슬레이브에도 안 걸리는 경우(설계 오류/잘못된 SA·DA)
    wire region_err_c;
    assign region_err_c = (slave_sel == 2'b00);

    wire req_pending;
    assign req_pending = (req_beat_cnt < total_beats_q);          // Remain request

    reg [1:0] outstanding_cnt;
    reg [1:0] active_slave;

    reg       ar_num_q;
    reg [1:0] ar_slave_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            outstanding_cnt <= 2'd0;
        end
        else if (init) begin
            outstanding_cnt <= 2'd0;
        end
        else begin
            case ({ar_hs, r_burst_end})
                2'b10:   outstanding_cnt <= outstanding_cnt + 2'd1;
                2'b01:   outstanding_cnt <= outstanding_cnt - 2'd1;
                default: outstanding_cnt <= outstanding_cnt;
            endcase
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            active_slave <= 2'd0;
        end
        else if (init) begin
            active_slave <= 2'd0;
        end
        else if (ar_hs && (outstanding_cnt == 2'd0)) begin
            active_slave <= ar_slave_q;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            num_busy <= 2'b00;
        end else if (init) begin
            num_busy <= 2'b00;
        end else begin
            if (ar_hs)       num_busy[ar_num_q] <= 1'b1;
            if (r_burst_end) num_busy[rid[0]]   <= 1'b0;
        end
    end

    reg abort_lat;
    always @(posedge clk) begin
        if (!rst_n) begin
            abort_lat <= 1'b0;
        end else if (init) begin
            abort_lat <= 1'b0;
        end else if (abort) begin
            abort_lat <= 1'b1;
        end
    end

    wire same_slave_ok, outstanding_ok;
    assign same_slave_ok = (outstanding_cnt == 2'd0) || (slave_sel == active_slave);
    assign outstanding_ok = (outstanding_cnt < 2'd2);

    //----------------------------------------------------------
    // (1) desired_beats      : burst_cfg, capped at MAX_BURST_BEATS
    // (2) beats_to_boundary  : cur_addr ~ 4KB boundary addr
    // (3) beats_to_region    : cur_addr ~ (선택된 슬레이브 영역의 끝 주소)
    // (4) remain_beats       : total_beats - req_beat_cnt
    //----------------------------------------------------------
    wire [LEN_WIDTH-1:0] remain_beats;
    wire [LEN_WIDTH-1:0] bytes_to_boundary;
    wire [LEN_WIDTH-1:0] beats_to_boundary;
    wire [LEN_WIDTH-1:0] bytes_to_region;
    wire [LEN_WIDTH-1:0] beats_to_region;
    wire [LEN_WIDTH-1:0] desired_raw;
    wire [LEN_WIDTH-1:0] desired_beats;
    wire [LEN_WIDTH-1:0] limit_beats;

    wire [LEN_WIDTH-1:0] safe_beats_incr;
    wire [LEN_WIDTH-1:0] safe_beats_fixed;
    wire [LEN_WIDTH-1:0] safe_beats_wrap;
    wire [LEN_WIDTH-1:0] safe_beats;
    wire [LEN_WIDTH-1:0] safe_bytes;
    wire [BURST_WIDTH-1:0] safe_arlen;

    assign remain_beats      = total_beats_q - req_beat_cnt;

    assign bytes_to_boundary = PAGE_BYTES -
                               {{(LEN_WIDTH-PAGE_LSB){1'b0}}, cur_addr[PAGE_LSB-1:0]};
    assign beats_to_boundary = bytes_to_boundary >> ADDR_LSB;

    // REGION_BYTES 균등분할 대신, region_end_addr()로 구한 실제 끝주소 - 현재주소
    assign bytes_to_region   = region_end_addr(cur_addr) - cur_addr;
    assign beats_to_region   = bytes_to_region >> ADDR_LSB;

    assign desired_raw       = {{(LEN_WIDTH-BURST_WIDTH){1'b0}}, burst_len_cfg} + 1'b1;

    assign desired_beats     = (desired_raw > MAX_BURST_BEATS) ? MAX_BURST_BEATS
                                                               : desired_raw;

    // min(4KB boundary, slave region boundary)
    assign limit_beats = (beats_to_boundary < beats_to_region) ? beats_to_boundary
                                                              : beats_to_region;

    assign safe_beats_fixed =
        (remain_beats < desired_beats) ?
            ((remain_beats  < 16) ? remain_beats  : 16) :
            ((desired_beats < 16) ? desired_beats : 16);

    assign safe_beats_incr =
        (desired_beats < limit_beats) ?
            ((desired_beats < remain_beats) ? desired_beats : remain_beats) :
            ((limit_beats   < remain_beats) ? limit_beats   : remain_beats);

    assign safe_beats_wrap =
        ((desired_beats >= 16) && (remain_beats >= 16)) ? 16 :
        ((desired_beats >=  8) && (remain_beats >=  8)) ?  8 :
        ((desired_beats >=  4) && (remain_beats >=  4)) ?  4 :
        ((desired_beats >=  2) && (remain_beats >=  2)) ?  2 : 0;

    assign safe_beats =
        (burst_type_cfg == 2'b00) ? safe_beats_fixed :
        (burst_type_cfg == 2'b01) ? safe_beats_incr  :
        (burst_type_cfg == 2'b10) ? safe_beats_wrap  : 0;

    assign safe_bytes = safe_beats << ADDR_LSB;
    assign safe_arlen = safe_beats[BURST_WIDTH-1:0] - 1'b1;

    // region_err_c(주소가 어느 슬레이브에도 안 걸림) 조건 추가
    wire ar_can_issue;
    assign ar_can_issue = en && !init && !arvalid && req_pending && !abort_lat
                          && outstanding_ok && same_slave_ok
                          && (safe_beats != {LEN_WIDTH{1'b0}})
                          && !cfg_err
                          && !region_err_c;

    wire no_progress;
    assign no_progress = en && !init && req_pending && !abort_lat
                         && (safe_beats == {LEN_WIDTH{1'b0}});

    always @(posedge clk) begin
        if (!rst_n) begin
            cfg_err <= 1'b0;
        end
        else if (init) begin
            cfg_err <= align_err_c;                 // 주소가 애초에 어느 영역에도 없으면 시작부터 에러
        end
        // region_err_c도 cfg_err을 세우는 조건에 추가
        else if (no_progress || align_err_q || region_err_c || (burst_type_cfg == 2'b11)) begin
            cfg_err <= 1'b1;
        end
    end

    wire ar_done;
    assign ar_done = !req_pending || abort_lat || cfg_err;

    assign xfer_done = !init && ar_done && (outstanding_cnt == 2'd0);

    always @(*) begin
        fifo_wr_en   = r_beat;
        fifo_wr_data = rdata;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= {LEN_WIDTH{1'b0}};
            align_err_q    <= 1'b0;
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
            slot_beat[0]   <= {BURST_WIDTH{1'b0}};
            slot_beat[1]   <= {BURST_WIDTH{1'b0}};
        end
        else if (init) begin
            cur_addr       <= src_addr;
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= total_beats_c;
            align_err_q    <= align_err_c;
            slot_addr[0]   <= {ADDR_WIDTH{1'b0}};
            slot_addr[1]   <= {ADDR_WIDTH{1'b0}};
            slot_beat[0]   <= {BURST_WIDTH{1'b0}};
            slot_beat[1]   <= {BURST_WIDTH{1'b0}};
        end
        else begin
            if (ar_hs)      slot_beat[ar_num_q] <= {BURST_WIDTH{1'b0}};
            if (r_beat)     slot_beat[rid[0]]   <= slot_beat[rid[0]] + 1'b1;

            if (ar_hs) begin
                case (burst_type_cfg)
                    2'b00: begin
                        cur_addr <= cur_addr;
                    end
                    2'b01: begin
                        cur_addr <= cur_addr + safe_bytes[ADDR_WIDTH-1:0];
                    end
                    2'b10: begin
                        // WRAP 주소 계산 미구현 (기존과 동일)
                    end
                    default: begin
                        cur_addr <= cur_addr;
                    end
                endcase
                req_beat_cnt          <= req_beat_cnt + safe_beats;
                slot_addr[ar_num_q]   <= araddr;
            end
        end
    end

    //======================================================================
    // AR channel
    //======================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            arvalid    <= 1'b0;
            araddr     <= {ADDR_WIDTH{1'b0}};
            arlen      <= {BURST_WIDTH{1'b0}};
            arid       <= 4'd0;
            ar_num_q   <= 1'b0;
            ar_slave_q <= 2'd0;
        end
        else if (ar_hs) begin
            arvalid <= 1'b0;
        end
        else if (ar_can_issue) begin
            arvalid    <= 1'b1;
            araddr     <= cur_addr;
            arlen      <= safe_arlen;
            arid       <= {MASTER_ID, slave_sel, next_num}; // ★ ID 상위 2bit이던 slave_sel이 이제 3값까지 표현 가능
            ar_num_q   <= next_num;
            ar_slave_q <= slave_sel;
        end
    end

    wire beat_err;
    assign beat_err = r_beat && rresp[1];

    wire [ADDR_WIDTH-1:0] beat_addr;
    assign beat_addr = (burst_type_cfg == 2'b00)
                     ? slot_addr[rid[0]]
                     : slot_addr[rid[0]] +
                       ({{(ADDR_WIDTH-BURST_WIDTH){1'b0}}, slot_beat[rid[0]]} << ADDR_LSB);

    reg [ADDR_WIDTH-1:0] err_addr_q;
    assign err_addr = err_addr_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
        end
        else if (init) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
        end
        else if (beat_err && !err_valid) begin
            err_valid  <= 1'b1;
            err_addr_q <= beat_addr;
        end
    end

endmodule