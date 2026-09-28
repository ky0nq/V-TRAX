`timescale 1ns / 1ps

// ============================================================================
//  AXI4_read_datapath
// ============================================================================
// "주소 SA 부터 BTT 바이트만큼 읽어와" 라는 일을 받아서,
// AXI 규칙에 맞게 여러 개의 burst로 쪼개서 요청하고, 돌아온 데이터를 FIFO에 넣음
//
// R0 : DDR (HP0)       카메라 프레임 버퍼
// R1 : BRAM 프레임 창   로딩화면 (ram_bridge가 만들어 주는 가상 프레임)
// ============================================================================
module AXI4_read_datapath #(
    parameter ADDR_WIDTH   = 32,        // 주소 폭
    parameter DATA_WIDTH   = 32,        // 데이터 폭 (1 beat)
    parameter LEN_WIDTH    = 32,        // 전송 길이(BTT) 폭
    parameter BURST_WIDTH  = 8,         // ARLEN 폭

    parameter [ADDR_WIDTH-1:0] R0_BASE = 32'h0000_0000,   // 영역0 시작 : DDR
    parameter [ADDR_WIDTH-1:0] R0_SIZE = 32'h4000_0000,   // 영역0 크기 : 1GB
    parameter [ADDR_WIDTH-1:0] R1_BASE = 32'h8000_0000,   // 영역1 시작 : BRAM 프레임 창
    parameter [ADDR_WIDTH-1:0] R1_SIZE = 32'h002A_3000,   // 영역1 크기 : 1280x720x3

    parameter MAX_BURST_BYTES = 64                        // burst 상한 64B = 16 beat
)(
    input                            clk,
    input                            rst_n,

    // ------------------------------------------------------------------
    // Controller가 주는 지시
    // ------------------------------------------------------------------
    input                            en,    // 1 = 일해도 됨 (controller가 S_DATA 일 때)
    input                            init,  // 1클럭 : 프레임 시작
    input                            abort, // 1클럭 : 새 요청 그만 

    // ------------------------------------------------------------------
    // 설정값 (레지스터 맵 / frame select 에서 옴)
    // ------------------------------------------------------------------
    input      [ADDR_WIDTH-1:0]      src_addr,    // 읽기 시작 주소
    input      [LEN_WIDTH-1:0]       length,      // 읽을 총 바이트 수
    input      [BURST_WIDTH+1:0]     burst_cfg,   // [7:0] 원하는 ARLEN, [9:8] burst 타입

    // ------------------------------------------------------------------
    // Controller에게 보고
    // ------------------------------------------------------------------
    output                           r_hs,        // R 채널 handshake
    output                           xfer_done,   // 이번 프레임 끝났음 신호
    output     [ADDR_WIDTH-1:0]      err_addr,    // 첫 에러 beat의 주소
    output reg                       err_valid,   // 에러 응답 받았음
    output reg                       cfg_err,     // 설정이 잘못돼서 진행 불가

    // ------------------------------------------------------------------
    // FIFO 로 데이터 넘기기
    // ------------------------------------------------------------------
    output reg                       fifo_wr_en,
    output reg [DATA_WIDTH-1:0]      fifo_wr_data,
    input                            fifo_full,

    // ------------------------------------------------------------------
    // AXI4 AR 채널 (주문서)
    // ------------------------------------------------------------------
    output reg [4:0]                 arid,        // {DMA표시 1bit, 예약 2bit, 슬롯번호 2bit}
    output reg [ADDR_WIDTH-1:0]      araddr,      // 이번 burst 시작 주소
    output reg [BURST_WIDTH-1:0]     arlen,       // beat 수 - 1
    output     [2:0]                 arsize,      // beat 크기 (4바이트 고정)
    output     [1:0]                 arburst,     // burst 타입
    output reg                       arvalid,     // "주문서 있어요"
    input                            arready,     // "주문서 받았어요"

    // ------------------------------------------------------------------
    // AXI4 R 채널 (물건)
    // ------------------------------------------------------------------
    input      [DATA_WIDTH-1:0]      rdata,       // 데이터
    input                            rvalid,      // "물건 왔어요"
    input                            rlast,       // "이게 이번 burst 마지막 beat"
    input      [4:0]                 rid,         // 어느 주문(ARID) 건지
    input      [1:0]                 rresp,       // 00 OK, 10 SLVERR, 11 DECERR
    output                           rready       // "받을 수 있어요"
);

    // ########################################################################
    //  0. 상수
    // ########################################################################
    localparam [0:0] MASTER_ID      = 1'b1;                     // ARID[4] : "이건 DMA 주문"
    localparam BYTES_PER_BEAT       = DATA_WIDTH/8;             // 4
    localparam ADDR_LSB             = $clog2(BYTES_PER_BEAT);   // 2 : 바이트 <-> beat 변환
    localparam PAGE_BYTES           = 4096;                     // AXI 규칙 : burst 는 4KB 경계를 못 넘음
    localparam PAGE_LSB             = $clog2(PAGE_BYTES);       // 12
    localparam MAX_BURST_BEATS      = MAX_BURST_BYTES / BYTES_PER_BEAT;  // 16
    localparam [2:0] ARSIZE_VAL     = ADDR_LSB;                 // 3'b010 = 4바이트/beat

    localparam MAX_OUTSTANDING   = 3;                            // 동시에 진행 가능한 주문 수
    localparam OUTSTANDING_CNT_W = $clog2(MAX_OUTSTANDING + 1);  // 0~3 셀 비트 수 = 2
    localparam SLOT_IDX_W        = (MAX_OUTSTANDING > 1) ? $clog2(MAX_OUTSTANDING) : 1;  // 슬롯번호 비트 = 2

    localparam [1:0] BURST_FIXED = 2'b00;                        // 주소 고정 (같은 주소 반복)
    localparam [1:0] BURST_INCR  = 2'b01;                        // 주소 증가 (보통 이거)

    localparam [ADDR_WIDTH-1:0] R0_END = R0_BASE + R0_SIZE;      // 영역0 끝 (이 주소는 포함 안 됨)
    localparam [ADDR_WIDTH-1:0] R1_END = R1_BASE + R1_SIZE;      // 영역1 끝 (이 주소는 포함 안 됨)

    assign arsize = ARSIZE_VAL;                                  // 항상 4바이트/beat

    // ########################################################################
    //  1. init 때 찍어두는 설정값들
    //     전송 도중에 CPU가 레지스터를 바꿔도 이번 프레임엔 영향 없게 하려고
    //     "시작 순간의 값"을 따로 저장해서 씀 (실제 저장은 9번 블럭에서)
    // ########################################################################
    reg  [BURST_WIDTH+1:0] burst_cfg_q;     // 찍어둔 BURST_CFG
    reg  [LEN_WIDTH-1:0]   total_beats_q;   // 찍어둔 총 beat 수 (= BTT / 4)
    reg                    align_err_q;     // 시작할 때 정렬 에러였는지
    reg  [ADDR_WIDTH-1:0]  cur_addr;        // 다음 주문서에 쓸 주소 (burst 마다 전진)
    reg  [LEN_WIDTH-1:0]   req_beat_cnt;    // 지금까지 주문한 beat 수

    // BURST_CFG를 두 조각으로 나눠 씀
    wire [1:0]             burst_type_cfg = burst_cfg_q[BURST_WIDTH+1:BURST_WIDTH];  // [9:8] 타입
    wire [BURST_WIDTH-1:0] burst_len_cfg  = burst_cfg_q[BURST_WIDTH-1:0];            // [7:0] 길이
    assign arburst = burst_type_cfg;

    // 타입이 10(WRAP) 이나 11(예약) 이면 윗비트가 1 -> 지원 안 함
    wire burst_type_err = burst_type_cfg[1];

    // 정렬 체크 : 주소와 길이가 4의 배수여야 함 (아래 2비트가 00)
    // beat가 4바이트라서, 4의 배수가 아니면 중간에 걸친 바이트를 처리할 수 없음
    wire align_err_c = (src_addr[ADDR_LSB-1:0] != {ADDR_LSB{1'b0}}) ||
                       (length[ADDR_LSB-1:0]   != {ADDR_LSB{1'b0}});

    // 총 바이트 -> 총 beat ( / 4)
    wire [LEN_WIDTH-1:0] total_beats_c = length >> ADDR_LSB;

    // ########################################################################
    //  2. 진행 상황
    // ########################################################################
    // 아직 주문 안 한 beat가 남았는지?
    wire req_pending = (req_beat_cnt < total_beats_q);

    // 남은 beat 수 (burst 길이 계산에 씀)
    wire [LEN_WIDTH-1:0] remain_beats = total_beats_q - req_beat_cnt;

    // ########################################################################
    //  3. 주소 영역 체크 : 지금 주소가 DDR이나 BRAM 창 안에 있나?
    // ########################################################################
    wire in_r0 = (cur_addr >= R0_BASE) && (cur_addr < R0_END);   // DDR 안?
    wire in_r1 = (cur_addr >= R1_BASE) && (cur_addr < R1_END);   // BRAM 창 안?

    // 지금 있는 영역의 끝 주소 (burst 가 영역을 넘어가지 않게 자를 때 씀)
    wire [ADDR_WIDTH-1:0] region_end_c = in_r0 ? R0_END : R1_END;

    // 아직 주문할 게 남았는데 주소가 두 영역 어디에도 없으면 에러
    wire region_err_c = req_pending && !in_r0 && !in_r1;

    // ########################################################################
    //  4. handshake 신호들 
    // ########################################################################
    wire ar_hs = arvalid && arready;

    assign r_hs   = rvalid && rready;      // beat 하나가 방금 도착함
    assign rready = !fifo_full;            // FIFO에 자리가 있으면 항상 받음

    wire r_mine      = (rid[4] == MASTER_ID);   // 내 주문(DMA) 건인가? (ID 맨 윗비트 확인)
    wire r_beat      = r_hs && r_mine;          // 내 주문의 beat 가 도착
    wire r_burst_end = r_beat && rlast;         // 그게 그 주문의 마지막 beat

    // ########################################################################
    //  5. outstanding 관리 (동시에 최대 3개)
    //
    //     슬롯 = 주문서 번호표 (0, 1, 2)
    //       num_busy[i]  : i번 슬롯이 사용 중인가
    //       slot_addr[i] : i번 주문의 시작 주소      -> 에러 주소 역산용
    //       slot_beat[i] : i번 주문에서 받은 beat 수 -> 에러 주소 역산용
    //     주문 보낼 때 ARID 아래 2비트에 슬롯 번호를 넣고,
    //     물건이 오면 RID 아래 2비트로 어느 슬롯인지 알아냄
    // ########################################################################
    reg [MAX_OUTSTANDING-1:0]   num_busy;
    reg [ADDR_WIDTH-1:0]        slot_addr [0:MAX_OUTSTANDING-1];
    reg [BURST_WIDTH-1:0]       slot_beat [0:MAX_OUTSTANDING-1];
    reg [OUTSTANDING_CNT_W-1:0] outstanding_cnt;   // 진행 중인 주문 수 (0~3)
    reg [SLOT_IDX_W-1:0]        ar_num_q;          // 지금 나가는 주문서의 슬롯 번호

    // 비어 있는 슬롯 중 번호가 제일 작은 것
    // (셋 다 차 있으면 2가 나오지만, 그땐 outstanding_ok = 0 이라 주문을 안 냄)
    wire [1:0] next_num = (!num_busy[0]) ? 2'd0 :
                          (!num_busy[1]) ? 2'd1 :
                                           2'd2;

    // 진행 중인 주문 수 세기
    //   주문 접수(ar_hs)   -> +1
    //   주문 완료(마지막 beat) -> -1
    //   같은 클럭에 둘 다  -> 그대로
    always @(posedge clk) begin
        if (!rst_n)      outstanding_cnt <= '0;
        else if (init)   outstanding_cnt <= '0;
        else begin
            case ({ar_hs, r_burst_end})
                2'b10:   outstanding_cnt <= outstanding_cnt + 1'b1;
                2'b01:   outstanding_cnt <= outstanding_cnt - 1'b1;
                default: outstanding_cnt <= outstanding_cnt;
            endcase
        end
    end
    // 슬롯 사용 표시
    //   주문 접수 -> 그 슬롯 사용 중
    //   마지막 beat 도착 -> RID로 찾은 슬롯 비움
    always @(posedge clk) begin
        if (!rst_n)      num_busy <= '0;
        else if (init)   num_busy <= '0;
        else begin
            if (ar_hs)       num_busy[ar_num_q] <= 1'b1;
            if (r_burst_end) num_busy[rid[1:0]] <= 1'b0;
        end
    end
    // 슬롯에 여유가 있나?
    wire outstanding_ok = (outstanding_cnt < MAX_OUTSTANDING);

    // ########################################################################
    //  6. abort 기억
    //     abort는 1클럭 펄스 -> 한 번 오면 이번 프레임 끝날 때까지 1로 세워둠
    //     -> 새 주문서는 더 안 보내지만 이미 보낸 주문의 물건은 끝까지 받음
    // ########################################################################
    reg abort_lat;
    always @(posedge clk) begin
        if (!rst_n)      abort_lat <= 1'b0;
        else if (init)   abort_lat <= 1'b0;
        else if (abort)  abort_lat <= 1'b1;
    end

    // ########################################################################
    //  7. 이번 burst 길이 정하기
    //     INCR는 아래 4개 중 제일 작은 값
    //       (a) 설정값   : BURST_CFG[7:0] + 1, 단 최대 16
    //       (b) 4KB 경계 : 다음 4KB 경계까지 남은 beat
    //       (c) 영역 끝  : 지금 영역 끝까지 남은 beat
    //       (d) 남은 양  : 아직 주문 안 한 beat
    // ########################################################################

    // (b) 4KB 경계까지 : 4096 - (주소 하위 12비트)
    wire [LEN_WIDTH-1:0] bytes_to_boundary = PAGE_BYTES -
                               {{(LEN_WIDTH-PAGE_LSB){1'b0}}, cur_addr[PAGE_LSB-1:0]};
    wire [LEN_WIDTH-1:0] beats_to_boundary = bytes_to_boundary >> ADDR_LSB;

    // (c) 영역 끝까지
    wire [LEN_WIDTH-1:0] bytes_to_region   = region_end_c - cur_addr;
    wire [LEN_WIDTH-1:0] beats_to_region   = bytes_to_region >> ADDR_LSB;

    // (a) 설정값 : ARLEN 은 "beat 수 - 1" 이라 +1 해서 beat 수로 바꾸고, 16 넘으면 16
    wire [LEN_WIDTH-1:0] desired_raw   = {{(LEN_WIDTH-BURST_WIDTH){1'b0}}, burst_len_cfg} + 1'b1;
    wire [LEN_WIDTH-1:0] desired_beats = (desired_raw > MAX_BURST_BEATS) ? MAX_BURST_BEATS
                                                                         : desired_raw;

    // (b) 와 (c) 중 작은 쪽
    wire [LEN_WIDTH-1:0] limit_beats = (beats_to_boundary < beats_to_region) ? beats_to_boundary
                                                                              : beats_to_region;

    // FIXED : 주소가 안 움직이니 경계 걱정 없음 -> min(남은 양, 설정값, 16)
    wire [LEN_WIDTH-1:0] safe_beats_fixed =
        (remain_beats < desired_beats) ?
            ((remain_beats  < 16) ? remain_beats  : 16) :
            ((desired_beats < 16) ? desired_beats : 16);

    // INCR : min(설정값, 경계, 남은 양)  (경계 = 4KB 와 영역 끝 중 작은 것)
    wire [LEN_WIDTH-1:0] safe_beats_incr =
        (desired_beats < limit_beats) ?
            ((desired_beats < remain_beats) ? desired_beats : remain_beats) :
            ((limit_beats   < remain_beats) ? limit_beats   : remain_beats);

    // 최종 beat 수 (지원 안 하는 타입이면 0 -> 주문 안 나감)
    wire [LEN_WIDTH-1:0] safe_beats =
        (burst_type_cfg == BURST_FIXED) ? safe_beats_fixed :
        (burst_type_cfg == BURST_INCR)  ? safe_beats_incr  : {LEN_WIDTH{1'b0}};

    wire [LEN_WIDTH-1:0]   safe_bytes = safe_beats << ADDR_LSB;            // beat -> 바이트 (×4) : 주소 전진량
    wire [BURST_WIDTH-1:0] safe_arlen = safe_beats[BURST_WIDTH-1:0] - 1'b1; // ARLEN = beat 수 - 1

    // ########################################################################
    //  8. 주문서를 내도 되는지 확인 / 설정 에러
    // ########################################################################
    wire ar_can_issue = en && !init          // 일하는 중이고, 시작 클럭이 아니고 (설정값 래치 중)
                        && !arvalid          // 이전 주문서가 아직 접수 대기 중이 아니고
                        && req_pending       // 주문할 게 남았고
                        && !abort_lat        // 멈추라는 말이 없었고
                        && outstanding_ok    // 슬롯 여유가 있고
                        && (safe_beats != {LEN_WIDTH{1'b0}})   // 길이가 0 이 아니고
                        && !cfg_err          // 설정 에러가 없고
                        && !region_err_c;    // 주소가 허용 영역 안

    // 남은 게 있는데 길이가 0으로 계산됨 = 더 이상 진행할 방법이 없음
    wire no_progress = en && !init && req_pending && !abort_lat
                       && (safe_beats == {LEN_WIDTH{1'b0}});

    // 설정 에러 플래그
    //   init 때 : 정렬 에러면 바로 1 (아니면 0 으로 초기화)
    //   일하는 중 : 진행 불가 / 정렬 에러 / 영역 밖 / 지원 안 하는 burst 타입 -> 1
    //   한 번 1 이 되면 다음 init 까지 유지 -> 주문 중단 -> 나간 주문 다 받으면 xfer_done
    always @(posedge clk) begin
        if (!rst_n)
            cfg_err <= 1'b0;
        else if (init)
            cfg_err <= align_err_c;
        else if (en && (no_progress || align_err_q || region_err_c || burst_type_err))
            cfg_err <= 1'b1;
    end

    // ########################################################################
    //  9. 설정값 래치 + 주소/카운트 전진
    // ########################################################################
    always @(posedge clk) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= {LEN_WIDTH{1'b0}};
            align_err_q    <= 1'b0;
            burst_cfg_q    <= {(BURST_WIDTH+2){1'b0}};
            slot_addr      <= '{default: '0};
            slot_beat      <= '{default: '0};
        end
        // ----- 프레임 시작 : 설정값 찰칵 -----
        else if (init) begin
            cur_addr       <= src_addr;         // 시작 주소
            req_beat_cnt   <= {LEN_WIDTH{1'b0}}; // 주문한 양 0 부터
            total_beats_q  <= total_beats_c;    // 총 beat 수
            align_err_q    <= align_err_c;      // 정렬 에러 여부
            burst_cfg_q    <= burst_cfg;        // burst 설정
            slot_addr      <= '{default: '0};   // 슬롯 기록 초기화
            slot_beat      <= '{default: '0};
        end
        // ----- 전송 중 -----
        else begin
            // 새 주문 접수 -> 그 슬롯의 받은 beat 수 0 부터
            if (ar_hs)  slot_beat[ar_num_q] <= {BURST_WIDTH{1'b0}};
            // beat 도착 -> RID 로 찾은 슬롯의 받은 beat 수 +1
            if (r_beat) slot_beat[rid[1:0]] <= slot_beat[rid[1:0]] + 1'b1;

            if (ar_hs) begin
                // INCR 면 다음 주문 주소를 이번 burst 크기만큼 전진 (FIXED 는 그대로)
                if (burst_type_cfg == BURST_INCR)
                    cur_addr <= cur_addr + safe_bytes[ADDR_WIDTH-1:0];
                req_beat_cnt        <= req_beat_cnt + safe_beats;   // 주문한 양 누적
                slot_addr[ar_num_q] <= araddr;                      // 이 슬롯의 시작 주소 기록
            end
        end
    end

    // ########################################################################
    //  10. AR 채널 : 주문서 내보내기
    //      AXI 규칙 : ARVALID 를 올리면 ARREADY 가 올 때까지 내용을 바꾸면 안 됨
    //      -> 올린 뒤엔 접수(ar_hs)될 때까지 가만히 두고, 접수되면 내림
    // ########################################################################
    always @(posedge clk) begin
        if (!rst_n) begin
            arvalid    <= 1'b0;
            araddr     <= {ADDR_WIDTH{1'b0}};
            arlen      <= {BURST_WIDTH{1'b0}};
            arid       <= 5'd0;
            ar_num_q   <= '0;
        end
        else if (ar_hs) begin
            arvalid <= 1'b0;                                // 접수됐으니 내림
        end
        else if (ar_can_issue) begin
            arvalid    <= 1'b1;                             // 새 주문서
            araddr     <= cur_addr;                         // 주소
            arlen      <= safe_arlen;                       // 길이
            arid       <= {MASTER_ID, 2'b00, next_num};     // {DMA, 예약, 슬롯번호}
            ar_num_q   <= next_num;                         // 어느 슬롯인지 기억
        end
    end

    // ########################################################################
    //  11. R 채널 -> FIFO
    //      내 주문의 beat 가 도착할 때마다 그대로 FIFO 에 씀
    //      (FIFO 가 차면 rready = 0 이라 도착 자체가 안 일어남 -> 넘칠 일 없음)
    // ########################################################################
    always @(*) begin
        fifo_wr_en   = r_beat;
        fifo_wr_data = rdata;
    end

    // ########################################################################
    //  12. 에러 주소 역산
    //      RRESP[1] = 1 이면 SLVERR(10) 또는 DECERR(11) = 불량
    //      불량 beat 의 주소 = 그 슬롯의 시작 주소 + (그 슬롯에서 앞서 받은 beat 수 × 4)
    //        예) 슬롯1 시작 0x1000_0100, 이미 3 beat 받았고 4번째가 불량
    //            -> 0x1000_0100 + 3×4 = 0x1000_010C
    //      FIXED 는 주소가 안 움직이니 시작 주소 그대로
    // ########################################################################
    wire beat_err = r_beat && rresp[1];

    wire [ADDR_WIDTH-1:0] beat_addr =
        (burst_type_cfg == BURST_FIXED)
            ? slot_addr[rid[1:0]]
            : slot_addr[rid[1:0]] +
              ({{(ADDR_WIDTH-BURST_WIDTH){1'b0}}, slot_beat[rid[1:0]]} << ADDR_LSB);

    reg [ADDR_WIDTH-1:0] err_addr_q;
    assign err_addr = err_addr_q;

    // 첫 번째 불량만 기록 (원인 파악엔 첫 번째가 제일 중요)
    // 참고 : 응답 에러가 나도 주문은 멈추지 않고 이번 프레임은 끝까지 받음.
    //        반복 여부는 controller 가 판단함 (에러 있으면 다음 프레임 안 함)
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

    // ########################################################################
    //  13. 끝났나? (xfer_done)
    //      "더 이상 주문할 게 없고" + "보낸 주문의 물건이 전부 도착" 이면 끝
    //        주문할 게 없는 경우 : 다 주문함 / abort / 설정 에러
    //      !init : 시작 클럭에는 이전 프레임 값이 남아 있을 수 있어서 제외
    //              (반복 모드에서 같은 프레임 끝을 두 번 세는 걸 막음)
    // ########################################################################
    wire ar_done = !req_pending || abort_lat || cfg_err;

    assign xfer_done = !init && ar_done && (outstanding_cnt == '0);

endmodule