`timescale 1ns / 1ps
//
// Zybo Z7-20 로딩화면 + AI 가속기 가중치 겸용 ram_bridge = RAM(Weight)
//   (가속기 입력 이미지는 별도 RAM(Capture)이 DDR3L/HP0에서 받아옴 -> 여기엔 없음)
//
// ===== BRAM 물리 배치 (32bit x 60,416워드, addr 16bit) =====
//   0x0000 ~ 0xB0C1 (0      ~ 45,249) : 가중치 (1개/워드 [23:0], 45,250개) <- CPU (GP1 -> AXI Interconnect)
//   0xB100 ~ 0xB12A (45,312 ~ 45,354) : Param  (1개/워드 [31:0], 43개)     <- CPU (GP1 -> AXI Interconnect)
//   0xB200 ~ 0xEB6B (45,568 ~ 60,267) : 로딩 이미지 (200x49, 3B/px 패킹)   <- coe
//                                        (coe 앞에 0을 45,568줄 채운 패딩 버전 사용)
//
// ===== AXI 주소 창 (AXI BRAM Controller byte address 22bit) =====
//   0x00_0000 ~ 0x2A_2FFF : 프레임 창 (1280x720x3B, 읽기 전용, DMA MM2S용)
//                           -> 박스 + 배경 로직 (보드 검증된 로직 그대로)
//   0x30_0000 ~ 0x33_FFFF : CPU 직접 접근 창 (256KB, BRAM 전체 1:1 매핑)
//                           -> CPU 주소 = 0x30_0000 + (BRAM 워드주소 x 4)
//                              가중치 시작 0x30_0000, Param 시작 0x32_C400
//                           -> 쓰기는 LOAD_IMG_BASE 미만만 허용 (로딩 이미지 보호)
//                           -> 화면 카운터를 건드리지 않음
//
// ===== 가속기 (BRAM Port B, 읽기 전용: web/dinb = 0) =====
//   가중치 : addrb = WEIGHT_BASE + w_idx   (= w_idx)
//   Param  : addrb = PARAM_BASE  + p_idx   (0xB100 + p_idx)
//
// ===== blk_mem_gen 설정 =====
//   - True Dual Port RAM, Width 32, Depth 60416
//   - Byte Write Enable 켜기, Byte Size 8 -> wea[3:0]
//   - Port A Output Register(Primitives/Core) 모두 끄기 (1클럭 지연 전제)
//   - Port B 클럭은 가속기 클럭 사용 가능 (clkb 분리)
//   - coe: test_image_padded.coe (make_padded_coe.py 로 생성)
//
module ram_bridge #(
    parameter [15:0] WEIGHT_BASE   = 16'h0000,    // 문서/가속기 공유용 (브릿지 로직에선 미사용)
    parameter [15:0] PARAM_BASE    = 16'hB100,    // 문서/가속기 공유용 (브릿지 로직에선 미사용)
    parameter [15:0] LOAD_IMG_BASE = 16'hB200
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        ctrl_en,
    input  wire [3:0]  ctrl_we,
    input  wire [21:0] ctrl_addr,
    input  wire [31:0] ctrl_din,
    output wire [31:0] ctrl_dout,

    output wire        bram_en,
    output wire [3:0]  bram_we,     // byte write enable
    output wire [15:0] bram_addr,
    output wire [31:0] bram_din,
    input  wire [31:0] bram_dout
);

    // ------------------------------------------------------------------
    // 화면/박스 파라미터 (기존 image_control과 동일)
    // ------------------------------------------------------------------
    localparam WORDS_PER_LINE = 960;
    localparam BOX_WORD_START = 330;
    localparam BOX_WORD_LEN   = 300;
    localparam START_Y        = 311;
    localparam BOX_H          = 98;

    localparam [31:0] BG_COLOR = 32'h00000000;   // 4바이트 전부 동일값 유지

    // ------------------------------------------------------------------
    // 접근 종류 판별
    // ------------------------------------------------------------------
    wire [15:0] direct_addr = ctrl_addr[17:2];                    // BRAM 워드주소
    wire is_direct  = (ctrl_addr[21:18] == 4'b1100);              // 0x30_0000 ~ 0x33_FFFF
    wire is_write   = (ctrl_we != 4'b0000);
    wire frame_rd   = ctrl_en && !is_write && !is_direct;         // 화면 스트림 읽기
    wire direct_wr  = ctrl_en &&  is_write &&  is_direct
                      && (direct_addr <  LOAD_IMG_BASE);          // 로딩 이미지 영역 쓰기 차단

    // ------------------------------------------------------------------
    // 화면 좌표 카운터 (기존 로직 그대로, frame_rd일 때만 전진)
    // ------------------------------------------------------------------
    reg [9:0]  word_in_line = 0;
    reg [10:0] y_coord      = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            word_in_line <= 0;
            y_coord      <= 0;
        end else if (frame_rd) begin
            if (ctrl_addr[21:2] == 20'd0) begin
                word_in_line <= 1;
                y_coord      <= 0;
            end else if (word_in_line == WORDS_PER_LINE - 1) begin
                word_in_line <= 0;
                y_coord      <= y_coord + 1;
            end else begin
                word_in_line <= word_in_line + 1;
            end
        end
    end

    wire is_box_row  = (y_coord >= START_Y) && (y_coord < START_Y + BOX_H);
    wire is_box_word = is_box_row &&
                        (word_in_line >= BOX_WORD_START) &&
                        (word_in_line <  BOX_WORD_START + BOX_WORD_LEN);

    wire [9:0]  sy          = (y_coord - START_Y) >> 1;
    wire [9:0]  word_in_row = word_in_line - BOX_WORD_START;
    wire [15:0] img_off     = is_box_word ? (sy * 300 + word_in_row) : 16'h0;

    // ------------------------------------------------------------------
    // BRAM 포트
    // ------------------------------------------------------------------
    assign bram_en   = ctrl_en;
    assign bram_we   = direct_wr ? ctrl_we : 4'b0000;
    assign bram_din  = ctrl_din;
    assign bram_addr = is_direct ? direct_addr
                                 : (LOAD_IMG_BASE + img_off);

    // ------------------------------------------------------------------
    // 읽기 응답 1클럭 지연 보정
    //   direct_rd_q   : 직전 읽기가 CPU 직접 창이었는지 (디버깅용 읽기)
    //   is_box_word_q : 직전 프레임 읽기가 박스 안이었는지 (기존과 동일)
    // ------------------------------------------------------------------
    reg direct_rd_q;
    reg is_box_word_q;

    always @(posedge clk) begin
        if (!rst_n) begin
            direct_rd_q   <= 1'b0;
            is_box_word_q <= 1'b0;
        end else if (ctrl_en && !is_write) begin
            direct_rd_q <= is_direct;
            if (!is_direct)
                is_box_word_q <= is_box_word;
        end
    end

    assign ctrl_dout = direct_rd_q   ? bram_dout :
                       is_box_word_q ? bram_dout : BG_COLOR;

endmodule