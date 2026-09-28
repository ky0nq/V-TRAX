// ============================================================================
// dma_scoreboard : DMA 통합 채점
//
//   monitor 5개에서 item 을 받음 (write 함수 이름 구분은 dma_pkg 의 imp_decl)
//     write_cam  : camera 가 넣은 프레임
//     write_disp : disp 로 나온 프레임
//     write_axil : 레지스터 접근 (DA0~2 주소, START 를 여기서 알아냄)
//     write_s2mm : S2MM 이 메모리에 쓴 버스트
//     write_mm2s : MM2S 가 메모리에서 읽은 버스트
//
//   채점 항목
//     [1] S2MM  : camera 픽셀을 byte 로 풀어서 = S2MM 이 쓴 데이터/주소
//     [2] MM2S  : MM2S 가 읽은 데이터를 픽셀로 묶어서 = disp 로 나온 픽셀
//     [3] E2E   : disp 로 나온 프레임이 최근 camera 프레임 중 하나와 같은가
//     [4] 프로토콜 : SOF 1번 / EOL 이 줄 수만큼, AXI 버스트가 4KB 경계를 안 넘는가
//     [5] 레지스터 : AXI-Lite 응답이 전부 OKAY 인가
//
//   설정 (config_db 또는 test 에서 env.sb.xxx 로 직접)
//     check_e2e  : 0 이면 [3] 끔 (LIVE=0 처럼 camera 와 상관없는 걸 읽을 때)
//     check_mm2s : 0 이면 [2] 끔 (MM2S 에러 주입처럼 출력 데이터가 정의 안 될 때)
//
//   byte 순서 (packetizer / depacketizer 와 동일, little endian)
//     픽셀 p0, p1 ... -> byte p0[7:0], p0[15:8], p0[23:16], p1[7:0] ...
// ============================================================================
class dma_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(dma_scoreboard)

    uvm_analysis_imp_cam  #(axis_item, dma_scoreboard) cam_imp;
    uvm_analysis_imp_disp #(axis_item, dma_scoreboard) disp_imp;
    uvm_analysis_imp_axil #(axil_item, dma_scoreboard) axil_imp;
    uvm_analysis_imp_s2mm #(s2mm_item, dma_scoreboard) s2mm_imp;
    uvm_analysis_imp_mm2s #(mm2s_item, dma_scoreboard) mm2s_imp;

    // ---------------- 설정 ----------------
    int unsigned width      = 64;
    int unsigned height     = 4;
    bit          check_e2e  = 1;
    bit          check_mm2s = 1;

    // ---------------- 레지스터에서 알아낸 값 ----------------
    bit [31:0]   da[3];

    // ---------------- [1] S2MM ----------------
    typedef struct { bit [31:0] addr; bit [31:0] data; } s2mm_beat_t;
    bit [7:0]    s2mm_exp_bytes[$];    // camera 픽셀을 풀어놓은 기대 byte
    s2mm_beat_t  s2mm_act_q[$];        // S2MM 이 쓴 word (기대값보다 먼저 올 수 있어서 모아둠)
    int unsigned s2mm_frame_idx;
    int unsigned s2mm_word_idx;
    bit          s2mm_frame_bad;

    // ---------------- [2] MM2S ----------------
    bit [7:0]    mm2s_rd_bytes[$];

    // ---------------- [3] E2E ----------------
    axis_item    cam_hist[$];
    localparam int HIST_MAX = 8;

    // ---------------- 카운터 ----------------
    int s2mm_pass_count = 0;   int s2mm_fail_count = 0;
    int mm2s_pass_count = 0;   int mm2s_fail_count = 0;
    int e2e_pass_count  = 0;   int e2e_fail_count  = 0;
    int proto_fail_count = 0;
    int reg_fail_count   = 0;
    int cam_frame_count  = 0;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        cam_imp  = new("cam_imp",  this);
        disp_imp = new("disp_imp", this);
        axil_imp = new("axil_imp", this);
        s2mm_imp = new("s2mm_imp", this);
        mm2s_imp = new("mm2s_imp", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        void'(uvm_config_db#(int unsigned)::get(this, "", "width",  width));
        void'(uvm_config_db#(int unsigned)::get(this, "", "height", height));
        void'(uvm_config_db#(bit)::get(this, "", "check_e2e",  check_e2e));
        void'(uvm_config_db#(bit)::get(this, "", "check_mm2s", check_mm2s));
    endfunction

    function int unsigned frame_words();
        return width * height * 3 / 4;
    endfunction

    // AXI 규칙 : INCR 버스트는 4KB 경계를 넘으면 안 됨
    function void check_4k(string ch, bit [31:0] addr, int unsigned beats, bit [1:0] burst);
        if (burst == 2'b01 && (addr[11:0] + beats * 4) > 4096) begin
            proto_fail_count++;
            `uvm_error(get_type_name(), $sformatf(
                "PROTO FAIL [%s]: 4KB 경계를 넘는 버스트 (addr=0x%08h, %0d beat -> 끝 0x%08h)",
                ch, addr, beats, addr + beats * 4 - 1))
        end
    endfunction

    // ========================================================================
    // camera
    // ========================================================================
    function void write_cam(axis_item tr);
        foreach (tr.pixels[i]) begin
            s2mm_exp_bytes.push_back(tr.pixels[i][7:0]);
            s2mm_exp_bytes.push_back(tr.pixels[i][15:8]);
            s2mm_exp_bytes.push_back(tr.pixels[i][23:16]);
        end
        cam_frame_count++;
        cam_hist.push_back(tr);
        if (cam_hist.size() > HIST_MAX) void'(cam_hist.pop_front());
        check_s2mm();
        `uvm_info(get_type_name(), $sformatf("CAM IN : %s", tr.convert2string()), UVM_MEDIUM)
    endfunction

    // ========================================================================
    // axil
    // ========================================================================
    function void write_axil(axil_item tr);
        if (tr.resp !== 2'b00) begin
            reg_fail_count++;
            `uvm_error(get_type_name(), $sformatf("REG FAIL: 응답 에러 %s", tr.convert2string()))
        end
        if (tr.is_write) begin
            case (tr.addr)
                7'h48: da[0] = tr.wdata;
                7'h4C: da[1] = tr.wdata;
                7'h50: da[2] = tr.wdata;
                7'h54: if (tr.wdata[0]) begin        // S2MM START
                           s2mm_frame_idx = 0;
                           s2mm_word_idx  = 0;
                           s2mm_frame_bad = 0;
                       end
                default: ;
            endcase
        end
    endfunction

    // ========================================================================
    // [1] S2MM
    // ========================================================================
    function void write_s2mm(s2mm_item tr);
        s2mm_beat_t b;
        check_4k("S2MM", tr.awaddr, tr.awlen + 1, tr.awburst);
        if (tr.bresp !== 2'b00)
            `uvm_info(get_type_name(), $sformatf("S2MM 버스트 에러 응답 (bresp=%0d, addr=0x%08h) : 에러 주입",
                      tr.bresp, tr.awaddr), UVM_LOW)
        foreach (tr.wdata[j]) begin
            b.addr = tr.awaddr + j * 4;
            b.data = tr.wdata[j];
            s2mm_act_q.push_back(b);
        end
        check_s2mm();
    endfunction

    function void check_s2mm();
        bit [31:0]  exp_word, exp_addr;
        s2mm_beat_t b;
        while (s2mm_act_q.size() > 0 && s2mm_exp_bytes.size() >= 4) begin
            b        = s2mm_act_q.pop_front();
            exp_word = {s2mm_exp_bytes[3], s2mm_exp_bytes[2], s2mm_exp_bytes[1], s2mm_exp_bytes[0]};
            repeat (4) void'(s2mm_exp_bytes.pop_front());
            exp_addr = da[s2mm_frame_idx % 3] + s2mm_word_idx * 4;

            if (b.data !== exp_word || b.addr !== exp_addr) begin
                s2mm_frame_bad = 1;
                `uvm_error(get_type_name(), $sformatf(
                    "S2MM FAIL [frame%0d word%0d]: addr=0x%08h data=0x%08h (기대값 addr=0x%08h data=0x%08h)",
                    s2mm_frame_idx, s2mm_word_idx, b.addr, b.data, exp_addr, exp_word))
            end

            s2mm_word_idx++;
            if (s2mm_word_idx == frame_words()) begin
                if (s2mm_frame_bad) s2mm_fail_count++;
                else begin
                    s2mm_pass_count++;
                    `uvm_info(get_type_name(), $sformatf("S2MM PASS [frame%0d -> buf%0d @0x%08h]",
                              s2mm_frame_idx, s2mm_frame_idx % 3, da[s2mm_frame_idx % 3]), UVM_MEDIUM)
                end
                s2mm_frame_idx++;
                s2mm_word_idx  = 0;
                s2mm_frame_bad = 0;
            end
        end
    endfunction

    // ========================================================================
    // MM2S 가 읽은 데이터 저장
    // ========================================================================
    function void write_mm2s(mm2s_item tr);
        check_4k("MM2S", tr.araddr, tr.arlen + 1, tr.arburst);
        foreach (tr.rdata[j]) begin
            if (tr.rresp[j] !== 2'b00)
                `uvm_info(get_type_name(), $sformatf("MM2S beat 에러 응답 (rresp=%0d, 버스트 addr 0x%08h) : 에러 주입",
                          tr.rresp[j], tr.araddr), UVM_LOW)
            mm2s_rd_bytes.push_back(tr.rdata[j][7:0]);
            mm2s_rd_bytes.push_back(tr.rdata[j][15:8]);
            mm2s_rd_bytes.push_back(tr.rdata[j][23:16]);
            mm2s_rd_bytes.push_back(tr.rdata[j][31:24]);
        end
    endfunction

    // ========================================================================
    // disp : [4] 프로토콜 -> [2] MM2S -> [3] E2E
    // ========================================================================
    function void write_disp(axis_item tr);
        bit [23:0] exp_pix;
        int        bad_cnt = 0;
        bit        found   = 0;

        `uvm_info(get_type_name(), $sformatf("DISP OUT: %s", tr.convert2string()), UVM_MEDIUM)

        if (tr.sof_cnt != 1 || tr.eol_cnt != height) begin
            proto_fail_count++;
            `uvm_error(get_type_name(), $sformatf("PROTO FAIL: SOF=%0d (기대 1), EOL=%0d (기대 %0d)",
                       tr.sof_cnt, tr.eol_cnt, height))
        end

        // ---------------- [2] MM2S ----------------
        foreach (tr.pixels[i]) begin
            if (mm2s_rd_bytes.size() < 3) begin
                bad_cnt++;
                if (check_mm2s && bad_cnt <= 5)
                    `uvm_error(get_type_name(), $sformatf("MM2S FAIL: px%0d 는 MM2S 가 읽은 적 없는 데이터", i))
                continue;
            end
            exp_pix = {mm2s_rd_bytes[2], mm2s_rd_bytes[1], mm2s_rd_bytes[0]};
            repeat (3) void'(mm2s_rd_bytes.pop_front());
            if (tr.pixels[i] !== exp_pix) begin
                bad_cnt++;
                if (check_mm2s && bad_cnt <= 5)
                    `uvm_error(get_type_name(), $sformatf("MM2S FAIL [px%0d (x=%0d,y=%0d)]: out=0x%06h (기대값=0x%06h)",
                               i, i % width, i / width, tr.pixels[i], exp_pix))
            end
        end
        if (check_mm2s) begin
            if (bad_cnt == 0) mm2s_pass_count++;
            else begin
                mm2s_fail_count++;
                `uvm_error(get_type_name(), $sformatf("MM2S FAIL: 픽셀 %0d개 불일치", bad_cnt))
            end
        end

        // ---------------- [3] E2E ----------------
        if (check_e2e) begin
            for (int h = cam_hist.size() - 1; h >= 0; h--) begin
                if (same_frame(cam_hist[h], tr)) begin
                    found = 1;
                    `uvm_info(get_type_name(), $sformatf("E2E PASS: 최근 camera 프레임 중 %0d번째 전 프레임과 일치",
                              cam_hist.size() - 1 - h), UVM_MEDIUM)
                    break;
                end
            end
            if (found) e2e_pass_count++;
            else begin
                e2e_fail_count++;
                `uvm_error(get_type_name(), "E2E FAIL: 나온 프레임이 최근 camera 프레임 어느 것과도 다름")
            end
        end
    endfunction

    function bit same_frame(axis_item a, axis_item b);
        if (a.pixels.size() != b.pixels.size()) return 0;
        foreach (a.pixels[i])
            if (a.pixels[i] !== b.pixels[i]) return 0;
        return 1;
    endfunction

    // ========================================================================
    // 리포트
    // ========================================================================
    function void report_phase(uvm_phase phase);
        int total_fail;

        if (s2mm_act_q.size() != 0) begin
            s2mm_fail_count++;
            `uvm_error("SCB", $sformatf("S2MM FAIL: camera 가 넣은 것보다 %0d word 더 씀", s2mm_act_q.size()))
        end

        total_fail = s2mm_fail_count + mm2s_fail_count + e2e_fail_count
                   + proto_fail_count + reg_fail_count;

        `uvm_info("SCB", "================================", UVM_LOW)
        `uvm_info("SCB", "=======Scoreboard Summary=======", UVM_LOW)
        `uvm_info("SCB", "================================", UVM_LOW)
        `uvm_info("SCB", "  [1] camera -> S2MM -> memory ", UVM_LOW)
        `uvm_info("SCB", $sformatf("pass count: %0d", s2mm_pass_count), UVM_LOW)
        `uvm_info("SCB", $sformatf("fail count: %0d", s2mm_fail_count), UVM_LOW)
        `uvm_info("SCB", "================================", UVM_LOW)
        `uvm_info("SCB", $sformatf("  [2] memory -> MM2S -> disp %s", check_mm2s ? "" : "(OFF)"), UVM_LOW)
        `uvm_info("SCB", $sformatf("pass count: %0d", mm2s_pass_count), UVM_LOW)
        `uvm_info("SCB", $sformatf("fail count: %0d", mm2s_fail_count), UVM_LOW)
        `uvm_info("SCB", "================================", UVM_LOW)
        `uvm_info("SCB", $sformatf("  [3] camera = disp (E2E) %s", check_e2e ? "" : "(OFF)"), UVM_LOW)
        `uvm_info("SCB", $sformatf("pass count: %0d", e2e_pass_count), UVM_LOW)
        `uvm_info("SCB", $sformatf("fail count: %0d", e2e_fail_count), UVM_LOW)
        `uvm_info("SCB", "================================", UVM_LOW)
        `uvm_info("SCB", $sformatf("  [4] protocol fail : %0d", proto_fail_count), UVM_LOW)
        `uvm_info("SCB", $sformatf("  [5] register fail : %0d", reg_fail_count), UVM_LOW)
        `uvm_info("SCB", "================================", UVM_LOW)

        if (s2mm_exp_bytes.size() != 0)
            `uvm_info("SCB", $sformatf("참고: S2MM 이 아직 안 쓴 camera 데이터 %0d byte", s2mm_exp_bytes.size()), UVM_LOW)
        if (mm2s_rd_bytes.size() != 0)
            `uvm_info("SCB", $sformatf("참고: disp 로 아직 안 나온 MM2S 데이터 %0d byte", mm2s_rd_bytes.size()), UVM_LOW)

        if (cam_frame_count > 0 && s2mm_pass_count + mm2s_pass_count + e2e_pass_count == 0)
            `uvm_error(get_type_name(), "TEST FAILED: 프레임을 넣었는데 채점된 프레임이 하나도 없음")
        else if (total_fail > 0)
            `uvm_error(get_type_name(), $sformatf("TEST FAILED: 총 fail=%0d", total_fail))
        else
            `uvm_info(get_type_name(), $sformatf("TEST PASSED: S2MM %0d / MM2S %0d / E2E %0d 프레임",
                      s2mm_pass_count, mm2s_pass_count, e2e_pass_count), UVM_LOW)
    endfunction
endclass