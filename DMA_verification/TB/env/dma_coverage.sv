// ============================================================================
// dma_coverage : "어떤 상황을 거쳐봤는지" 기록 (채점은 안 함)
//
//   cg_reg   : 어떤 레지스터를 read/write 해봤는지
//   cg_mode  : MM2S 모드 (LIVE / CYCLIC / IDX_SW) 조합
//   cg_cfg   : BURST_CFG 에 어떤 길이/타입을 설정해봤는지 (DUT 가 16 beat 로 자르는 경우 포함)
//   cg_s2mm  : S2MM 이 어떤 버퍼에 썼고 어떤 응답을 받았는지
//   cg_mm2s  : MM2S 가 어떤 버퍼를, 어떤 길이/타입으로 읽었고 어떤 응답을 받았는지
//   cg_frame : 경계값 픽셀 프레임을 넣어봤는지
//
//   ※ 설계상 갈 수 없는 bin 은 ignore_bins 로 빼고 이유를 적어둠 (coverage closure 근거)
// ============================================================================
class dma_coverage extends uvm_component;
    `uvm_component_utils(dma_coverage)

    uvm_analysis_imp_cam  #(axis_item, dma_coverage) cam_imp;
    uvm_analysis_imp_axil #(axil_item, dma_coverage) axil_imp;
    uvm_analysis_imp_s2mm #(s2mm_item, dma_coverage) s2mm_imp;
    uvm_analysis_imp_mm2s #(mm2s_item, dma_coverage) mm2s_imp;

    int unsigned width  = 64;
    int unsigned height = 4;
    bit [31:0]   da[3];

    // ---------------- 레지스터 ----------------
    covergroup cg_reg with function sample(bit [6:0] addr, bit is_write);
        cp_addr : coverpoint addr {
            bins MM2S_CR   = {7'h00};  bins MM2S_SR   = {7'h04};
            bins SA        = {7'h18};  bins READ_ERR  = {7'h1C};
            bins BTT       = {7'h28};  bins BURST_CFG = {7'h30};
            bins NUM_BUF   = {7'h38};  bins SW_IDX    = {7'h3C};
            bins S2MM_CR   = {7'h40};  bins S2MM_SR   = {7'h44};
            bins DA0       = {7'h48};  bins DA1       = {7'h4C};
            bins DA2       = {7'h50};  bins START     = {7'h54};
            bins WRITE_ERR = {7'h58};
        }
        cp_dir  : coverpoint is_write { bins RD = {0}; bins WR = {1}; }
        x_addr_dir : cross cp_addr, cp_dir {
            // 읽기 전용(에러 주소)에 쓰기, 펄스 레지스터(START) 읽기는 의미 없음
            ignore_bins ro_wr = binsof(cp_addr) intersect {7'h1C, 7'h58} && binsof(cp_dir) intersect {1};
            ignore_bins wo_rd = binsof(cp_addr) intersect {7'h54}        && binsof(cp_dir) intersect {0};
        }
    endgroup

    // ---------------- MM2S 모드 (MM2S_CR 쓸 때) ----------------
    covergroup cg_mode with function sample(bit live, bit cyclic, bit idx_sw);
        cp_live   : coverpoint live;
        cp_cyclic : coverpoint cyclic;
        cp_idx_sw : coverpoint idx_sw;
        x_mode    : cross cp_live, cp_cyclic;
    endgroup

    // ---------------- BURST_CFG 설정값 (BURST_CFG 쓸 때) ----------------
    covergroup cg_cfg with function sample(bit [7:0] len, bit [1:0] btype);
        cp_len  : coverpoint len   { bins ONE = {0}; bins SHORT = {[1:14]}; bins MAX16 = {15};
                                     bins OVER16 = {[16:255]}; }          // OVER16 : DUT 가 16 으로 잘라야 함
        cp_type : coverpoint btype { bins FIXED = {2'b00}; bins INCR = {2'b01};
                                     bins WRAP_ERR = {2'b10}; bins RSVD_ERR = {2'b11}; }
    endgroup

    // ---------------- S2MM 버스트 ----------------
    covergroup cg_s2mm with function sample(int buf_idx, bit [3:0] len, bit [1:0] resp);
        cp_buf  : coverpoint buf_idx { bins BUF[] = {0, 1, 2}; bins OTHER = default; }
        // S2MM writer 는 BURST_LEN=16 고정 -> 항상 awlen=15. 다른 길이는 설계상 안 나옴
        cp_len  : coverpoint len     { bins FULL16 = {15}; ignore_bins fixed_by_design = {[0:14]}; }
        cp_resp : coverpoint resp    { bins OKAY = {2'b00}; bins SLVERR = {2'b10}; bins DECERR = {2'b11}; }
        x_buf_resp : cross cp_buf, cp_resp;
    endgroup

    // ---------------- MM2S 버스트 ----------------
    covergroup cg_mm2s with function sample(int buf_idx, bit [7:0] len, bit [1:0] burst, bit [1:0] resp);
        cp_buf   : coverpoint buf_idx { bins BUF[] = {0, 1, 2}; bins SA_OR_OTHER = default; }
        // DUT 가 최대 16 beat 로 자르므로 awlen 16 이상은 설계상 안 나옴
        cp_len   : coverpoint len     { bins SINGLE = {0}; bins SHORT = {[1:14]}; bins LEN16 = {15};
                                        ignore_bins capped_by_design = {[16:255]}; }
        cp_burst : coverpoint burst   { bins FIXED = {2'b00}; bins INCR = {2'b01}; }
        cp_resp  : coverpoint resp    { bins OKAY = {2'b00}; bins SLVERR = {2'b10}; bins DECERR = {2'b11}; }
        x_len_burst : cross cp_len, cp_burst;
    endgroup

    // ---------------- 입력 프레임 ----------------
    covergroup cg_frame with function sample(bit [23:0] first_px, bit solid);
        cp_corner : coverpoint first_px iff (solid) {
            bins BLACK = {24'h000000}; bins WHITE = {24'hFFFFFF};
            bins H55   = {24'h555555}; bins HAA   = {24'hAAAAAA};
        }
        cp_solid  : coverpoint solid { bins SOLID = {1}; bins MIXED = {0}; }
    endgroup

    function new(string name, uvm_component parent);
        super.new(name, parent);
        cam_imp  = new("cam_imp",  this);
        axil_imp = new("axil_imp", this);
        s2mm_imp = new("s2mm_imp", this);
        mm2s_imp = new("mm2s_imp", this);
        cg_reg   = new();
        cg_mode  = new();
        cg_cfg   = new();
        cg_s2mm  = new();
        cg_mm2s  = new();
        cg_frame = new();
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        void'(uvm_config_db#(int unsigned)::get(this, "", "width",  width));
        void'(uvm_config_db#(int unsigned)::get(this, "", "height", height));
    endfunction

    function int find_buf(bit [31:0] addr);
        int unsigned frame_bytes = width * height * 3;
        for (int i = 0; i < 3; i++)
            if (addr >= da[i] && addr < da[i] + frame_bytes) return i;
        return -1;
    endfunction

    function void write_axil(axil_item tr);
        cg_reg.sample(tr.addr, tr.is_write);
        if (tr.is_write) begin
            case (tr.addr)
                7'h48: da[0] = tr.wdata;
                7'h4C: da[1] = tr.wdata;
                7'h50: da[2] = tr.wdata;
                7'h00: cg_mode.sample(tr.wdata[5], tr.wdata[4], tr.wdata[6]);   // LIVE, CYCLIC, IDX_SW
                7'h30: cg_cfg.sample(tr.wdata[7:0], tr.wdata[9:8]);             // ARLEN, 타입
                default: ;
            endcase
        end
    endfunction

    function void write_s2mm(s2mm_item tr);
        cg_s2mm.sample(find_buf(tr.awaddr), tr.awlen, tr.bresp);
    endfunction

    function void write_mm2s(mm2s_item tr);
        bit [1:0] worst = 2'b00;
        foreach (tr.rresp[i]) if (tr.rresp[i] != 2'b00) worst = tr.rresp[i];
        cg_mm2s.sample(find_buf(tr.araddr), tr.arlen, tr.arburst, worst);
    endfunction

    function void write_cam(axis_item tr);
        bit solid = 1;
        foreach (tr.pixels[i]) if (tr.pixels[i] !== tr.pixels[0]) begin solid = 0; break; end
        cg_frame.sample(tr.pixels[0], solid);
    endfunction

    function void report_phase(uvm_phase phase);
        `uvm_info("COV", "================================", UVM_LOW)
        `uvm_info("COV", "========Coverage Summary========", UVM_LOW)
        `uvm_info("COV", "  (이 test 하나 기준, 전체는 make cov)", UVM_LOW)
        `uvm_info("COV", "================================", UVM_LOW)
        `uvm_info("COV", $sformatf("register   : %5.1f %%", cg_reg.get_inst_coverage()),   UVM_LOW)
        `uvm_info("COV", $sformatf("mm2s mode  : %5.1f %%", cg_mode.get_inst_coverage()),  UVM_LOW)
        `uvm_info("COV", $sformatf("burst cfg  : %5.1f %%", cg_cfg.get_inst_coverage()),   UVM_LOW)
        `uvm_info("COV", $sformatf("s2mm burst : %5.1f %%", cg_s2mm.get_inst_coverage()),  UVM_LOW)
        `uvm_info("COV", $sformatf("mm2s burst : %5.1f %%", cg_mm2s.get_inst_coverage()),  UVM_LOW)
        `uvm_info("COV", $sformatf("frame      : %5.1f %%", cg_frame.get_inst_coverage()), UVM_LOW)
        `uvm_info("COV", "================================", UVM_LOW)
    endfunction
endclass