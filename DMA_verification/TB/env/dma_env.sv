// ============================================================================
// dma_env : agent 5개 + memory model + scoreboard + coverage 를 만들고 연결
// ============================================================================
class dma_env extends uvm_env;
    `uvm_component_utils(dma_env)

    axil_agent     axil_agt;
    axis_agent     camera_agt;     // axis_agent 클래스 하나를 두 번 씀
    axis_agent     disp_agt;
    mm2s_agent     mm2s_agt;
    s2mm_agent     s2mm_agt;

    dma_mem_model  mem;
    dma_scoreboard sb;
    dma_coverage   cov;

    function new(string name = "dma_env", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);

        // camera = master(픽셀 보냄), disp = slave(tready 만)
        // ※ agent 만들기 전에 넣어야 driver 의 build_phase 에서 받을 수 있음
        uvm_config_db#(bit)::set(this, "camera_agt*", "is_master", 1);
        uvm_config_db#(bit)::set(this, "disp_agt*",   "is_master", 0);

        // 이름이 tb_top config_db 경로(*axil_agt* 등)와 같아야 vif 를 받음
        axil_agt   = axil_agent::type_id::create("axil_agt",   this);
        camera_agt = axis_agent::type_id::create("camera_agt", this);
        disp_agt   = axis_agent::type_id::create("disp_agt",   this);
        mm2s_agt   = mm2s_agent::type_id::create("mm2s_agt",   this);
        s2mm_agt   = s2mm_agent::type_id::create("s2mm_agt",   this);

        mem = dma_mem_model::type_id::create("mem");
        sb  = dma_scoreboard::type_id::create("sb",  this);
        cov = dma_coverage::type_id::create("cov", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);

        // memory model 을 두 driver 가 같이 씀
        mm2s_agt.drv.mem = mem;
        s2mm_agt.drv.mem = mem;

        // monitor -> scoreboard
        camera_agt.mon.ap.connect(sb.cam_imp);
        disp_agt.mon.ap.connect(sb.disp_imp);
        axil_agt.mon.ap.connect(sb.axil_imp);
        s2mm_agt.mon.ap.connect(sb.s2mm_imp);
        mm2s_agt.mon.ap.connect(sb.mm2s_imp);

        // monitor -> coverage (analysis port 는 여러 곳에 동시에 연결 가능)
        camera_agt.mon.ap.connect(cov.cam_imp);
        axil_agt.mon.ap.connect(cov.axil_imp);
        s2mm_agt.mon.ap.connect(cov.s2mm_imp);
        mm2s_agt.mon.ap.connect(cov.mm2s_imp);
    endfunction

    function void report_phase(uvm_phase phase);
        mem.report();
    endfunction
endclass