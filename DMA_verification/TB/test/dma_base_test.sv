// ============================================================================
// dma_base_test : env 생성 + 백그라운드 sequence(disp/mm2s/s2mm) + 공통 task
//   자식 test 는 run_scenario() 만 바꾸면 됨
//
//   plusarg
//     +NUM_FRAMES=N : 프레임 수를 쓰는 test 에서 기본값 대신 N 장
//   실행 : make TEST=dma_random_test PLUSARGS=+NUM_FRAMES=30
// ============================================================================
class dma_base_test extends uvm_test;
    `uvm_component_utils(dma_base_test)
    dma_env env;

    // ---------------- test 마다 바꾸는 설정 ----------------
    int unsigned frame_w      = 64;   // tb_top DUT 파라미터와 같아야 함 (64 의 배수)
    int unsigned frame_h      = 4;
    bit          random_delay = 0;    // 1 = mm2s/s2mm 메모리 응답을 랜덤하게 늦춤
    int unsigned cam_gap      = 0;    // camera tvalid 쉬는 최대 cycle
    int unsigned disp_gap     = 0;    // disp tready 내리는 최대 cycle
    int          num_frames   = 0;    // +NUM_FRAMES 로 받은 값 (0 이면 test 기본값 사용)

    // 백그라운드 sequence 핸들 (에러 test 에서 err_burst 를 알아내려고 저장)
    mm2s_sequence mm2s_bg;
    s2mm_sequence s2mm_bg;

    // 레지스터 주소 (axil_sequence 에 정의된 값 그대로)
    localparam bit [6:0] MM2S_CR   = axil_sequence::MM2S_CR;
    localparam bit [6:0] MM2S_SR   = axil_sequence::MM2S_SR;
    localparam bit [6:0] READ_ERR  = axil_sequence::READ_ERR;
    localparam bit [6:0] S2MM_SR   = axil_sequence::S2MM_SR;
    localparam bit [6:0] WRITE_ERR = axil_sequence::WRITE_ERR;

    function new(string name = "dma_base_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        void'($value$plusargs("NUM_FRAMES=%d", num_frames));
        uvm_config_db#(int unsigned)::set(this, "*", "width",  frame_w);
        uvm_config_db#(int unsigned)::set(this, "*", "height", frame_h);
        env = dma_env::type_id::create("env", this);
        uvm_top.set_timeout(20ms, 0);
    endfunction

    function void end_of_elaboration_phase(uvm_phase phase);
        uvm_top.print_topology();
    endfunction

    task run_phase(uvm_phase phase);
        uvm_objection obj = phase.get_objection();
        obj.set_drain_time(this, 2us);
        phase.raise_objection(this);
        start_background();
        run_scenario();
        phase.drop_objection(this);
    endtask

    virtual task run_scenario();
    endtask

    // plusarg 로 받은 프레임 수가 있으면 그걸, 없으면 test 기본값
    function int frames(int def);
        return (num_frames > 0) ? num_frames : def;
    endfunction

    // ------------------------------------------------------------------
    // 백그라운드 : slave 쪽 sequence 는 test 내내 돌아야 해서 join_none
    //   ※ create() 로 만들기 때문에 test 에서 factory override 하면 다른 seq 로 바뀜
    // ------------------------------------------------------------------
    task start_background();
        axis_ready_seq disp_s;
        disp_s = axis_ready_seq::type_id::create("disp_seq");
        disp_s.frame_w = frame_w;  disp_s.frame_h = frame_h;  disp_s.gap_max = disp_gap;

        if (random_delay) mm2s_bg = mm2s_random_seq::type_id::create("mm2s_seq");
        else              mm2s_bg = mm2s_basic_seq::type_id::create("mm2s_seq");
        if (random_delay) s2mm_bg = s2mm_random_seq::type_id::create("s2mm_seq");
        else              s2mm_bg = s2mm_basic_seq::type_id::create("s2mm_seq");
        void'(mm2s_bg.randomize());     // err_seq 로 바뀌었으면 err_burst / err_resp 가 정해짐
        void'(s2mm_bg.randomize());

        fork
            disp_s.start(env.disp_agt.sqr);
            mm2s_bg.start(env.mm2s_agt.sqr);
            s2mm_bg.start(env.s2mm_agt.sqr);
        join_none
    endtask

    // ------------------------------------------------------------------
    // 레지스터 한 개 읽기 / 쓰기 / 비트 기다리기 / 값 확인
    // ------------------------------------------------------------------
    task reg_write(bit [6:0] addr, bit [31:0] data);
        axil_write_seq s = axil_write_seq::type_id::create("reg_wr");
        s.addr = addr;  s.data = data;
        s.start(env.axil_agt.sqr);
    endtask

    task reg_read(bit [6:0] addr, output bit [31:0] data);
        axil_read_seq s = axil_read_seq::type_id::create("reg_rd");
        s.addr = addr;
        s.start(env.axil_agt.sqr);
        data = s.data;
    endtask

    task wait_bit(bit [6:0] addr, int bit_pos, int max_reads = 2000);
        axil_wait_bit_seq s = axil_wait_bit_seq::type_id::create("wait_bit");
        s.addr = addr;  s.bit_pos = bit_pos;  s.max_reads = max_reads;
        s.start(env.axil_agt.sqr);
    endtask

    // 읽은 값의 mask 부분이 기대값과 같은지 확인
    task reg_check(bit [6:0] addr, bit [31:0] mask, bit [31:0] exp, string what);
        bit [31:0] v;
        reg_read(addr, v);
        if ((v & mask) !== (exp & mask))
            `uvm_error(get_type_name(), $sformatf("CHECK FAIL [%s]: addr 0x%02h = 0x%08h (기대값 0x%08h, mask 0x%08h)",
                       what, addr, v, exp, mask))
        else
            `uvm_info(get_type_name(), $sformatf("CHECK PASS [%s]: 0x%08h", what, v & mask), UVM_LOW)
    endtask

    // ------------------------------------------------------------------
    // DMA 제어
    // ------------------------------------------------------------------
    task s2mm_start(bit [31:0] da0 = 32'h1000_0000, bit [31:0] da1 = 32'h1001_0000,
                    bit [31:0] da2 = 32'h1002_0000);
        axil_s2mm_start_seq s = axil_s2mm_start_seq::type_id::create("s2mm_start");
        s.da0 = da0;  s.da1 = da1;  s.da2 = da2;
        s.start(env.axil_agt.sqr);
    endtask

    task mm2s_start(bit live = 1, bit cyclic = 0, bit idx_sw = 0, bit [2:0] sw_idx = 0,
                    bit [31:0] sa = 32'h8000_0000, bit [7:0] arlen = 8'd15, bit [1:0] burst = 2'b01,
                    int btt = -1);
        axil_mm2s_start_seq s = axil_mm2s_start_seq::type_id::create("mm2s_start");
        s.live   = live;    s.cyclic = cyclic;
        s.idx_sw = idx_sw;  s.sw_idx = sw_idx;
        s.sa     = sa;      s.arlen  = arlen;   s.burst = burst;
        s.btt    = (btt < 0) ? frame_w * frame_h * 3 : btt;
        s.start(env.axil_agt.sqr);
    endtask

    task wait_frame(bit is_mm2s);
        axil_wait_frame_seq s = axil_wait_frame_seq::type_id::create("wait_frame");
        s.is_mm2s = is_mm2s;
        s.start(env.axil_agt.sqr);
    endtask

    // CYCLIC 끄고 MM2S 가 지금 프레임까지만 하고 멈출 때까지 대기
    task stop_cyclic();
        bit [31:0] cr;
        reg_read(MM2S_CR, cr);
        reg_write(MM2S_CR, cr & ~(32'h1 << axil_sequence::CR_CYCLIC));
        wait_bit(MM2S_SR, axil_sequence::SR_IDLE);
        `uvm_info(get_type_name(), "MM2S cyclic 정지 확인", UVM_LOW)
    endtask

    task send_camera(axis_sequence s);
        s.frame_w = frame_w;
        s.frame_h = frame_h;
        s.gap_max = cam_gap;
        s.start(env.camera_agt.sqr);
    endtask

    task send_random_frame(string name = "cam_seq");
        axis_random_seq s = axis_random_seq::type_id::create(name);
        if (!s.randomize() with { num == 1; }) `uvm_error(get_type_name(), "randomize 실패")
        send_camera(s);
    endtask

    // 프레임 한 장 왕복 : 넣기 -> S2MM 완료 -> MM2S(LIVE) -> MM2S 완료
    task one_frame(axis_sequence s, bit [7:0] arlen = 8'd15, bit [1:0] burst = 2'b01);
        send_camera(s);
        wait_frame(0);
        mm2s_start(.live(1), .arlen(arlen), .burst(burst));
        wait_frame(1);
    endtask
endclass