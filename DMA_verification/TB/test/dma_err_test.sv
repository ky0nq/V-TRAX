// ----------------------------------------------------------------------------
// 응답 에러 : 메모리가 SLVERR/DECERR 를 주면 DUT 가 에러를 제대로 기록하는지
//   - factory override 로 basic_seq 대신 err_seq 가 만들어지게 함 (agent/env 코드 수정 없음)
//   - 확인 항목 (S2MM, MM2S 각각)
//       SR[4]  error 비트가 1
//       SR[14] Err_Irq 가 1 -> W1C 로 지우면 0
//       WRITE_ERR / READ_ERR = 에러 난 버스트의 시작 주소 (버퍼 시작 + err_burst * 64)
//   - MM2S 에러 뒤 출력 데이터는 정의돼 있지 않아서 MM2S 채점/E2E 는 끔
// ----------------------------------------------------------------------------
class dma_err_test extends dma_base_test;
    `uvm_component_utils(dma_err_test)
    localparam bit [31:0] DA0         = 32'h1000_0000;
    localparam int        BURST_BYTES = 64;

    function new(string name = "dma_err_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        mm2s_basic_seq::type_id::set_type_override(mm2s_err_seq::get_type());
        s2mm_basic_seq::type_id::set_type_override(s2mm_err_seq::get_type());
        super.build_phase(phase);
        uvm_config_db#(bit)::set(this, "env.sb", "check_e2e",  0);
        uvm_config_db#(bit)::set(this, "env.sb", "check_mm2s", 0);
    endfunction

    task run_scenario();
        mm2s_err_seq me;
        s2mm_err_seq se;
        if (!$cast(me, mm2s_bg)) `uvm_fatal(get_type_name(), "mm2s err_seq override 실패")
        if (!$cast(se, s2mm_bg)) `uvm_fatal(get_type_name(), "s2mm err_seq override 실패")

        // ---------------- S2MM ----------------
        `uvm_info(get_type_name(), $sformatf("---- S2MM : %0d번째 버스트 에러 ----", se.err_burst), UVM_LOW)
        s2mm_start(.da0(DA0));
        send_random_frame("cam_seq_0");
        wait_bit(S2MM_SR, axil_sequence::SR_ERR_IRQ);
        reg_check(S2MM_SR,   32'h0000_0010, 32'h0000_0010, "S2MM SR.error");
        reg_check(WRITE_ERR, 32'hFFFF_FFFF, DA0 + se.err_burst * BURST_BYTES, "WRITE_ERR 주소");
        reg_write(S2MM_SR, 32'h1 << axil_sequence::SR_ERR_IRQ);                 // W1C
        reg_check(S2MM_SR,   32'h0000_4000, 32'h0, "S2MM Err_Irq W1C 로 지워짐");

        // ---------------- MM2S ----------------
        // S2MM 이 frame0 을 buf0 에 다 쓸 때까지 기다린 뒤 LIVE 로 읽음 -> 에러 주소도 buf0 기준
        wait_frame(0);
        `uvm_info(get_type_name(), $sformatf("---- MM2S : %0d번째 버스트 에러 ----", me.err_burst), UVM_LOW)
        mm2s_start(.live(1));
        wait_bit(MM2S_SR, axil_sequence::SR_ERR_IRQ);
        reg_check(MM2S_SR,  32'h0000_0010, 32'h0000_0010, "MM2S SR.error");
        reg_check(READ_ERR, 32'hFFFF_FFFF, DA0 + me.err_burst * BURST_BYTES, "READ_ERR 주소");
        reg_write(MM2S_SR, 32'h1 << axil_sequence::SR_ERR_IRQ);                 // W1C
        reg_check(MM2S_SR,  32'h0000_4000, 32'h0, "MM2S Err_Irq W1C 로 지워짐");
    endtask
endclass