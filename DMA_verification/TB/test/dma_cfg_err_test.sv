// ----------------------------------------------------------------------------
// 설정 오류 : MM2S 에 잘못된 설정을 주면 DUT 가 시작하지 않고 에러를 내는지
//   (a) SA 가 4byte 정렬 안 됨      (b) BTT 가 4 의 배수 아님
//   (c) BURST 타입이 WRAP(지원 안 함) (d) SA 가 DDR/BRAM 어디에도 없는 주소
//   각 경우 SR.error 가 뜨는지 확인 -> Err_Irq W1C 로 지움 -> 다음 경우
//   마지막에 정상 설정으로 한 번 더 돌려서 에러 뒤에도 정상 동작하는지 확인
// ----------------------------------------------------------------------------
class dma_cfg_err_test extends dma_base_test;
    `uvm_component_utils(dma_cfg_err_test)

    function new(string name = "dma_cfg_err_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        uvm_config_db#(bit)::set(this, "env.sb", "check_e2e", 0);
    endfunction

    task expect_cfg_err(string what);
        bit [31:0] ea;
        wait_bit(MM2S_SR, axil_sequence::SR_ERROR, 200);
        reg_check(MM2S_SR, 32'h0000_0010, 32'h0000_0010, $sformatf("%s -> SR.error", what));
        reg_read(READ_ERR, ea);
        `uvm_info(get_type_name(), $sformatf("%s : READ_ERR = 0x%08h", what, ea), UVM_LOW)
        reg_write(MM2S_SR, (32'h1 << axil_sequence::SR_ERR_IRQ) | (32'h1 << axil_sequence::SR_IOC_IRQ));
    endtask

    task run_scenario();
        bit [31:0] sa = 32'h8000_0000;
        int        fb = frame_w * frame_h * 3;

        `uvm_info(get_type_name(), "---- (a) SA 정렬 안 됨 ----", UVM_LOW)
        mm2s_start(.live(0), .sa(sa + 2));
        expect_cfg_err("SA 비정렬");

        `uvm_info(get_type_name(), "---- (b) BTT 4 의 배수 아님 ----", UVM_LOW)
        mm2s_start(.live(0), .sa(sa), .btt(fb - 2));
        expect_cfg_err("BTT 비정렬");

        `uvm_info(get_type_name(), "---- (c) WRAP 버스트 ----", UVM_LOW)
        mm2s_start(.live(0), .sa(sa), .burst(2'b10));
        expect_cfg_err("WRAP 버스트");

        `uvm_info(get_type_name(), "---- (d) 영역 밖 주소 ----", UVM_LOW)
        mm2s_start(.live(0), .sa(32'h5000_0000));
        expect_cfg_err("영역 밖 SA");

        `uvm_info(get_type_name(), "---- 에러 뒤 정상 동작 ----", UVM_LOW)
        env.mem.preload(sa, fb / 4, 1, $urandom());
        mm2s_start(.live(0), .sa(sa));
        wait_frame(1);
        reg_check(MM2S_SR, 32'h0000_0010, 32'h0, "정상 설정 후 SR.error 가 0");
    endtask
endclass