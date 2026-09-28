// ----------------------------------------------------------------------------
// LIVE=0 : camera 없이 SA(BRAM 창 0x8000_0000)에 미리 채워둔 이미지를 읽어서 출력
//   1) 한 장만 (cyclic=0)
//   2) 반복 (cyclic=1) 으로 N 장 -> cyclic 끄고 정지 확인
//   camera 와 상관없는 데이터라 E2E 는 끔 (MM2S 채점 = 메모리 내용 그대로 나오는지)
// ----------------------------------------------------------------------------
class dma_live0_test extends dma_base_test;
    `uvm_component_utils(dma_live0_test)
    bit [31:0] sa = 32'h8000_0000;

    function new(string name = "dma_live0_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        uvm_config_db#(bit)::set(this, "env.sb", "check_e2e", 0);
    endfunction

    task run_scenario();
        int n = frames(3);
        env.mem.preload(sa, frame_w * frame_h * 3 / 4, 1, $urandom());

        `uvm_info(get_type_name(), "---- LIVE=0 한 장 ----", UVM_LOW)
        mm2s_start(.live(0), .cyclic(0), .sa(sa));
        wait_frame(1);

        `uvm_info(get_type_name(), $sformatf("---- LIVE=0 CYCLIC %0d 장 ----", n), UVM_LOW)
        mm2s_start(.live(0), .cyclic(1), .sa(sa));
        repeat (n) wait_frame(1);
        stop_cyclic();
    endtask
endclass