// ----------------------------------------------------------------------------
// LIVE + CYCLIC : MM2S 가 계속 도는 동안 camera 가 새 프레임을 계속 넣음
//   MM2S 는 프레임이 끝날 때마다 S2MM 이 가장 최근에 다 쓴 버퍼로 갈아탐 (VDMA 방식)
//   -> 나오는 프레임은 전부 "최근 camera 프레임 중 하나" 여야 함 (E2E)
// ----------------------------------------------------------------------------
class dma_cyclic_test extends dma_base_test;
    `uvm_component_utils(dma_cyclic_test)

    function new(string name = "dma_cyclic_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_scenario();
        int n = frames(6);
        s2mm_start();
        send_random_frame("cam_seq_0");
        wait_frame(0);
        mm2s_start(.live(1), .cyclic(1));

        for (int i = 1; i < n; i++) begin
            `uvm_info(get_type_name(), $sformatf("---- camera frame %0d / %0d (MM2S 는 계속 도는 중) ----", i + 1, n), UVM_LOW)
            send_random_frame($sformatf("cam_seq_%0d", i));
            wait_frame(0);
        end
        wait_frame(1);      // 마지막 프레임을 한 번은 읽고 나서
        stop_cyclic();
    endtask
endclass