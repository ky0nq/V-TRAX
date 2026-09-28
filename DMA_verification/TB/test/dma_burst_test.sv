// ----------------------------------------------------------------------------
// 버스트 길이 : MM2S BURST_CFG 의 ARLEN 을 프레임마다 바꿔서 왕복
//   0(1beat), 1~14(짧은), 15(16beat), 16~255(DUT 가 16 으로 잘라야 함) 을 전부 거침
//   마지막 프레임은 FIXED 버스트 (같은 주소 반복 읽기) -> 출력이 camera 와 달라서 E2E 끔
// ----------------------------------------------------------------------------
class dma_burst_test extends dma_base_test;
    `uvm_component_utils(dma_burst_test)

    function new(string name = "dma_burst_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_scenario();
        bit [7:0] lens[$] = {8'd0, 8'd1, 8'd3, 8'd7, 8'd14, 8'd15, 8'd16, 8'd31, 8'd255};
        int n = frames(lens.size());
        bit [7:0] l;
        s2mm_start();
        for (int i = 0; i < n; i++) begin
            l = (i < lens.size()) ? lens[i] : 8'($urandom_range(255, 0));
            `uvm_info(get_type_name(), $sformatf("---- frame %0d : ARLEN 설정 %0d ----", i + 1, l), UVM_LOW)
            send_random_frame($sformatf("cam_seq_%0d", i));
            wait_frame(0);
            mm2s_start(.live(1), .arlen(l));
            wait_frame(1);
        end

        // FIXED 버스트 : 버스트마다 같은 주소를 반복해서 읽음 (MM2S 채점은 그대로, E2E 만 끔)
        `uvm_info(get_type_name(), "---- FIXED 버스트 ----", UVM_LOW)
        env.sb.check_e2e = 0;
        mm2s_start(.live(1), .arlen(8'd15), .burst(2'b00));
        wait_frame(1);
    endtask
endclass