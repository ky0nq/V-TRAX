// ----------------------------------------------------------------------------
// IDX_SW : S2MM 이 버퍼 0,1,2 를 다 채운 뒤, SW_IDX 로 읽을 버퍼를 직접 골라서 출력
//   sw_idx = 2, 0, 1 : 그 버퍼를 읽어야 함
//   sw_idx = 3       : NUM_BUF(3) 이상이라 DUT 가 0 번으로 바꿔야 함 (경계 조건)
//   매번 MM2S_SR[10:8] (cur_buf) 이 기대 버퍼와 같은지 확인
// ----------------------------------------------------------------------------
class dma_idx_sw_test extends dma_base_test;
    `uvm_component_utils(dma_idx_sw_test)

    function new(string name = "dma_idx_sw_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_scenario();
        bit [2:0] sel[$] = {3'd2, 3'd0, 3'd1, 3'd3};
        bit [2:0] exp_buf;
        s2mm_start();
        for (int i = 0; i < 3; i++) begin
            send_random_frame($sformatf("cam_seq_%0d", i));
            wait_frame(0);
        end
        foreach (sel[i]) begin
            exp_buf = (sel[i] < 3) ? sel[i] : 3'd0;
            `uvm_info(get_type_name(), $sformatf("---- SW_IDX=%0d -> buf%0d 기대 ----", sel[i], exp_buf), UVM_LOW)
            mm2s_start(.live(1), .idx_sw(1), .sw_idx(sel[i]));
            wait_frame(1);
            reg_check(MM2S_SR, 32'h0000_0700, {21'd0, exp_buf, 8'd0}, $sformatf("cur_buf (sw_idx=%0d)", sel[i]));
        end
    endtask
endclass