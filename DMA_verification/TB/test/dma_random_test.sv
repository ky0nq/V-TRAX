// ----------------------------------------------------------------------------
// 랜덤 : 랜덤 프레임 N 장(기본 6, +NUM_FRAMES 로 변경) + 메모리/camera/disp 랜덤 지연
// ----------------------------------------------------------------------------
class dma_random_test extends dma_base_test;
    `uvm_component_utils(dma_random_test)

    function new(string name = "dma_random_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        random_delay = 1;
        cam_gap      = 3;
        disp_gap     = 3;
        super.build_phase(phase);
    endfunction

    task run_scenario();
        axis_random_seq s;
        int n = frames(6);
        s2mm_start();
        for (int i = 0; i < n; i++) begin
            `uvm_info(get_type_name(), $sformatf("---- frame %0d / %0d ----", i + 1, n), UVM_LOW)
            s = axis_random_seq::type_id::create($sformatf("cam_seq_%0d", i));
            if (!s.randomize() with { num == 1; }) `uvm_error(get_type_name(), "randomize 실패")
            one_frame(s);
        end
    endtask
endclass