// ----------------------------------------------------------------------------
// 경계값 : 000000 / FFFFFF / 555555 / AAAAAA 단색 프레임을 한 장씩 왕복
// ----------------------------------------------------------------------------
class dma_corner_test extends dma_base_test;
    `uvm_component_utils(dma_corner_test)

    function new(string name = "dma_corner_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_scenario();
        logic [23:0]   corner_vals[$] = {24'h000000, 24'hFFFFFF, 24'h555555, 24'hAAAAAA};
        axis_solid_seq s;
        s2mm_start();
        foreach (corner_vals[i]) begin
            s = axis_solid_seq::type_id::create($sformatf("cam_seq_%0d", i));
            s.color = corner_vals[i];
            one_frame(s);
        end
    endtask
endclass