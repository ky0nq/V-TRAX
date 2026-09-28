// ----------------------------------------------------------------------------
// 레지스터 read/write
// ----------------------------------------------------------------------------
class dma_reg_test extends dma_base_test;
    `uvm_component_utils(dma_reg_test)

    function new(string name = "dma_reg_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_scenario();
        axil_reg_rw_seq s = axil_reg_rw_seq::type_id::create("reg_seq");
        if (!s.randomize()) `uvm_error(get_type_name(), "randomize 실패")
        s.start(env.axil_agt.sqr);
    endtask
endclass