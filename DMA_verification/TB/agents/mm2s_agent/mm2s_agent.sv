class mm2s_agent extends uvm_agent;
    `uvm_component_utils(mm2s_agent)
    uvm_sequencer #(mm2s_item) sqr;
    mm2s_driver  drv;
    mm2s_monitor mon;

    function new(string name = "mm2s_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        sqr = uvm_sequencer#(mm2s_item)::type_id::create("sqr", this);
        drv = mm2s_driver::type_id::create("drv", this);
        mon = mm2s_monitor::type_id::create("mon", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
        // drv.mem 은 env 의 connect_phase 에서 넣어줌
    endfunction
endclass