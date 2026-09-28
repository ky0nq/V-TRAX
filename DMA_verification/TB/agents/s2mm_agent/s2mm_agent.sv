class s2mm_agent extends uvm_agent;
    `uvm_component_utils(s2mm_agent)
    uvm_sequencer #(s2mm_item) sqr;
    s2mm_driver  drv;
    s2mm_monitor mon;

    function new(string name = "s2mm_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        sqr = uvm_sequencer#(s2mm_item)::type_id::create("sqr", this);
        drv = s2mm_driver::type_id::create("drv", this);
        mon = s2mm_monitor::type_id::create("mon", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
        // drv.mem 은 env 의 connect_phase 에서 넣어줌
    endfunction
endclass