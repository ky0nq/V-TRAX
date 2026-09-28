class axil_agent extends uvm_agent;
    `uvm_component_utils(axil_agent)
    uvm_sequencer #(axil_item) sqr;
    axil_driver  drv;
    axil_monitor mon;

    function new(string name = "axil_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        sqr = uvm_sequencer#(axil_item)::type_id::create("sqr", this);
        drv = axil_driver::type_id::create("drv", this); // axil driver 안에 있는 type_id에게 이름은 drv,부모는 나로 해서 객체를 하나 만들어라
        mon = axil_monitor::type_id::create("mon", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
endclass