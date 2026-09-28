// camera_agt, disp_agt 둘 다 이 클래스 (env 에서 is_master 로 구분)
class axis_agent extends uvm_agent;
    `uvm_component_utils(axis_agent)
    uvm_sequencer #(axis_item) sqr;
    axis_driver  drv;
    axis_monitor mon;

    function new(string name = "axis_agent", uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        sqr = uvm_sequencer#(axis_item)::type_id::create("sqr", this);
        drv = axis_driver::type_id::create("drv", this);
        mon = axis_monitor::type_id::create("mon", this);
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
endclass