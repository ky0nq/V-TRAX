// ============================================================================
// s2mm_monitor : AW -> W beats (wlast 까지) -> B 순서로 버스트 하나를 모아서 보냄
//   aw_q : W 를 기다리는 AW
//   b_q  : W 는 다 받았고 B 를 기다리는 버스트
// ============================================================================
class s2mm_monitor extends uvm_monitor;
    `uvm_component_utils(s2mm_monitor)
    virtual s2mm_interface s2mm_vif;
    uvm_analysis_port#(s2mm_item) ap;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual s2mm_interface)::get(this, "", "s2mm_vif", s2mm_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
    endfunction

    task run_phase(uvm_phase phase);
        s2mm_item tr;
        s2mm_item aw_q[$];
        s2mm_item b_q[$];

        wait (s2mm_vif.rst_n === 1'b1);
        forever begin
            @(s2mm_vif.mon_cb);

            // AW handshake
            if (s2mm_vif.mon_cb.awvalid && s2mm_vif.mon_cb.awready) begin
                tr         = s2mm_item::type_id::create("tr");
                tr.awid    = s2mm_vif.mon_cb.awid;
                tr.awaddr  = s2mm_vif.mon_cb.awaddr;
                tr.awlen   = s2mm_vif.mon_cb.awlen;
                tr.awsize  = s2mm_vif.mon_cb.awsize;
                tr.awburst = s2mm_vif.mon_cb.awburst;
                aw_q.push_back(tr);
            end

            // W handshake
            if (s2mm_vif.mon_cb.wvalid && s2mm_vif.mon_cb.wready) begin
                if (aw_q.size() == 0)
                    `uvm_error(get_type_name(), "AW 없이 W 가 들어옴")
                else begin
                    aw_q[0].wdata.push_back(s2mm_vif.mon_cb.wdata);
                    aw_q[0].wstrb.push_back(s2mm_vif.mon_cb.wstrb);
                    if (s2mm_vif.mon_cb.wlast) begin
                        tr = aw_q.pop_front();
                        if (tr.wdata.size() != tr.num_beats())
                            `uvm_error(get_type_name(), $sformatf("beat 수 불일치: 기대 %0d, 실제 %0d",
                                       tr.num_beats(), tr.wdata.size()))
                        b_q.push_back(tr);
                    end
                end
            end

            // B handshake
            if (s2mm_vif.mon_cb.bvalid && s2mm_vif.mon_cb.bready) begin
                if (b_q.size() == 0)
                    `uvm_error(get_type_name(), "W 완료 없이 B 가 들어옴")
                else begin
                    tr       = b_q.pop_front();
                    tr.bresp = s2mm_vif.mon_cb.bresp;
                    `uvm_info(get_type_name(), tr.convert2string(), UVM_HIGH)
                    ap.write(tr);
                end
            end
        end
    endtask
endclass