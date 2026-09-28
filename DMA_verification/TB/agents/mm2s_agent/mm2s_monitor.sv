// ============================================================================
// mm2s_monitor : AR handshake 를 큐에 넣고, R beat 를 모아서
//                rlast 가 오면 버스트 하나를 item 으로 완성해서 보냄
// ============================================================================
class mm2s_monitor extends uvm_monitor;
    `uvm_component_utils(mm2s_monitor)
    virtual mm2s_interface mm2s_vif;
    uvm_analysis_port#(mm2s_item) ap;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mm2s_interface)::get(this, "", "mm2s_vif", mm2s_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
    endfunction

    task run_phase(uvm_phase phase);
        mm2s_item tr;
        mm2s_item ar_q[$];     // R 을 기다리는 AR 들 (순서대로)

        wait (mm2s_vif.rst_n === 1'b1);
        forever begin
            @(mm2s_vif.mon_cb);

            // AR handshake
            if (mm2s_vif.mon_cb.arvalid && mm2s_vif.mon_cb.arready) begin
                tr         = mm2s_item::type_id::create("tr");
                tr.arid    = mm2s_vif.mon_cb.arid;
                tr.araddr  = mm2s_vif.mon_cb.araddr;
                tr.arlen   = mm2s_vif.mon_cb.arlen;
                tr.arsize  = mm2s_vif.mon_cb.arsize;
                tr.arburst = mm2s_vif.mon_cb.arburst;
                ar_q.push_back(tr);
            end

            // R handshake
            if (mm2s_vif.mon_cb.rvalid && mm2s_vif.mon_cb.rready) begin
                if (ar_q.size() == 0)
                    `uvm_error(get_type_name(), "AR 없이 R 이 들어옴")
                else begin
                    ar_q[0].rdata.push_back(mm2s_vif.mon_cb.rdata);
                    ar_q[0].rresp.push_back(mm2s_vif.mon_cb.rresp);
                    if (mm2s_vif.mon_cb.rlast) begin
                        tr = ar_q.pop_front();
                        if (tr.rdata.size() != tr.num_beats())
                            `uvm_error(get_type_name(), $sformatf("beat 수 불일치: 기대 %0d, 실제 %0d",
                                       tr.num_beats(), tr.rdata.size()))
                        `uvm_info(get_type_name(), tr.convert2string(), UVM_HIGH)
                        ap.write(tr);
                    end
                end
            end
        end
    endtask
endclass