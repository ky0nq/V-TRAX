// ============================================================================
// axil_monitor : AXI4-Lite 버스를 보고 read/write 한 번마다 item 하나를 만듦
//   write : AW + W handshake 후 B handshake 에서 완성
//   read  : AR handshake 후 R handshake 에서 완성
// ============================================================================
class axil_monitor extends uvm_monitor;
    `uvm_component_utils(axil_monitor)
    virtual axil_interface axil_vif;
    uvm_analysis_port#(axil_item) ap;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual axil_interface)::get(this, "", "axil_vif", axil_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
    endfunction

    task run_phase(uvm_phase phase);
        wait (axil_vif.rst_n === 1'b1);
        fork
            collect_write();
            collect_read();
            watch_irq();
        join
    endtask

    task collect_write();
        axil_item tr;
        bit got_aw, got_w;
        forever begin
            tr     = axil_item::type_id::create("tr");  // 새 기록지 준비
            tr.is_write = 1;                            // 쓰기라고 표시
            got_aw = 0;                                 // 주소 아직 못 봄
            got_w  = 0;                                 // 데이터 아직 못 봄
            // AW, W 는 순서가 다를 수 있어서 둘 다 올 때까지 봄
            while (!(got_aw && got_w)) begin            // 둘 다 볼 때까지 봄
                @(axil_vif.mon_cb);                     // 클럭 하나 기다리고
                if (!got_aw && axil_vif.mon_cb.awvalid && axil_vif.mon_cb.awready) begin
                    tr.addr = axil_vif.mon_cb.awaddr;   // AW handshake -> 주소 기록
                    got_aw  = 1;
                end
                if (!got_w && axil_vif.mon_cb.wvalid && axil_vif.mon_cb.wready) begin
                    tr.wdata = axil_vif.mon_cb.wdata;   // W handshake -> 데이터 기록
                    tr.wstrb = axil_vif.mon_cb.wstrb;
                    got_w    = 1;
                end
            end
            do @(axil_vif.mon_cb); while (!(axil_vif.mon_cb.bvalid && axil_vif.mon_cb.bready));
            tr.resp = axil_vif.mon_cb.bresp;            // B handshake -> 응답 기록

            `uvm_info(get_type_name(), tr.convert2string(), UVM_HIGH)
            ap.write(tr);                               // 완성된 기록지를 scoreboard로 전송
        end
    endtask

    task collect_read();
        axil_item tr;
        forever begin
            do @(axil_vif.mon_cb); while (!(axil_vif.mon_cb.arvalid && axil_vif.mon_cb.arready));
            tr          = axil_item::type_id::create("tr");
            tr.is_write = 0;
            tr.addr     = axil_vif.mon_cb.araddr;       // AR handshake -> 주소 기록

            do @(axil_vif.mon_cb); while (!(axil_vif.mon_cb.rvalid && axil_vif.mon_cb.rready));
            tr.rdata = axil_vif.mon_cb.rdata;           // R handshake -> 데이터 기록
            tr.resp  = axil_vif.mon_cb.rresp;

            `uvm_info(get_type_name(), tr.convert2string(), UVM_HIGH)
            ap.write(tr);
        end
    endtask

    // irq는 일단 로그만 (나중에 scoreboard로 보내고 싶으면 port 추가)
    task watch_irq();
        bit mm2s_q, s2mm_q;
        forever begin
            @(axil_vif.mon_cb);
            if (axil_vif.mon_cb.mm2s_irq && !mm2s_q)
                `uvm_info(get_type_name(), "mm2s_irq 상승", UVM_MEDIUM)
            if (axil_vif.mon_cb.s2mm_irq && !s2mm_q)
                `uvm_info(get_type_name(), "s2mm_irq 상승", UVM_MEDIUM)
            mm2s_q = axil_vif.mon_cb.mm2s_irq;
            s2mm_q = axil_vif.mon_cb.s2mm_irq;
        end
    endtask
endclass