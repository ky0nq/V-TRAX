// ============================================================================
// mm2s_driver : TB = AXI4 read slave (메모리 역할)
//   item 하나 = 다음에 들어올 AR 버스트 하나를 "어떻게 응답할지"
//     arready_delay : AR 받기 전 대기 cycle
//     rvalid_gap    : R beat 사이 쉬는 cycle
//     rresp_inj     : 강제로 넣을 응답 (00 = OKAY)
//   데이터는 mem(dma_mem_model) 에서 꺼냄
//   ※ 버스트를 하나씩 순서대로 처리 (R 끝날 때까지 다음 AR 안 받음)
// ============================================================================
class mm2s_driver extends uvm_driver#(mm2s_item);
    `uvm_component_utils(mm2s_driver)
    virtual mm2s_interface mm2s_vif;
    dma_mem_model     mem;     // env 가 connect_phase 에서 넣어줌

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual mm2s_interface)::get(this, "", "mm2s_vif", mm2s_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
    endfunction

    task run_phase(uvm_phase phase);
        if (mem == null)
            `uvm_fatal(get_type_name(), "mem 이 연결되지 않음 (env connect_phase 확인)")

        // 초기화 (slave 가 구동하는 신호 전부 0)
        mm2s_vif.drv_cb.arready <= 0;
        mm2s_vif.drv_cb.rid     <= 0;
        mm2s_vif.drv_cb.rdata   <= 0;
        mm2s_vif.drv_cb.rresp   <= 0;
        mm2s_vif.drv_cb.rlast   <= 0;
        mm2s_vif.drv_cb.rvalid  <= 0;

        wait (mm2s_vif.rst_n === 1'b1);
        @(mm2s_vif.drv_cb);

        forever begin
            seq_item_port.get_next_item(req);
            respond_burst(req);
            seq_item_port.item_done();
        end
    endtask

    task respond_burst(mm2s_item req);
        bit [31:0] a;

        // ---------------- AR ----------------
        do @(mm2s_vif.drv_cb); while (mm2s_vif.drv_cb.arvalid !== 1'b1);
        repeat (req.arready_delay) @(mm2s_vif.drv_cb);
        mm2s_vif.drv_cb.arready <= 1;
        @(mm2s_vif.drv_cb);                       // 이 edge 에서 handshake
        req.arid    = mm2s_vif.drv_cb.arid;
        req.araddr  = mm2s_vif.drv_cb.araddr;
        req.arlen   = mm2s_vif.drv_cb.arlen;
        req.arsize  = mm2s_vif.drv_cb.arsize;
        req.arburst = mm2s_vif.drv_cb.arburst;
        mm2s_vif.drv_cb.arready <= 0;

        // ---------------- R ----------------
        for (int unsigned i = 0; i <= req.arlen; i++) begin
            if (i > 0 && req.rvalid_gap > 0) begin
                mm2s_vif.drv_cb.rvalid <= 0;
                repeat (req.rvalid_gap) @(mm2s_vif.drv_cb);
            end
            a = mem.beat_addr(req.araddr, req.arlen, req.arsize, req.arburst, i);

            mm2s_vif.drv_cb.rid    <= req.arid;
            mm2s_vif.drv_cb.rdata  <= mem.read_word(a);
            mm2s_vif.drv_cb.rresp  <= mem.is_err_addr(a) ? 2'b10 : req.rresp_inj;
            mm2s_vif.drv_cb.rlast  <= (i == req.arlen);
            mm2s_vif.drv_cb.rvalid <= 1;
            do @(mm2s_vif.drv_cb); while (mm2s_vif.drv_cb.rready !== 1'b1);
        end
        mm2s_vif.drv_cb.rvalid <= 0;
        mm2s_vif.drv_cb.rlast  <= 0;

        `uvm_info(get_type_name(), $sformatf("응답완료: %s", req.convert2string()), UVM_HIGH)
    endtask
endclass