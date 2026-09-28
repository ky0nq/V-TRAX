// ============================================================================
// s2mm_driver : TB = AXI3 write slave (메모리 역할)
//   item 하나 = 다음에 들어올 AW 버스트 하나를 "어떻게 응답할지"
//     awready_delay : AW 받기 전 대기 cycle
//     wready_gap    : W beat 사이 wready 내리는 cycle
//     bvalid_delay  : 마지막 W 이후 B 주기까지 cycle
//     bresp_inj     : 강제로 넣을 응답 (00 = OKAY)
//   받은 데이터는 mem(dma_mem_model) 에 저장
//   ※ AW 를 먼저 받고 나서 W 를 받음 (AXI 규칙상 slave 가 이렇게 해도 됨)
// ============================================================================
class s2mm_driver extends uvm_driver#(s2mm_item);
    `uvm_component_utils(s2mm_driver)
    virtual s2mm_interface s2mm_vif;
    dma_mem_model     mem;     // env 가 connect_phase 에서 넣어줌

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual s2mm_interface)::get(this, "", "s2mm_vif", s2mm_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
    endfunction

    task run_phase(uvm_phase phase);
        if (mem == null)
            `uvm_fatal(get_type_name(), "mem 이 연결되지 않음 (env connect_phase 확인)")

        // 초기화 (slave 가 구동하는 신호 전부 0)
        s2mm_vif.drv_cb.awready <= 0;
        s2mm_vif.drv_cb.wready  <= 0;
        s2mm_vif.drv_cb.bid     <= 0;
        s2mm_vif.drv_cb.bresp   <= 0;
        s2mm_vif.drv_cb.bvalid  <= 0;

        wait (s2mm_vif.rst_n === 1'b1);
        @(s2mm_vif.drv_cb);

        forever begin
            seq_item_port.get_next_item(req);
            respond_burst(req);
            seq_item_port.item_done();
        end
    endtask

    task respond_burst(s2mm_item req);
        bit [31:0] a;
        bit [31:0] data;
        bit [3:0]  strb;
        bit        last;
        bit        err = 0;

        // ---------------- AW ----------------
        do @(s2mm_vif.drv_cb); while (s2mm_vif.drv_cb.awvalid !== 1'b1);
        repeat (req.awready_delay) @(s2mm_vif.drv_cb);
        s2mm_vif.drv_cb.awready <= 1;
        @(s2mm_vif.drv_cb);                       // 이 edge 에서 handshake
        req.awid    = s2mm_vif.drv_cb.awid;
        req.awaddr  = s2mm_vif.drv_cb.awaddr;
        req.awlen   = s2mm_vif.drv_cb.awlen;
        req.awsize  = s2mm_vif.drv_cb.awsize;
        req.awburst = s2mm_vif.drv_cb.awburst;
        s2mm_vif.drv_cb.awready <= 0;

        // ---------------- W ----------------
        for (int unsigned i = 0; i <= req.awlen; i++) begin
            if (i > 0 && req.wready_gap > 0) begin
                s2mm_vif.drv_cb.wready <= 0;
                repeat (req.wready_gap) @(s2mm_vif.drv_cb);
            end
            s2mm_vif.drv_cb.wready <= 1;
            do @(s2mm_vif.drv_cb); while (s2mm_vif.drv_cb.wvalid !== 1'b1);

            data = s2mm_vif.drv_cb.wdata;
            strb = s2mm_vif.drv_cb.wstrb;
            last = s2mm_vif.drv_cb.wlast;
            a    = mem.beat_addr(req.awaddr, req.awlen, req.awsize, req.awburst, i);

            if (last != (i == req.awlen))
                `uvm_error(get_type_name(), $sformatf("wlast 위치 이상: beat %0d / len %0d, wlast=%0b",
                           i, req.awlen, last))

            if (mem.is_err_addr(a)) err = 1;     // 에러 영역이면 저장 안 함
            else                    mem.write_word(a, data, strb);

            req.wdata.push_back(data);
            req.wstrb.push_back(strb);
        end
        s2mm_vif.drv_cb.wready <= 0;

        // ---------------- B ----------------
        repeat (req.bvalid_delay) @(s2mm_vif.drv_cb);
        s2mm_vif.drv_cb.bid    <= req.awid;
        s2mm_vif.drv_cb.bresp  <= err ? 2'b10 : req.bresp_inj;
        s2mm_vif.drv_cb.bvalid <= 1;
        do @(s2mm_vif.drv_cb); while (s2mm_vif.drv_cb.bready !== 1'b1);
        req.bresp = err ? 2'b10 : req.bresp_inj;
        s2mm_vif.drv_cb.bvalid <= 0;

        `uvm_info(get_type_name(), $sformatf("응답완료: %s", req.convert2string()), UVM_HIGH)
    endtask
endclass