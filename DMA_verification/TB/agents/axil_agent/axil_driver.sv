// ============================================================================
// axil_driver : TB = AXI4-Lite master (CPU 역할)
//   item 하나 = 레지스터 read 또는 write 한 번
// ============================================================================
class axil_driver extends uvm_driver#(axil_item);
    `uvm_component_utils(axil_driver)
    virtual axil_interface axil_vif;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual axil_interface)::get(this, "", "axil_vif", axil_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
    endfunction

    task run_phase(uvm_phase phase);
        // 초기화 (master 가 구동하는 신호 전부 0)
        axil_vif.drv_cb.awaddr  <= 0;
        axil_vif.drv_cb.awprot  <= 0;
        axil_vif.drv_cb.awvalid <= 0;
        axil_vif.drv_cb.wdata   <= 0;
        axil_vif.drv_cb.wstrb   <= 0;
        axil_vif.drv_cb.wvalid  <= 0;
        axil_vif.drv_cb.bready  <= 0;
        axil_vif.drv_cb.araddr  <= 0;
        axil_vif.drv_cb.arprot  <= 0;
        axil_vif.drv_cb.arvalid <= 0;
        axil_vif.drv_cb.rready  <= 0;

        // 리셋 풀릴 때까지 대기
        wait (axil_vif.rst_n === 1'b1);
        @(axil_vif.drv_cb);

        forever begin
            seq_item_port.get_next_item(req);
            if (req.is_write) drive_write(req);
            else              drive_read(req);
            seq_item_port.item_done();
        end
    endtask

    // ------------------------------------------------------------------
    // write : AW 와 W 를 동시에 올리고 각각 handshake 되면 내림 -> B 받기
    // ------------------------------------------------------------------
    task drive_write(axil_item req);
        axil_vif.drv_cb.awaddr  <= req.addr;    // 어디에 쓸지
        axil_vif.drv_cb.awprot  <= 3'b000;      // 보호 속성(안 씀: 0)
        axil_vif.drv_cb.awvalid <= 1;           // 주소가 준비됐음
        axil_vif.drv_cb.wdata   <= req.wdata;   // 뭘 쓸지
        axil_vif.drv_cb.wstrb   <= req.wstrb;   // 몇 번째 바이트를 쓸지 
        axil_vif.drv_cb.wvalid  <= 1;           // 데이터 준비됨

        fork // 두 쪽이 모두 끝나야 넘어감 
            begin   // AW handshake
                do @(axil_vif.drv_cb); while (axil_vif.drv_cb.awready !== 1'b1);
                axil_vif.drv_cb.awvalid <= 0;
            end
            begin   // W handshake
                do @(axil_vif.drv_cb); while (axil_vif.drv_cb.wready !== 1'b1);
                axil_vif.drv_cb.wvalid <= 0;
            end
        join

        // B 응답 받기
        axil_vif.drv_cb.bready <= 1; // 응답 받을 준비됨
        do @(axil_vif.drv_cb); while (axil_vif.drv_cb.bvalid !== 1'b1); // dut가 응답 줄 때까지 
        req.resp = axil_vif.drv_cb.bresp;   // 응답 코드 저장
        axil_vif.drv_cb.bready <= 0;    // 다 받음 -> 내림 

        `uvm_info(get_type_name(), $sformatf("구동완료: %s", req.convert2string()), UVM_HIGH)
    endtask

    // ------------------------------------------------------------------
    // read : AR 올리고 handshake -> R 받기
    // ------------------------------------------------------------------
    task drive_read(axil_item req);
        axil_vif.drv_cb.araddr  <= req.addr;                                // 어디를 읽을지
        axil_vif.drv_cb.arprot  <= 3'b000;                                  // 안 씀: 0
        axil_vif.drv_cb.arvalid <= 1;                                       // 주소 준비됐음
        do @(axil_vif.drv_cb); while (axil_vif.drv_cb.arready !== 1'b1);    // dut가 받아갈 때까지
        axil_vif.drv_cb.arvalid <= 0;

        axil_vif.drv_cb.rready <= 1;                                        // 데이터 받을 준비됨
        do @(axil_vif.drv_cb); while (axil_vif.drv_cb.rvalid !== 1'b1);     // dut가 데이터 줄 때까지
        req.rdata = axil_vif.drv_cb.rdata;                                  // 읽은 값 저장
        req.resp  = axil_vif.drv_cb.rresp;                                  // 응답 코드 저장
        axil_vif.drv_cb.rready <= 0;

        `uvm_info(get_type_name(), $sformatf("구동완료: %s", req.convert2string()), UVM_HIGH)
    endtask
endclass