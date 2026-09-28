// ============================================================================
// axis_driver
//   is_master = 1 (camera) : item(프레임) 하나를 픽셀 단위로 전송
//                             tuser = 프레임 첫 픽셀(SOF), tlast = 줄 끝(EOL)
//   is_master = 0 (disp)   : item 하나 = 프레임 한 장 받는 동안의 tready 패턴
//                             tvalid_gap_max 를 "tready 내리는 최대 cycle" 로 사용
// ============================================================================
class axis_driver extends uvm_driver#(axis_item);
    `uvm_component_utils(axis_driver)
    virtual axis_interface axis_vif;
    bit is_master = 1;

    function new(string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual axis_interface)::get(this, "", "axis_vif", axis_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
        void'(uvm_config_db#(bit)::get(this, "", "is_master", is_master));
        `uvm_info(get_type_name(), $sformatf("mode = %s", is_master ? "MASTER" : "SLAVE"), UVM_MEDIUM)
    endfunction

    task run_phase(uvm_phase phase);
        // 초기화
        if (is_master) begin
            axis_vif.mst_cb.tdata  <= 0;
            axis_vif.mst_cb.tkeep  <= 0;
            axis_vif.mst_cb.tuser  <= 0;
            axis_vif.mst_cb.tlast  <= 0;
            axis_vif.mst_cb.tvalid <= 0;
        end
        else begin
            axis_vif.slv_cb.tready <= 0;
        end

        wait (axis_vif.rst_n === 1'b1);
        @(axis_vif.mst_cb);

        forever begin
            seq_item_port.get_next_item(req);
            if (is_master) drive_frame(req);
            else           accept_frame(req);
            seq_item_port.item_done();
        end
    endtask

    // ------------------------------------------------------------------
    // master : 프레임 전송
    // ------------------------------------------------------------------
    task drive_frame(axis_item req);
        int unsigned gap;
        for (int unsigned y = 0; y < req.height; y++) begin
            for (int unsigned x = 0; x < req.width; x++) begin
                axis_vif.mst_cb.tdata  <= req.get_pixel(x, y);
                axis_vif.mst_cb.tkeep  <= '1;
                axis_vif.mst_cb.tuser  <= (x == 0 && y == 0);      // SOF
                axis_vif.mst_cb.tlast  <= (x == req.width - 1);    // EOL
                axis_vif.mst_cb.tvalid <= 1;
                do @(axis_vif.mst_cb); while (axis_vif.mst_cb.tready !== 1'b1);

                // 랜덤하게 tvalid 쉬기
                gap = $urandom_range(req.tvalid_gap_max, 0);
                if (gap > 0) begin
                    axis_vif.mst_cb.tvalid <= 0;
                    repeat (gap) @(axis_vif.mst_cb);
                end
            end
        end
        axis_vif.mst_cb.tvalid <= 0;
        axis_vif.mst_cb.tuser  <= 0;
        axis_vif.mst_cb.tlast  <= 0;

        `uvm_info(get_type_name(), $sformatf("구동완료: %s", req.convert2string()), UVM_HIGH)
    endtask

    // ------------------------------------------------------------------
    // slave : 프레임 한 장(width*height beat) 받는 동안 tready 구동
    // ------------------------------------------------------------------
    task accept_frame(axis_item req);
        int unsigned gap;
        repeat (req.width * req.height) begin
            gap = $urandom_range(req.tvalid_gap_max, 0);
            if (gap > 0) begin
                axis_vif.slv_cb.tready <= 0;
                repeat (gap) @(axis_vif.slv_cb);
            end
            axis_vif.slv_cb.tready <= 1;
            do @(axis_vif.slv_cb); while (axis_vif.slv_cb.tvalid !== 1'b1);
        end
        axis_vif.slv_cb.tready <= 0;

        `uvm_info(get_type_name(), $sformatf("수신완료: %0dx%0d beats", req.width, req.height), UVM_HIGH)
    endtask
endclass