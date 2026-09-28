// ============================================================================
// axis_monitor : SOF(tuser) 가 뜬 beat 부터 width*height 개 픽셀을 모아서
//                프레임 item 하나로 보냄 (camera / disp 공통)
//   width, height 는 config_db 로 받음 (기본 16 x 4)
// ============================================================================
class axis_monitor extends uvm_monitor;
    `uvm_component_utils(axis_monitor)
    virtual axis_interface axis_vif;
    uvm_analysis_port#(axis_item) ap;
    int unsigned width  = 16;
    int unsigned height = 4;

    function new(string name, uvm_component parent);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual axis_interface)::get(this, "", "axis_vif", axis_vif))
            `uvm_fatal(get_type_name(), "virtual interface를 config_db에서 찾지 못함")
        void'(uvm_config_db#(int unsigned)::get(this, "", "width",  width));
        void'(uvm_config_db#(int unsigned)::get(this, "", "height", height));
    endfunction

    task run_phase(uvm_phase phase);
        axis_item    tr;
        int unsigned idx;

        wait (axis_vif.rst_n === 1'b1);
        forever begin
            // SOF 인 beat 기다림
            do @(axis_vif.mon_cb);
            while (!(axis_vif.mon_cb.tvalid && axis_vif.mon_cb.tready && axis_vif.mon_cb.tuser[0]));

            tr        = axis_item::type_id::create("tr");
            tr.width  = width;
            tr.height = height;
            tr.pixels = new[width * height];
            tr.sof_cnt = 1;
            tr.eol_cnt = axis_vif.mon_cb.tlast ? 1 : 0;
            tr.pixels[0] = axis_vif.mon_cb.tdata;
            idx = 1;

            // 나머지 픽셀
            while (idx < width * height) begin
                @(axis_vif.mon_cb);
                if (axis_vif.mon_cb.tvalid && axis_vif.mon_cb.tready) begin
                    if (axis_vif.mon_cb.tuser[0]) tr.sof_cnt++;   // 중간에 SOF 가 또 뜨면 이상한 것
                    if (axis_vif.mon_cb.tlast)    tr.eol_cnt++;
                    tr.pixels[idx] = axis_vif.mon_cb.tdata;
                    idx++;
                end
            end

            `uvm_info(get_type_name(), tr.convert2string(), UVM_HIGH)
            ap.write(tr);
        end
    endtask
endclass