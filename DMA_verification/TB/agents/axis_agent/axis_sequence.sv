// ============================================================================
// axis_sequence : 영상 프레임 sequence 모음
//   camera 용 : 프레임을 만들어서 보냄 (random / pattern / corner)
//   disp   용 : 프레임 한 장 받는 동안의 tready 패턴 (axis_ready_seq)
// ============================================================================
class axis_sequence extends uvm_sequence#(axis_item);
    `uvm_object_utils(axis_sequence)
    int unsigned frame_w = 16;     // tb_top DUT 파라미터와 같아야 함
    int unsigned frame_h = 4;
    int unsigned gap_max = 0;      // 0 = 쉬지 않고 연속

    function new(string name = "axis_sequence");
        super.new(name);
    endfunction

    // 픽셀 배열을 받아서 프레임 하나 보내기
    task send_frame(logic [23:0] pix[]);
        axis_item item;
        item = axis_item::type_id::create("item");
        start_item(item);
        item.width          = frame_w;
        item.height         = frame_h;
        item.pixels         = pix;
        item.tvalid_gap_max = gap_max;
        finish_item(item);
    endtask

    // 한 색으로 꽉 찬 프레임 만들기
    function void make_solid(ref logic [23:0] pix[], input logic [23:0] color);
        pix = new[frame_w * frame_h];
        foreach (pix[i]) pix[i] = color;
    endfunction
endclass


// ----------------------------------------------------------------------------
// 랜덤 프레임 num 장 (camera)
// ----------------------------------------------------------------------------
class axis_random_seq extends axis_sequence;
    `uvm_object_utils(axis_random_seq)
    rand int num;
    constraint c_num { soft num inside {[1:5]}; }

    function new(string name = "axis_random_seq");
        super.new(name);
    endfunction

    task body();
        axis_item item;
        `uvm_info(get_type_name(), $sformatf("랜덤 프레임 시나리오 시작 (%0d장)", num), UVM_LOW)
        repeat (num) begin
            item = axis_item::type_id::create("item");
            start_item(item);
            if (!item.randomize() with { width          == local::frame_w;
                                         height         == local::frame_h;
                                         tvalid_gap_max <= local::gap_max; })
                `uvm_error("SEQ", "randomize 실패")
            finish_item(item);
        end
        `uvm_info(get_type_name(), "랜덤 프레임 시나리오 종료", UVM_LOW)
    endtask
endclass


// ----------------------------------------------------------------------------
// 패턴 프레임 num 장 (camera) : 픽셀 값 = {프레임번호, y, x}
//   틀렸을 때 로그만 봐도 "몇 번째 프레임의 어느 좌표"인지 바로 보임 -> 디버깅용
// ----------------------------------------------------------------------------
class axis_pattern_seq extends axis_sequence;
    `uvm_object_utils(axis_pattern_seq)
    rand int num;
    constraint c_num { soft num == 3; }

    function new(string name = "axis_pattern_seq");
        super.new(name);
    endfunction

    task body();
        logic [23:0] pix[];
        `uvm_info(get_type_name(), $sformatf("패턴 프레임 시나리오 시작 (%0d장)", num), UVM_LOW)
        for (int f = 0; f < num; f++) begin
            pix = new[frame_w * frame_h];
            for (int y = 0; y < frame_h; y++)
                for (int x = 0; x < frame_w; x++)
                    pix[y*frame_w + x] = {8'(f), 8'(y), 8'(x)};
            send_frame(pix);
        end
        `uvm_info(get_type_name(), "패턴 프레임 시나리오 종료", UVM_LOW)
    endtask
endclass


// ----------------------------------------------------------------------------
// 경계값 프레임 (camera) : 000000 / FFFFFF / 555555 / AAAAAA 단색 프레임
// ----------------------------------------------------------------------------
class axis_corner_seq extends axis_sequence;
    `uvm_object_utils(axis_corner_seq)

    function new(string name = "axis_corner_seq");
        super.new(name);
    endfunction

    task body();
        logic [23:0] corner_vals[$] = {24'h000000, 24'hFFFFFF, 24'h555555, 24'hAAAAAA};
        logic [23:0] pix[];
        `uvm_info(get_type_name(), $sformatf("경계값 프레임 시나리오 시작 (%0d장)", corner_vals.size()), UVM_LOW)
        foreach (corner_vals[i]) begin
            make_solid(pix, corner_vals[i]);
            send_frame(pix);
        end
        `uvm_info(get_type_name(), "경계값 프레임 시나리오 종료", UVM_LOW)
    endtask
endclass


// ----------------------------------------------------------------------------
// disp 용 : 프레임 num 장 받는 동안 tready 구동
//   gap_max = 0 : tready 항상 1 / gap_max > 0 : 랜덤하게 0~gap_max cycle 씩 내림
// ----------------------------------------------------------------------------
class axis_ready_seq extends axis_sequence;
    `uvm_object_utils(axis_ready_seq)
    int num = 1000;     // 넉넉하게 (test 끝나면 알아서 멈춤)

    function new(string name = "axis_ready_seq");
        super.new(name);
    endfunction

    task body();
        axis_item item;
        `uvm_info(get_type_name(), $sformatf("disp 수신 시작 (tready gap_max=%0d)", gap_max), UVM_LOW)
        repeat (num) begin
            item = axis_item::type_id::create("item");
            start_item(item);
            item.width          = frame_w;
            item.height         = frame_h;
            item.tvalid_gap_max = gap_max;    // slave 모드에서는 tready 내리는 최대 cycle
            finish_item(item);
        end
    endtask
endclass


// ----------------------------------------------------------------------------
// 단색 프레임 1장 (camera) : test 에서 경계값을 한 장씩 보낼 때 사용
// ----------------------------------------------------------------------------
class axis_solid_seq extends axis_sequence;
    `uvm_object_utils(axis_solid_seq)
    logic [23:0] color = 24'h000000;

    function new(string name = "axis_solid_seq");
        super.new(name);
    endfunction

    task body();
        logic [23:0] pix[];
        make_solid(pix, color);
        send_frame(pix);
        `uvm_info(get_type_name(), $sformatf("단색 프레임 전송 (0x%06h)", color), UVM_LOW)
    endtask
endclass