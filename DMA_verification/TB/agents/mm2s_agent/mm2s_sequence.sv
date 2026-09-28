// ============================================================================
// mm2s_sequence : MM2S read slave(메모리)의 "응답 방식" sequence 모음
//   item 하나 = AR 버스트 하나에 어떻게 응답할지
//   ※ slave 라서 item 을 계속 넣어줘야 DUT 가 안 멈춤 -> num 을 넉넉하게
// ============================================================================
class mm2s_sequence extends uvm_sequence#(mm2s_item);
    `uvm_object_utils(mm2s_sequence)
    int num = 100000;

    function new(string name = "mm2s_sequence");
        super.new(name);
    endfunction

    task respond(int unsigned arready_delay, int unsigned rvalid_gap, bit [1:0] resp);
        mm2s_item item;
        item = mm2s_item::type_id::create("item");
        start_item(item);
        item.arready_delay = arready_delay;
        item.rvalid_gap    = rvalid_gap;
        item.rresp_inj     = resp;
        finish_item(item);
    endtask

    task respond_random();
        mm2s_item item;
        item = mm2s_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with { rresp_inj == 2'b00; })
            `uvm_error("SEQ", "randomize 실패")
        finish_item(item);
    endtask
endclass


// 기본 : 지연 없이 OKAY
class mm2s_basic_seq extends mm2s_sequence;
    `uvm_object_utils(mm2s_basic_seq)

    function new(string name = "mm2s_basic_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), "MM2S 메모리 응답 시작 (지연 없음)", UVM_LOW)
        repeat (num) respond(0, 0, 2'b00);
    endtask
endclass


// 랜덤 지연 (backpressure)
class mm2s_random_seq extends mm2s_sequence;
    `uvm_object_utils(mm2s_random_seq)

    function new(string name = "mm2s_random_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), "MM2S 메모리 응답 시작 (랜덤 지연)", UVM_LOW)
        repeat (num) respond_random();
    endtask
endclass


// 에러 주입 : err_burst 번째 버스트에만 SLVERR 또는 DECERR, 나머지는 정상
//   test 에서 factory override 로 mm2s_basic_seq 대신 이게 만들어지게 함
class mm2s_err_seq extends mm2s_basic_seq;
    `uvm_object_utils(mm2s_err_seq)
    rand int       err_burst;
    rand bit [1:0] err_resp;
    constraint c_err  { err_burst inside {[0:11]}; }          // 64x4 프레임 = 버스트 12개
    constraint c_resp { err_resp inside {2'b10, 2'b11}; }     // SLVERR, DECERR

    function new(string name = "mm2s_err_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), $sformatf("MM2S 에러 주입 시작 (%0d번째 버스트에 %s)",
                  err_burst, err_resp == 2'b10 ? "SLVERR" : "DECERR"), UVM_LOW)
        for (int i = 0; i < num; i++) begin
            if (i == err_burst) respond(0, 0, err_resp);
            else                respond(0, 0, 2'b00);
        end
    endtask
endclass