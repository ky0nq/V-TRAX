// ============================================================================
// s2mm_sequence : S2MM write slave(메모리)의 "응답 방식" sequence 모음
//   item 하나 = AW 버스트 하나에 어떻게 응답할지
//   ※ slave 라서 item 을 계속 넣어줘야 DUT 가 안 멈춤 -> num 을 넉넉하게
// ============================================================================
class s2mm_sequence extends uvm_sequence#(s2mm_item);
    `uvm_object_utils(s2mm_sequence)
    int num = 100000;

    function new(string name = "s2mm_sequence");
        super.new(name);
    endfunction

    task respond(int unsigned awready_delay, int unsigned wready_gap,
                 int unsigned bvalid_delay,  bit [1:0] resp);
        s2mm_item item;
        item = s2mm_item::type_id::create("item");
        start_item(item);
        item.awready_delay = awready_delay;
        item.wready_gap    = wready_gap;
        item.bvalid_delay  = bvalid_delay;
        item.bresp_inj     = resp;
        finish_item(item);
    endtask

    task respond_random();
        s2mm_item item;
        item = s2mm_item::type_id::create("item");
        start_item(item);
        if (!item.randomize() with { bresp_inj == 2'b00; })
            `uvm_error("SEQ", "randomize 실패")
        finish_item(item);
    endtask
endclass


// 기본 : 지연 없이 OKAY
class s2mm_basic_seq extends s2mm_sequence;
    `uvm_object_utils(s2mm_basic_seq)

    function new(string name = "s2mm_basic_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), "S2MM 메모리 응답 시작 (지연 없음)", UVM_LOW)
        repeat (num) respond(0, 0, 0, 2'b00);
    endtask
endclass


// 랜덤 지연 (backpressure)
class s2mm_random_seq extends s2mm_sequence;
    `uvm_object_utils(s2mm_random_seq)

    function new(string name = "s2mm_random_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), "S2MM 메모리 응답 시작 (랜덤 지연)", UVM_LOW)
        repeat (num) respond_random();
    endtask
endclass


// 에러 주입 : err_burst 번째 버스트에만 SLVERR 또는 DECERR, 나머지는 정상
//   test 에서 factory override 로 s2mm_basic_seq 대신 이게 만들어지게 함
class s2mm_err_seq extends s2mm_basic_seq;
    `uvm_object_utils(s2mm_err_seq)
    rand int       err_burst;
    rand bit [1:0] err_resp;
    constraint c_err  { err_burst inside {[0:11]}; }          // 64x4 프레임 = 버스트 12개
    constraint c_resp { err_resp inside {2'b10, 2'b11}; }     // SLVERR, DECERR

    function new(string name = "s2mm_err_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), $sformatf("S2MM 에러 주입 시작 (%0d번째 버스트에 %s)",
                  err_burst, err_resp == 2'b10 ? "SLVERR" : "DECERR"), UVM_LOW)
        for (int i = 0; i < num; i++) begin
            if (i == err_burst) respond(0, 0, 0, err_resp);
            else                respond(0, 0, 0, 2'b00);
        end
    endtask
endclass