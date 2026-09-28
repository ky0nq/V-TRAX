// ============================================================================
// s2mm_item : S2MM AXI3 write burst 한 번 (AW 1개 + W beat 여러 개 + B 1개)
//   TB = slave  -> 주소/데이터는 DUT 가 정함 (monitor 가 채움)
//               -> sequence 는 "어떻게 응답할지"(지연, 에러)만 정함
// ============================================================================
class s2mm_item extends uvm_sequence_item;
    // ---- DUT 가 보낸 요청 (monitor 가 채움) ----
    logic [3:0]  awid;
    logic [31:0] awaddr;
    logic [3:0]  awlen;          // AXI3 : 최대 16 beat
    logic [2:0]  awsize;
    logic [1:0]  awburst;
    logic [31:0] wdata[$];
    logic [3:0]  wstrb[$];

    // ---- 돌려준 응답 (monitor 가 채움) ----
    logic [1:0]  bresp;

    // ---- 응답 방식 (sequence 가 랜덤으로 정함) ----
    rand int unsigned awready_delay;   // AW 받기 전 대기 cycle
    rand int unsigned wready_gap;      // W beat 사이 ready 내리는 cycle
    rand int unsigned bvalid_delay;    // 마지막 W 이후 B 주기까지 cycle
    rand logic [1:0]  bresp_inj;       // 00=OKAY, 10=SLVERR, 11=DECERR

    constraint c_delay {
        awready_delay inside {[0:3]};
        wready_gap    inside {[0:2]};
        bvalid_delay  inside {[0:3]};
    }
    constraint c_resp { soft bresp_inj == 2'b00; }

    function new(string name = "s2mm_item");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(s2mm_item)
        `uvm_field_int(awid,          UVM_ALL_ON)
        `uvm_field_int(awaddr,        UVM_ALL_ON)
        `uvm_field_int(awlen,         UVM_ALL_ON)
        `uvm_field_int(awsize,        UVM_ALL_ON)
        `uvm_field_int(awburst,       UVM_ALL_ON)
        `uvm_field_queue_int(wdata,   UVM_ALL_ON)
        `uvm_field_queue_int(wstrb,   UVM_ALL_ON)
        `uvm_field_int(bresp,         UVM_ALL_ON)
        `uvm_field_int(awready_delay, UVM_ALL_ON)
        `uvm_field_int(wready_gap,    UVM_ALL_ON)
        `uvm_field_int(bvalid_delay,  UVM_ALL_ON)
        `uvm_field_int(bresp_inj,     UVM_ALL_ON)
    `uvm_object_utils_end

    function int unsigned num_beats();
        return awlen + 1;
    endfunction

    function string convert2string();
        return $sformatf("S2MM WR | id=%0d addr=0x%08h len=%0d(beats=%0d) size=%0d burst=%0d | %0d beats written, bresp=%0d",
                         awid, awaddr, awlen, num_beats(), awsize, awburst, wdata.size(), bresp);
    endfunction
endclass