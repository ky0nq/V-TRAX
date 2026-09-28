// ============================================================================
// mm2s_item : MM2S AXI4 read burst 한 번 (AR 1개 + R beat 여러 개)
//   TB = slave  -> 주소/길이는 DUT 가 정함 (monitor 가 채움)
//               -> sequence 는 "어떻게 응답할지"(지연, 에러)만 정함
//   실제 읽어줄 데이터는 메모리 모델에서 가져옴
// ============================================================================

class mm2s_item extends uvm_sequence_item;
    // ---- DUT 가 보낸 요청 (monitor가 채움) ----
    logic [4:0]  arid;
    logic [31:0] araddr;
    logic [7:0]  arlen;          // beat 수 - 1
    logic [2:0]  arsize;
    logic [1:0]  arburst;

    // ---- R 채널로 돌려준 데이터 (monitor가 채움) ----
    logic [31:0] rdata[$];
    logic [1:0]  rresp[$];

    // ---- 응답 방식 (sequence가 랜덤으로 정함) ----
    rand int unsigned arready_delay;   // AR 받기 전 대기 cycle
    rand int unsigned rvalid_gap;      // R beat 사이 빈 cycle
    rand logic [1:0]  rresp_inj;       // 00=OKAY, 10=SLVERR, 11=DECERR

    constraint c_delay {
        arready_delay inside {[0:3]};
        rvalid_gap    inside {[0:2]};
    }
    constraint c_resp { soft rresp_inj == 2'b00; }   // 기본은 정상 응답

    function new(string name = "mm2s_item");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(mm2s_item)
        `uvm_field_int(arid,          UVM_ALL_ON)
        `uvm_field_int(araddr,        UVM_ALL_ON)
        `uvm_field_int(arlen,         UVM_ALL_ON)
        `uvm_field_int(arsize,        UVM_ALL_ON)
        `uvm_field_int(arburst,       UVM_ALL_ON)
        `uvm_field_queue_int(rdata,   UVM_ALL_ON)
        `uvm_field_queue_int(rresp,   UVM_ALL_ON)
        `uvm_field_int(arready_delay, UVM_ALL_ON)
        `uvm_field_int(rvalid_gap,    UVM_ALL_ON)
        `uvm_field_int(rresp_inj,     UVM_ALL_ON)
    `uvm_object_utils_end

    function int unsigned num_beats();
        return arlen + 1;
    endfunction

    function string convert2string();
        return $sformatf("MM2S RD | id=%0d addr=0x%08h len=%0d(beats=%0d) size=%0d burst=%0d | got %0d beats",
                         arid, araddr, arlen, num_beats(), arsize, arburst, rdata.size());
    endfunction
endclass