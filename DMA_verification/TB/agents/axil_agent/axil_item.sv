// ============================================================================
// axil_item : AXI4-Lite 레지스터 read/write 한 번
//   TB = master  -> sequence 가 addr/data 를 정해서 보냄
// ============================================================================

class axil_item extends uvm_sequence_item;
    rand bit          is_write;     // 1 = write, 0 = read
    rand logic [6:0]  addr;
    rand logic [31:0] wdata;
    rand logic [3:0]  wstrb;

    // DUT 응답 (driver/monitor 가 채움)
    logic [31:0]      rdata;
    logic [1:0]       resp;         // write 면 BRESP, read 면 RRESP

    constraint c_align { addr[1:0] == 2'b00; }   // 주소가 0, 4, 8로 4byte 간격이므로 4의 배수로 고정시킴
    constraint c_strb  { soft wstrb == 4'hF; }   // wstrb: 어떤 바이트를 실제로 쓸지 정하는 신호 -> 일단 전체 쓰기

    function new(string name = "axil_item");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(axil_item) // 이 클래스를 uvm factory에 등록
        `uvm_field_int(is_write, UVM_ALL_ON) // 기능 전부 켜기 ~
        `uvm_field_int(addr,     UVM_ALL_ON)
        `uvm_field_int(wdata,    UVM_ALL_ON)
        `uvm_field_int(wstrb,    UVM_ALL_ON)
        `uvm_field_int(rdata,    UVM_ALL_ON)
        `uvm_field_int(resp,     UVM_ALL_ON)
    `uvm_object_utils_end

    function string convert2string(); // item 내용을 로그용 한줄 문자열로 바꿔주는 함수
        // bresp = 0은 okay, 2는 slverr, 3은 decerr (레지스터 접근에서 에러 났다는 뜻)
        if (is_write)
            return $sformatf("AXIL WR | addr=0x%02h wdata=0x%08h strb=%04b | bresp=%0d",
                             addr, wdata, wstrb, resp);
        else
            return $sformatf("AXIL RD | addr=0x%02h | rdata=0x%08h rresp=%0d",
                             addr, rdata, resp);
    endfunction
endclass