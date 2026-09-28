// ============================================================================
// axil_sequence : 레지스터 접근 sequence 모음
//   base 클래스에 레지스터 주소 + write_reg / read_reg / wait_bit 를 만들어두고
//   시나리오별 sequence 는 이걸 상속해서 씀 (spi_sequence 의 do_slave0 같은 역할)
// ============================================================================
class axil_sequence extends uvm_sequence#(axil_item);
    `uvm_object_utils(axil_sequence)

    // ---------------- 레지스터 주소 (dma_regmap 과 동일) ----------------
    localparam bit [6:0] MM2S_CR   = 7'h00;
    localparam bit [6:0] MM2S_SR   = 7'h04;
    localparam bit [6:0] SA        = 7'h18;
    localparam bit [6:0] READ_ERR  = 7'h1C;
    localparam bit [6:0] BTT       = 7'h28;   // 쓰면 MM2S 시작
    localparam bit [6:0] BURST_CFG = 7'h30;
    localparam bit [6:0] NUM_BUF   = 7'h38;
    localparam bit [6:0] SW_IDX    = 7'h3C;
    localparam bit [6:0] S2MM_CR   = 7'h40;
    localparam bit [6:0] S2MM_SR   = 7'h44;
    localparam bit [6:0] DA0       = 7'h48;
    localparam bit [6:0] DA1       = 7'h4C;
    localparam bit [6:0] DA2       = 7'h50;
    localparam bit [6:0] START     = 7'h54;   // [0] 에 1 쓰면 S2MM 시작
    localparam bit [6:0] WRITE_ERR = 7'h58;

    // ---------------- 비트 위치 ----------------
    localparam int CR_ABORT     = 2;
    localparam int CR_CYCLIC    = 4;
    localparam int CR_LIVE      = 5;
    localparam int CR_IDX_SW    = 6;
    localparam int CR_IOC_IRQEN = 12;
    localparam int CR_ERR_IRQEN = 14;
    localparam int SR_BUSY      = 0;
    localparam int SR_IDLE      = 1;
    localparam int SR_ERROR     = 4;
    localparam int SR_IOC_IRQ   = 12;
    localparam int SR_ERR_IRQ   = 14;

    function new(string name = "axil_sequence");
        super.new(name);
    endfunction

    // 레지스터 쓰기
    task write_reg(bit [6:0] addr, bit [31:0] data);
        axil_item item;
        item = axil_item::type_id::create("item");
        start_item(item);
        item.is_write = 1;
        item.addr     = addr;
        item.wdata    = data;
        item.wstrb    = 4'hF;
        finish_item(item);
    endtask

    // 레지스터 읽기 (finish_item 이 끝나면 driver 가 rdata 를 채워둠)
    task read_reg(bit [6:0] addr, output bit [31:0] data);
        axil_item item;
        item = axil_item::type_id::create("item");
        start_item(item);
        item.is_write = 0;
        item.addr     = addr;
        finish_item(item);
        data = item.rdata;
    endtask

    // 레지스터의 특정 비트가 1 이 될 때까지 계속 읽기 (polling)
    task wait_bit(bit [6:0] addr, int bit_pos, int max_reads = 2000);
        bit [31:0] val;
        for (int i = 0; i < max_reads; i++) begin
            read_reg(addr, val);
            if (val[bit_pos]) return;
        end
        `uvm_error(get_type_name(), $sformatf("timeout: addr 0x%02h bit%0d 가 %0d번 읽는 동안 안 뜸",
                   addr, bit_pos, max_reads))
    endtask
endclass


// ----------------------------------------------------------------------------
// S2MM 시작 : 버퍼 주소 3개 설정 -> 인터럽트 enable -> START
// ----------------------------------------------------------------------------
class axil_s2mm_start_seq extends axil_sequence;
    `uvm_object_utils(axil_s2mm_start_seq)
    bit [31:0] da0 = 32'h1000_0000;
    bit [31:0] da1 = 32'h1001_0000;
    bit [31:0] da2 = 32'h1002_0000;

    function new(string name = "axil_s2mm_start_seq");
        super.new(name);
    endfunction

    task body();
        `uvm_info(get_type_name(), $sformatf("S2MM 시작 (DA0=0x%08h DA1=0x%08h DA2=0x%08h)",
                  da0, da1, da2), UVM_LOW)
        write_reg(DA0, da0);
        write_reg(DA1, da1);
        write_reg(DA2, da2);
        write_reg(S2MM_CR, (1 << CR_IOC_IRQEN) | (1 << CR_ERR_IRQEN));
        write_reg(START, 32'h1);
    endtask
endclass


// ----------------------------------------------------------------------------
// MM2S 시작 : CR / BURST_CFG / NUM_BUF / SA 설정 후 BTT 를 마지막에 써서 출발
//   live   = 1 : S2MM 이 방금 다 쓴 버퍼(newest)를 읽음 (기본)
//   live   = 0 : SA 주소를 읽음 (메모리 preload 해서 MM2S 만 볼 때)
//   cyclic = 1 : 프레임 끝나면 자동으로 다음 프레임
// ----------------------------------------------------------------------------
class axil_mm2s_start_seq extends axil_sequence;
    `uvm_object_utils(axil_mm2s_start_seq)
    bit        live      = 1;
    bit        cyclic    = 0;
    bit        idx_sw    = 0;
    bit [2:0]  sw_idx    = 0;
    bit [31:0] sa        = 32'h8000_0000;
    bit [31:0] btt       = 16 * 4 * 3;     // 16x4 프레임, 픽셀당 3byte = 192
    bit [7:0]  arlen     = 8'd15;          // 16 beat
    bit [1:0]  burst     = 2'b01;          // INCR
    bit [3:0]  num_buf   = 4'd3;

    function new(string name = "axil_mm2s_start_seq");
        super.new(name);
    endfunction

    task body();
        bit [31:0] cr;
        cr = (1 << CR_IOC_IRQEN) | (1 << CR_ERR_IRQEN);
        if (cyclic) cr |= (1 << CR_CYCLIC);
        if (live)   cr |= (1 << CR_LIVE);
        if (idx_sw) cr |= (1 << CR_IDX_SW);

        `uvm_info(get_type_name(), $sformatf("MM2S 시작 (live=%0d cyclic=%0d btt=%0d arlen=%0d)",
                  live, cyclic, btt, arlen), UVM_LOW)
        write_reg(MM2S_CR,   cr);
        write_reg(BURST_CFG, {22'd0, burst, arlen});
        write_reg(NUM_BUF,   {28'd0, num_buf});
        write_reg(SW_IDX,    {29'd0, sw_idx});
        write_reg(SA,        sa);
        write_reg(BTT,       btt);              // 이게 start
    endtask
endclass


// ----------------------------------------------------------------------------
// 프레임 완료 대기 : SR 의 IOC_Irq(bit12) 가 뜰 때까지 polling -> W1C 로 지움
// ----------------------------------------------------------------------------
class axil_wait_frame_seq extends axil_sequence;
    `uvm_object_utils(axil_wait_frame_seq)
    bit is_mm2s = 0;     // 0 = S2MM 기다림, 1 = MM2S 기다림

    function new(string name = "axil_wait_frame_seq");
        super.new(name);
    endfunction

    task body();
        bit [6:0] sr = is_mm2s ? MM2S_SR : S2MM_SR;
        wait_bit(sr, SR_IOC_IRQ);
        write_reg(sr, (1 << SR_IOC_IRQ));      // W1C : 1 을 써서 지움
        `uvm_info(get_type_name(), $sformatf("%s 프레임 완료 확인", is_mm2s ? "MM2S" : "S2MM"), UVM_LOW)
    endtask
endclass


// ----------------------------------------------------------------------------
// MM2S abort : CR 을 읽어서 ABORT 비트만 1 로 써줌 (나머지 설정 유지)
// ----------------------------------------------------------------------------
class axil_mm2s_abort_seq extends axil_sequence;
    `uvm_object_utils(axil_mm2s_abort_seq)

    function new(string name = "axil_mm2s_abort_seq");
        super.new(name);
    endfunction

    task body();
        bit [31:0] cr;
        read_reg(MM2S_CR, cr);
        write_reg(MM2S_CR, cr | (1 << CR_ABORT));
        `uvm_info(get_type_name(), "MM2S abort", UVM_LOW)
    endtask
endclass


// ----------------------------------------------------------------------------
// 레지스터 read/write 테스트 : 경계값 4개 + 랜덤 num 번 -> 쓴 값 다시 읽어서 비교
//   ※ BTT / START 는 쓰면 DMA 가 출발해서 제외
// ----------------------------------------------------------------------------
class axil_reg_rw_seq extends axil_sequence;
    `uvm_object_utils(axil_reg_rw_seq)
    rand int num;
    constraint c_num { num inside {[5:10]}; }

    int pass_count = 0;
    int fail_count = 0;

    function new(string name = "axil_reg_rw_seq");
        super.new(name);
    endfunction

    // 주소 + 읽었을 때 살아있는 비트 mask
    typedef struct { bit [6:0] addr; bit [31:0] mask; string name; } reg_t;

    task check_reg(reg_t r, bit [31:0] wval);
        bit [31:0] rval;
        write_reg(r.addr, wval);
        read_reg(r.addr, rval);
        if ((rval & r.mask) === (wval & r.mask)) begin
            pass_count++;
            `uvm_info(get_type_name(), $sformatf("REG PASS [%s]: wr=0x%08h rd=0x%08h",
                      r.name, wval & r.mask, rval), UVM_MEDIUM)
        end else begin
            fail_count++;
            `uvm_error(get_type_name(), $sformatf("REG FAIL [%s]: wr=0x%08h rd=0x%08h (기대값=0x%08h)",
                       r.name, wval, rval, wval & r.mask))
        end
    endtask

    task body();
        reg_t regs[$];
        bit [31:0] corner_vals[$] = {32'h0000_0000, 32'hFFFF_FFFF, 32'h5555_5555, 32'hAAAA_AAAA};

        regs.push_back('{MM2S_CR,   ~(32'h1 << CR_ABORT), "MM2S_CR"});   // ABORT 는 저장 안 됨
        regs.push_back('{SA,        32'hFFFF_FFFF,        "SA"});
        regs.push_back('{BURST_CFG, 32'h0000_03FF,        "BURST_CFG"});
        regs.push_back('{NUM_BUF,   32'h0000_000F,        "NUM_BUF"});
        regs.push_back('{SW_IDX,    32'h0000_0007,        "SW_IDX"});
        regs.push_back('{S2MM_CR,   32'hFFFF_FFFF,        "S2MM_CR"});
        regs.push_back('{DA0,       32'hFFFF_FFFF,        "DA0"});
        regs.push_back('{DA1,       32'hFFFF_FFFF,        "DA1"});
        regs.push_back('{DA2,       32'hFFFF_FFFF,        "DA2"});

        `uvm_info(get_type_name(), $sformatf("레지스터 시나리오 시작 (경계값 %0d개 + 랜덤 %0d번) x 레지스터 %0d개",
                  corner_vals.size(), num, regs.size()), UVM_LOW)

        foreach (corner_vals[i])
            foreach (regs[j]) check_reg(regs[j], corner_vals[i]);

        repeat (num)
            foreach (regs[j]) check_reg(regs[j], $urandom());

        // 읽기 전용 / 상태 레지스터도 한 번씩 읽어봄 (coverage 용, 값 비교는 안 함)
        begin
            bit [6:0]  ro_regs[$] = {MM2S_SR, READ_ERR, BTT, S2MM_SR, WRITE_ERR};
            bit [31:0] v;
            foreach (ro_regs[i]) read_reg(ro_regs[i], v);
        end

        // 다음 테스트에 영향 안 주게 CR 은 0 으로 되돌림
        write_reg(MM2S_CR, 0);
        write_reg(S2MM_CR, 0);

        `uvm_info(get_type_name(), $sformatf("레지스터 시나리오 종료 (pass=%0d fail=%0d)",
                  pass_count, fail_count), UVM_LOW)
    endtask
endclass



// ============================================================================
// test 에서 레지스터 하나만 읽고/쓰고/기다릴 때 쓰는 작은 sequence 들
//   (sequence 의 task 는 start() 된 sequence 안에서만 쓸 수 있어서 따로 만듦)
// ============================================================================
class axil_write_seq extends axil_sequence;
    `uvm_object_utils(axil_write_seq)
    bit [6:0]  addr;
    bit [31:0] data;

    function new(string name = "axil_write_seq");
        super.new(name);
    endfunction

    task body();
        write_reg(addr, data);
    endtask
endclass


class axil_read_seq extends axil_sequence;
    `uvm_object_utils(axil_read_seq)
    bit [6:0]  addr;
    bit [31:0] data;

    function new(string name = "axil_read_seq");
        super.new(name);
    endfunction

    task body();
        read_reg(addr, data);
    endtask
endclass


class axil_wait_bit_seq extends axil_sequence;
    `uvm_object_utils(axil_wait_bit_seq)
    bit [6:0] addr;
    int       bit_pos;
    int       max_reads = 2000;

    function new(string name = "axil_wait_bit_seq");
        super.new(name);
    endfunction

    task body();
        wait_bit(addr, bit_pos, max_reads);
    endtask
endclass