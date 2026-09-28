// ============================================================================
// dma_mem_model : DDR/BRAM 역할을 하는 메모리 모델
//   - env 가 하나 만들어서 mm2s driver, s2mm driver 에 같은 핸들을 나눠줌
//       s2mm driver : DUT가 쓴 데이터를 write_word() 로 저장
//       mm2s driver : DUT가 읽으려는 주소를 read_word() 로 꺼내서 R 응답
//   - byte 단위 연관 배열이라 쓴 주소만 메모리를 차지함 (4GB 전체 할당 X)
//   - 에러 영역을 지정하면 그 주소 접근 시 SLVERR 를 돌려주게 할 수 있음
// ============================================================================
class dma_mem_model extends uvm_object;
    `uvm_object_utils(dma_mem_model)

    // ---------------- 저장 공간 ----------------
    protected bit [7:0] mem[bit [31:0]];      // byte 주소 -> 1byte

    // ---------------- 설정 ----------------
    bit [31:0]   uninit_value = 32'hDEAD_BEEF; // 안 쓴 주소 읽을 때 돌려줄 값
    bit          warn_uninit  = 0;             // 1 이면 안 쓴 주소 읽을 때 warning

    // ---------------- 에러 영역 ----------------
    typedef struct { bit [31:0] lo; bit [31:0] hi; } region_t;
    protected region_t err_regions[$];

    // ---------------- 통계 (디버깅/리포트용) ----------------
    int unsigned wr_cnt;      // write_word 호출 횟수
    int unsigned rd_cnt;      // read_word  호출 횟수
    int unsigned uninit_cnt;  // 안 쓴 주소 읽은 횟수

    function new(string name = "dma_mem_model");
        super.new(name);
    endfunction

    // ========================================================================
    // 기본 read / write (32bit word, strobe 지원)
    // ========================================================================
    // wstrb[i] 가 1 인 byte 만 저장 (AXI W 채널 그대로)
    function void write_word(bit [31:0] addr, bit [31:0] data, bit [3:0] strb = 4'hF);
        bit [31:0] base = {addr[31:2], 2'b00};  // word 정렬
        for (int i = 0; i < 4; i++) begin
            if (strb[i])
                mem[base + i] = data[i*8 +: 8];
        end
        wr_cnt++;
    endfunction

    function bit [31:0] read_word(bit [31:0] addr);
        bit [31:0] base = {addr[31:2], 2'b00};
        bit [31:0] data;
        bit        hit_uninit = 0;
        for (int i = 0; i < 4; i++) begin
            if (mem.exists(base + i))
                data[i*8 +: 8] = mem[base + i];
            else begin
                data[i*8 +: 8] = uninit_value[i*8 +: 8];
                hit_uninit     = 1;
            end
        end
        if (hit_uninit) begin
            uninit_cnt++;
            if (warn_uninit)
                `uvm_warning("MEM_UNINIT", $sformatf("read uninitialized addr 0x%08h", base))
        end
        rd_cnt++;
        return data;
    endfunction

    // 해당 word 가 한 번이라도 써졌는지
    function bit is_written(bit [31:0] addr);
        bit [31:0] base = {addr[31:2], 2'b00};
        return mem.exists(base) || mem.exists(base + 1) ||
               mem.exists(base + 2) || mem.exists(base + 3);
    endfunction

    // ========================================================================
    // 테스트 편의 함수
    // ========================================================================
    // MM2S 만 따로 볼 때 : 미리 메모리를 채워둠 (값 = 주소, 또는 seed 기반 랜덤)
    function void preload(bit [31:0] start_addr, int unsigned num_words,
                          bit use_random = 0, int unsigned seed = 1);
        int unsigned s = seed;
        for (int unsigned i = 0; i < num_words; i++) begin
            bit [31:0] a = start_addr + i*4;
            if (use_random) write_word(a, $urandom(s + i));
            else            write_word(a, a);
        end
        `uvm_info("MEM", $sformatf("preload 0x%08h ~ 0x%08h (%0d words, %s)",
                  start_addr, start_addr + num_words*4 - 1, num_words,
                  use_random ? "random" : "addr pattern"), UVM_MEDIUM)
    endfunction

    // 메모리 전체 비우기 (테스트 사이 초기화용)
    function void clear();
        mem.delete();
        err_regions.delete();
        wr_cnt = 0; rd_cnt = 0; uninit_cnt = 0;
    endfunction

    // 특정 구간 출력 (디버깅용)
    function void dump(bit [31:0] start_addr, int unsigned num_words);
        for (int unsigned i = 0; i < num_words; i++) begin
            bit [31:0] a = start_addr + i*4;
            `uvm_info("MEM_DUMP", $sformatf("[0x%08h] = 0x%08h%s", a, read_word(a),
                      is_written(a) ? "" : "  (uninit)"), UVM_LOW)
        end
        rd_cnt -= num_words;   // dump 는 통계에서 제외
    endfunction

    // ========================================================================
    // 에러 주입 : 이 구간에 접근하면 driver 가 SLVERR 를 돌려주게 함
    // ========================================================================
    function void add_err_region(bit [31:0] lo, bit [31:0] hi);
        region_t r;
        r.lo = lo; r.hi = hi;
        err_regions.push_back(r);
        `uvm_info("MEM", $sformatf("error region 0x%08h ~ 0x%08h", lo, hi), UVM_MEDIUM)
    endfunction

    function bit is_err_addr(bit [31:0] addr);
        foreach (err_regions[i])
            if (addr >= err_regions[i].lo && addr <= err_regions[i].hi)
                return 1;
        return 0;
    endfunction

    // ========================================================================
    // AXI burst 주소 계산 : beat_idx 번째 beat 의 주소
    //   burst : 0 = FIXED, 1 = INCR, 2 = WRAP
    //   len   : AxLEN (beat 수 - 1), size : AxSIZE (2 = 4byte)
    //   mm2s / s2mm driver 둘 다 이 함수로 beat 주소를 구함
    // ========================================================================
    function bit [31:0] beat_addr(bit [31:0] start, int unsigned len,
                                         bit [2:0] size, bit [1:0] burst,
                                         int unsigned beat_idx);
        int unsigned nbytes = 1 << size;
        bit [31:0]   aligned = (start / nbytes) * nbytes;
        case (burst)
            2'b00: return start;                                  // FIXED
            2'b01: return (beat_idx == 0) ? start                 // INCR
                                          : aligned + beat_idx * nbytes;
            2'b10: begin                                          // WRAP
                int unsigned wrap_bytes = (len + 1) * nbytes;
                bit [31:0]   lower      = (start / wrap_bytes) * wrap_bytes;
                return lower + ((start - lower + beat_idx * nbytes) % wrap_bytes);
            end
            default: begin
                `uvm_error("MEM", $sformatf("reserved burst type %0d", burst))
                return start;
            end
        endcase
    endfunction

    // 리포트 (env 의 report_phase 에서 호출)
    function void report();
        `uvm_info("MEM", $sformatf("writes=%0d reads=%0d uninit_reads=%0d bytes_used=%0d",
                  wr_cnt, rd_cnt, uninit_cnt, mem.num()), UVM_LOW)
    endfunction

endclass