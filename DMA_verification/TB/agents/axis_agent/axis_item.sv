// ============================================================================
// axis_item : 영상 프레임 한 장 (24bit RGB 픽셀 width x height 개)
//   vin  : sequence 가 픽셀을 랜덤으로 만들어서 driver 가 보냄
//   vout : monitor 가 출력 픽셀을 모아서 채움 (scoreboard 비교용)
// ============================================================================

class axis_item extends uvm_sequence_item;
    rand int unsigned width;
    rand int unsigned height;
    rand logic [23:0] pixels[];        // 행 우선 순서 (line0 px0, px1, ... line1 px0 ...)

    // vin driver 용 : tvalid 를 중간중간 내릴지
    rand int unsigned tvalid_gap_max;  // 0이면 연속 전송

    // vout monitor 가 채움 : 프로토콜 체크 결과
    int unsigned sof_cnt;              // tuser가 뜬 횟수 (1 이어야 정상)
    int unsigned eol_cnt;              // tlast가 뜬 횟수 (height 와 같아야 정상)

    // tb_top 의 DUT 파라미터와 맞춰야 함 (16 x 4)
    constraint c_size {
        soft width  == 16;
        soft height == 4;
        pixels.size() == width * height;
    }
    constraint c_gap { tvalid_gap_max inside {[0:2]}; }

    function new(string name = "axis_item");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(axis_item)
        `uvm_field_int(width,          UVM_ALL_ON)
        `uvm_field_int(height,         UVM_ALL_ON)
        `uvm_field_array_int(pixels,   UVM_ALL_ON | UVM_NOPRINT)  // 너무 길어서 print 생략
        `uvm_field_int(tvalid_gap_max, UVM_ALL_ON)
        `uvm_field_int(sof_cnt,        UVM_ALL_ON)
        `uvm_field_int(eol_cnt,        UVM_ALL_ON)
    `uvm_object_utils_end

    // 좌표로 픽셀 꺼내기 (scoreboard 에서 편하게 쓰려고)
    function logic [23:0] get_pixel(int unsigned x, int unsigned y);
        return pixels[y * width + x];
    endfunction

    function string convert2string();
        if (pixels.size() >= 2)
            return $sformatf("FRAME %0dx%0d | px[0]=0x%06h px[1]=0x%06h ... px[last]=0x%06h | sof=%0d eol=%0d",
                             width, height, pixels[0], pixels[1], pixels[pixels.size()-1],
                             sof_cnt, eol_cnt);
        else
            return $sformatf("FRAME %0dx%0d | %0d pixels | sof=%0d eol=%0d",
                             width, height, pixels.size(), sof_cnt, eol_cnt);
    endfunction
endclass