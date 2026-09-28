// ----------------------------------------------------------------------------
// 기본 : 패턴 프레임 1장 왕복 (지연 없음) -> 제일 먼저 통과시킬 테스트
// ----------------------------------------------------------------------------
class dma_frame_test extends dma_base_test;
    `uvm_component_utils(dma_frame_test)

    function new(string name = "dma_frame_test", uvm_component parent);
        super.new(name, parent);
    endfunction

    task run_scenario();
        axis_pattern_seq s = axis_pattern_seq::type_id::create("cam_seq");
        if (!s.randomize() with { num == 1; }) `uvm_error(get_type_name(), "randomize 실패")
        s2mm_start();
        one_frame(s);
    endtask
endclass