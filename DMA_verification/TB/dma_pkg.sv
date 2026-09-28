// ============================================================================
// dma_pkg : class 들을 전부 모아둔 package (interface 는 package 에 못 넣어서 tb.f 에 따로)
//   ※ include 순서 중요 : 쓰이는 쪽보다 먼저 정의돼야 함
// ============================================================================
package dma_pkg;
    import uvm_pkg::*;
    `include "uvm_macros.svh"

    // scoreboard / coverage 가 같이 쓰는 analysis imp (write_cam, write_disp ...)
    `uvm_analysis_imp_decl(_cam)
    `uvm_analysis_imp_decl(_disp)
    `uvm_analysis_imp_decl(_axil)
    `uvm_analysis_imp_decl(_s2mm)
    `uvm_analysis_imp_decl(_mm2s)

    // item
    `include "axil_item.sv"
    `include "axis_item.sv"
    `include "mm2s_item.sv"
    `include "s2mm_item.sv"

    // memory model (driver 보다 먼저)
    `include "dma_mem_model.sv"

    // agent : driver -> monitor -> agent -> sequence
    `include "axil_driver.sv"
    `include "axil_monitor.sv"
    `include "axil_agent.sv"
    `include "axil_sequence.sv"

    `include "axis_driver.sv"
    `include "axis_monitor.sv"
    `include "axis_agent.sv"
    `include "axis_sequence.sv"

    `include "mm2s_driver.sv"
    `include "mm2s_monitor.sv"
    `include "mm2s_agent.sv"
    `include "mm2s_sequence.sv"

    `include "s2mm_driver.sv"
    `include "s2mm_monitor.sv"
    `include "s2mm_agent.sv"
    `include "s2mm_sequence.sv"

    // env / test
    `include "dma_scoreboard.sv"
    `include "dma_coverage.sv"
    `include "dma_env.sv"
    `include "dma_base_test.sv"
    `include "dma_frame_test.sv"
    `include "dma_reg_test.sv"
    `include "dma_random_test.sv"
    `include "dma_corner_test.sv"
    `include "dma_burst_test.sv"
    `include "dma_live0_test.sv"
    `include "dma_cyclic_test.sv"
    `include "dma_idx_sw_test.sv"
    `include "dma_err_test.sv"
    `include "dma_cfg_err_test.sv"
endpackage