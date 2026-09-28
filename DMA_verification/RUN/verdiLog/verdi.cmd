simSetSimulator "-vcssv" -exec "simv" -args
debImport "-dbdir" "simv.daidir"
debLoadSimResult /home/pedu19/workspace/final_dma/RUN/dma_cfg_err_test.fsdb
wvCreateWindow
verdiSetActWin -win $_nWave2
verdiWindowResize -win $_Verdi_1 "635" "31" "1807" "1360"
verdiSetActWin -dock widgetDock_MTB_SOURCE_TAB_1
srcHBSelect "tb_top.mm2s" -win $_nTrace1
verdiSetActWin -dock widgetDock_<Inst._Tree>
srcHBSelect "tb_top.dut.U_MM2S.U_DMA_MM2S.U_READ_ENGINE.U_READ_DATAPATH" -win \
           $_nTrace1
srcHBSelect "tb_top.dut.U_MM2S.U_DMA_MM2S.U_READ_ENGINE.U_READ_CNTL" -win \
           $_nTrace1
srcHBDrag -win $_nTrace1
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 0)}
wvRenameGroup -win $_nWave2 {G1} {U_READ_CNTL}
wvAddSignal -win $_nWave2 \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/clk" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/rst_n" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/start" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/abort" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/cyclic" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/busy" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/done" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/error" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/error_addr\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/frame_done" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/xfer_done" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/err_valid" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/cfg_err" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/err_addr\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/en" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_CNTL/init"
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 0)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 16)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 16)}
srcHBDrag -win $_nTrace1
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 1)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 2)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 4)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 7)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 8)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 10)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 11)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 13)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 14)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 15)}
wvSetPosition -win $_nWave2 {("U_READ_CNTL" 16)}
wvSetPosition -win $_nWave2 {("G2" 0)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 0)}
wvRenameGroup -win $_nWave2 {G2} {U_READ_DATAPATH}
wvAddSignal -win $_nWave2 \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/clk" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rst_n" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/en" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/init" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/abort" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/src_addr\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/length\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/burst_cfg\[9:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/r_hs" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/xfer_done" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/err_addr\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/err_valid" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/cfg_err" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/fifo_wr_en" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/fifo_wr_data\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/fifo_full" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/arid\[4:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/araddr\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/arlen\[7:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/arsize\[2:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/arburst\[1:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/arvalid" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/arready" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rdata\[31:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rvalid" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rlast" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rid\[4:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rresp\[1:0\]" \
           "/tb_top/dut/U_MM2S/U_DMA_MM2S/U_READ_ENGINE/U_READ_DATAPATH/rready"
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 0)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 29)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 29)}
verdiSetActWin -win $_nWave2
wvSelectSignal -win $_nWave2 {( "U_READ_DATAPATH" 19 )} 
wvScrollUp -win $_nWave2 24
wvScrollDown -win $_nWave2 10
wvSelectSignal -win $_nWave2 {( "U_READ_DATAPATH" 13 )} 
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 10)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 9)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 8)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 7)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 6)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 5)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 4)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 3)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 2)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 1)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 0)}
wvMoveSelected -win $_nWave2
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 0)}
wvSetPosition -win $_nWave2 {("U_READ_DATAPATH" 1)}
wvSelectSignal -win $_nWave2 {( "U_READ_CNTL" 16 )} {( "U_READ_DATAPATH" 1 )} \
           
wvScrollUp -win $_nWave2 1
wvScrollUp -win $_nWave2 1
wvZoom -win $_nWave2 477245.308311 588602.546917
wvZoomOut -win $_nWave2
wvZoomOut -win $_nWave2
wvZoomOut -win $_nWave2
wvScrollUp -win $_nWave2 1
