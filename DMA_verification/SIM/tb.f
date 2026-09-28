// TB : RUN/ 폴더 기준 상대 경로
+incdir+../TB
+incdir+../TB/agents/axil_agent
+incdir+../TB/agents/axis_agent
+incdir+../TB/agents/mm2s_agent
+incdir+../TB/agents/s2mm_agent
+incdir+../TB/env
+incdir+../TB/test

// interface (package 밖)
../TB/agents/axil_agent/axil_interface.sv
../TB/agents/axis_agent/axis_interface.sv
../TB/agents/mm2s_agent/mm2s_interface.sv
../TB/agents/s2mm_agent/s2mm_interface.sv

// package (나머지 class 전부 include)
../TB/dma_pkg.sv

// top
../TB/tb_top.sv