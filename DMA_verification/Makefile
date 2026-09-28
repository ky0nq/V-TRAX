# ============================================================================
#  DMA UVM Makefile
#    - Makefile 은 프로젝트 최상위에 두고, 생기는 파일은 전부 RUN/ 폴더 안에 만듦
#    - filelist(SIM/*.f) 안의 경로는 "RUN/ 폴더 기준" 상대 경로 (../RTL, ../TB)
#
#  사용법
#    make                          : 컴파일 + dma_frame_test 실행
#    make TEST=dma_random_test     : 다른 test 실행
#    make run TEST=... SEED=5      : 컴파일 없이 다시 실행 (seed 만 바꿀 때)
#    make regress NSEED=10         : TESTS 전부 x seed 10개 실행 + 결과 요약
#    make run TEST=... PLUSARGS=+NUM_FRAMES=30 : 프레임 수 늘리기
#    make sim WAVE=1 TEST=...      : 파형 저장 (RUN/<TEST>.fsdb)
#    make verdi TEST=...           : 그 test 의 파형 보기
#    make cov                      : coverage 리포트 (html)
#    make verdi_cov                : Verdi 로 coverage 보기
#    make clean                    : RUN/ 폴더 통째로 삭제
# ============================================================================

ROOT     := $(CURDIR)
# RUN : 생성 파일이 쌓이는 곳 (make clean 하면 통째로 삭제)
# SIM : rtl.f, tb.f 보관 (절대 RUN 과 바꾸지 말 것)
RUN_DIR  := $(ROOT)/RUN
FLIST    := $(ROOT)/SIM

TEST       ?= dma_frame_test
SEED       ?= 1
INFO_LEVEL ?= UVM_LOW
WAVE       ?= 0
NSEED      ?= 1
SEEDS      ?= $(shell seq 1 $(NSEED))
PLUSARGS   ?=
TESTS      ?= dma_reg_test dma_frame_test dma_corner_test dma_random_test \
              dma_burst_test dma_live0_test dma_cyclic_test dma_idx_sw_test \
              dma_err_test dma_cfg_err_test

CM_TYPES  = line+cond+fsm+tgl+branch+assert

# ---------------- compile ----------------
VCS_OPTS  = -full64 -sverilog
VCS_OPTS += -ntb_opts uvm-1.2
VCS_OPTS += -timescale=1ns/1ps
VCS_OPTS += -debug_access+all -kdb -lca
VCS_OPTS += -cm $(CM_TYPES)
VCS_OPTS += -top tb_top
VCS_OPTS += -f $(FLIST)/rtl.f
VCS_OPTS += -f $(FLIST)/tb.f
VCS_OPTS += -l compile.log
VCS_OPTS += -o simv

# ---------------- run ----------------
SIMV_OPTS  = +UVM_TESTNAME=$(TEST)
SIMV_OPTS += +UVM_VERBOSITY=$(INFO_LEVEL)
SIMV_OPTS += +ntb_random_seed=$(SEED)
SIMV_OPTS += -cm $(CM_TYPES)
SIMV_OPTS += -cm_dir coverage.vdb
SIMV_OPTS += -cm_name $(TEST)_$(SEED)
SIMV_OPTS += -l $(TEST)_$(SEED).log
SIMV_OPTS += $(PLUSARGS)
ifeq ($(WAVE),1)
SIMV_OPTS += +WAVE=$(TEST).fsdb
endif

.PHONY: all compile sim run regress verdi verdi_cov cov clean

all: sim

compile:
	@mkdir -p $(RUN_DIR)
	cd $(RUN_DIR) && vcs $(VCS_OPTS)

sim: compile run

run:
	cd $(RUN_DIR) && ./simv $(SIMV_OPTS)

# TESTS x SEEDS 전부 실행 -> RUN/regress_summary.txt 에 한 줄씩 기록 -> 요약 출력
#   make regress NSEED=10                  : seed 1~10
#   make regress SEEDS="7 42 1234"         : seed 직접 지정
#   make regress TESTS=dma_random_test NSEED=20 PLUSARGS=+NUM_FRAMES=30
regress: compile
	@rm -f $(RUN_DIR)/regress_summary.txt
	@for t in $(TESTS); do for s in $(SEEDS); do \
		printf "running %-20s seed=%s\n" $$t $$s; \
		( cd $(RUN_DIR) && ./simv +UVM_TESTNAME=$$t +UVM_VERBOSITY=$(INFO_LEVEL) \
			+ntb_random_seed=$$s $(PLUSARGS) -cm $(CM_TYPES) -cm_dir coverage.vdb \
			-cm_name $${t}_$$s -l $${t}_$$s.log > /dev/null 2>&1 ); \
		err=$$(grep -E "^UVM_ERROR :" $(RUN_DIR)/$${t}_$$s.log | awk '{print $$3}'); \
		fat=$$(grep -E "^UVM_FATAL :" $(RUN_DIR)/$${t}_$$s.log | awk '{print $$3}'); \
		if [ "$$err" = "0" ] && [ "$$fat" = "0" ]; then res=PASS; else res=FAIL; fi; \
		printf "%-20s seed=%-6s %s  (UVM_ERROR=%s UVM_FATAL=%s)\n" $$t $$s $$res "$$err" "$$fat" \
			>> $(RUN_DIR)/regress_summary.txt; \
	done; done
	@echo ""
	@echo "=============== REGRESSION SUMMARY ==============="
	@cat $(RUN_DIR)/regress_summary.txt
	@echo "--------------------------------------------------"
	@echo "total=$$(grep -c . $(RUN_DIR)/regress_summary.txt)  pass=$$(grep -c ' PASS ' $(RUN_DIR)/regress_summary.txt)  fail=$$(grep -c ' FAIL ' $(RUN_DIR)/regress_summary.txt)"

verdi:
	cd $(RUN_DIR) && verdi -dbdir simv.daidir -ssf $(TEST).fsdb &

verdi_cov:
	cd $(RUN_DIR) && verdi -cov -covdir coverage.vdb &

cov:
	cd $(RUN_DIR) && urg -dir coverage.vdb -report cov_report
	@echo "open $(RUN_DIR)/cov_report/dashboard.html"

clean:
	rm -rf $(RUN_DIR)
