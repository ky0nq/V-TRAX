// ============================================================================
// axis_interface : AXI4-Stream (camera 입력 / disp 출력 공통)
//
//   [ICPSD 경고 해결 방식]
//   - 버스 신호(tdata ~ tready)는 wire 로 선언 -> DUT 와 TB 가 같이 물려도 됨
//   - TB 가 실제로 값을 넣는 곳은 m_* / s_tready 변수이고, assign 으로 버스에 연결
//   - 이 변수들의 초기값은 'z  -> 안 쓰는 쪽은 z 로 남아서 DUT 값을 방해하지 않음
//       camera(vin)  : master 라서 m_* 만 씀  -> s_tready 는 z  -> tready 는 DUT 값
//       disp  (vout) : slave  라서 s_tready 만 씀 -> m_* 는 z   -> tdata 등은 DUT 값
//   - clocking 안의 이름(mst_cb.tdata 등)은 그대로라서 driver / monitor 코드는 수정 없음
// ============================================================================
interface axis_interface #(parameter int DW = 24, parameter int KW = 3, parameter int UW = 1)
                          (input logic clk, input logic rst_n);

    // ---------------- 버스 (DUT 와 연결되는 신호) ----------------
    wire [DW-1:0] tdata;
    wire [KW-1:0] tkeep;
    wire [UW-1:0] tuser;
    wire          tlast;
    wire          tvalid;
    wire          tready;

    // ---------------- TB 가 구동하는 쪽 (안 쓰면 z 로 남음) ----------------
    logic [DW-1:0] m_tdata  = 'z;
    logic [KW-1:0] m_tkeep  = 'z;
    logic [UW-1:0] m_tuser  = 'z;
    logic          m_tlast  = 1'bz;
    logic          m_tvalid = 1'bz;
    logic          s_tready = 1'bz;

    assign tdata  = m_tdata;
    assign tkeep  = m_tkeep;
    assign tuser  = m_tuser;
    assign tlast  = m_tlast;
    assign tvalid = m_tvalid;
    assign tready = s_tready;

    // ---------------- master (camera) ----------------
    clocking mst_cb @(posedge clk);
        default input #1step output #0;
        output tdata  = m_tdata;
        output tkeep  = m_tkeep;
        output tuser  = m_tuser;
        output tlast  = m_tlast;
        output tvalid = m_tvalid;
        input  tready;
    endclocking

    // ---------------- slave (disp) ----------------
    clocking slv_cb @(posedge clk);
        default input #1step output #0;
        input  tdata, tkeep, tuser, tlast, tvalid;
        output tready = s_tready;
    endclocking

    // ---------------- monitor ----------------
    clocking mon_cb @(posedge clk);
        default input #1step;
        input tdata, tkeep, tuser, tlast, tvalid, tready;
    endclocking

    modport MST(clocking mst_cb, input clk, input rst_n);
    modport SLV(clocking slv_cb, input clk, input rst_n);
    modport MON(clocking mon_cb, input clk, input rst_n);
endinterface