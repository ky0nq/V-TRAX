`timescale 1ns / 1ps
//
// 쓰기 쪽(Read Engine)   : fifo_wr_en / fifo_wr_data / fifo_full
// 읽기 쪽(depacketizer)  : fifo_rd_en / fifo_rd_data / fifo_empty
//

module fifo #(
    parameter integer DATA_WIDTH = 32,
    parameter integer DEPTH      = 16,
    parameter integer ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ===== 쓰기 쪽 (AXI4_read_engine과 연결) =====
    input  wire                    fifo_wr_en,
    input  wire [DATA_WIDTH-1:0]   fifo_wr_data,
    output wire                    fifo_full,

    // ===== 읽기 쪽 (axis_frame_depacketizer 어댑터와 연결) =====
    input  wire                    fifo_rd_en,
    output wire [DATA_WIDTH-1:0]   fifo_rd_data,
    output wire                    fifo_empty,

    // ===== 참고용 상태 (디버깅/almost-full 판단 등에 필요하면 사용) =====
    output wire [ADDR_WIDTH:0]     fifo_count      // 0 ~ DEPTH, 현재 채워진 개수
);

    // ------------------------------------------------------------------
    // 메모리 배열 (합성 시 분산 RAM 또는 BRAM으로 자동 매핑됨, DEPTH 작으면 LUT-RAM)
    // ------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   count;   // 1비트 더 넓게 둬서 '꽉 참'과 '텅 빔' 구분

    assign fifo_full  = (count == DEPTH[ADDR_WIDTH:0]);
    assign fifo_empty = (count == {(ADDR_WIDTH+1){1'b0}});
    assign fifo_count = count;

    wire do_write = fifo_wr_en && !fifo_full;
    wire do_read  = fifo_rd_en && !fifo_empty;

    // ------------------------------------------------------------------
    // 쓰기 포인터 + 메모리 쓰기
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= {ADDR_WIDTH{1'b0}};
        end else if (do_write) begin
            mem[wr_ptr] <= fifo_wr_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // 읽기 포인터 (읽기 데이터는 아래 조합논리 read로 처리 - fall-through 방식)
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {ADDR_WIDTH{1'b0}};
        end else if (do_read) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    assign fifo_rd_data = mem[rd_ptr];

    // ------------------------------------------------------------------
    // 개수 카운터
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            count <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            case ({do_write, do_read})
                2'b10:   count <= count + 1'b1;   // 쓰기만
                2'b01:   count <= count - 1'b1;   // 읽기만
                default: count <= count;          // 둘 다 안 하거나, 동시에 하면 상쇄
            endcase
        end
    end

`ifndef SYNTHESIS
    // 시뮬레이션 안전장치: full인데 write 시도, empty인데 read 시도하면 경고
    always @(posedge clk) begin
        if (rst_n && fifo_wr_en && fifo_full)
            $display("[WARN][channel1_data_fifo] write attempted while FULL at time %0t", $time);
        if (rst_n && fifo_rd_en && fifo_empty)
            $display("[WARN][channel1_data_fifo] read attempted while EMPTY at time %0t", $time);
    end
`endif

endmodule