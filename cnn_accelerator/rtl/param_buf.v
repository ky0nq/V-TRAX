`timescale 1ns / 1ps

module param_buf #(
    parameter integer PARAM_TOTAL = 43      // 43 : Conv0 6 + Conv1 4 + fc1 32 + fc2 1
) (
    input  wire        clk,
    input  wire        rst_n,

    // cnn_cntl -> out_path
    input  wire               i_load_start,
    input  wire               i_wr_en,
    input  wire        [ 5:0] i_wr_addr,
    input  wire signed [31:0] i_wr_data,
    output reg                o_load_done,
    output reg                o_loaded,

    // -> post_process
    input  wire               i_rd_req_valid,
    output wire               o_rd_req_ready,
    input  wire        [ 5:0] i_rd_addr,
    output reg  signed [31:0] o_rd_data,
    output reg                o_rd_rsp_valid,
    input  wire               i_rd_rsp_ready      
);

    reg signed [31:0] mem [0:63];                // 64-depth. Using only 0 ~ 42
    reg        loading;                     // i_load_start ~ last store = High-state
    reg [ 5:0] wr_cnt;                      // Load counter 0 ~ 43

    wire wr_fire = loading && !i_load_start && i_wr_en;
    wire wr_last   = wr_fire && (wr_cnt == PARAM_TOTAL - 1); // 43th store
    wire req_fire  = i_rd_req_valid && o_rd_req_ready;
    wire rsp_fire  = o_rd_rsp_valid && i_rd_rsp_ready;

    // Read request Access state condition
    // 응답이 같은 클럭에 수락되면 다음 요청을 바로 받는다 (연속 읽기. 핸드셰이크 정리 2026-09-23)
    assign o_rd_req_ready = o_loaded && !loading && !i_load_start && (!o_rd_rsp_valid || i_rd_rsp_ready);

    // memory Read/Write
    always @(posedge clk) begin
        if (wr_fire)  mem[i_wr_addr] <= i_wr_data;
        if (req_fire) o_rd_data      <= mem[i_rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            loading        <= 1'b0;
            wr_cnt         <= 6'd0;
            o_load_done    <= 1'b0;
            o_loaded       <= 1'b0;
            o_rd_rsp_valid <= 1'b0;
        end else begin
            o_load_done <= 1'b0;                 

            if (i_load_start) begin // Load Start
                loading        <= 1'b1;
                wr_cnt         <= 6'd0;
                o_loaded       <= 1'b0;
                o_rd_rsp_valid <= 1'b0;
            end else if (wr_fire) begin
                wr_cnt <= wr_cnt + 6'd1;
                if (wr_last) begin // Last Parameter Store
                    loading     <= 1'b0;
                    o_loaded    <= 1'b1;
                    o_load_done <= 1'b1;
                end
            end

            // Read response
            if      (req_fire)      o_rd_rsp_valid <= 1'b1;
            else if (rsp_fire)      o_rd_rsp_valid <= 1'b0;
        end
    end

endmodule
