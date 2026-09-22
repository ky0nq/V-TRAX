`timescale 1ns / 1ps

// ============================================================================
// wgt_path : Weight 경로 Top
//
//   RAM -> wgt_ld_unit -> wgt_buf -> wgt_patch_gen -> wgt_feeder -> PE (skew)

// ============================================================================
module wgt_path (
    input  wire        clk,
    input  wire        rst_n,

    // ---- memory load ----
    input  wire        i_ld_start,
    input  wire [31:0] i_mem_base,
    input  wire [6:0]  i_chunk_word_count,
    output wire        o_mem_rd_en,
    output wire [31:0] o_mem_rd_addr,
    input  wire [23:0] i_mem_rdata,
    input  wire        i_mem_rvalid,
    output wire        o_ld_done,
    output wire        o_ld_ready,
    input  wire        i_buf_free,

    // ---- cnn_cntl ----
    input  wire        i_chunk_start,
    input  wire [2:0]  i_col_mask,
    output wire        o_chunk_done,
    input  wire        i_step_en,
    input  wire        i_clear,

    // ---- PE array (skew) ----
    output wire [23:0] o_data,
    output wire [2:0]  o_keep,
    output wire        o_valid,
    input  wire        i_ready
);

    wire        buf_we;
    wire [5:0]  buf_waddr;
    wire [23:0] buf_wdata;

    wire        gen_ren;
    wire [5:0]  gen_raddr;
    wire [23:0] buf_rdata;
    wire        buf_rvalid;

    wire [23:0] gen_data;
    wire [2:0]  gen_keep;
    wire        gen_valid;
    wire        gen_ready;

    wgt_ld_unit WGT_LD_UNIT (
        .clk          (clk),
        .rst_n        (rst_n),
        .i_ld_start (i_ld_start),
        .i_mem_base   (i_mem_base),
        .i_chunk_word_count  (i_chunk_word_count),
        .o_mem_rd_en  (o_mem_rd_en),
        .o_mem_rd_addr(o_mem_rd_addr),
        .i_mem_rdata  (i_mem_rdata),
        .i_mem_rvalid (i_mem_rvalid),
        .o_buf_we     (buf_we),
        .o_buf_waddr  (buf_waddr),
        .o_buf_wdata  (buf_wdata),
        .o_ld_done    (o_ld_done),
        .o_ld_ready   (o_ld_ready),
        .i_buf_free   (i_buf_free)
    );

    wgt_buf WGT_BUF (
        .clk     (clk),
        .rst_n   (rst_n),
        .i_we    (buf_we),
        .i_waddr (buf_waddr),
        .i_wdata (buf_wdata),
        .i_ren   (gen_ren),
        .i_raddr (gen_raddr),
        .o_rdata (buf_rdata),
        .o_rvalid(buf_rvalid)
    );

    wgt_patch_gen WGT_PATCH_GEN (
        .clk          (clk),
        .rst_n        (rst_n),
        .i_chunk_start(i_chunk_start),
        .i_chunk_word_count  (i_chunk_word_count),
        .i_col_mask   (i_col_mask),
        .o_ren        (gen_ren),
        .o_raddr      (gen_raddr),
        .i_rdata      (buf_rdata),
        .i_rvalid     (buf_rvalid),
        .o_data       (gen_data),
        .o_keep       (gen_keep),
        .o_valid      (gen_valid),
        .i_ready      (gen_ready),
        .o_chunk_done (o_chunk_done)
    );

    wgt_feeder WGT_FEEDER (
        .clk      (clk),
        .rst_n    (rst_n),
        .i_data   (gen_data),
        .i_keep   (gen_keep),
        .i_valid  (gen_valid),
        .o_ready  (gen_ready),
        .i_step_en(i_step_en),
        .i_clear  (i_clear),
        .o_data   (o_data),
        .o_keep   (o_keep),
        .o_valid  (o_valid),
        .i_ready  (i_ready)
    );

endmodule


// ============================================================================
// wgt_ld_unit : RAM -> wgt_buf 복사 (chunk_len word)
// ============================================================================
module wgt_ld_unit (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        i_ld_start,
    input  wire [31:0] i_mem_base,
    input  wire [6:0]  i_chunk_word_count,

    output wire        o_mem_rd_en,
    output wire [31:0] o_mem_rd_addr,
    input  wire [23:0] i_mem_rdata,
    input  wire        i_mem_rvalid,

    output wire        o_buf_we,
    output wire [5:0]  o_buf_waddr,
    output wire [23:0] o_buf_wdata,

    output reg         o_ld_done,
    output wire        o_ld_ready,
    input  wire        i_buf_free
);
    localparam [1:0] S_IDLE = 2'd0,
                     S_REQ  = 2'd1,
                     S_WAIT = 2'd2;

    reg [1:0]  state;
    reg [31:0] mem_base;
    reg [6:0]  chunk_len;
    reg [6:0]  idx;

    assign o_mem_rd_en   = rst_n && (state == S_REQ);
    assign o_mem_rd_addr = mem_base + {25'd0, idx};

    assign o_buf_we    = rst_n && (state == S_WAIT) && i_mem_rvalid;
    assign o_buf_waddr = idx[5:0];
    assign o_buf_wdata = i_mem_rdata;
    assign o_ld_ready  = rst_n && (state == S_IDLE) && i_buf_free;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            mem_base    <= 32'd0;
            chunk_len   <= 7'd0;
            idx         <= 7'd0;
            o_ld_done <= 1'b0;
        end else begin
            o_ld_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (i_ld_start && i_buf_free) begin
                        mem_base  <= i_mem_base;
                        chunk_len <= i_chunk_word_count;
                        idx       <= 7'd0;
                        if (i_chunk_word_count == 7'd0)
                            o_ld_done <= 1'b1;
                        else
                            state <= S_REQ;
                    end
                end

                S_REQ: state <= S_WAIT;

                S_WAIT: begin
                    if (i_mem_rvalid) begin
                        if (idx == chunk_len - 7'd1) begin
                            o_ld_done <= 1'b1;
                            state       <= S_IDLE;
                        end else begin
                            idx   <= idx + 7'd1;
                            state <= S_REQ;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule


// ============================================================================
// wgt_buf : 64 x 24bit
// ============================================================================
module wgt_buf (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        i_we,
    input  wire [5:0]  i_waddr,
    input  wire [23:0] i_wdata,

    input  wire        i_ren,
    input  wire [5:0]  i_raddr,

    output reg  [23:0] o_rdata,
    output reg         o_rvalid
);
    reg [23:0] mem [0:63];

    always @(posedge clk or negedge rst_n) begin
        if (rst_n && i_we)
            mem[i_waddr] <= i_wdata;

        if (rst_n && i_ren)
            o_rdata <= mem[i_raddr];

        if (!rst_n)
            o_rvalid <= 1'b0;
        else
            o_rvalid <= i_ren;
    end
endmodule


// ============================================================================
// wgt_patch_gen : buf를 순서대로 읽어 beat로 내보냄 (1 beat/cycle)
// ============================================================================
module wgt_patch_gen (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        i_chunk_start,
    input  wire [6:0]  i_chunk_word_count,
    input  wire [2:0]  i_col_mask,

    output wire        o_ren,
    output wire [5:0]  o_raddr,
    input  wire [23:0] i_rdata,
    input  wire        i_rvalid,

    output wire [23:0] o_data,
    output wire [2:0]  o_keep,
    output wire        o_valid,
    input  wire        i_ready,

    output reg         o_chunk_done
);
    reg        running;
    reg [6:0]  len_q;
    reg [2:0]  mask_q;
    reg [6:0]  req_idx;      // 다음에 읽을 word
    reg [1:0]  pend;         // 요청했지만 아직 소비 안 된 word (in-flight + FIFO)

    // 2-entry FIFO
    reg [23:0] fifo0, fifo1;
    reg        head, tail;
    reg [1:0]  cnt;

    wire pop   = o_valid && i_ready;
    wire issue = running && (req_idx != len_q) && ((pend < 2'd2) || pop);

    assign o_ren   = issue;
    assign o_raddr = req_idx[5:0];

    wire [23:0] head_word = head ? fifo1 : fifo0;
    assign o_valid = running && (cnt != 2'd0);
    assign o_keep  = mask_q;
    assign o_data  = { mask_q[2] ? head_word[23:16] : 8'd0,
                       mask_q[1] ? head_word[15:8]  : 8'd0,
                       mask_q[0] ? head_word[7:0]   : 8'd0 };

    wire push = running && i_rvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running      <= 1'b0;
            len_q        <= 7'd0;
            mask_q       <= 3'b000;
            req_idx      <= 7'd0;
            pop_idx      <= 7'd0;
            pend         <= 2'd0;
            head         <= 1'b0;
            tail         <= 1'b0;
            cnt          <= 2'd0;
            o_chunk_done <= 1'b0;
        end else begin
            o_chunk_done <= 1'b0;

            if (!running) begin
                if (i_chunk_start) begin
                    len_q   <= i_chunk_word_count;
                    mask_q  <= i_col_mask;
                    req_idx <= 7'd0;
                    pop_idx <= 7'd0;
                    pend    <= 2'd0;
                    head    <= 1'b0;
                    tail    <= 1'b0;
                    cnt     <= 2'd0;
                    if (i_chunk_word_count == 7'd0)
                        o_chunk_done <= 1'b1;
                    else
                        running <= 1'b1;
                end
            end else begin
                if (issue)
                    req_idx <= req_idx + 7'd1;

                pend <= pend + {1'b0, issue} - {1'b0, pop};

                if (push) begin
                    if (tail) fifo1 <= i_rdata;
                    else      fifo0 <= i_rdata;
                    tail <= ~tail;
                end

                if (pop)
                    head <= ~head;

                cnt <= cnt + {1'b0, push} - {1'b0, pop};

                if (pop && (req_idx == len_q) && (pend == 2'd1)) begin
                    running      <= 1'b0;
                    o_chunk_done <= 1'b1;
                end
            end
        end
    end

endmodule

// ============================================================================
// wgt_feeder : 1-stage register slice + step_en (act_feeder와 동일 구조)
//   skew 버퍼로 넘기는 출력단.
// ============================================================================
module wgt_feeder (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [23:0] i_data,
    input  wire [2:0]  i_keep,
    input  wire        i_valid,
    output wire        o_ready,

    input  wire        i_step_en,
    input  wire        i_clear,

    output reg  [23:0] o_data,
    output reg  [2:0]  o_keep,
    output reg         o_valid,
    input  wire        i_ready
);
    assign o_ready = rst_n && !i_clear && i_step_en && (!o_valid || i_ready);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_data  <= 24'd0;
            o_keep  <= 3'b000;
            o_valid <= 1'b0;
        end else if (i_clear) begin
            o_data  <= 24'd0;
            o_keep  <= 3'b000;
            o_valid <= 1'b0;
        end else begin
            if (o_ready) begin
                o_valid <= i_valid;
                if (i_valid) begin
                    o_data <= i_data;
                    o_keep <= i_keep;
                end
            end else if (o_valid && i_ready) begin
                o_valid <= 1'b0;
            end
        end
    end
endmodule
