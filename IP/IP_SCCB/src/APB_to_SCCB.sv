`timescale 1ns / 1ps

module APB_to_SCCB (
    input  logic        PCLK,
    input  logic        PRESETn,

    // ===== Bridge와 연결되는 APB 신호 =====
    input  logic        PSEL,
    input  logic        PENABLE,
    input  logic        PWRITE,
    input  logic [31:0] PADDR,
    input  logic [15:0] PWDATA,
    input  logic [1:0]  PSTRB,
    input  logic [2:0]  PPROT,

    output logic [15:0] PRDATA,
    output logic        PREADY,
    output logic        PSLVERR,

    // ===== SCCB_sequencer로 나가는 제어신호 =====
    output logic        o_wstart,
    output logic        o_rstart,
    output logic        o_op,
    output logic [7:0]  o_reg_addr,
    output logic [7:0]  o_reg_data,

    // ===== OV7670_engine에서 들어오는 결과 =====
    input  logic        i_done,
    input  logic [7:0]  i_read_data,

    // ===== 인터럽트 출력 =====
    output logic        o_irq
);

    logic [7:0] reg_addr_r;
    logic [7:0] wdata_r;
    logic       irq_en_r;
    logic       busy_r;
    logic       done_r;

    assign o_reg_addr  = reg_addr_r;
    assign o_irq       = irq_en_r & done_r;

    logic strb_lo;
    logic strb_hi;
    assign strb_lo = PSTRB[0];
    assign strb_hi = PSTRB[1];

    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            PREADY     <= 1'b0;
            PSLVERR    <= 1'b0;
            PRDATA     <= 16'h0;
            o_wstart   <= 1'b0;
            o_rstart   <= 1'b0;
            o_op       <= 1'b0;
            o_reg_data <= 8'h0;
            reg_addr_r <= 8'h0;
            wdata_r    <= 8'h0;
            irq_en_r   <= 1'b0;
            busy_r     <= 1'b0;
            done_r     <= 1'b0;
        end
        else begin
            o_wstart <= 1'b0;
            o_rstart <= 1'b0;

            if (i_done) begin
                busy_r <= 1'b0;
                done_r <= 1'b1;
            end

            if (PSEL && PENABLE) begin
                PREADY  <= 1'b1;
                PSLVERR <= 1'b0;

                if (PWRITE) begin
                    case (PADDR[3:1])
                        3'd0 : begin                      // REG (offset 0)
                            if (strb_lo) reg_addr_r <= PWDATA[7:0];
                        end
                        3'd1 : begin                      // WDATA (offset 2)
                            if (strb_lo) begin
                                o_op       <= PWDATA[1];
                                wdata_r    <= PWDATA[9:2];
                                o_reg_data <= PWDATA[9:2];
                                if (PWDATA[0] && !busy_r) begin
                                    o_wstart <= 1'b1;
                                    busy_r   <= 1'b1;
                                    done_r   <= 1'b0;
                                end
                            end
                        end
                        3'd2 : begin                      // RDATA (offset 4)
                            if (strb_lo) begin
                                if (PWDATA[0] && !busy_r) begin
                                    o_rstart <= 1'b1;
                                    busy_r   <= 1'b1;
                                    done_r   <= 1'b0;
                                end
                            end
                        end
                        3'd4 : begin                      // IRQ (offset 8)
                            if (strb_lo) irq_en_r <= PWDATA[0];
                        end
                        default : PSLVERR <= 1'b1;
                    endcase
                end
                else begin
                    case (PADDR[3:1])
                        3'd0 : PRDATA <= {8'h0, reg_addr_r};
                        3'd1 : PRDATA <= {6'h0, wdata_r, o_op, 1'b0};
                        3'd2 : PRDATA <= {7'h0, i_read_data, 1'b0};
                        3'd3 : PRDATA <= {14'h0, done_r, busy_r};
                        3'd4 : PRDATA <= {15'h0, irq_en_r};
                        default : begin
                            PRDATA  <= 16'h0;
                            PSLVERR <= 1'b1;
                        end
                    endcase
                end
            end
            else begin
                PREADY  <= 1'b0;
                PSLVERR <= 1'b0;
            end
        end
    end

endmodule
