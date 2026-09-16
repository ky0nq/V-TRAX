`timescale 1ns / 1ps

module APB_to_UART (
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

    // ===== uart로 나가는 신호 =====
    output logic [7:0]  o_tx_data,  
    output logic        o_tx_valid,
    input  logic        i_tx_ready, 

    input  logic        i_rx_valid,
    input  logic [7:0]  i_rx_data, 

    // ===== 인터럽트 출력 =====
    output logic        o_irq
);

    logic [7:0]  rx_data_r;   // 마지막으로 받은 rx 데이터
    logic [7:0]  tx_data_r;   // 마지막으로 쓴 tx 데이터
    logic        irq_en_r;    // 인터럽트 활성화 여부

    logic w_busy;
    assign w_busy = ~i_tx_ready;
    logic strb_lo;
    assign strb_lo = PSTRB[0];
	assign o_irq = i_rx_valid & irq_en_r;


    always_ff @(posedge PCLK, negedge PRESETn) begin
        if (!PRESETn) begin
            PREADY     <= 1'b0;
            PSLVERR    <= 1'b0;
            PRDATA     <= 16'h0;
            o_tx_valid <= 1'b0;
            o_tx_data  <= 8'h0;
            rx_data_r  <= 8'h0;
            tx_data_r  <= 8'h0;
            irq_en_r   <= 1'b0;
        end else begin
            o_tx_valid <= 1'b0; 
            // ---- UART core 쪽 수신 신호 반영 ----
            if (PSEL && PENABLE) begin
                PREADY  <= 1'b1;
                PSLVERR <= 1'b0;
                // =========== Write Access ===========
                if (PWRITE) begin
                    case (PADDR[3:0])
                        4'd2 : begin                              // TX_DATA (offset 2)
                            if (strb_lo) begin
                                tx_data_r <= PWDATA[8:1];
                                o_tx_data <= PWDATA[8:1];
                                if (PWDATA[0] && i_tx_ready) begin // start bit && idle일 때만
                                    o_tx_valid <= 1'b1;
                                end
                            end
                        end
                        4'd6 : begin                              // IRQ_EN (offset 6)
                            if (strb_lo) irq_en_r <= PWDATA[0];
                        end
                        default : PSLVERR <= 1'b1;
                    endcase
                // =========== Read Access ===========
                end else begin
                    case (PADDR[3:0])
                        4'd0 : PRDATA <= {7'h0, i_rx_data, i_rx_valid};
                        4'd2 : PRDATA <= {7'h0, tx_data_r, 1'b0};  // TX_DATA (offset 2)
                        4'd4 : PRDATA <= {15'h0, w_busy};          // STATUS (offset 4)
                        4'd6 : PRDATA <= {15'h0, irq_en_r};        // IRQ_EN (offset 6)
                        default : begin
                            PRDATA  <= 16'h0;
                            PSLVERR <= 1'b1;
                        end
                    endcase
                end
            end else begin
                PREADY  <= 1'b0;
                PSLVERR <= 1'b0;
            end
        end
    end

endmodule
