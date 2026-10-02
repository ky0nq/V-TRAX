`timescale 1ns / 1ps


module APB_to_GPIO(
    input  logic        PCLK,
    input  logic        PRESETn,

    // ===== APB signals connected to the Bridge =====
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

    // ===== Signals going out to GPIO =====
    output logic [15:0] o_cr,
    output logic [15:0] o_aodr,

    // ===== Signals coming in from GPIO =====
    input  logic [15:0] i_idr,

    // ===== Interrupt output =====
    output logic [7:0]  o_irq
    );

    logic [15:0] cr_r, aodr_r;
    logic [15:0] idr_prev_r; // prev holds the idr value from the previous clock
    logic [7:0]  irq_r;

    assign o_cr   = cr_r;
    assign o_aodr = aodr_r;
    assign o_irq  = irq_r;

    logic strb_lo;
    assign strb_lo = PSTRB[0];
    logic strb_hi;
    assign strb_hi = PSTRB[1];

    // 0->1 posedge: pin is 1 now & pin was 0 on the previous clock & currently in input mode
    logic [7:0] w_posedge ;
    assign w_posedge = i_idr[7:0] & ~idr_prev_r & ~cr_r[7:0];

    always_ff @(posedge PCLK, negedge PRESETn) begin
        if (!PRESETn) begin
            PREADY     <= 1'b0;
            PSLVERR    <= 1'b0;
            PRDATA     <= 16'h0;
            cr_r       <= 16'h0;
            aodr_r     <= 16'h0;
            idr_prev_r <= 16'h0;
            irq_r      <= 8'h0;
        end else begin
            idr_prev_r <= i_idr; // store previous value
            irq_r      <= irq_r | w_posedge; 
            if (PSEL && PENABLE) begin
                PREADY  <= 1'b1;
                PSLVERR <= 1'b0;
                // =========== Write Access ===========
                if (PWRITE) begin
                    case (PADDR[3:0])
                        4'd0: begin                                   // CR (offset 0)
                            if (strb_lo) cr_r[7:0]  <= PWDATA[7:0];
                            if (strb_hi) cr_r[15:8] <= PWDATA[15:8];
                        end
                        4'd4: begin                                   // OUT (offset 4)
                            if (strb_lo) aodr_r[7:0]  <= PWDATA[7:0];
                            if (strb_hi) aodr_r[15:8] <= PWDATA[15:8];
                        end
                        4'd6: begin
                            // the CPU writes 1 to the bits it wants to clear, so
                            // irq_r & ~PWDATA: buttons the CPU wrote 1 to become 0, buttons written with 0 keep their value
                            if (strb_lo) irq_r <= (irq_r & ~PWDATA[7:0]) | w_posedge;
                        end
                        default : PSLVERR <= 1'b1;
                    endcase
                // =========== Read Access ===========
                end else begin
                    case (PADDR[3:0])
                        4'd0 : PRDATA <= cr_r;
                        4'd2 : PRDATA <= i_idr;
                        4'd4 : PRDATA <= aodr_r;
                        4'd6 : PRDATA <= {8'h0, irq_r};
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
