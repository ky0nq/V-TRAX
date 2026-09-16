`timescale 1ns / 1ps

module SCCB_sequencer #(BYTE_SWAP = 0)(
        input  logic        clk,
        input  logic        rst,

        input  logic        i_wstart,     // 쓰기 시작 (WDATA 레지스터의 Wstart)
        input  logic        i_rstart,     // 읽기 시작 (RDATA 레지스터의 Rstart)
        input  logic        i_op,         // 0=수동, 1=자동 (wstart일때만 의미있음)
        input  logic [7:0]  i_reg_addr,   // 수동용 레지스터주소
        input  logic [7:0]  i_reg_data,   // 수동용 데이터
        input  logic        i_done,       // Camera가 주는 완료신호 (레지스터 하나 쓸때마다 뜸)

        output logic        o_rw,        // 카메라의 rw 포트로
        output logic        o_start,     // 카메라의 set_start 포트로
        output logic [6:0]  o_addr,      // 카메라의 ADDRESS 포트로 (고정값)
        output logic [7:0]  o_reg_addr,  // 카메라의 REG 포트로
        output logic [7:0]  o_reg_data,  // 카메라의 WDATA 포트로
        output logic        o_seq_done   // 전체 작업 정말 끝났을 때
    );

    localparam write_num = 74;
    localparam IDLE = 0, WRITE = 1, READ = 2, WAIT = 3;

    logic [15:0] set_data;
    logic [1:0]  state;
    logic [6:0]  w_cnt;
    logic        END;

    assign o_addr     = 7'h21;          // 카메라 고정 슬레이브주소
    assign o_reg_addr = set_data[15:8];
    assign o_reg_data = set_data[7:0];

    // ===== 엣지검출: wstart/rstart 각각으로 =====
    logic wstart_d, rstart_d;
    logic wstart_pe, rstart_pe;

    always @(posedge clk, posedge rst) begin
        if (rst) begin wstart_d <= 0; rstart_d <= 0; end
        else begin wstart_d <= i_wstart; rstart_d <= i_rstart; end
    end

    assign wstart_pe = (i_wstart && !wstart_d);
    assign rstart_pe = (i_rstart && !rstart_d);

    // ===== done 신호 =====
    logic done_hold;

    logic [$clog2(150_000)-1:0] reset_wait;
    logic reset_wait_start;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            state <= IDLE;
            w_cnt <= 0;
            set_data <= 0;
            o_rw <= 0;
            o_start <= 0;
            reset_wait <= 0;
            END <= 0;
            reset_wait_start <= 0;
            done_hold <= 0;
            o_seq_done <= 0;
        end
        else begin
            if (i_done) done_hold <= 1;
            o_seq_done <= 0;

            case (state)
                IDLE : begin
                    o_start <= 0;
                    if      (wstart_pe) state <= WRITE;   //  둘다 WRITE로
                    else if (rstart_pe) state <= READ;    // 읽기는 항상 수동
                    else                state <= IDLE;
                    END <= 0;
                end

                WRITE : begin
                    o_rw <= 0;
                    o_start <= 1;
                    state <= WAIT;

                    if (i_op) begin   // 자동모드
                        END <= 0;
                        case (w_cnt)
                            7'd0 : set_data = 16'h12_80;
                            7'd1 : set_data = 16'h12_14;
                            7'd2 : set_data = {8'h0C, BYTE_SWAP ? 8'h44 : 8'h04};
                            7'd3 : set_data = 16'h3E_19;
                            7'd4 : set_data = 16'h11_01;
                            7'd5 : set_data = 16'h40_D0;
                            7'd6 : set_data = 16'h3A_04;
                            7'd7 : set_data = 16'h14_18;
                            7'd8 : set_data = 16'h4F_B3;
                            7'd9 : set_data = 16'h50_B3;
                            7'd10: set_data = 16'h51_00;
                            7'd11: set_data = 16'h52_3D;
                            7'd12: set_data = 16'h53_A7;
                            7'd13: set_data = 16'h54_E4;
                            7'd14: set_data = 16'h58_9E;
                            7'd15: set_data = 16'h3D_C0;
                            7'd16: set_data = 16'h17_13;
                            7'd17: set_data = 16'h18_01;
                            7'd18: set_data = 16'h32_B6;
                            7'd19: set_data = 16'h19_02;
                            7'd20: set_data = 16'h1A_7A;
                            7'd21: set_data = 16'h03_0A;
                            7'd22: set_data = 16'h0F_41;
                            7'd23: set_data = 16'h1E_00;
                            7'd24: set_data = 16'h33_0B;
                            7'd25: set_data = 16'h3C_78;
                            7'd26: set_data = 16'h69_00;
                            7'd27: set_data = 16'h74_00;
                            7'd28: set_data = 16'hB0_84;
                            7'd29: set_data = 16'hB1_0C;
                            7'd30: set_data = 16'hB2_0E;
                            7'd31: set_data = 16'hB3_80;
                            7'd32: set_data = 16'h70_3A;
                            7'd33: set_data = 16'h71_35;
                            7'd34: set_data = 16'h72_11;
                            7'd35: set_data = 16'h73_F1;
                            7'd36: set_data = 16'hA2_02;
                            7'd37: set_data = 16'h7A_20;
                            7'd38: set_data = 16'h7B_10;
                            7'd39: set_data = 16'h7C_1E;
                            7'd40: set_data = 16'h7D_35;
                            7'd41: set_data = 16'h7E_5A;
                            7'd42: set_data = 16'h7F_69;
                            7'd43: set_data = 16'h80_76;
                            7'd44: set_data = 16'h81_80;
                            7'd45: set_data = 16'h82_88;
                            7'd46: set_data = 16'h83_8F;
                            7'd47: set_data = 16'h84_96;
                            7'd48: set_data = 16'h85_A3;
                            7'd49: set_data = 16'h86_AF;
                            7'd50: set_data = 16'h87_C4;
                            7'd51: set_data = 16'h88_D7;
                            7'd52: set_data = 16'h89_E8;
                            7'd53: set_data = 16'h13_E0;
                            7'd54: set_data = 16'h00_00;
                            7'd55: set_data = 16'h10_00;
                            7'd56: set_data = 16'h0D_40;
                            7'd57: set_data = 16'h15_00;
                            7'd58: set_data = 16'hA5_05;
                            7'd59: set_data = 16'hAB_07;
                            7'd60: set_data = 16'h24_70;
                            7'd61: set_data = 16'h25_20;
                            7'd62: set_data = 16'h26_E3;
                            7'd63: set_data = 16'h9F_78;
                            7'd64: set_data = 16'hA0_68;
                            7'd65: set_data = 16'hA1_03;
                            7'd66: set_data = 16'hA6_D8;
                            7'd67: set_data = 16'hA7_D8;
                            7'd68: set_data = 16'hA8_F0;
                            7'd69: set_data = 16'hA9_90;
                            7'd70: set_data = 16'hAA_94;
                            7'd71: set_data = 16'h13_E7;
                            7'd72: set_data = 16'h69_07;
                            7'd73: set_data <= {8'h42, 8'h00};
                            default: set_data <= {8'h15, 8'h00};
                        endcase
                    end
                    else begin   // 수동모드
                        END <= 1;
                        set_data <= {i_reg_addr, i_reg_data};
                    end
                end

                READ : begin   // 항상 수동 읽기
                    o_rw <= 1;
                    o_start <= 1;
                    state <= WAIT;
                    set_data <= {i_reg_addr, 8'h00};
                end

                WAIT : begin
                    o_start <= 0;
                    if (reset_wait_start) begin
                        if (reset_wait >= (150_000-1)) begin
                            reset_wait <= 0;
                            state <= WRITE;
                            w_cnt <= w_cnt+1;
                            reset_wait_start <= 0;
                        end
                        else begin
                            reset_wait <= reset_wait+1;
                            state <= WAIT;
                        end
                    end
                    else if (done_hold && o_rw) begin
                        // 읽기 완료 
                        state <= IDLE;
                        done_hold <= 0;
                        o_seq_done <= 1;
                    end
                    else if (done_hold && !o_rw) begin
                        done_hold <= 0;
                        if (w_cnt == 0) reset_wait_start <= 1;
                        else begin
                            reset_wait_start <= 0;
                            if (i_op) begin
                                if (w_cnt >= write_num-1) begin
                                    w_cnt <= 0;
                                    state <= IDLE;
                                    o_seq_done <= 1;
                                end
                                else begin w_cnt <= w_cnt+1; state <= WRITE; end
                            end
                            else begin
                                state <= IDLE;
                                o_seq_done <= 1;
                            end
                        end
                    end
                    else state <= WAIT;
                end

                default : state <= IDLE;
            endcase
        end
    end
endmodule