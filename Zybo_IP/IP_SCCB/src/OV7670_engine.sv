`timescale 1ns / 1ps

module OV7670_engine #(
    parameter len_x = 320,
    parameter len_y = 240
)(
    // ===== System =====
    input  logic                     clk,
    input  logic                     rst,
    input  logic                     pclk,
    input  logic                     i_href,
    input  logic                     i_vsync,
    input  logic [7:0]               i_cam_data,

    // ===== SCCB_sequencer로부터 받는 제어신호 =====
    input  logic                     i_rw,
    input  logic                     i_start,
    input  logic [6:0]               i_addr,
    input  logic [7:0]               i_reg_addr,
    input  logic [7:0]               i_reg_data,

    output logic [15:0]              o_pixel_data,
    output logic                     o_pixel_valid,
    output logic [$clog2(len_x)-1:0] o_pixel_x,
    output logic [$clog2(len_y)-1:0] o_pixel_y,
    output logic [7:0]               o_reg_data,
    output logic                     o_done,

    input  logic                     i_siod,
    output logic                     o_sioc,
    output logic                     o_siod_en,
    output logic                     o_siod,
    output logic                     o_vga_start,
    output logic                     o_pwdn
);

    localparam C_IDLE=0, C_PHASE1=1, C_PHASE2=2;
    localparam S_IDLE=0, S_START=1, S_SADDR=2, S_ADDR=3, S_REG=4, S_DATA=5,
               S_ADDR_ACK=6, S_REG_ACK=7, S_DATA_ACK=8, S_STOP=9, S_WAIT=10;

    logic                     sioc_en;
    logic                     sioc_d;
    logic [1:0]               c_state;
    logic [3:0]               s_state;
    logic [8:0]               mode_cnt;
    logic [$clog2(len_x)-1:0] n_pixel_x;
    logic [$clog2(len_y)-1:0] n_pixel_y;
    logic                     sioc_tick;
    logic                     sioc_pe;
    logic                     sioc_fe;

    tick_gen #(.COUNT(500)) tick_500 (.clk(clk), .rst(rst), .tick(sioc_tick));

    assign o_pwdn  = 1'b0;

    logic vsync_d;

    always @(posedge pclk, posedge rst) begin
        if (rst) begin
            c_state <= C_PHASE1;
            vsync_d <= 0;
            o_pixel_valid <= 0;
            o_pixel_x <= 0;
            o_pixel_y <= 0;
            n_pixel_x <= 0;
            n_pixel_y <= 0;
            o_pixel_data <= 0;
            o_vga_start <= 0;
        end else begin
            if (!i_vsync && i_href && c_state == C_PHASE2)
                o_pixel_valid <= 1;
            else o_pixel_valid <= 0;
            vsync_d <= i_vsync;

            if (i_vsync) begin
                c_state <= C_PHASE1;
                o_vga_start <= 1;
                o_pixel_valid <= 0;
                o_pixel_x <= 0;
                o_pixel_y <= 0;
                n_pixel_x <= 0;
                n_pixel_y <= 0;
            end else if (!i_href) begin
                c_state <= C_PHASE1;
                o_vga_start <= 0;
                o_pixel_valid <= 0;
                o_pixel_x <= 0;
                n_pixel_x <= 0;
            end else begin
                o_vga_start <= 0;
                case (c_state)
                    C_PHASE1 : begin
                        if (i_href) begin
                            c_state <= C_PHASE2;
                            o_pixel_data[15:11] <= i_cam_data[7:3];
                            o_pixel_data[10:8]  <= i_cam_data[2:0];
                            o_pixel_x <= n_pixel_x;
                            o_pixel_y <= n_pixel_y;
                        end else c_state <= C_PHASE1;
                    end
                    C_PHASE2 : begin
                        if (!i_vsync && i_href) begin
                            c_state <= C_PHASE1;
                            o_pixel_data[7:5] <= i_cam_data[7:5];
                            o_pixel_data[4:0] <= i_cam_data[4:0];

                            if (n_pixel_x >= len_x-1) begin
                                n_pixel_x <= 0;
                                if (n_pixel_y >= len_y-1) n_pixel_y <= 0;
                                else n_pixel_y <= n_pixel_y + 1;
                            end else n_pixel_x <= n_pixel_x + 1;
                        end else c_state <= C_PHASE1;
                    end
                    default : c_state <= C_PHASE1;
                endcase
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) o_sioc <= 1'b1;
        else if (sioc_tick) begin
            if (sioc_en) o_sioc <= ~o_sioc;
            else o_sioc <= 1'b1;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) sioc_d <= 1'b0;
        else sioc_d <= o_sioc;
    end

    assign sioc_pe = (!sioc_d && o_sioc) ? 1 : 0;
    assign sioc_fe = (sioc_d && !o_sioc) ? 1 : 0;

    logic [7:0] siod_addr_r;
    logic [7:0] siod_reg_r;
    logic [7:0] siod_rdata_r;
    logic [7:0] siod_wdata_r;

    logic flag_pe_en;
    logic flag_fe_en;

    logic read_phase;
    logic [2:0] count_8;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            flag_pe_en <= 0;
            flag_fe_en <= 0;
            s_state <= S_IDLE;
            count_8 <= 0;
            o_siod <= 1;
            o_siod_en <= 0;
            sioc_en <= 0;
            siod_addr_r <= 0;
            siod_reg_r <= 0;
            siod_wdata_r <= 0;
            o_done <= 0;
            siod_rdata_r <= 0;
            read_phase <= 0;
            mode_cnt <= 0;
            o_reg_data <= 0;
        end else begin
            case (s_state)
                S_IDLE : begin
                    flag_pe_en <= 0;
                    flag_fe_en <= 0;
                    o_done <= 0;
                    count_8 <= 0;
                    o_siod_en <= 1;
                    sioc_en <= 0;
                    o_siod <= 1;
                    siod_rdata_r <= 0;
                    mode_cnt <= 0;
                    if (i_start) begin
                        s_state <= S_START;
                        sioc_en <= 0;
                        o_siod <= 0;
                    end
                end
                S_START : begin
                    o_siod_en <= 1;
                    o_siod <= 0;
                    if (mode_cnt >= 60) begin
                        sioc_en <= 1;
                        s_state <= S_SADDR;
                        mode_cnt <= 0;
                    end else mode_cnt <= mode_cnt + 1;
                    if (!i_rw) begin
                        siod_reg_r <= i_reg_addr;
                        siod_wdata_r <= i_reg_data;
                        siod_addr_r <= {i_addr, 1'b0};
                    end else begin
                        siod_wdata_r <= 0;
                        if (read_phase) begin
                            siod_reg_r <= 0;
                            siod_addr_r <= {i_addr, 1'b1};
                        end else begin
                            siod_reg_r <= i_reg_addr;
                            siod_addr_r <= {i_addr, 1'b0};
                        end
                    end
                end
                S_SADDR : begin
                    if (sioc_pe) flag_pe_en <= 1;
                    else if (sioc_fe) flag_fe_en <= 1;

                    if (flag_fe_en) begin
                        if (mode_cnt >= 100) begin
                            mode_cnt <= 0;
                            o_siod_en <= 1;
                            flag_fe_en <= 0;
                            o_siod <= siod_addr_r[7-count_8];
                        end else mode_cnt <= mode_cnt + 1;
                    end else if (flag_pe_en) begin
                        count_8 <= count_8 + 1;
                        s_state <= S_ADDR;
                        flag_pe_en <= 0;
                    end
                end
                S_ADDR : begin
                    if (sioc_pe) flag_pe_en <= 1;
                    else if (sioc_fe) flag_fe_en <= 1;

                    if (flag_fe_en) begin
                        if (mode_cnt >= 50) begin
                            mode_cnt <= 0;
                            o_siod_en <= 1;
                            flag_fe_en <= 0;
                            o_siod <= siod_addr_r[7-count_8];
                        end else mode_cnt <= mode_cnt + 1;
                    end else if (flag_pe_en) begin
                        if (count_8 == 7) begin
                            count_8 <= 0;
                            s_state <= S_ADDR_ACK;
                        end else count_8 <= count_8 + 1;
                        flag_pe_en <= 0;
                    end
                end
                S_ADDR_ACK : begin
                    if (sioc_fe) begin
                        s_state <= S_ADDR_ACK;
                        o_siod_en <= 0;
                    end
                    if (sioc_pe) begin
                        if (!i_rw) s_state <= S_REG;
                        else begin
                            if (read_phase) s_state <= S_DATA;
                            else s_state <= S_REG;
                        end
                    end
                end
                S_REG : begin
                    if (sioc_pe) flag_pe_en <= 1;
                    else if (sioc_fe) flag_fe_en <= 1;

                    if (flag_fe_en) begin
                        if (mode_cnt >= 50) begin
                            mode_cnt <= 0;
                            o_siod_en <= 1;
                            flag_fe_en <= 0;
                            o_siod <= siod_reg_r[7-count_8];
                        end else mode_cnt <= mode_cnt + 1;
                    end else if (flag_pe_en) begin
                        if (count_8 == 7) begin
                            count_8 <= 0;
                            s_state <= S_REG_ACK;
                        end else count_8 <= count_8 + 1;
                        flag_pe_en <= 0;
                    end
                end
                S_REG_ACK : begin
                    if (sioc_fe) begin
                        s_state <= S_REG_ACK;
                        o_siod_en <= 0;
                    end
                    if (sioc_pe) begin
                        if (i_rw) s_state <= S_STOP;
                        else s_state <= S_DATA;
                    end
                end
                S_DATA : begin
                    if (sioc_pe) flag_pe_en <= 1;
                    else if (sioc_fe) flag_fe_en <= 1;

                    if (flag_fe_en) begin
                        if (!i_rw) begin
                            if (mode_cnt >= 50) begin
                                mode_cnt <= 0;
                                o_siod_en <= 1;
                                flag_fe_en <= 0;
                                o_siod <= siod_wdata_r[7-count_8];
                            end else mode_cnt <= mode_cnt + 1;
                        end else flag_fe_en <= 0;
                    end else if (flag_pe_en) begin
                        flag_pe_en <= 0;
                        if (i_rw) begin
                            o_siod_en <= 0;
                            siod_rdata_r[7-count_8] <= i_siod;
                        end
                        if (count_8 == 7) begin
                            count_8 <= 0;
                            s_state <= S_DATA_ACK;
                        end else count_8 <= count_8 + 1;
                    end
                end
                S_DATA_ACK : begin
                    if (sioc_fe) begin
                        if (i_rw) begin
                            o_siod_en <= 1;
                            o_siod <= 1'b1;
                        end else o_siod_en <= 0;
                    end
                    if (sioc_pe) s_state <= S_STOP;
                end
                S_STOP : begin
                    if (sioc_fe) begin
                        s_state <= S_STOP;
                        o_siod_en <= 1;
                        sioc_en <= 1;
                        o_siod <= 0;
                    end
                    if (sioc_pe) begin
                        sioc_en <= 0;
                        s_state <= S_WAIT;
                        mode_cnt <= 0;
                    end
                end
                S_WAIT : begin
                    if (mode_cnt >= 500) begin
                        mode_cnt <= 0;
                        if (!i_rw) begin
                            s_state <= S_IDLE;
                            o_done <= 1;
                            read_phase <= 0;
                        end else begin
                            if (read_phase) begin
                                read_phase <= 0;
                                o_done <= 1;
                                s_state <= S_IDLE;
                                o_reg_data <= siod_rdata_r;
                            end else read_phase <= 1;
                            o_done <= 0;
                            s_state <= S_START;
                        end
                    end else if (mode_cnt >= 250) begin
                        o_siod <= 1;
                        mode_cnt <= mode_cnt + 1;
                    end else mode_cnt <= mode_cnt + 1;
                end
                default : s_state <= S_IDLE;
            endcase
        end
    end
endmodule