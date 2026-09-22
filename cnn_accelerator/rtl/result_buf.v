`timescale 1ns / 1ps

module result_buf (
    input  wire        clk,
    input  wire        rst_n,

    // ---- layer config ----
    input  wire        i_cfg_valid,         
    input  wire        i_is_final_layer,    
    input  wire [13:0] i_dst_base,          
    input  wire [ 5:0] i_dst_channels,      

    // ---- pooling_unit ----
    input  wire [23:0] i_data,            
    input  wire        i_valid,           
    output wire        o_ready,           
    input  wire [ 2:0] i_keep,            
    input  wire [18:0] i_meta,            
    input  wire        i_write_grant,     
    output wire        o_wr_en,           
    output wire [13:0] o_wr_addr,         
    output wire [23:0] o_wr_data,         
    output wire [ 2:0] o_wr_be,           

    // ---- post_process scalar ----
    input wire signed [7:0] i_final_data,  
    input  wire        i_final_valid,      
    output wire        o_final_ready,      
    output reg         o_final_tile_done,  

    // ---- Done / CSR ----
    output reg               o_layer_done,       
    input  wire              i_irq_en,           
    input  wire              i_irq_clear,        
    output reg signed [31:0] o_final_result,  
    output reg               o_done_status,      
    output wire              o_irq               
);

    // ---- cfg ----
    reg        is_final;
    reg [13:0] dst_base;
    reg [ 3:0] wpp;                         // ceil(C / 3) : C <= 32 -> 1 ~ 11
    reg        final_stored;                // 이 레이어의 scalar 를 저장했다
    reg        layer_done_sent;             // 이 레이어의 layer_done 을 냈다 (중복 금지, R472)

    function [3:0] ceil_div3;               // (C + 2) / 3, C 는 6bit
        input [5:0] c;
        begin ceil_div3 = ({2'b00, c} + 8'd2) / 8'd3; end
    endfunction

    // ---- 중간 레이어 쓰기 ----
    wire [11:0] m_pos  = i_meta[11:0];
    wire [ 4:0] m_ocb  = i_meta[16:12];
    wire        m_lend = i_meta[18];

    wire [15:0] pix_off = m_pos * {12'd0, wpp};                 // pos * ceil(C/3), 최대 4095 * 11
    wire [ 3:0] ch_off  = m_ocb / 5'd3;                         // out_ch_base 는 3 의 배수 -> 그룹 번호 0 ~ 10

    assign o_ready = rst_n && !i_cfg_valid && !is_final && !layer_done_sent && i_write_grant;         // 큐 없음 : grant 가 있으면 바로 쓴다
    assign o_wr_en   = i_valid && o_ready;
    assign o_wr_addr = dst_base + pix_off[13:0] + {10'd0, ch_off};
    assign o_wr_data = i_data;
    assign o_wr_be   = i_keep;

    wire wr_fire     = o_wr_en;
    wire lend_fire   = wr_fire && m_lend && !layer_done_sent;  

    assign o_final_ready = rst_n && !i_cfg_valid && is_final && !final_stored;
    wire   final_fire    = i_final_valid && o_final_ready;

    assign o_irq = i_irq_en && o_done_status;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_final          <= 1'b0;
            dst_base          <= 14'd0;
            wpp               <= 4'd1;
            final_stored      <= 1'b0;
            layer_done_sent   <= 1'b0;
            o_final_tile_done <= 1'b0;
            o_layer_done      <= 1'b0;
            o_final_result    <= 32'd0;
            o_done_status     <= 1'b0;
        end else begin
            o_final_tile_done <= 1'b0;
            o_layer_done      <= 1'b0;

            if (i_cfg_valid) begin
                is_final        <= i_is_final_layer;
                dst_base        <= i_dst_base;
                wpp             <= ceil_div3(i_dst_channels);
                final_stored    <= 1'b0;
                layer_done_sent <= 1'b0;
            end

            // Layer Change : next clk = layer_done High-state
            if (lend_fire) begin
                o_layer_done    <= 1'b1;
                layer_done_sent <= 1'b1;
            end

            // Final : angle store
            if (final_fire) begin
                o_final_result    <= {{24{i_final_data[7]}}, i_final_data};
                final_stored      <= 1'b1;
                layer_done_sent   <= 1'b1;
                o_final_tile_done <= 1'b1;
                o_layer_done      <= 1'b1;
                o_done_status     <= 1'b1;
            end else if (i_irq_clear) begin
                o_done_status     <= 1'b0;
            end
        end
    end

endmodule