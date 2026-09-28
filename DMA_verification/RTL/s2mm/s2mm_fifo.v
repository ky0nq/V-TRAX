module s2mm_fifo #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 1024
)(
    input  wire                         clk,
    input  wire                         resetn,

    input  wire                         wr_en,
    input  wire [DATA_WIDTH-1:0]        wr_data,
    output wire                         full,

    input  wire                         rd_en,
    output reg  [DATA_WIDTH-1:0]        rd_data,
    output wire                         empty,

    output reg  [$clog2(DEPTH+1)-1:0]   count
);

    localparam PTR_WIDTH = $clog2(DEPTH);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_WIDTH-1:0]  wr_ptr;
    reg [PTR_WIDTH-1:0]  rd_ptr;

    wire do_write;
    wire do_read;

    assign full  = (count == DEPTH);
    assign empty = (count == 0);

    assign do_write = wr_en && !full;
    assign do_read  = rd_en && !empty;

    always @(posedge clk) begin
        if (!resetn) begin
            wr_ptr  <= 0;
            rd_ptr  <= 0;
            count   <= 0;
            rd_data <= 0;
        end
        else begin
            if (do_write) begin
                mem[wr_ptr] <= wr_data;

                if (wr_ptr == DEPTH-1)
                    wr_ptr <= 0;
                else
                    wr_ptr <= wr_ptr + 1'b1;
            end

            if (do_read) begin
                rd_data <= mem[rd_ptr];

                if (rd_ptr == DEPTH-1)
                    rd_ptr <= 0;
                else
                    rd_ptr <= rd_ptr + 1'b1;
            end

            case ({do_write, do_read})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
