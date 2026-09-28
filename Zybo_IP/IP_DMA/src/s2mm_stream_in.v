module s2mm_stream_in #(
    parameter DATA_WIDTH = 32,
    parameter KEEP_WIDTH = DATA_WIDTH / 8,
    parameter USER_WIDTH = 16
)(
    input  wire                  ACLK,
    input  wire                  ARESETN,
    input  wire [DATA_WIDTH-1:0] S_AXIS_TDATA,
    input  wire                  S_AXIS_TVALID,
    output wire                  S_AXIS_TREADY,
    input  wire                  S_AXIS_TLAST,
    input  wire [KEEP_WIDTH-1:0] S_AXIS_TKEEP,
    input  wire [USER_WIDTH-1:0] S_AXIS_TUSER,
    input  wire                  fifo_full,
    output wire                  fifo_we,
    output wire [DATA_WIDTH-1:0] fifo_wdata
);

    localparam IDLE  = 1'b0;
    localparam FRAME = 1'b1;

    reg STATE;

    wire handshake;

    assign S_AXIS_TREADY = !fifo_full;
    assign handshake     = S_AXIS_TVALID && S_AXIS_TREADY;
    assign fifo_wdata    = S_AXIS_TDATA;
    assign fifo_we       = handshake &&
                           ((STATE == FRAME) || S_AXIS_TUSER[0]);

    always @(posedge ACLK) begin
        if (!ARESETN) begin
            STATE <= IDLE;
        end
        else begin
            case (STATE)
                IDLE : begin
                    if (handshake && S_AXIS_TUSER[0]) begin
                        if (S_AXIS_TLAST)
                            STATE <= IDLE;
                        else
                            STATE <= FRAME;
                    end
                end

                FRAME : begin
                    if (handshake && S_AXIS_TLAST)
                        STATE <= IDLE;
                end

                default : STATE <= IDLE;
            endcase
        end
    end

    wire _unused_tkeep = &S_AXIS_TKEEP;

endmodule
