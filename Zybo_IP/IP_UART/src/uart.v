`timescale 1 ns / 1 ps


// UART top module: simple wrapper integrating TX and RX modules.
// Default config: 100 MHz clock, 115200 baud, 8N1 format

module uart #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    // TX interface
    input  wire [7:0] tx_data,
    input  wire       tx_valid,
    output wire       tx_ready,
    output wire       tx,
    // RX interface
    input  wire       rx,
    output wire [7:0] rx_data,
    output wire       rx_valid
);

    uart_tx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .data_in  (tx_data),
        .valid    (tx_valid),
        .ready    (tx_ready),
        .tx       (tx)
    );

    uart_rx #(
        .CLK_FREQ  (CLK_FREQ),
        .BAUD_RATE (BAUD_RATE)
    ) u_rx (
        .clk      (clk),
        .rst_n    (rst_n),
        .rx       (rx),
        .data_out (rx_data),
        .valid    (rx_valid)
    );

endmodule



// UART receiver (8N1: 8 data bits, no parity, 1 stop bit)

module uart_rx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,         // Serial input line
    output reg  [7:0] data_out,   // Received byte
    output reg        valid       // Receive done pulse (1-cycle high)
);

    localparam CLKS_PER_BIT  = CLK_FREQ / BAUD_RATE;
    localparam HALF_BIT      = CLKS_PER_BIT / 2;

    // RX FSM states
    localparam S_IDLE  = 2'd0; // Idle (detect rx falling edge)
    localparam S_START = 2'd1; // Wait until start bit center, then re-check
    localparam S_DATA  = 2'd2; // Sample 8 data bits
    localparam S_STOP  = 2'd3; // Pass stop bit

    reg [1:0]                    state;
    reg [$clog2(CLKS_PER_BIT):0] clk_cnt;
    reg [2:0]                    bit_idx;
    reg [7:0]                    shift_reg;

    // 2-stage synchronizer bringing rx into clk domain (metastability prevention)
    reg rx_sync0, rx_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync0 <= 1'b1;
            rx_sync  <= 1'b1;
        end else begin
            rx_sync0 <= rx;
            rx_sync  <= rx_sync0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 8'h00;
            data_out  <= 8'h00;
            valid     <= 1'b0;
        end else begin
            valid <= 1'b0; // Default: deassert

            case (state)
                S_IDLE: begin
                    clk_cnt <= 0;
                    bit_idx <= 0;
                    if (rx_sync == 1'b0) // Falling edge -> start bit candidate
                        state <= S_START;
                end

                // Wait until start bit center
                S_START: begin
                    if (clk_cnt == HALF_BIT - 1) begin
                        clk_cnt <= 0;
                        if (rx_sync == 1'b0) // Still low -> valid start bit
                            state <= S_DATA;
                        else
                            state <= S_IDLE; // Treated as noise, discard
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // Sample at the center of each bit period
                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt              <= 0;
                        shift_reg[bit_idx]   <= rx_sync; // Fill from LSB
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                // Wait one stop bit period, then output data
                S_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt  <= 0;
                        data_out <= shift_reg;
                        valid    <= 1'b1; // 1-cycle pulse
                        state    <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule


// UART transmitter (8N1: 8 data bits, no parity, 1 stop bit)

module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data_in,   // Byte to transmit
    input  wire       valid,     // Transmit start pulse (1-cycle high)
    output reg        ready,     // Idle state (can accept next data)
    output reg        tx         // Serial output line
);

    // Number of clocks per bit
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    // TX FSM states
    localparam S_IDLE  = 2'd0; // Idle
    localparam S_START = 2'd1; // Output start bit (0)
    localparam S_DATA  = 2'd2; // Output 8 data bits
    localparam S_STOP  = 2'd3; // Output stop bit (1)

    reg [1:0]                    state;
    reg [$clog2(CLKS_PER_BIT):0] clk_cnt;   // Bit period counter
    reg [2:0]                    bit_idx;   // Index of bit currently being sent
    reg [7:0]                    shift_reg; // Holds TX data

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 0;
            bit_idx   <= 0;
            shift_reg <= 8'h00;
            tx        <= 1'b1; // Line is high when idle
            ready     <= 1'b1;
        end else begin
            case (state)
                S_IDLE: begin
                    tx    <= 1'b1;
                    ready <= 1'b1;
                    if (valid) begin
                        // Capture data and enter start bit
                        shift_reg <= data_in;
                        clk_cnt   <= 0;
                        ready     <= 1'b0;
                        state     <= S_START;
                    end
                end

                S_START: begin
                    tx <= 1'b0; // Start bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        bit_idx <= 0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_DATA: begin
                    tx <= shift_reg[bit_idx]; // Send from LSB in order
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        if (bit_idx == 3'd7) begin
                            state <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                S_STOP: begin
                    tx <= 1'b1; // Stop bit
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
