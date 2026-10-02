`timescale 1ns / 1ps

// ============================================================================
//  mm2s_controller  (= the DMA's Control Unit)
// ============================================================================
// Manages "whether the DMA is working or idle" and acts as the manager that gives the start signal to the datapath!!
//
// [Operating modes]
//    One-shot mode (cyclic = 0) : start -> 1 frame -> IDLE
//    Cyclic mode   (cyclic = 1) : start -> each time a frame ends, immediately the next frame -> ...
//                                 keeps running even without CPU intervention (VDMA circular mode)
//    3 conditions that stop the repetition
//        1) abort arrives (stop signal issued by the CPU)
//        2) the cyclic bit is cleared (CPU commands to stop after finishing the current frame)
//        3) an error occurs (something went wrong, so do not repeat any further)
// ============================================================================

module mm2s_controller #(
    parameter ADDR_WIDTH = 32  // address width for error storage
)(
    input                           clk,
    input                           rst_n,

    // ------------------------------------------------------------------
    // Signals exchanged with the CPU
    // ------------------------------------------------------------------
    input                           start,        // pulses for 1 clock when the BTT register is written -> start
    input                           abort,        // pulses for 1 clock when 1 is written to CR[2] -> stop
    input                           cyclic,       // CR[4] : 1 = cyclic mode
    output reg                      busy,         // 1 = working -> SR[0]
    output reg                      done,         // 1 = work finished -> held until the next start
    output reg                      error,        // 1 = error occurred -> SR[4]
    output reg  [ADDR_WIDTH-1:0]    error_addr,   // address where the error occurred -> READ_ERR register
    output reg                      frame_done,   // 1 clock each time a frame finishes -> completion IRQ, cur_buf update

    // ------------------------------------------------------------------
    // Signals reported by the Datapath
    // ------------------------------------------------------------------
    input                           xfer_done,    // this frame has been fully read
    input                           err_valid,    // the slave returned an error response (SLVERR/DECERR)
    input                           cfg_err,      // invalid configuration (alignment, address range, burst type, etc.)
    input       [ADDR_WIDTH-1:0]    err_addr,     // error address computed by the datapath

    // ------------------------------------------------------------------
    // Commands given to the Datapath
    // ------------------------------------------------------------------
    output                          en,           // 1 = OK to work now~
    output reg                      init          // 1 clock: start of a new frame
);

    // ------------------------------------------------------------------
    //   S_IDLE : idle. waits only for start
    //   S_DATA : working. the datapath is reading data over AXI
    // ------------------------------------------------------------------
    localparam S_IDLE = 1'b0;
    localparam S_DATA = 1'b1;

    reg state;          // current state

    // A memo that "remembers" abort.
    // abort is a 1-clock pulse, so unless it is remembered until the moment the frame ends (xfer_done),
    // we would forget "was I told to stop?" and repeat again. So once it arrives, it is held at 1.
    reg stop_req;

    // in the S_DATA state en = 1 -> the datapath may issue AR requests
    assign en = (state == S_DATA);

    //   error     : error already recorded (occurred on a previous clock)
    //   err_valid : response error that just arrived
    //   cfg_err   : config error that was just detected
    // If any one of the three is present, do not repeat.
    // (the error register rises one clock late... so err_valid / cfg_err are also checked to avoid missing an error on the same clock)
    wire err_now = error || err_valid || cfg_err;

    always @(posedge clk) begin
        // ==============================================================
        // reset
        // ==============================================================
        if (!rst_n) begin
            state      <= S_IDLE;
            init       <= 1'b0;
            frame_done <= 1'b0;
            stop_req   <= 1'b0;
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            error_addr <= {ADDR_WIDTH{1'b0}};
        end
        else begin
            init       <= 1'b0;
            frame_done <= 1'b0;

            case (state)
                // ======================================================
                // S_IDLE : idle
                // ======================================================
                S_IDLE: begin
                    if (start) begin                // CPU wrote BTT = go signal
                        state    <= S_DATA;         // move to the working state
                        init     <= 1'b1;           // notify start of a new frame
                        stop_req <= 1'b0;           // clear the previous abort memo
                        busy     <= 1'b1;           // mark busy in SR
                        done     <= 1'b0;           // clear the previous done flag
                        error    <= 1'b0;           // clear the previous error record
                    end
                end

                // ======================================================
                // S_DATA : working
                // ======================================================
                S_DATA: begin
                    // --------------------------------------------------
                    // (1) remember if abort arrives
                    //     the datapath also receives abort directly, stops issuing new AR requests,
                    //     and raises xfer_done only after receiving all responses for requests already issued.
                    //     the controller only remembers it so it can decide "don't repeat" at that point.
                    // --------------------------------------------------
                    if (abort)
                        stop_req <= 1'b1;
                    // --------------------------------------------------
                    // (2) error record : store only the "first error"
                    //     !error condition: ignore errors from the second one onward -> preserve the original cause
                    //     note: for cfg_err cases the datapath gives address 0, so error_addr = 0
                    // --------------------------------------------------
                    if ((err_valid || cfg_err) && !error) begin
                        error      <= 1'b1;
                        error_addr <= err_addr;
                    end
                    // --------------------------------------------------
                    // (3) when the frame ends : decide whether to repeat or go idle
                    // --------------------------------------------------
                    if (xfer_done) begin
                        frame_done <= 1'b1; // either way, first raise the signal that one frame has finished
                        //   cyclic    : cyclic mode is on, and
                        //   !stop_req : no abort has arrived before, and
                        //   !abort    : no abort arrived on this very clock either
                        //               (stop_req only becomes 1 on the next clock, so this is checked as well)
                        //   !err_now  : and there is no error
                        if (cyclic && !stop_req && !abort && !err_now) begin
                            init <= 1'b1;           // start the next frame immediately (state stays S_DATA)
                                                    // busy also stays 1 -> from the CPU's point of view it is continuously busy!
                        end else begin
                            state <= S_IDLE;
                            busy  <= 1'b0;          // lower busy
                            done  <= 1'b1;          // "done" flag (held until the next start)
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule