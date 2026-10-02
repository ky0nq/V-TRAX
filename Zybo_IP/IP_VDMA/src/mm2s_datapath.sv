`timescale 1ns / 1ps

// ============================================================================
//  mm2s_datapath
// ============================================================================
// Takes the job "read BTT bytes starting at address SA",
// splits it into multiple bursts according to AXI rules, issues the requests, and puts the returned data into the FIFO
//
// R0 : DDR (HP0)          camera frame buffer
// R1 : BRAM frame window  loading screen (virtual frame provided by ram_bridge)
// ============================================================================
module mm2s_datapath #(
    parameter ADDR_WIDTH   = 32,        // address width
    parameter DATA_WIDTH   = 32,        // data width (1 beat)
    parameter LEN_WIDTH    = 32,        // transfer length (BTT) width
    parameter BURST_WIDTH  = 8,         // ARLEN width

    parameter [ADDR_WIDTH-1:0] R0_BASE = 32'h0000_0000,   // region 0 start : DDR
    parameter [ADDR_WIDTH-1:0] R0_SIZE = 32'h4000_0000,   // region 0 size  : 1GB
    parameter [ADDR_WIDTH-1:0] R1_BASE = 32'h8000_0000,   // region 1 start : BRAM frame window
    parameter [ADDR_WIDTH-1:0] R1_SIZE = 32'h002A_3000,   // region 1 size  : 1280x720x3

    parameter MAX_BURST_BYTES = 64                        // burst limit 64B = 16 beat
)(
    input                            clk,
    input                            rst_n,

    // ------------------------------------------------------------------
    // Commands from the Controller
    // ------------------------------------------------------------------
    input                            en,    // 1 = OK to work (when the controller is in S_DATA)
    input                            init,  // 1 clock : frame start
    input                            abort, // 1 clock : stop new requests

    // ------------------------------------------------------------------
    // Config values (from the register map / frame select)
    // ------------------------------------------------------------------
    input      [ADDR_WIDTH-1:0]      src_addr,    // read start address
    input      [LEN_WIDTH-1:0]       length,      // total number of bytes to read
    input      [BURST_WIDTH+1:0]     burst_cfg,   // [7:0] desired ARLEN, [9:8] burst type

    // ------------------------------------------------------------------
    // Report to the Controller
    // ------------------------------------------------------------------
    output                           r_hs,        // R channel handshake
    output                           xfer_done,   // this frame is finished
    output     [ADDR_WIDTH-1:0]      err_addr,    // address of the first error beat
    output reg                       err_valid,   // error response received
    output reg                       cfg_err,     // invalid configuration, cannot proceed

    // ------------------------------------------------------------------
    // Pass data to the FIFO
    // ------------------------------------------------------------------
    output reg                       fifo_wr_en,
    output reg [DATA_WIDTH-1:0]      fifo_wr_data,
    input                            fifo_full,

    // ------------------------------------------------------------------
    // AXI4 AR channel (the order form)
    // ------------------------------------------------------------------
    output reg [4:0]                 arid,        // {DMA flag 1bit, reserved 2bit, slot number 2bit}
    output reg [ADDR_WIDTH-1:0]      araddr,      // start address of this burst
    output reg [BURST_WIDTH-1:0]     arlen,       // number of beats - 1
    output     [2:0]                 arsize,      // beat size (fixed at 4 bytes)
    output     [1:0]                 arburst,     // burst type
    output reg                       arvalid,     // "I have an order"
    input                            arready,     // "Order received"

    // ------------------------------------------------------------------
    // AXI4 R channel (the goods)
    // ------------------------------------------------------------------
    input      [DATA_WIDTH-1:0]      rdata,       // data
    input                            rvalid,      // "Goods have arrived"
    input                            rlast,       // "This is the last beat of this burst"
    input      [4:0]                 rid,         // which order (ARID) it belongs to
    input      [1:0]                 rresp,       // 00 OK, 10 SLVERR, 11 DECERR
    output                           rready       // "I can accept"
);

    // ########################################################################
    //  0. Constants
    // ########################################################################
    localparam [0:0] MASTER_ID      = 1'b1;                     // ARID[4] : "this is a DMA order"
    localparam BYTES_PER_BEAT       = DATA_WIDTH/8;             // 4
    localparam ADDR_LSB             = $clog2(BYTES_PER_BEAT);   // 2 : byte <-> beat conversion
    localparam PAGE_BYTES           = 4096;                     // AXI rule : a burst cannot cross a 4KB boundary
    localparam PAGE_LSB             = $clog2(PAGE_BYTES);       // 12
    localparam MAX_BURST_BEATS      = MAX_BURST_BYTES / BYTES_PER_BEAT;  // 16
    localparam [2:0] ARSIZE_VAL     = ADDR_LSB;                 // 3'b010 = 4 bytes/beat

    localparam MAX_OUTSTANDING   = 3;                            // number of orders that can be in flight at the same time
    localparam OUTSTANDING_CNT_W = $clog2(MAX_OUTSTANDING + 1);  // number of bits to count 0~3 = 2
    localparam SLOT_IDX_W        = (MAX_OUTSTANDING > 1) ? $clog2(MAX_OUTSTANDING) : 1;  // slot number bits = 2

    localparam [1:0] BURST_FIXED = 2'b00;                        // fixed address (repeat the same address)
    localparam [1:0] BURST_INCR  = 2'b01;                        // incrementing address (the usual one)

    localparam [ADDR_WIDTH-1:0] R0_END = R0_BASE + R0_SIZE;      // end of region 0 (this address is not included)
    localparam [ADDR_WIDTH-1:0] R1_END = R1_BASE + R1_SIZE;      // end of region 1 (this address is not included)

    assign arsize = ARSIZE_VAL;                                  // always 4 bytes/beat

    // ########################################################################
    //  1. Config values captured at init
    //     so that the CPU changing registers mid-transfer does not affect this frame,
    //     the "value at the moment of start" is stored separately and used (the actual storing is in block 9)
    // ########################################################################
    reg  [BURST_WIDTH+1:0] burst_cfg_q;     // captured BURST_CFG
    reg  [LEN_WIDTH-1:0]   total_beats_q;   // captured total beat count (= BTT / 4)
    reg                    align_err_q;     // whether there was an alignment error at start
    reg  [ADDR_WIDTH-1:0]  cur_addr;        // address for the next order form (advances every burst)
    reg  [LEN_WIDTH-1:0]   req_beat_cnt;    // number of beats ordered so far

    // BURST_CFG is used split into two parts
    wire [1:0]             burst_type_cfg = burst_cfg_q[BURST_WIDTH+1:BURST_WIDTH];  // [9:8] type
    wire [BURST_WIDTH-1:0] burst_len_cfg  = burst_cfg_q[BURST_WIDTH-1:0];            // [7:0] length
    assign arburst = burst_type_cfg;

    // if the type is 10(WRAP) or 11(reserved), the upper bit is 1 -> not supported
    wire burst_type_err = burst_type_cfg[1];

    // alignment check : address and length must be multiples of 4 (lower 2 bits are 00)
    // a beat is 4 bytes, so if not a multiple of 4, bytes straddling a beat cannot be handled
    wire align_err_c = (src_addr[ADDR_LSB-1:0] != {ADDR_LSB{1'b0}}) ||
                       (length[ADDR_LSB-1:0]   != {ADDR_LSB{1'b0}});

    // total bytes -> total beats ( / 4)
    wire [LEN_WIDTH-1:0] total_beats_c = length >> ADDR_LSB;

    // ########################################################################
    //  2. Progress
    // ########################################################################
    // are there beats that have not been ordered yet?
    wire req_pending = (req_beat_cnt < total_beats_q);

    // remaining beat count (used to calculate the burst length)
    wire [LEN_WIDTH-1:0] remain_beats = total_beats_q - req_beat_cnt;

    // ########################################################################
    //  3. Address region check : is the current address inside DDR or the BRAM window?
    // ########################################################################
    wire in_r0 = (cur_addr >= R0_BASE) && (cur_addr < R0_END);   // inside DDR?
    wire in_r1 = (cur_addr >= R1_BASE) && (cur_addr < R1_END);   // inside the BRAM window?

    // end address of the current region (used to cut the burst so it does not cross the region)
    wire [ADDR_WIDTH-1:0] region_end_c = in_r0 ? R0_END : R1_END;

    // error if there is still something to order but the address is in neither region
    wire region_err_c = req_pending && !in_r0 && !in_r1;

    // ########################################################################
    //  4. Handshake signals
    // ########################################################################
    wire ar_hs = arvalid && arready;

    assign r_hs   = rvalid && rready;      // one beat just arrived
    assign rready = !fifo_full;            // always accept as long as the FIFO has room

    wire r_mine      = (rid[4] == MASTER_ID);   // is it my (DMA) order? (check the top bit of the ID)
    wire r_beat      = r_hs && r_mine;          // a beat of my order has arrived
    wire r_burst_end = r_beat && rlast;         // and it is the last beat of that order

    // ########################################################################
    //  5. Outstanding management (up to 3 at the same time)
    //
    //     slot = order ticket number (0, 1, 2)
    //       num_busy[i]  : whether slot i is in use
    //       slot_addr[i] : start address of order i   -> for back-calculating the error address
    //       slot_beat[i] : beats received for order i -> for back-calculating the error address
    //     when issuing an order, the slot number is put in the lower 2 bits of ARID,
    //     and when the goods arrive, the lower 2 bits of RID tell which slot it is
    // ########################################################################
    reg [MAX_OUTSTANDING-1:0]   num_busy;
    reg [ADDR_WIDTH-1:0]        slot_addr [0:MAX_OUTSTANDING-1];
    reg [BURST_WIDTH-1:0]       slot_beat [0:MAX_OUTSTANDING-1];
    reg [OUTSTANDING_CNT_W-1:0] outstanding_cnt;   // number of orders in progress (0~3)
    reg [SLOT_IDX_W-1:0]        ar_num_q;          // slot number of the order form currently going out

    // the lowest-numbered free slot
    // (if all three are taken this yields 2, but then outstanding_ok = 0 so no order is issued)
    wire [1:0] next_num = (!num_busy[0]) ? 2'd0 : (!num_busy[1]) ? 2'd1 : 2'd2;

    // count orders in progress
    //   order accepted (ar_hs)      -> +1
    //   order completed (last beat) -> -1
    //   both on the same clock      -> unchanged
    always @(posedge clk) begin
        if (!rst_n)      outstanding_cnt <= '0;
        else if (init)   outstanding_cnt <= '0;
        else begin
            case ({ar_hs, r_burst_end})
                2'b10:   outstanding_cnt <= outstanding_cnt + 1'b1;
                2'b01:   outstanding_cnt <= outstanding_cnt - 1'b1;
                default: outstanding_cnt <= outstanding_cnt;
            endcase
        end
    end
    // slot in-use flags
    //   order accepted    -> that slot is in use
    //   last beat arrived -> free the slot found via RID
    always @(posedge clk) begin
        if (!rst_n)      num_busy <= '0;
        else if (init)   num_busy <= '0;
        else begin
            if (ar_hs)       num_busy[ar_num_q] <= 1'b1;
            if (r_burst_end) num_busy[rid[1:0]] <= 1'b0;
        end
    end
    // is there a free slot?
    wire outstanding_ok = (outstanding_cnt < MAX_OUTSTANDING);

    // ########################################################################
    //  6. Remember abort
    //     abort is a 1-clock pulse -> once it arrives, hold it at 1 until this frame ends
    //     -> no more new order forms are sent, but the goods for orders already sent are received to the end
    // ########################################################################
    reg abort_lat;
    always @(posedge clk) begin
        if (!rst_n)      abort_lat <= 1'b0;
        else if (init)   abort_lat <= 1'b0;
        else if (abort)  abort_lat <= 1'b1;
    end

    // ########################################################################
    //  7. Determine the length of this burst
    //     for INCR, the smallest of the 4 values below
    //       (a) config     : BURST_CFG[7:0] + 1, max 16
    //       (b) 4KB bound  : beats left until the next 4KB boundary
    //       (c) region end : beats left until the end of the current region
    //       (d) remaining  : beats not yet ordered
    // ########################################################################

    // (b) up to the 4KB boundary : 4096 - (lower 12 bits of the address)
    wire [LEN_WIDTH-1:0] bytes_to_boundary = PAGE_BYTES - {{(LEN_WIDTH-PAGE_LSB){1'b0}}, cur_addr[PAGE_LSB-1:0]};
    wire [LEN_WIDTH-1:0] beats_to_boundary = bytes_to_boundary >> ADDR_LSB;

    // (c) up to the region end
    wire [LEN_WIDTH-1:0] bytes_to_region   = region_end_c - cur_addr;
    wire [LEN_WIDTH-1:0] beats_to_region   = bytes_to_region >> ADDR_LSB;

    // (a) config : ARLEN is "beats - 1", so add 1 to convert to beats, capped at 16
    wire [LEN_WIDTH-1:0] desired_raw   = {{(LEN_WIDTH-BURST_WIDTH){1'b0}}, burst_len_cfg} + 1'b1;
    wire [LEN_WIDTH-1:0] desired_beats = (desired_raw > MAX_BURST_BEATS) ? MAX_BURST_BEATS
                                                                         : desired_raw;

    // the smaller of (b) and (c)
    //wire [LEN_WIDTH-1:0] limit_beats = (beats_to_boundary < beats_to_region) ? beats_to_boundary : beats_to_region;
    wire [LEN_WIDTH-1:0] limit_beats = beats_to_boundary;

    // FIXED : the address does not move, so no boundary concern -> min(remaining, config, 16)
    wire [LEN_WIDTH-1:0] safe_beats_fixed = (remain_beats < desired_beats) ?  ((remain_beats  < 16) ? remain_beats  : 16) : ((desired_beats < 16) ? desired_beats : 16);

    // INCR : min(config, boundary, remaining)  (boundary = the smaller of 4KB and region end)
    wire [LEN_WIDTH-1:0] safe_beats_incr = (desired_beats < limit_beats) ?
            ((desired_beats < remain_beats) ? desired_beats : remain_beats) : ((limit_beats   < remain_beats) ? limit_beats   : remain_beats);

    // final beat count (0 for an unsupported type -> no order is issued)
    wire [LEN_WIDTH-1:0] safe_beats = (burst_type_cfg == BURST_FIXED) ? safe_beats_fixed : (burst_type_cfg == BURST_INCR)  ? safe_beats_incr  : {LEN_WIDTH{1'b0}};

	reg [LEN_WIDTH-1:0] safe_beats_reg;
	reg                 safe_beats_valid;
	
	
	//is it OK to compute the burst length and store it in the FF?
	wire safe_calc_en = en && !init
			&& !arvalid
	    	&& !safe_beats_valid
	    	&& req_pending
			&& !abort
	    	&& !abort_lat
	    	&& outstanding_ok
	    	&& !cfg_err
	    	&& !region_err_c;
	
	always @(posedge clk) begin
	    if (!rst_n || init) begin
	        safe_beats_reg     <= {LEN_WIDTH{1'b0}};
	        safe_beats_valid <= 1'b0;
	    end
	    else begin
	        // consumed in the next stage
	        if (!arvalid && safe_beats_valid)
	            safe_beats_valid <= 1'b0;
	
	        // store the newly computed burst length
	        if (safe_calc_en) begin
	            safe_beats_reg     <= safe_beats;
	            safe_beats_valid <= 1'b1;
	        end
	    end
	end


    wire [LEN_WIDTH-1:0]   safe_bytes = safe_beats_reg << ADDR_LSB;            // beat -> bytes (×4) : address advance amount
    wire [BURST_WIDTH-1:0] safe_arlen = safe_beats_reg[BURST_WIDTH-1:0] - 1'b1; // ARLEN = beats - 1

    // ########################################################################
    //  8. Check whether an order form may be issued / config error
    // ########################################################################
	// may the stored burst info be sent out on the AXI AR channel?
    wire ar_can_issue = en && !init          // working, and not the start clock (config is being latched)
                        && !arvalid          // the previous order form is not still waiting to be accepted
                        && req_pending       // there is still something to order
                        && !abort_lat        // no stop request has been made
                        && outstanding_ok    // there is a free slot
                        && (safe_beats_reg != {LEN_WIDTH{1'b0}})   // the length is not 0
                        && !cfg_err          // there is no config error
                        && !region_err_c     // the address is within an allowed region
						&& safe_beats_valid
						&& !abort ;

    // something remains but the length is computed as 0 = no way to proceed any further
    wire no_progress = en && !init && req_pending && !abort_lat && !abort
                       && (safe_beats_reg == {LEN_WIDTH{1'b0}})&& safe_beats_valid;

    // config error flag
    //   at init : 1 immediately on an alignment error (otherwise cleared to 0)
    //   while working : cannot proceed / alignment error / out of region / unsupported burst type -> 1
    //   once 1, held until the next init -> orders stop -> xfer_done once all issued orders are received
    always @(posedge clk) begin
        if (!rst_n) cfg_err <= 1'b0;
        else if (init) cfg_err <= align_err_c;
        else if (en && (no_progress || align_err_q || region_err_c || burst_type_err))
            cfg_err <= 1'b1;
    end

    // ########################################################################
    //  9. Latch config values + advance address/count
    // ########################################################################
    always @(posedge clk) begin
        if (!rst_n) begin
            cur_addr       <= {ADDR_WIDTH{1'b0}};
            req_beat_cnt   <= {LEN_WIDTH{1'b0}};
            total_beats_q  <= {LEN_WIDTH{1'b0}};
            align_err_q    <= 1'b0;
            burst_cfg_q    <= {(BURST_WIDTH+2){1'b0}};
            slot_addr      <= '{default: '0};
            slot_beat      <= '{default: '0};
        end
        // ----- frame start : snapshot the config values -----
        else if (init) begin
            cur_addr       <= src_addr;         // start address
            req_beat_cnt   <= {LEN_WIDTH{1'b0}}; // ordered amount starts from 0
            total_beats_q  <= total_beats_c;    // total beat count
            align_err_q    <= align_err_c;      // alignment error flag
            burst_cfg_q    <= burst_cfg;        // burst config
            slot_addr      <= '{default: '0};   // clear slot records
            slot_beat      <= '{default: '0};
        end
        // ----- during transfer -----
        else begin
            // new order accepted -> that slot's received beat count starts from 0
            if (ar_hs)  slot_beat[ar_num_q] <= {BURST_WIDTH{1'b0}};
            // beat arrived -> +1 to the received beat count of the slot found via RID
            if (r_beat) slot_beat[rid[1:0]] <= slot_beat[rid[1:0]] + 1'b1;

            if (ar_hs) begin
                // for INCR, advance the next order address by this burst size (FIXED stays put)
                if (burst_type_cfg == BURST_INCR) cur_addr <= cur_addr + safe_bytes[ADDR_WIDTH-1:0];
				req_beat_cnt        <= req_beat_cnt + safe_beats_reg;   // accumulate the ordered amount
                slot_addr[ar_num_q] <= araddr;                      // record this slot's start address
            end
        end
    end

    // ########################################################################
    //  10. AR channel : send out the order form
    //      AXI rule : once ARVALID is raised, the contents must not change until ARREADY arrives
    //      -> after raising it, leave it alone until accepted (ar_hs), then lower it
    // ########################################################################
    always @(posedge clk) begin
        if (!rst_n) begin
            arvalid    <= 1'b0;
            araddr     <= {ADDR_WIDTH{1'b0}};
            arlen      <= {BURST_WIDTH{1'b0}};
            arid       <= 5'd0;
            ar_num_q   <= '0;
        end
        else if (ar_hs) begin
            arvalid <= 1'b0;                                // accepted, so lower it
        end
        else if (ar_can_issue) begin
            arvalid    <= 1'b1;                             // new order form
            araddr     <= cur_addr;                         // address
            arlen      <= safe_arlen;                       // length
            arid       <= {MASTER_ID, 2'b00, next_num};     // {DMA, reserved, slot number}
            ar_num_q   <= next_num;                         // remember which slot
        end
    end

    // ########################################################################
    //  11. R channel -> FIFO
    //      every time a beat of my order arrives, write it into the FIFO as is
    //      (when the FIFO is full rready = 0, so the arrival itself does not happen -> no overflow)
    // ########################################################################
    always @(*) begin
        fifo_wr_en   = r_beat;
        fifo_wr_data = rdata;
    end

    // ########################################################################
    //  12. Back-calculate the error address
    //      RRESP[1] = 1 means SLVERR(10) or DECERR(11) = bad
    //      address of the bad beat = slot start address + (beats previously received in that slot × 4)
    //        e.g.) slot 1 starts at 0x1000_0100, 3 beats already received and the 4th is bad
    //            -> 0x1000_0100 + 3×4 = 0x1000_010C
    //      FIXED does not move the address, so it is the start address as is
    // ########################################################################
    wire beat_err = r_beat && rresp[1];

    wire [ADDR_WIDTH-1:0] beat_addr = (burst_type_cfg == BURST_FIXED) ? slot_addr[rid[1:0]] : slot_addr[rid[1:0]] +
              ({{(ADDR_WIDTH-BURST_WIDTH){1'b0}}, slot_beat[rid[1:0]]} << ADDR_LSB);

    reg [ADDR_WIDTH-1:0] err_addr_q;
    assign err_addr = err_addr_q;

    // record only the first bad one (the first one matters most for finding the cause)
    // note : even on a response error, ordering does not stop and this frame is received to the end.
    //        whether to repeat is decided by the controller (no next frame if there is an error)
    always @(posedge clk) begin
        if (!rst_n) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
        end
        else if (init) begin
            err_valid  <= 1'b0;
            err_addr_q <= {ADDR_WIDTH{1'b0}};
        end
        else if (beat_err && !err_valid) begin
            err_valid  <= 1'b1;
            err_addr_q <= beat_addr;
        end
    end

    // ########################################################################
    //  13. Done? (xfer_done)
    //      done when "nothing more to order" + "all goods for the sent orders have arrived"
    //        cases with nothing to order : everything ordered / abort / config error
    //      !init : excluded because values from the previous frame may remain on the start clock
    //              (prevents counting the same frame end twice in cyclic mode)
    // ########################################################################
    wire ar_done = !req_pending || abort_lat || cfg_err;

    assign xfer_done = !init && ar_done && (outstanding_cnt == '0);

endmodule
