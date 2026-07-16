//==============================================================================
// tx_arbiter - round-robin 4:1 CSI-2 packet mux with virtual-channel tagging
//==============================================================================
// Runs in the TX byte-clock domain. Drains per-camera frame_buffer token
// streams (FWFT) and replays whole packets into the csi2_tx sink, tagging each
// with VC = camera index. A camera is only selected once its frame_buffer holds
// a COMPLETE packet (pkt_avail), so the TX never starts a packet it cannot
// finish (no mid-HS underflow).
//
// Rewrite vs old design: works on the FWFT token stream (old code read stale
// metadata at ST_SELECT before data was valid), and there is no byte/word-count
// bookkeeping to get wrong -- the packet boundary comes from the token 'last'
// flag produced by csi2_rx.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

import csi2_pkg::*;

module tx_arbiter #(
    parameter int NUM_CAMERAS = 4
)(
    input  wire        clk,            // TX byte clock
    input  wire        rst_n,

    // Per-camera frame_buffer read interfaces (FWFT token stream)
    input  wire [NUM_CAMERAS-1:0]        buf_rd_valid,
    input  wire [NUM_CAMERAS-1:0]        buf_rd_is_sop,
    input  wire [NUM_CAMERAS-1:0]        buf_rd_last,
    input  wire [NUM_CAMERAS-1:0][31:0]  buf_rd_data,
    input  wire [NUM_CAMERAS-1:0]        buf_pkt_avail,
    output logic [NUM_CAMERAS-1:0]       buf_rd_en,

    // csi2_tx packet sink
    output logic        sop_valid,
    input  wire         sop_ready,
    output logic [1:0]  sop_vc,
    output logic [5:0]  sop_dt,
    output logic [15:0] sop_wc,
    output logic        sop_short,
    output logic [31:0] pay_data,
    output logic        pay_valid,
    input  wire         pay_ready
);

    localparam int CW = (NUM_CAMERAS <= 1) ? 1 : $clog2(NUM_CAMERAS);

    typedef enum logic [1:0] {ST_SEL, ST_SOP, ST_PAY} state_t;
    state_t state;

    logic [CW-1:0] cur, nxt;

    // round-robin next camera with a complete packet whose head is a SOP
    // (found-guard instead of 'break' for simulator portability)
    always_comb begin
        logic found;
        nxt   = cur;
        found = 1'b0;
        for (int i = 1; i <= NUM_CAMERAS; i++) begin
            logic [CW-1:0] c;
            c = CW'((int'(cur) + i) % NUM_CAMERAS);
            if (!found && buf_pkt_avail[c] && buf_rd_valid[c] && buf_rd_is_sop[c]) begin
                nxt   = c;
                found = 1'b1;
            end
        end
    end

    wire any_ready = |(buf_pkt_avail & buf_rd_valid & buf_rd_is_sop);

    // unpacked head of current camera
    wire [31:0] head = buf_rd_data[cur];

    always_comb begin
        buf_rd_en = '0;
        sop_valid = 1'b0;
        pay_valid = 1'b0;
        sop_vc    = cur[1:0];
        sop_dt    = sop_dt_f(head);
        sop_wc    = sop_wc_f(head);
        sop_short = sop_short_f(head);
        pay_data  = head;

        unique case (state)
            ST_SOP: begin
                sop_valid = 1'b1;
                if (sop_ready) buf_rd_en[cur] = 1'b1;   // pop the SOP token
            end
            ST_PAY: begin
                pay_valid = buf_rd_valid[cur] && !buf_rd_is_sop[cur];
                if (pay_valid && pay_ready) buf_rd_en[cur] = 1'b1;
            end
            default: ;
        endcase
    end

    // Hang escapes. A camera disable/unplug mid-packet can leave a TRUNCATED
    // packet in a frame_buffer (its 'last' token never arrives) or empty the
    // buffer under us (per-camera FIFO reset). Without an escape the FSM would
    // wait in ST_PAY forever and stall all four cameras. Abort the in-flight
    // packet if the head token is unexpectedly a SOP, or if the buffer stays
    // silent mid-packet for 2^15 byte clocks (~0.3 ms; a legitimate packet is
    // fully buffered upstream and never stalls). csi2_tx has its own matching
    // underflow escape for the packet it is transmitting.
    logic [15:0] stall_cnt;
    wire         stall_abort = stall_cnt[15];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)                            stall_cnt <= '0;
        else if (state != ST_PAY || pay_valid) stall_cnt <= '0;
        else                                   stall_cnt <= stall_cnt + 1'b1;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_SEL;
            cur   <= '0;
        end else begin
            unique case (state)
                ST_SEL: begin
                    if (any_ready) begin
                        cur   <= nxt;
                        state <= ST_SOP;
                    end
                end
                ST_SOP: begin
                    if (sop_ready) begin
                        if (sop_short_f(head)) state <= ST_SEL;
                        else                   state <= ST_PAY;
                    end
                end
                ST_PAY: begin
                    if (pay_valid && pay_ready && buf_rd_last[cur])
                        state <= ST_SEL;
                    else if ((buf_rd_valid[cur] && buf_rd_is_sop[cur]) || stall_abort)
                        state <= ST_SEL;   // truncated packet: abort
                end
                default: state <= ST_SEL;
            endcase
        end
    end

endmodule

`default_nettype wire
