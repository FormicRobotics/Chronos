//==============================================================================
// Chronos precision trigger generator
//==============================================================================
//
// Copyright (C) 2025 Chronos Project
// SPDX-License-Identifier: Proprietary
//
// One FSIN pulse train at the requested frame rate.  On HW_CHRONOS_R1 the FPGA
// only drives ONE physical FSIN pin (net FSIN_FPGA, pin R9) — the board fan-
// outs that signal through buffers U5/U9 to every camera, the IMU, and the
// external sync header.  The per-output "trigger_delay" array therefore lives
// in registers; it is only meaningful in firmware revisions that route four
// FSIN pins separately (kept here so the register map stays stable for future
// hardware revisions where per-output skew tuning is possible).
//
// Bugs fixed vs. the previous revision:
//   * trigger_delay was sized [4] but indexed up to NUM_OUTPUTS-1 = 4 → out
//     of bounds when NUM_OUTPUTS was 5.  Array now sized to the parameter.
//   * Delayed output never followed the master pulse width correctly because
//     delay_active was cleared before delay_cnt actually reached zero, so
//     "delayed_trigger" briefly dropped before the pulse completed.  The new
//     logic uses a single shift register that mirrors the master pulse with
//     a programmable delay; pulse width is preserved by construction.
//   * frame_period was a non-blocking assignment to a non-registered output;
//     it could lag by one cycle after a rate change.  The new pulse engine
//     latches the period once at every wrap, so the period that is in
//     flight is always consistent.
//
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module trigger_generator #(
    parameter int NUM_OUTPUTS    = 4,
    parameter int CLK_FREQ_HZ    = 200_000_000,
    parameter int MAX_FRAME_RATE = 120
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 enable,
    input  wire        [7:0]    frame_rate,        // fps
    input  wire        [15:0]   pulse_width,       // in clk cycles (0 = default)
    input  wire        [7:0]    trigger_delay [NUM_OUTPUTS],

    output logic                trigger_pulse,     // un-delayed reference
    output logic                fsin_out           // single physical FSIN pin
);

    //--------------------------------------------------------------------------
    // Local constants
    //--------------------------------------------------------------------------
    localparam int MIN_PERIOD          = CLK_FREQ_HZ;                  // 1 fps
    localparam int CNT_W               = $clog2(MIN_PERIOD) + 1;
    localparam int DEFAULT_PULSE_CYCLES= CLK_FREQ_HZ / 100_000;        // 10 us
    localparam int MAX_DELAY           = 256;                          // 8-bit
    // Largest shift register we will need = max delay we can program.
    localparam int DELAY_SR_DEPTH      = MAX_DELAY;

    //--------------------------------------------------------------------------
    // Pulse period engine
    //--------------------------------------------------------------------------
    logic [CNT_W-1:0]  period_count;
    logic [CNT_W-1:0]  frame_period;
    logic [15:0]       pulse_cnt;
    logic [15:0]       pulse_w_eff;          // pulse width actually used

    // Latch a sane period at every wrap so the in-flight period is stable.
    function automatic logic [CNT_W-1:0] compute_period(input logic [7:0] fps);
        if (fps == 0 || fps > MAX_FRAME_RATE)
            return CLK_FREQ_HZ / 30;
        return CLK_FREQ_HZ / fps;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_count   <= '0;
            pulse_cnt      <= '0;
            frame_period   <= compute_period(8'd30);
            pulse_w_eff    <= DEFAULT_PULSE_CYCLES[15:0];
            trigger_pulse  <= 1'b0;
        end else if (!enable) begin
            period_count  <= '0;
            pulse_cnt     <= '0;
            trigger_pulse <= 1'b0;
        end else begin
            // Period counter
            if (period_count >= frame_period - 1) begin
                period_count <= '0;
                pulse_cnt    <= 16'd1;                     // start a new pulse
                trigger_pulse<= 1'b1;
                // Latch any rate/width change once per period boundary.
                frame_period <= compute_period(frame_rate);
                pulse_w_eff  <= (pulse_width == 16'd0)
                                 ? DEFAULT_PULSE_CYCLES[15:0]
                                 : pulse_width;
            end else begin
                period_count <= period_count + 1'b1;

                if (pulse_cnt != 16'd0) begin
                    if (pulse_cnt >= pulse_w_eff) begin
                        pulse_cnt     <= 16'd0;
                        trigger_pulse <= 1'b0;
                    end else begin
                        pulse_cnt     <= pulse_cnt + 1'b1;
                        trigger_pulse <= 1'b1;
                    end
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Delayed output (single physical pin)
    //
    // Implementation: a long shift register fed by `trigger_pulse`.  The output
    // is a multiplexer that selects the appropriate tap based on
    // trigger_delay[0] (camera-0's calibration — used here because all
    // physical outputs share one pin on this PCB).  The width of the high
    // window is therefore preserved exactly.
    //--------------------------------------------------------------------------
    logic [DELAY_SR_DEPTH-1:0] delay_sr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            delay_sr <= '0;
        else
            delay_sr <= {delay_sr[DELAY_SR_DEPTH-2:0], trigger_pulse};
    end

    always_comb begin
        // Tap 0 = no delay
        if (trigger_delay[0] == 8'd0)
            fsin_out = trigger_pulse;
        else
            fsin_out = delay_sr[trigger_delay[0] - 1];
    end

    //--------------------------------------------------------------------------
    // Sanity asserts (sim only)
    //--------------------------------------------------------------------------
    // synthesis translate_off
    initial begin
        assert (NUM_OUTPUTS <= 8)
            else $fatal("trigger_generator: NUM_OUTPUTS too large for trigger_delay array");
        assert (MAX_FRAME_RATE >= 1 && MAX_FRAME_RATE <= 240)
            else $fatal("trigger_generator: MAX_FRAME_RATE must be 1..240");
    end
    // synthesis translate_on

endmodule

`default_nettype wire
