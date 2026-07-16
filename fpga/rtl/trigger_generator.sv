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
// Rate generation: a phase accumulator adds `fps` every clock and fires a
// trigger when it wraps CLK_FREQ_HZ.  The previous revision computed
// CLK_FREQ_HZ / fps with a runtime divisor - a ~28-bit combinational divider
// sitting in the clk_sys datapath that cannot close timing at ~200 MHz.  The
// accumulator needs only an adder and a compare; the average rate is EXACT
// (including rates that do not divide CLK_FREQ_HZ) and edge jitter is at most
// one clk cycle, which is irrelevant for a camera FSIN.
//
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module trigger_generator #(
    parameter int NUM_OUTPUTS    = 4,
    parameter int CLK_FREQ_HZ    = 192_000_000,
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
    localparam int ACC_W               = $clog2(CLK_FREQ_HZ) + 1;
    localparam int DEFAULT_PULSE_CYCLES= CLK_FREQ_HZ / 100_000;        // 10 us
    localparam int MAX_DELAY           = 256;                          // 8-bit
    // Largest shift register we will need = max delay we can program.
    localparam int DELAY_SR_DEPTH      = MAX_DELAY;

    //--------------------------------------------------------------------------
    // Pulse period engine: fractional phase accumulator, no divider.
    //   acc += fps each cycle; when acc crosses CLK_FREQ_HZ, subtract it back
    //   and fire.  Pulses/second therefore averages exactly `fps`.
    //--------------------------------------------------------------------------
    logic [ACC_W-1:0]  acc;
    logic [7:0]        fps_lat;              // rate latched at each pulse start
    logic [15:0]       pulse_cnt;
    logic [15:0]       pulse_w_eff;          // pulse width actually used

    // Sanitised rate request (0 or out-of-range falls back to 30 fps).
    wire [7:0] fps_req = (frame_rate == 8'd0 || frame_rate > MAX_FRAME_RATE[7:0])
                       ? 8'd30 : frame_rate;

    wire [ACC_W-1:0] acc_next = acc + {{(ACC_W-8){1'b0}}, fps_lat};
    wire             fire     = acc_next >= CLK_FREQ_HZ[ACC_W-1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc           <= CLK_FREQ_HZ[ACC_W-1:0];   // first pulse fires at once
            fps_lat       <= 8'd30;
            pulse_cnt     <= '0;
            pulse_w_eff   <= DEFAULT_PULSE_CYCLES[15:0];
            trigger_pulse <= 1'b0;
        end else if (!enable) begin
            acc           <= CLK_FREQ_HZ[ACC_W-1:0];   // re-arm for prompt start
            fps_lat       <= fps_req;
            pulse_cnt     <= '0;
            trigger_pulse <= 1'b0;
        end else if (fire) begin
            acc           <= acc_next - CLK_FREQ_HZ[ACC_W-1:0];
            pulse_cnt     <= 16'd1;                    // start a new pulse
            trigger_pulse <= 1'b1;
            // Latch any rate/width change once per period boundary.
            fps_lat       <= fps_req;
            pulse_w_eff   <= (pulse_width == 16'd0)
                              ? DEFAULT_PULSE_CYCLES[15:0]
                              : pulse_width;
        end else begin
            acc <= acc_next;

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
