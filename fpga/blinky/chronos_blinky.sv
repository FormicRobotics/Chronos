//==============================================================================
// Chronos blinky — minimal bring-up bitstream
//
// Purpose:
//   Validate that the Radiant toolchain, the bitstream we generate, and the
//   FT2232-based JTAG path can all program user logic into the LIFCL-40 on
//   HW_CHRONOS_R1.  This design has no external dependencies — no PLL, no PDC
//   clock, no MIPI, no I2C — so it is the simplest possible smoke test.
//
// What it does:
//   * Instantiates the internal LIFCL high-frequency oscillator (~450 MHz),
//     pre-divides it down to ~45 MHz inside the OSC_CORE block.
//   * Runs a free-running 28-bit counter on that clock.
//   * Drives the four green status LEDs (D14..D17 in the schematic, "LED0..3"
//     nets) from the top four bits of the counter so each LED toggles at a
//     visibly different rate (~1 Hz, ~0.5 Hz, ~0.25 Hz, ~0.12 Hz).
//
// Visible result:
//   When the bitstream is programmed via Radiant Programmer, the four LEDs
//   should immediately start a recognisable binary-count pattern.  If you see
//   that pattern, the entire FPGA stack is healthy and we move on to the full
//   design.
//
// LED polarity:
//   LEDs are driven through NPN transistors (Q1..Q4 on the rev1 board) so a
//   '1' on the FPGA pin turns the LED ON.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module chronos_blinky (
    input  wire        rst_n,         // SW4 / GSRN button (active-low)
    output logic [3:0] led_status
);

    //--------------------------------------------------------------------------
    // Internal oscillator (450 MHz divided down).  OSC_CORE is the hardened
    // RC oscillator inside every CrossLink-NX device.  HF_CLK_DIV is the
    // pre-scaler that drops the clock to something the fabric can route
    // comfortably; 9 gives ~45 MHz.
    //--------------------------------------------------------------------------
    logic clk_int;

    OSC_CORE #(
        .HF_CLK_DIV   ("9"),         // 450 MHz / 10 ≈ 45 MHz
        .HF_OSC_EN    ("ENABLED"),
        .LF_OUTPUT_EN ("ENABLED")
    ) u_osc (
        .HFOUTEN      (1'b1),
        .HFCLKOUT     (clk_int),
        .LFCLKOUT     ()             // unused
    );

    //--------------------------------------------------------------------------
    // Reset synchroniser — bring the asynchronous GSRN button into clk_int.
    //--------------------------------------------------------------------------
    logic rst_n_meta, rst_n_sync;
    always_ff @(posedge clk_int or negedge rst_n) begin
        if (!rst_n) begin
            rst_n_meta <= 1'b0;
            rst_n_sync <= 1'b0;
        end else begin
            rst_n_meta <= 1'b1;
            rst_n_sync <= rst_n_meta;
        end
    end

    //--------------------------------------------------------------------------
    // Free-running counter.  28 bits at 45 MHz wraps every ~6 s, so the slowest
    // LED toggles every ~3 s and the fastest every ~0.37 s.
    //--------------------------------------------------------------------------
    logic [27:0] counter;
    always_ff @(posedge clk_int or negedge rst_n_sync) begin
        if (!rst_n_sync)
            counter <= '0;
        else
            counter <= counter + 28'd1;
    end

    //--------------------------------------------------------------------------
    // LED outputs — pick four well-separated taps so a human eye sees the
    // binary-count pattern clearly.
    //--------------------------------------------------------------------------
    assign led_status[0] = counter[24];   // fastest
    assign led_status[1] = counter[25];
    assign led_status[2] = counter[26];
    assign led_status[3] = counter[27];   // slowest

endmodule

`default_nettype wire
