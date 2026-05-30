//==============================================================================
// Chronos blinky - minimal bring-up bitstream  (Verilog-2001)
//
// Plain Verilog-2001 version for Radiant 2.0 SP1, which targets the ES1
// silicon (JTAG IDCODE 0x010F1043) on HW_CHRONOS_R1.  Radiant 2.0 SP1's LSE
// does not fully support SystemVerilog, so this file deliberately avoids all
// SV-only constructs (logic, always_ff, '0, etc.).
//
// Behaviour is identical to chronos_blinky.sv:
//   * OSC_CORE internal RC oscillator (~450 MHz) pre-divided to ~45 MHz.
//   * Free-running 28-bit counter.
//   * Top four counter bits drive the four green status LEDs (D14..D17).
//
// LED polarity: driven through NPN transistors (Q1..Q4) so '1' = LED ON.
//==============================================================================

`timescale 1ns / 1ps

module chronos_blinky (
    input  wire       rst_n,         // SW4 / GSRN button (active-low)
    output wire [3:0] led_status
);

    //--------------------------------------------------------------------------
    // Internal oscillator (450 MHz divided down by HF_CLK_DIV=9 -> ~45 MHz).
    //--------------------------------------------------------------------------
    wire clk_int;

    OSC_CORE #(
        .HF_CLK_DIV   ("9"),
        .HF_OSC_EN    ("ENABLED"),
        .LF_OUTPUT_EN ("ENABLED")
    ) u_osc (
        .HFOUTEN      (1'b1),
        .HFCLKOUT     (clk_int),
        .LFCLKOUT     ()
    );

    //--------------------------------------------------------------------------
    // Reset synchroniser - bring the asynchronous GSRN button into clk_int.
    //--------------------------------------------------------------------------
    reg rst_n_meta, rst_n_sync;
    always @(posedge clk_int or negedge rst_n) begin
        if (!rst_n) begin
            rst_n_meta <= 1'b0;
            rst_n_sync <= 1'b0;
        end else begin
            rst_n_meta <= 1'b1;
            rst_n_sync <= rst_n_meta;
        end
    end

    //--------------------------------------------------------------------------
    // Free-running counter.
    //--------------------------------------------------------------------------
    reg [27:0] counter;
    always @(posedge clk_int or negedge rst_n_sync) begin
        if (!rst_n_sync)
            counter <= 28'd0;
        else
            counter <= counter + 28'd1;
    end

    //--------------------------------------------------------------------------
    // LED outputs.
    //--------------------------------------------------------------------------
    assign led_status[0] = counter[24];   // fastest
    assign led_status[1] = counter[25];
    assign led_status[2] = counter[26];
    assign led_status[3] = counter[27];   // slowest

endmodule
