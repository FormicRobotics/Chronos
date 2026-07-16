//------------------------------------------------------------------------------
// Chronos PLL — generates the two fabric clocks from the 12 MHz board ref.
//------------------------------------------------------------------------------
//
// HW_CHRONOS_R1 routes a 12 MHz CMOS clock from the FT2232HL oscillator to
// FPGA pin L13 through jumper JP2 (net "12MHZ").  This module instantiates
// the LIFCL-40 hardened PLL_CORE primitive to derive the two clocks that the
// rest of the design needs:
//
//   clk_sys   192 MHz  (fabric logic, trigger generator, control plane)
//   clk_byte   96 MHz  (spare; the real byte clocks come from the D-PHY IPs)
//
// With FEEDBK_PATH = "CLKOP" the loop locks CLKOP itself to fPD x CLKFB_DIV:
//   CLKOP = 12 MHz x 16 = 192 MHz,  VCO = CLKOP x CLKOP_DIV = 1152 MHz (in the
//   800-1600 MHz CrossLink-NX range),  CLKOS = VCO / 12 = 96 MHz.
// NOTE: exactly 200 MHz is NOT reachable from a 12 MHz reference with integer
// dividers (200/12 is not an integer and fPD must stay >= 10 MHz), which is
// why clk_sys is 192 MHz. The previous parameter set (CLKFB_DIV=50, CLKOP_DIV=3)
// implied VCO = 1800 MHz - out of range, the PLL could never lock.
//
// If the JP2 jumper is missing the design will not get a reference clock; we
// also expose a simulation/back-up path that uses the internal high-frequency
// oscillator (OSC_CORE).  Pick one by setting `USE_INTERNAL_OSC` in synthesis.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module chronos_pll (
    input  wire  clk_ref,
    input  wire  rst_n,
    output logic clk_sys,        // 192 MHz
    output logic clk_byte,       // 96 MHz
    output logic locked
);

`ifdef SIMULATION
    //----------------------------------------------------------------------
    // Simulation-only model — generates clk_sys/clk_byte directly.
    //----------------------------------------------------------------------
    logic clk_sys_int  = 1'b0;
    logic clk_byte_int = 1'b0;
    always #2.604 clk_sys_int  = ~clk_sys_int;    // ~192 MHz
    always #5.208 clk_byte_int = ~clk_byte_int;   // ~96 MHz

    logic [7:0] lock_cnt;
    always_ff @(posedge clk_ref or negedge rst_n) begin
        if (!rst_n) begin
            lock_cnt <= 8'h0;
            locked   <= 1'b0;
        end else if (lock_cnt < 8'd200) begin
            lock_cnt <= lock_cnt + 1'b1;
        end else begin
            locked <= 1'b1;
        end
    end

    assign clk_sys  = clk_sys_int;
    assign clk_byte = clk_byte_int;

`else
    //----------------------------------------------------------------------
    // Synthesis target — Lattice CrossLink-NX hardened PLL.
    // The primitive name and parameter set below match the LIFCL/Radiant
    // PLL_CORE atom; Radiant's IP Catalog usually wraps this in its own
    // template, but the raw primitive is legal too.
    //----------------------------------------------------------------------
    //
    // Settings (CLKI_DIV / CLKFB_DIV / CLKOP_DIV / CLKOS_DIV), with the loop
    // closed through CLKOP (FEEDBK_PATH="CLKOP" => fCLKOP = fPD x CLKFB_DIV
    // and fVCO = fCLKOP x CLKOP_DIV):
    //   CLKI_DIV  = 1     -> phase detector at 12 MHz  (>= 10 MHz minimum)
    //   CLKFB_DIV = 16    -> CLKOP = 12 x 16 = 192 MHz (drives clk_sys)
    //   CLKOP_DIV = 6     -> VCO   = 192 x 6 = 1152 MHz (800..1600 MHz range)
    //   CLKOS_DIV = 12    -> CLKOS = 1152/12 =  96 MHz (drives clk_byte)
    //
    // Regenerate through the Radiant IP Catalog before the final build to get
    // the tool-blessed bandwidth/phase settings for this exact ratio.
    //----------------------------------------------------------------------
    /* verilator lint_off DECLFILENAME */
    PLL_CORE #(
        .REF_OSC_FREQ      ( "12.0"  ),
        .CLKI_DIV          ( 1       ),
        .CLKFB_DIV         ( 16      ),
        .CLKOP_DIV         ( 6       ),
        .CLKOP_ENABLE      ( "ENABLED" ),
        .CLKOS_DIV         ( 12      ),
        .CLKOS_ENABLE      ( "ENABLED" ),
        .FEEDBK_PATH       ( "CLKOP" )
    ) u_pll_core (
        .CLKI       ( clk_ref ),
        .CLKFB      ( clk_sys ),
        .RST        ( ~rst_n  ),
        .ENCLKOP    ( 1'b1    ),
        .ENCLKOS    ( 1'b1    ),
        .CLKOP      ( clk_sys ),
        .CLKOS      ( clk_byte ),
        .LOCK       ( locked  )
    );
    /* verilator lint_on DECLFILENAME */
`endif

endmodule

//------------------------------------------------------------------------------
// Reset synchroniser (2-FF, async assert, sync deassert)
//------------------------------------------------------------------------------
module reset_sync (
    input  wire  clk,
    input  wire  rst_n_async,
    output logic rst_n_sync
);
    logic rst_n_meta;

    always_ff @(posedge clk or negedge rst_n_async) begin
        if (!rst_n_async) begin
            rst_n_meta <= 1'b0;
            rst_n_sync <= 1'b0;
        end else begin
            rst_n_meta <= 1'b1;
            rst_n_sync <= rst_n_meta;
        end
    end
endmodule
