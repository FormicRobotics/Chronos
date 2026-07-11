//==============================================================================
// ov9281_init - OV9281 SCCB configuration sequencer (one per camera)
//==============================================================================
// After the camera reset is released and a power-up delay elapses, walks an
// init ROM of {reg16, val8} entries and programs them over SCCB via i2c_master
// (3 bytes/entry: reg hi, reg lo, value). Asserts cfg_done when finished.
//
// IMPORTANT: INIT_ROM below is a STRUCTURAL placeholder containing only the
// reset/standby/stream-control registers. Paste the full OV9281 register table
// between the marked rows for production. The sequencing mechanism is complete.
//
// Module: SincereFirst SF-AOV9281 (OV9281 CSP), connector AXT624124 mating the
// board AXT524124 socket directly (see docs/sf_aov9281_direct_connect.md).
//   * I2C address depends on the port's SID strap (netlist): J3/J5 (cam0/2)
//     ground SID -> 0x60; J4/J6 (cam1/3) pull SID to VCCIO6 -> 0x10.
//     chronos_top overrides DEV_ADDR per instance; the 0x60 below is only the
//     parameter default.
//   * XCLK = 27 MHz: the module is externally clocked; the FPGA forwards the
//     X2 oscillator (L5 in, M3..M6 out), NOT 24 MHz. The pasted table's PLL
//     registers MUST target a 27 MHz input clock for the intended link rate.
//
// Design point (see fpga/DESIGN_DECISIONS.md): 30 fps, SYNCHRONIZED via FPGA
// FSIN (conn pin 16). The pasted table MUST configure 1280x800 RAW10, 2-lane
// MIPI, AND put the sensor in EXTERNAL-TRIGGER / slave-strobe mode (frame
// exposed per FSIN pulse), with VTS (regs 0x380E/0x380F) large enough for the
// 30 fps window. The frame *rate* itself comes from the FPGA trigger period,
// not free-running sensor timing, so all four cameras stay frame-locked.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ov9281_init #(
    parameter int  CLK_HZ        = 200_000_000,
    parameter int  SCL_HZ        = 400_000,
    parameter [6:0] DEV_ADDR     = 7'h60,
    parameter int  PUP_DELAY_CYC = 200_000   // ~1 ms @200 MHz after reset release
)(
    input  wire  clk,
    input  wire  rst_n,
    input  wire  start,          // level: camera powered + reset released
    inout  wire  scl,
    inout  wire  sda,
    output logic cfg_done,
    output wire  ack_error
);

    //--------------------------------------------------------------------------
    // Init ROM
    //--------------------------------------------------------------------------
    localparam int N = 4;
    // {reg[15:0], val[7:0]}
    localparam logic [23:0] INIT_ROM [0:N-1] = '{
        24'h0103_01,   // software reset
        24'h0100_00,   // standby (stream off) during configuration
        // ---- paste full OV9281 1280x800 RAW10 2-lane mode table here ----
        24'h3000_00,   // placeholder mode register
        // -----------------------------------------------------------------
        24'h0100_01    // stream on (must remain last)
    };

    //--------------------------------------------------------------------------
    // i2c master
    //--------------------------------------------------------------------------
    logic       tx_valid, tx_first, tx_last;
    logic [7:0] tx_data;
    wire        tx_ready, i2c_busy;

    i2c_master #(.CLK_HZ(CLK_HZ), .SCL_HZ(SCL_HZ)) u_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .dev_addr (DEV_ADDR),
        .scl      (scl),
        .sda      (sda),
        .tx_valid (tx_valid),
        .tx_data  (tx_data),
        .tx_first (tx_first),
        .tx_last  (tx_last),
        .tx_ready (tx_ready),
        .busy     (i2c_busy),
        .ack_error(ack_error)
    );

    //--------------------------------------------------------------------------
    // Sequencer
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {S_IDLE, S_DELAY, S_PUSH, S_WAIT, S_NEXT, S_DONE} st_t;
    st_t st;

    logic [$clog2(N+1)-1:0] idx;
    logic [1:0]             bsel;     // 0: reg hi, 1: reg lo, 2: value
    logic [31:0]            dly;

    wire [23:0] entry = INIT_ROM[idx];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            idx      <= '0;
            bsel     <= 2'd0;
            dly      <= 32'd0;
            cfg_done <= 1'b0;
            tx_valid <= 1'b0;
            tx_first <= 1'b0;
            tx_last  <= 1'b0;
            tx_data  <= 8'd0;
        end else begin
            tx_valid <= 1'b0;
            unique case (st)
                S_IDLE: begin
                    cfg_done <= 1'b0;
                    idx <= '0; bsel <= 2'd0;
                    if (start) begin dly <= PUP_DELAY_CYC; st <= S_DELAY; end
                end
                S_DELAY: begin
                    if (dly == 0) st <= S_PUSH;
                    else dly <= dly - 1'b1;
                end
                S_PUSH: begin
                    tx_valid <= 1'b1;
                    tx_first <= (bsel == 2'd0);
                    tx_last  <= (bsel == 2'd2);
                    case (bsel)
                        2'd0: tx_data <= entry[23:16];  // reg hi
                        2'd1: tx_data <= entry[15:8];   // reg lo
                        default: tx_data <= entry[7:0]; // value
                    endcase
                    st <= S_WAIT;
                end
                S_WAIT: begin
                    if (tx_ready) begin
                        if (bsel == 2'd2) st <= S_NEXT;
                        else begin bsel <= bsel + 2'd1; st <= S_PUSH; end
                    end
                end
                S_NEXT: begin
                    bsel <= 2'd0;
                    // extra settle after the software-reset entry
                    if (idx == 0) begin dly <= PUP_DELAY_CYC; end
                    if (idx == (N-1)) begin
                        st <= S_DONE;
                    end else begin
                        idx <= idx + 1'b1;
                        st  <= (idx == 0) ? S_DELAY : S_PUSH;
                    end
                end
                S_DONE: begin
                    cfg_done <= 1'b1;
                    if (!start) st <= S_IDLE;   // re-arm if camera is reset
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    /* verilator lint_off UNUSED */
    wire _unused = &{1'b0, i2c_busy, 1'b0};
    /* verilator lint_on UNUSED */

endmodule

`default_nettype wire
