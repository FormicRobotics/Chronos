//==============================================================================
// mipi_dphy_models - simulation-only behavioral D-PHY IP replacements
//==============================================================================
// These replace fpga/rtl/mipi/mipi_dphy_rx.v and mipi_dphy_tx.v during
// simulation so the CSI-2 packet path can be exercised without the Lattice
// primitives. Compile fpga/sim/ + fpga/rtl/ but EXCLUDE fpga/rtl/mipi/ (see
// fpga/sim/filelist_sim.f). Same module names/ports as the real wrappers.
//
// RX model: generates a 100 MHz byte clock and presents one {lane1,lane0} word
// per byte clock from an internal queue; tb pushes byte-aligned PPI words with
// feed(even, odd). TX model: captures the PPI words while data_en is high into
// a queue the tb can pop with get(word).
//==============================================================================

`timescale 1ns / 1ps

module mipi_dphy_rx #(
    parameter integer NUM_LANE = 2,
    parameter integer GEAR     = 8
)(
    input  wire                     clk_ref_i,
    input  wire                     rst_i,
    input  wire                     pll_lock_i,
    input  wire                     hs_rx_en_i,
    output wire                     byte_clk_o,
    output wire [NUM_LANE*GEAR-1:0] hs_data_o,
    output wire                     ready_o,
    inout  wire                     clk_p_io,
    inout  wire                     clk_n_io,
    inout  wire [NUM_LANE-1:0]      data_p_io,
    inout  wire [NUM_LANE-1:0]      data_n_io
);
    reg byte_clk = 1'b0;
    always #5 byte_clk = ~byte_clk;        // 100 MHz
    assign byte_clk_o = byte_clk;

    reg ready = 1'b0;
    always @(posedge byte_clk) ready <= ~rst_i & pll_lock_i;
    assign ready_o = ready;

    reg [15:0] q [$];
    reg [15:0] cur = 16'h0000;
    always @(posedge byte_clk) begin
        if (q.size() > 0) cur <= q.pop_front();
        else              cur <= 16'h0000;
    end
    assign hs_data_o = cur;                 // {lane1(odd), lane0(even)}

    // tb stimulus: push one byte-aligned PPI word (even=lane0, odd=lane1)
    task feed(input [7:0] even, input [7:0] odd);
        q.push_back({odd, even});
    endtask
    task feed_idle(input int n);
        for (int k = 0; k < n; k++) q.push_back(16'h0000);
    endtask
endmodule


module mipi_dphy_tx #(
    parameter integer NUM_LANE       = 2,
    parameter integer GEAR           = 8,
    parameter         REF_CLOCK_FREQ = 100,
    parameter         INT_DATA_RATE  = 800.000000,
    parameter         CM             = "00000000",
    parameter         CN             = "00000",
    parameter         CO             = "000"
)(
    input  wire                     clk_ref_i,
    input  wire                     rst_i,
    output wire                     byte_clk_o,
    output wire                     pll_lock_o,
    output wire                     ready_o,
    input  wire                     hs_tx_en_i,
    input  wire                     hs_tx_clk_en_i,
    input  wire [NUM_LANE*GEAR-1:0] hs_data_i,
    input  wire                     hs_data_en_i,
    inout  wire                     clk_p_io,
    inout  wire                     clk_n_io,
    inout  wire [NUM_LANE-1:0]      data_p_io,
    inout  wire [NUM_LANE-1:0]      data_n_io
);
    reg byte_clk = 1'b0;
    always #5 byte_clk = ~byte_clk;
    assign byte_clk_o = byte_clk;

    reg ready = 1'b0;
    always @(posedge byte_clk) ready <= ~rst_i;
    assign ready_o    = ready;
    assign pll_lock_o = ready;

    // capture transmitted PPI words
    reg [15:0] cap [$];
    always @(posedge byte_clk)
        if (hs_tx_en_i && hs_data_en_i) cap.push_back(hs_data_i[15:0]);

    task get(output [15:0] w, output bit ok);
        if (cap.size() > 0) begin w = cap.pop_front(); ok = 1'b1; end
        else                begin w = 16'h0;          ok = 1'b0; end
    endtask
    function int count(); return cap.size(); endfunction
endmodule
