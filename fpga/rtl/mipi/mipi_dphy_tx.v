// =============================================================================
// mipi_dphy_tx - hard MIPI D-PHY TX wrapper (PPI in)
// -----------------------------------------------------------------------------
// Thin wrapper around Lattice lscc_mipi_wrapper_tx in HARD_IP mode with the
// internal D-PHY PLL. Serializes the PPI byte stream to the 2-lane CSI-2 link
// toward the Jetson. The packet layer (csi2_tx) drives hs_data_i in the
// byte_clk_o domain.
//
// CLOCKING: the hard D-PHY PLL takes clk_ref_i as CLKREF and multiplies it to
// the HS bit clock using CM/CN/CO. Those divider strings MUST be taken from an
// IP Catalog run for the chosen clk_ref_i frequency (REF_CLOCK_FREQ) and target
// line rate (e.g. 800 Mbps). The defaults below are placeholders -- override
// them when instantiating, or generate the IP and use its emitted parameters.
// =============================================================================

`default_nettype none

module mipi_dphy_tx #(
    parameter integer NUM_LANE       = 2,
    parameter integer GEAR           = 8,
    parameter         REF_CLOCK_FREQ = 100,                 // MHz on clk_ref_i
    parameter         INT_DATA_RATE  = 800.000000,          // Mbps/lane target
    parameter         CM             = "00000000",          // <-- from IP Catalog
    parameter         CN             = "00000",             // <-- from IP Catalog
    parameter         CO             = "000"                // <-- from IP Catalog
)(
    input  wire                       clk_ref_i,   // CLKREF to D-PHY PLL
    input  wire                       rst_i,       // active-high sync reset

    output wire                       byte_clk_o,  // TX PPI byte clock
    output wire                       pll_lock_o,
    output wire                       ready_o,

    input  wire                       hs_tx_en_i,
    input  wire                       hs_tx_clk_en_i,
    input  wire [NUM_LANE*GEAR-1:0]   hs_data_i,
    input  wire                       hs_data_en_i,

    // D-PHY pads
    inout  wire                       clk_p_io,
    inout  wire                       clk_n_io,
    inout  wire [NUM_LANE-1:0]        data_p_io,
    inout  wire [NUM_LANE-1:0]        data_n_io
);

    lscc_mipi_wrapper_tx #(
        .NUM_LANE        (NUM_LANE),
        .GEAR            (GEAR),
        .INTF            ("CSI2"),
        .DPHY_IP         ("HARD_IP"),
        .CLK_MODE        ("ENABLED"),
        .DPHY_CIL_BYPASS ("CIL_BYPASSED"),
        .PLL_MODE        ("INTERNAL"),
        .REF_CLOCK_FREQ  (REF_CLOCK_FREQ),
        .INT_DATA_RATE   (INT_DATA_RATE),
        .CM              (CM),
        .CN              (CN),
        .CO              (CO)
    ) u_tx (
        .sync_clk_i        (clk_ref_i),
        .sync_rst_i        (rst_i),
        // LMMI unused
        .lmmi_clk_i        (clk_ref_i),
        .lmmi_resetn_i     (~rst_i),
        .lmmi_wdata_i      (4'd0),
        .lmmi_wr_rdn_i     (1'b0),
        .lmmi_offset_i     (5'd0),
        .lmmi_request_i    (1'b0),
        .lmmi_ready_o      (),
        .lmmi_rdata_o      (),
        .lmmi_rdata_valid_o(),
        // external PLL feeds (unused with INTERNAL PLL)
        .pll_clkop_i       (1'b0),
        .pll_clkos_i       (1'b0),
        .pll_lock_i        (1'b0),
        // HS TX PPI
        .hs_tx_en_i        (hs_tx_en_i),
        .hs_tx_clk_en_i    (hs_tx_clk_en_i),
        .hs_tx_data_i      (hs_data_i),
        .hs_tx_data_en_i   (hs_data_en_i),
        // LP TX (unused, continuous-clock CSI-2)
        .lp_tx_en_i        (1'b0),
        .lp_tx_data_p_i    ({NUM_LANE{1'b0}}),
        .lp_tx_data_n_i    ({NUM_LANE{1'b0}}),
        .lp_tx_data_en_i   (1'b0),
        .lp_tx_clk_p_i     (1'b0),
        .lp_tx_clk_n_i     (1'b0),
        // LP RX (unused)
        .lp_rx_en_i        (1'b0),
        .lp_rx_data_p_o    (),
        .lp_rx_data_n_o    (),
        // misc
        .usrstdby_i        (1'b0),
        .pd_dphy_i         (1'b0),
        .txclk_hsgate_i    (1'b0),
        .pll_lock_o        (pll_lock_o),
        .clk_byte_o        (byte_clk_o),
        .ready_o           (ready_o),
        .hs_tx_cil_ready_o (),
        .data_lane_ss_o    (),
        // pads
        .clk_p_io          (clk_p_io),
        .clk_n_io          (clk_n_io),
        .data_p_io         (data_p_io),
        .data_n_io         (data_n_io)
    );

endmodule

`default_nettype wire
