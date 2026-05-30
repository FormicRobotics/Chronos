// =============================================================================
// mipi_dphy_rx - soft MIPI D-PHY RX wrapper (PPI out)
// -----------------------------------------------------------------------------
// Thin wrapper around Lattice lscc_mipi_wrapper_rx in LATTICE (soft) mode.
// Recovers the byte clock from the camera's clock lane (ECLKSYNC/ECLKDIV) and
// deserializes each data lane (IDDRX4) into GEAR-bit words. There is no SoT /
// data-enable from the soft IP, so byte/word alignment is done in fabric by the
// csi2_rx packet layer (0xB8 detection).
//
// PPI:
//   byte_clk_o  : recovered byte clock (NUM_LANE*data_rate/GEAR), drives parser
//   hs_data_o   : {laneN-1, ..., lane1, lane0}, GEAR bits per lane, per byte clk
//   ready_o     : GDDR calibration done (data valid after this)
//
// IP source: this instantiates lscc_mipi_wrapper_rx. Add the Radiant install IP
// RTL to the project search path (see fpga/ip/README.md) or generate the
// "mipi_dphy" IP (RX, LATTICE soft, NUM_LANE, GEAR=8) from the IP Catalog.
// =============================================================================

`default_nettype none

module mipi_dphy_rx #(
    parameter integer NUM_LANE = 2,
    parameter integer GEAR     = 8
)(
    input  wire                       clk_ref_i,   // stable reference (clk_sys) for GDDR sync
    input  wire                       rst_i,       // active-high sync reset
    input  wire                       pll_lock_i,  // start calibration (ref PLL locked)
    input  wire                       hs_rx_en_i,  // enable HS receive

    output wire                       byte_clk_o,  // recovered byte clock
    output wire [NUM_LANE*GEAR-1:0]   hs_data_o,   // deserialized lane bytes
    output wire                       ready_o,     // calibration complete

    // D-PHY pads
    inout  wire                       clk_p_io,
    inout  wire                       clk_n_io,
    inout  wire [NUM_LANE-1:0]        data_p_io,
    inout  wire [NUM_LANE-1:0]        data_n_io
);

    lscc_mipi_wrapper_rx #(
        .NUM_LANE   (NUM_LANE),
        .GEAR       (GEAR),
        .INTF       ("CSI2_APP"),
        .CLK_MODE   ("ENABLED"),       // continuous clock from camera
        .CIL_BYPASS ("CIL_BYPASSED"),
        .DPHY_IP    ("LATTICE")        // soft D-PHY (GDDR/IDDRX4 based)
    ) u_rx (
        .sync_clk_i        (clk_ref_i),
        .sync_rst_i        (rst_i),
        // LMMI unused in soft mode
        .lmmi_clk_i        (clk_ref_i),
        .lmmi_resetn_i     (~rst_i),
        .lmmi_wdata_i      (4'd0),
        .lmmi_wr_rdn_i     (1'b0),
        .lmmi_offset_i     (5'd0),
        .lmmi_request_i    (1'b0),
        .lmmi_ready_o      (),
        .lmmi_rdata_o      (),
        .lmmi_rdata_valid_o(),
        .pll_lock_i        (pll_lock_i),
        // HS RX
        .hs_rx_en_i        (hs_rx_en_i),
        .hs_rx_data_o      (hs_data_o),
        .hs_rx_data_en_o   (),          // not driven by soft IP
        // LP RX (unused)
        .lp_rx_en_i        (1'b0),
        .lp_rx_data_p_o    (),
        .lp_rx_data_n_o    (),
        .lp_rx_clk_p_o     (),
        .lp_rx_clk_n_o     (),
        // LP TX (unused)
        .lp_tx_en_i        (1'b0),
        .lp_tx_data_p_i    (1'b0),
        .lp_tx_data_n_i    (1'b0),
        .lp_tx_data_en_i   (1'b0),
        .lp_tx_clk_p_i     (1'b0),
        .lp_tx_clk_n_i     (1'b0),
        // misc
        .ready_o           (ready_o),
        .pd_dphy_i         (1'b0),
        .usrstdby_i        (1'b0),
        .clk_byte_o        (byte_clk_o),
        // pads
        .clk_p_io          (clk_p_io),
        .clk_n_io          (clk_n_io),
        .data_p_io         (data_p_io),
        .data_n_io         (data_n_io)
    );

endmodule

`default_nettype wire
