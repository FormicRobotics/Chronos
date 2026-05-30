//==============================================================================
// csi2_tx - MIPI CSI-2 transmitter (packet sink -> hard D-PHY PPI)
//==============================================================================
// Builds CSI-2 packets and drives the hard MIPI D-PHY TX IP, entirely in the TX
// byte-clock domain recovered from the IP (clk_byte_o), which also clocks the
// arbiter. 2 lanes, GEAR=8 => 2 bytes/byte-clock (lane0 even, lane1 odd).
//
// Payload is buffered in a small prefetch FIFO and the first word is preloaded
// before HS starts, so high-speed transmission never stalls mid-packet (a stall
// would corrupt the CSI-2 stream). Upstream (frame_buffer) guarantees the whole
// packet is present before the arbiter starts it.
//
// Fixes vs old design: correct header byte order, shared ECC/CRC from csi2_pkg,
// CRC chained byte-by-byte in order, real IP back end (no placeholder).
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

import csi2_pkg::*;

module csi2_tx #(
    parameter int NUM_LANES = 2,
    parameter int GEAR       = 8,
    parameter         REF_CLOCK_FREQ = 100,
    parameter         INT_DATA_RATE  = 800.000000,
    parameter         CM             = "00000000",
    parameter         CN             = "00000",
    parameter         CO             = "000"
)(
    input  wire        clk_ref,        // CLKREF to TX D-PHY PLL (clk_sys)
    input  wire        rst_n,

    output wire        clk_byte_o,     // TX byte clock (clocks the arbiter)
    output wire        tx_ready,       // IP ready (PLL locked)

    // Packet sink (clk_byte_o domain)
    input  wire        sop_valid,
    output logic       sop_ready,
    input  wire [1:0]  sop_vc,
    input  wire [5:0]  sop_dt,
    input  wire [15:0] sop_wc,
    input  wire        sop_short,
    input  wire [31:0] pay_data,
    input  wire        pay_valid,
    output wire        pay_ready,

    // D-PHY pads (D-PHY cells are bidirectional)
    inout  wire        dphy_clk_p,
    inout  wire        dphy_clk_n,
    inout  wire [NUM_LANES-1:0] dphy_data_p,
    inout  wire [NUM_LANES-1:0] dphy_data_n
);

    wire clk_byte;
    wire rst = ~rst_n;
    wire ip_ready;
    assign clk_byte_o = clk_byte;
    assign tx_ready   = ip_ready;

    // byte-clock-domain reset (released after IP calibration)
    logic brst_n_meta, brst_n;
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin brst_n_meta <= 1'b0; brst_n <= 1'b0; end
        else        begin brst_n_meta <= ip_ready; brst_n <= brst_n_meta; end
    end

    //--------------------------------------------------------------------------
    // Payload prefetch FIFO
    //--------------------------------------------------------------------------
    wire        pf_full, pf_empty;
    wire [31:0] pf_rd_data;
    logic       pf_rd_en;

    sync_fifo #(.WIDTH(32), .DEPTH(16)) u_payfifo (
        .clk    (clk_byte),
        .rst_n  (brst_n),
        .wr_en  (pay_valid),
        .wr_data(pay_data),
        .full   (pf_full),
        .rd_en  (pf_rd_en),
        .rd_data(pf_rd_data),
        .empty  (pf_empty)
    );
    assign pay_ready = !pf_full;

    //--------------------------------------------------------------------------
    // Packetizer FSM
    //--------------------------------------------------------------------------
    typedef enum logic [2:0]
        {ST_IDLE, ST_PRELOAD, ST_SYNC, ST_HDR1, ST_HDR2, ST_PAY, ST_CRC, ST_GAP} state_t;
    state_t state;

    logic [1:0]  vc_r;
    logic [5:0]  dt_r;
    logic [15:0] wc_r;
    logic        short_r;
    logic [7:0]  ecc_r;
    logic [15:0] crc_reg;
    logic [15:0] rem;
    logic [1:0]  bidx;

    logic [7:0]  byte_lo, byte_hi;
    logic        hs_active;
    logic        data_en;

    always_ff @(posedge clk_byte or negedge brst_n) begin
        if (!brst_n) begin
            state    <= ST_IDLE;
            vc_r <= 2'd0; dt_r <= 6'd0; wc_r <= 16'd0; short_r <= 1'b0;
            ecc_r <= 8'd0; crc_reg <= 16'hFFFF; rem <= 16'd0; bidx <= 2'd0;
            byte_lo <= 8'd0; byte_hi <= 8'd0;
            hs_active <= 1'b0; data_en <= 1'b0;
            sop_ready <= 1'b0; pf_rd_en <= 1'b0;
        end else begin
            sop_ready <= 1'b0;
            pf_rd_en  <= 1'b0;
            data_en   <= 1'b0;

            unique case (state)
                //--------------------------------------------------------------
                ST_IDLE: begin
                    hs_active <= 1'b0;
                    if (sop_valid) begin
                        vc_r    <= sop_vc;
                        dt_r    <= sop_dt;
                        wc_r    <= sop_wc;
                        short_r <= sop_short;
                        ecc_r   <= calc_ecc({sop_wc, sop_vc, sop_dt});
                        crc_reg <= 16'hFFFF;
                        rem     <= sop_wc;
                        bidx    <= 2'd0;
                        sop_ready <= 1'b1;
                        state   <= sop_short ? ST_SYNC : ST_PRELOAD;
                    end
                end
                //--------------------------------------------------------------
                ST_PRELOAD: begin
                    // hold in LP until the first payload word is buffered
                    if (!pf_empty) state <= ST_SYNC;
                end
                //--------------------------------------------------------------
                ST_SYNC: begin
                    hs_active <= 1'b1;
                    byte_lo   <= SYNC_BYTE;
                    byte_hi   <= SYNC_BYTE;
                    data_en   <= 1'b1;
                    state     <= ST_HDR1;
                end
                //--------------------------------------------------------------
                ST_HDR1: begin
                    byte_lo <= {vc_r, dt_r};   // B0
                    byte_hi <= wc_r[7:0];      // B1
                    data_en <= 1'b1;
                    state   <= ST_HDR2;
                end
                //--------------------------------------------------------------
                ST_HDR2: begin
                    byte_lo <= wc_r[15:8];     // B2
                    byte_hi <= ecc_r;          // B3
                    data_en <= 1'b1;
                    state   <= short_r ? ST_GAP : ST_PAY;
                end
                //--------------------------------------------------------------
                ST_PAY: begin
                    if (!pf_empty) begin
                        logic [7:0]  lo, hi;
                        logic [15:0] crc_t;
                        logic [1:0]  nbi;
                        logic        last_word;
                        lo  = pf_rd_data[bidx*8 +: 8];
                        nbi = bidx + 2'd1;
                        hi  = pf_rd_data[nbi*8 +: 8];
                        crc_t = crc_reg;

                        byte_lo <= lo;
                        crc_t   = crc16_step(crc_t, lo);
                        if (rem > 1) begin
                            byte_hi <= hi;
                            crc_t   = crc16_step(crc_t, hi);
                        end else begin
                            byte_hi <= 8'h00;
                        end
                        data_en <= 1'b1;
                        crc_reg <= crc_t;

                        last_word = (rem <= NUM_LANES[15:0]) ||
                                    ((bidx + 2'd2) == 2'd0);
                        if (last_word) pf_rd_en <= 1'b1;   // advance to next word

                        if (rem <= NUM_LANES[15:0]) begin
                            state <= ST_CRC;
                            bidx  <= 2'd0;
                        end else begin
                            rem  <= rem - NUM_LANES[15:0];
                            bidx <= bidx + 2'd2;
                        end
                    end
                end
                //--------------------------------------------------------------
                ST_CRC: begin
                    byte_lo <= crc_reg[7:0];   // CRC low byte first
                    byte_hi <= crc_reg[15:8];
                    data_en <= 1'b1;
                    state   <= ST_GAP;
                end
                //--------------------------------------------------------------
                ST_GAP: begin
                    hs_active <= 1'b1;
                    data_en   <= 1'b0;
                    state     <= ST_IDLE;
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    //--------------------------------------------------------------------------
    // Hard D-PHY TX IP
    //--------------------------------------------------------------------------
    wire [NUM_LANES*GEAR-1:0] hs_data = {byte_hi, byte_lo};

    mipi_dphy_tx #(
        .NUM_LANE       (NUM_LANES),
        .GEAR           (GEAR),
        .REF_CLOCK_FREQ (REF_CLOCK_FREQ),
        .INT_DATA_RATE  (INT_DATA_RATE),
        .CM             (CM),
        .CN             (CN),
        .CO             (CO)
    ) u_dphy_tx (
        .clk_ref_i      (clk_ref),
        .rst_i          (rst),
        .byte_clk_o     (clk_byte),
        .pll_lock_o     (),
        .ready_o        (ip_ready),
        .hs_tx_en_i     (hs_active),
        .hs_tx_clk_en_i (1'b1),
        .hs_data_i      (hs_data),
        .hs_data_en_i   (data_en),
        .clk_p_io       (dphy_clk_p),
        .clk_n_io       (dphy_clk_n),
        .data_p_io      (dphy_data_p),
        .data_n_io      (dphy_data_n)
    );

endmodule

`default_nettype wire
