//==============================================================================
// csi2_rx - MIPI CSI-2 receiver (soft D-PHY PPI -> packet token stream)
//==============================================================================
// Pipeline: pads -> mipi_dphy_rx (soft) -> per-lane 0xB8 word aligner -> packet
// FSM. 2 data lanes, GEAR=8 => 2 bytes per recovered byte clock
// (lane0 = even byte, lane1 = odd byte in CSI-2 transmission order).
//
// Output is a token stream in the recovered byte-clock domain (byte_clk_o),
// identical in shape to the csi2_tx sink so the frame_buffer can store it and
// the arbiter can replay it verbatim:
//   * sop_valid : one beat per packet, with {vc,dt,wc,short} + fs/fe/ls/le
//   * pay_valid : payload words (long packets), pay_last marks the final word
//
// Fixes vs old design: real IP D-PHY (not a placeholder), correct 4-byte header
// assembly, ECC over the true {WC,VC,DT} bits, and a CRC chained byte-by-byte
// in order across both lanes (old for-loop kept only the last lane).
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

import csi2_pkg::*;

module csi2_rx #(
    parameter int NUM_LANES   = 2,
    parameter int CAMERA_ID   = 0,
    parameter int GEAR        = 8,
    parameter bit BIT_REVERSE = 1'b0   // set if IDDRX4 captures MSB-first
)(
    input  wire        clk_sys,        // reference clock for D-PHY GDDR calibration
    input  wire        rst_n,
    input  wire        pll_locked,     // start D-PHY calibration
    input  wire        enable,         // host: enable this camera RX

    // D-PHY pads
    inout  wire        dphy_clk_p,
    inout  wire        dphy_clk_n,
    inout  wire [NUM_LANES-1:0] dphy_data_p,
    inout  wire [NUM_LANES-1:0] dphy_data_n,

    // recovered byte clock (write clock of the async frame buffer)
    output wire        byte_clk_o,

    // Packet token stream (byte_clk_o domain)
    output logic        sop_valid,
    output logic        sop_short,
    output logic [1:0]  sop_vc,
    output logic [5:0]  sop_dt,
    output logic [15:0] sop_wc,
    output logic        sop_fs,
    output logic        sop_fe,
    output logic        sop_ls,
    output logic        sop_le,
    output logic        pay_valid,
    output logic [31:0] pay_data,
    output logic        pay_last,
    output logic        error
);

    //--------------------------------------------------------------------------
    // Soft D-PHY RX IP
    //--------------------------------------------------------------------------
    wire                      byte_clk;
    wire [NUM_LANES*GEAR-1:0] hs_data;     // {lane1, lane0}
    wire                      dphy_ready;
    wire                      rst = ~rst_n;

    assign byte_clk_o = byte_clk;

    mipi_dphy_rx #(
        .NUM_LANE (NUM_LANES),
        .GEAR     (GEAR)
    ) u_dphy_rx (
        .clk_ref_i  (clk_sys),
        .rst_i      (rst),
        .pll_lock_i (pll_locked),
        .hs_rx_en_i (enable),
        .byte_clk_o (byte_clk),
        .hs_data_o  (hs_data),
        .ready_o    (dphy_ready),
        .clk_p_io   (dphy_clk_p),
        .clk_n_io   (dphy_clk_n),
        .data_p_io  (dphy_data_p),
        .data_n_io  (dphy_data_n)
    );

    // reset into byte-clock domain (released after IP calibration)
    logic brst_n_meta, brst_n;
    always_ff @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin brst_n_meta <= 1'b0; brst_n <= 1'b0; end
        else        begin brst_n_meta <= dphy_ready; brst_n <= brst_n_meta; end
    end

    //--------------------------------------------------------------------------
    // Raw lane bytes (optionally bit-reversed to match IDDRX4 bit order)
    //--------------------------------------------------------------------------
    logic [7:0] raw [NUM_LANES];
    always_comb begin
        for (int l = 0; l < NUM_LANES; l++) begin
            logic [7:0] b;
            b = hs_data[l*GEAR +: 8];
            if (BIT_REVERSE)
                for (int k = 0; k < 8; k++) raw[l][k] = b[7-k];
            else
                raw[l] = b;
        end
    end

    //--------------------------------------------------------------------------
    // Per-lane word aligner: detect 0xB8 across the 8 bit offsets, lock the
    // offset, then emit one aligned byte/cycle. hist holds the previous cycle's
    // bits so an aligned byte can straddle the cycle boundary.
    //--------------------------------------------------------------------------
    logic [7:0] hist [NUM_LANES];
    logic [2:0] shift [NUM_LANES];
    logic       locked [NUM_LANES];
    logic [7:0] aligned [NUM_LANES];
    logic       all_locked;

    always_comb begin
        for (int l = 0; l < NUM_LANES; l++) begin
            logic [15:0] win;
            win        = {raw[l], hist[l]};
            aligned[l] = win[shift[l] +: 8];
        end
    end

    always_comb begin
        all_locked = 1'b1;
        for (int l = 0; l < NUM_LANES; l++)
            all_locked &= locked[l];
    end

    genvar gl;
    generate
        for (gl = 0; gl < NUM_LANES; gl++) begin : g_align
            always_ff @(posedge byte_clk or negedge brst_n) begin
                if (!brst_n) begin
                    hist[gl]   <= 8'h00;
                    shift[gl]  <= 3'd0;
                    locked[gl] <= 1'b0;
                end else begin
                    hist[gl] <= raw[gl];
                    if (!locked[gl]) begin
                        logic [15:0] win;
                        logic        found;
                        win   = {raw[gl], hist[gl]};
                        found = 1'b0;
                        // Lock onto the LOWEST bit offset whose byte == 0xB8.
                        // 'found' guards against later offsets overwriting it
                        // (without it, the highest matching offset would win).
                        for (int s = 0; s < 8; s++)
                            if (!found && (win[s +: 8] == SYNC_BYTE)) begin
                                shift[gl]  <= s[2:0];
                                locked[gl] <= 1'b1;
                                found       = 1'b1;
                            end
                    end
                end
            end
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Packet FSM
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {ST_SYNC, ST_HDR1, ST_HDR2, ST_PAY, ST_CRC} state_t;
    state_t state;

    logic [7:0]  b_even, b_odd;
    assign b_even = aligned[0];
    assign b_odd  = (NUM_LANES > 1) ? aligned[1] : 8'h00;

    logic [5:0]  dt;
    logic [1:0]  vc;
    logic [7:0]  wc_lo;
    logic [15:0] crc_reg;
    logic [15:0] rem;
    logic [1:0]  word_fill;
    logic [31:0] word_acc;

    always_ff @(posedge byte_clk or negedge brst_n) begin
        if (!brst_n) begin
            state     <= ST_SYNC;
            dt <= 6'd0; vc <= 2'd0; wc_lo <= 8'd0;
            crc_reg   <= 16'hFFFF;
            rem       <= 16'd0;
            word_fill <= 2'd0;
            word_acc  <= 32'd0;
            sop_valid <= 1'b0; sop_short <= 1'b0;
            sop_vc <= 2'd0; sop_dt <= 6'd0; sop_wc <= 16'd0;
            sop_fs <= 1'b0; sop_fe <= 1'b0; sop_ls <= 1'b0; sop_le <= 1'b0;
            pay_valid <= 1'b0; pay_data <= 32'd0; pay_last <= 1'b0;
            error     <= 1'b0;
        end else begin
            sop_valid <= 1'b0;
            sop_fs <= 1'b0; sop_fe <= 1'b0; sop_ls <= 1'b0; sop_le <= 1'b0;
            pay_valid <= 1'b0; pay_last <= 1'b0;
            error     <= 1'b0;

            unique case (state)
                //--------------------------------------------------------------
                ST_SYNC: begin
                    word_fill <= 2'd0;
                    crc_reg   <= 16'hFFFF;
                    if (all_locked && (b_even == SYNC_BYTE))
                        state <= ST_HDR1;
                end
                //--------------------------------------------------------------
                ST_HDR1: begin
                    dt    <= b_even[5:0];
                    vc    <= b_even[7:6];
                    wc_lo <= b_odd;
                    state <= ST_HDR2;
                end
                //--------------------------------------------------------------
                ST_HDR2: begin
                    // B2 = wc_hi = b_even, B3 = ecc = b_odd
                    logic [15:0] wc_full;
                    logic        shortp;
                    wc_full = {b_even, wc_lo};
                    shortp  = is_short_packet(dt);
                    if (calc_ecc({wc_full, vc, dt}) != b_odd) begin
                        error <= 1'b1;
                        state <= ST_SYNC;
                    end else begin
                        sop_valid <= 1'b1;
                        sop_vc    <= vc;
                        sop_dt    <= dt;
                        sop_wc    <= wc_full;
                        sop_short <= shortp;
                        sop_fs    <= (dt == DT_FRAME_START);
                        sop_fe    <= (dt == DT_FRAME_END);
                        sop_ls    <= (dt == DT_LINE_START);
                        sop_le    <= (dt == DT_LINE_END);
                        if (shortp) begin
                            state <= ST_SYNC;
                        end else begin
                            rem       <= wc_full;
                            word_fill <= 2'd0;
                            crc_reg   <= 16'hFFFF;
                            state     <= ST_PAY;
                        end
                    end
                end
                //--------------------------------------------------------------
                ST_PAY: begin
                    logic [15:0] crc_t;
                    logic [31:0] wacc;
                    logic [1:0]  wf;
                    logic        emit;
                    logic [15:0] n;
                    crc_t = crc_reg;
                    wacc  = word_acc;
                    wf    = word_fill;
                    emit  = 1'b0;
                    n     = (rem >= NUM_LANES[15:0]) ? NUM_LANES[15:0] : rem;

                    // byte 0 (lane0, even)
                    crc_t          = crc16_step(crc_t, b_even);
                    wacc[wf*8 +: 8]= b_even;
                    wf             = wf + 2'd1;
                    if (wf == 2'd0) emit = 1'b1;
                    // byte 1 (lane1, odd) when present
                    if (n > 1) begin
                        crc_t           = crc16_step(crc_t, b_odd);
                        wacc[wf*8 +: 8] = b_odd;
                        wf              = wf + 2'd1;
                        if (wf == 2'd0) emit = 1'b1;
                    end

                    crc_reg   <= crc_t;
                    word_acc  <= wacc;
                    word_fill <= wf;

                    if (rem <= NUM_LANES[15:0]) begin
                        // final payload cycle: emit whatever we have, mark last
                        pay_valid <= 1'b1;
                        pay_data  <= wacc;
                        pay_last  <= 1'b1;
                        state     <= ST_CRC;
                    end else begin
                        rem <= rem - NUM_LANES[15:0];
                        if (emit) begin
                            pay_valid <= 1'b1;
                            pay_data  <= wacc;
                        end
                    end
                end
                //--------------------------------------------------------------
                ST_CRC: begin
                    if ({b_odd, b_even} != crc_reg)
                        error <= 1'b1;
                    state <= ST_SYNC;
                end
                default: state <= ST_SYNC;
            endcase
        end
    end

    /* verilator lint_off UNUSED */
    wire _unused = &{1'b0, CAMERA_ID[0], 1'b0};
    /* verilator lint_on UNUSED */

endmodule

`default_nettype wire
