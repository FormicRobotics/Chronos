//==============================================================================
// tb_chronos_csi2 - self-checking CSI-2 datapath loopback testbench
//==============================================================================
// Exercises the real RTL data plane end-to-end using the behavioral D-PHY
// models in mipi_dphy_models.sv:
//
//   csi2_rx (cam0) -> token write -> frame_buffer (async CDC) ->
//   tx_arbiter (4:1, VC tag) -> csi2_tx -> [captured PPI words]
//
// The mini-DUT below mirrors the chronos_top generate-block wiring for the
// data plane (only cam0 is driven; cam1..3 are tied idle). A synthetic OV9281
// frame (FS short / RAW10 line long / FE short) is pushed into cam0's RX byte
// stream; the testbench then decodes the captured TX PPI words and checks data
// type, virtual channel (== camera index), header ECC and payload CRC.
//
// Run (Questa/ModelSim example):
//   vlog -sv fpga/rtl/csi2_pkg.sv fpga/sim/mipi_dphy_models.sv \
//        fpga/rtl/sync_fifo.sv fpga/rtl/csi2_rx.sv fpga/rtl/frame_buffer.sv \
//        fpga/rtl/tx_arbiter.sv fpga/rtl/csi2_tx.sv fpga/sim/tb_chronos_csi2.sv
//   vsim -c tb_chronos_csi2 -do "run -all; quit"
// (Do NOT include fpga/rtl/mipi/* - the models replace those wrappers.)
//==============================================================================

`timescale 1ns / 1ps

import csi2_pkg::*;

module tb_chronos_csi2;

    localparam int NUM_CAMERAS = 4;
    localparam int NUM_LANES   = 2;

    //--------------------------------------------------------------------------
    // Clocks / reset
    //--------------------------------------------------------------------------
    logic clk_sys = 1'b0;
    always #2.604 clk_sys = ~clk_sys;   // ~192 MHz reference (chronos_pll clk_sys)

    logic rst_n      = 1'b0;
    logic pll_locked = 1'b0;
    logic cam_enable = 1'b0;

    //--------------------------------------------------------------------------
    // cam0 RX -> token write
    //--------------------------------------------------------------------------
    wire        rx0_byte_clk;
    logic       rx0_sop_v, rx0_sop_short, rx0_sop_fs, rx0_sop_fe, rx0_sop_ls, rx0_sop_le;
    logic [1:0] rx0_sop_vc;
    logic [5:0] rx0_sop_dt;
    logic [15:0]rx0_sop_wc;
    logic       rx0_pay_v, rx0_pay_last, rx0_err;
    logic [31:0]rx0_pay_data;

    wire dummy_clk_p, dummy_clk_n;
    wire [NUM_LANES-1:0] dummy_data_p, dummy_data_n;

    csi2_rx #(.NUM_LANES(NUM_LANES), .CAMERA_ID(0)) u_rx0 (
        .clk_sys(clk_sys), .rst_n(rst_n), .pll_locked(pll_locked), .enable(cam_enable),
        .dphy_clk_p(dummy_clk_p), .dphy_clk_n(dummy_clk_n),
        .dphy_data_p(dummy_data_p), .dphy_data_n(dummy_data_n),
        .byte_clk_o(rx0_byte_clk),
        .sop_valid(rx0_sop_v), .sop_short(rx0_sop_short),
        .sop_vc(rx0_sop_vc), .sop_dt(rx0_sop_dt), .sop_wc(rx0_sop_wc),
        .sop_fs(rx0_sop_fs), .sop_fe(rx0_sop_fe), .sop_ls(rx0_sop_ls), .sop_le(rx0_sop_le),
        .pay_valid(rx0_pay_v), .pay_data(rx0_pay_data), .pay_last(rx0_pay_last),
        .error(rx0_err)
    );

    wire        wr0_en     = rx0_sop_v | rx0_pay_v;
    wire        wr0_is_sop = rx0_sop_v;
    wire        wr0_last   = rx0_sop_v ? rx0_sop_short : rx0_pay_last;
    wire [31:0] wr0_data   = rx0_sop_v ?
        pack_sop(rx0_sop_short, rx0_sop_vc, rx0_sop_dt, rx0_sop_wc,
                 rx0_sop_fs, rx0_sop_fe, rx0_sop_ls, rx0_sop_le)
        : rx0_pay_data;

    //--------------------------------------------------------------------------
    // Per-camera frame_buffer read interfaces to the arbiter
    //--------------------------------------------------------------------------
    wire [NUM_CAMERAS-1:0]       buf_rd_valid;
    wire [NUM_CAMERAS-1:0]       buf_rd_is_sop;
    wire [NUM_CAMERAS-1:0]       buf_rd_last;
    wire [NUM_CAMERAS-1:0][31:0] buf_rd_data;
    wire [NUM_CAMERAS-1:0]       buf_pkt_avail;
    wire [NUM_CAMERAS-1:0]       buf_rd_en;

    wire tx_byte_clk, tx_ready;

    frame_buffer #(.ADDR_WIDTH(11), .CAMERA_ID(0)) u_fb0 (
        .wr_clk(rx0_byte_clk), .wr_rst_n(rst_n),
        .wr_en(wr0_en), .wr_is_sop(wr0_is_sop), .wr_last(wr0_last),
        .wr_data(wr0_data), .full(), .overflow(),
        .rd_clk(tx_byte_clk), .rd_rst_n(rst_n),
        .rd_en(buf_rd_en[0]), .rd_valid(buf_rd_valid[0]),
        .rd_is_sop(buf_rd_is_sop[0]), .rd_last(buf_rd_last[0]),
        .rd_data(buf_rd_data[0]), .empty(), .pkt_avail(buf_pkt_avail[0])
    );

    // cam1..3 idle
    assign buf_rd_valid [NUM_CAMERAS-1:1] = '0;
    assign buf_rd_is_sop[NUM_CAMERAS-1:1] = '0;
    assign buf_rd_last  [NUM_CAMERAS-1:1] = '0;
    assign buf_pkt_avail[NUM_CAMERAS-1:1] = '0;
    genvar gi;
    generate
        for (gi = 1; gi < NUM_CAMERAS; gi++) assign buf_rd_data[gi] = 32'h0;
    endgenerate

    //--------------------------------------------------------------------------
    // Arbiter + TX
    //--------------------------------------------------------------------------
    wire        arb_sop_v, arb_sop_short, arb_pay_v;
    wire [1:0]  arb_sop_vc;
    wire [5:0]  arb_sop_dt;
    wire [15:0] arb_sop_wc;
    wire [31:0] arb_pay_data;
    wire        tx_sop_ready, tx_pay_ready;

    tx_arbiter #(.NUM_CAMERAS(NUM_CAMERAS)) u_arb (
        .clk(tx_byte_clk), .rst_n(rst_n),
        .buf_rd_valid(buf_rd_valid), .buf_rd_is_sop(buf_rd_is_sop),
        .buf_rd_last(buf_rd_last), .buf_rd_data(buf_rd_data),
        .buf_pkt_avail(buf_pkt_avail), .buf_rd_en(buf_rd_en),
        .sop_valid(arb_sop_v), .sop_ready(tx_sop_ready),
        .sop_vc(arb_sop_vc), .sop_dt(arb_sop_dt), .sop_wc(arb_sop_wc),
        .sop_short(arb_sop_short), .pay_data(arb_pay_data),
        .pay_valid(arb_pay_v), .pay_ready(tx_pay_ready)
    );

    wire tx_clk_p, tx_clk_n;
    wire [NUM_LANES-1:0] tx_data_p, tx_data_n;

    csi2_tx #(.NUM_LANES(NUM_LANES)) u_tx (
        .clk_ref(clk_sys), .rst_n(rst_n),
        .clk_byte_o(tx_byte_clk), .tx_ready(tx_ready),
        .sop_valid(arb_sop_v), .sop_ready(tx_sop_ready),
        .sop_vc(arb_sop_vc), .sop_dt(arb_sop_dt), .sop_wc(arb_sop_wc),
        .sop_short(arb_sop_short), .pay_data(arb_pay_data),
        .pay_valid(arb_pay_v), .pay_ready(tx_pay_ready),
        .dphy_clk_p(tx_clk_p), .dphy_clk_n(tx_clk_n),
        .dphy_data_p(tx_data_p), .dphy_data_n(tx_data_n)
    );

    //--------------------------------------------------------------------------
    // Stimulus helpers (push byte-aligned PPI words into cam0 RX model)
    //--------------------------------------------------------------------------
    task automatic feed_idle(input int n);
        u_rx0.u_dphy_rx.feed_idle(n);
    endtask

    task automatic feed_short(input logic [1:0] vc, input logic [5:0] dt,
                              input logic [15:0] sdata);
        logic [7:0] b0, b1, b2, b3;
        b0 = {vc, dt};
        b1 = sdata[7:0];
        b2 = sdata[15:8];
        b3 = calc_ecc({sdata, vc, dt});
        u_rx0.u_dphy_rx.feed(SYNC_BYTE, SYNC_BYTE);
        u_rx0.u_dphy_rx.feed(b0, b1);
        u_rx0.u_dphy_rx.feed(b2, b3);
    endtask

    // payload taken from module-scope line_pay (Icarus does not support 'ref'
    // task ports); len even for simplicity
    byte unsigned line_pay [];

    task automatic feed_long(input logic [1:0] vc, input logic [5:0] dt);
        int len; logic [15:0] wc, crc; logic [7:0] b3; int k;
        len = line_pay.size();
        wc  = len[15:0];
        b3  = calc_ecc({wc, vc, dt});
        u_rx0.u_dphy_rx.feed(SYNC_BYTE, SYNC_BYTE);
        u_rx0.u_dphy_rx.feed({vc, dt}, wc[7:0]);
        u_rx0.u_dphy_rx.feed(wc[15:8], b3);
        crc = 16'hFFFF;
        for (k = 0; k < len; k += 2) begin
            u_rx0.u_dphy_rx.feed(line_pay[k], line_pay[k+1]);
            crc = crc16_step(crc, line_pay[k]);
            crc = crc16_step(crc, line_pay[k+1]);
        end
        u_rx0.u_dphy_rx.feed(crc[7:0], crc[15:8]);
    endtask

    //--------------------------------------------------------------------------
    // TX decode / scoreboard
    //--------------------------------------------------------------------------
    int errors    = 0;
    int pkt_count = 0;

    task automatic get_word(output logic [15:0] w);
        bit ok; int spin;
        spin = 0;
        forever begin
            u_tx.u_dphy_tx.get(w, ok);
            if (ok) break;
            @(posedge tx_byte_clk);
            spin++;
            if (spin > 100000) begin
                $error("[TB] timeout waiting for TX word"); errors++; w = 16'h0; break;
            end
        end
    endtask

    task automatic check(input bit cond, input string msg);
        if (!cond) begin $error("[TB] CHECK FAILED: %s", msg); errors++; end
    endtask

    // decode one TX packet, verify ECC/CRC/VC, return dt
    task automatic decode_packet(input logic [1:0] exp_vc, output logic [5:0] dt);
        logic [15:0] w, wc, crc, crc_calc;
        logic [1:0]  vc;
        logic [7:0]  b0, b1, b2, b3;
        int k;
        get_word(w);
        check((w == {SYNC_BYTE, SYNC_BYTE}), "TX sync word");
        get_word(w); b0 = w[7:0];  b1 = w[15:8];
        get_word(w); b2 = w[7:0];  b3 = w[15:8];
        vc = b0[7:6]; dt = b0[5:0]; wc = {b2, b1};
        check((vc == exp_vc), $sformatf("VC tag (got %0d exp %0d)", vc, exp_vc));
        check((calc_ecc({wc, vc, dt}) == b3), "TX header ECC");
        if (!is_short_packet(dt)) begin
            crc_calc = 16'hFFFF;
            for (k = 0; k < wc; k += 2) begin
                get_word(w);
                crc_calc = crc16_step(crc_calc, w[7:0]);
                if (k+1 < wc) crc_calc = crc16_step(crc_calc, w[15:8]);
            end
            get_word(w); crc = {w[15:8], w[7:0]};
            check((crc == crc_calc), $sformatf("TX payload CRC (got %04h exp %04h)", crc, crc_calc));
        end
        pkt_count++;
    endtask

    // Any RX protocol error during the run is a failure (counted here so the
    // final check is meaningful; a single-cycle pulse is easy to miss).
    int rx_err_seen = 0;
    always @(posedge rx0_byte_clk) if (rx0_err) rx_err_seen++;

    //--------------------------------------------------------------------------
    // Test sequence
    //--------------------------------------------------------------------------
    logic [5:0]   dt_fs, dt_ln, dt_fe;

    initial begin
        // build a 16-byte RAW10 payload
        line_pay = new[16];
        foreach (line_pay[i]) line_pay[i] = 8'h10 + i[7:0];

        rst_n = 1'b0; pll_locked = 1'b0; cam_enable = 1'b0;
        #50;
        rst_n = 1'b1; pll_locked = 1'b1; cam_enable = 1'b1;
        #50;

        // RX stream: prime aligner + a full frame (FS / RAW10 line / FE)
        feed_idle(8);
        feed_short(2'd0, DT_FRAME_START, 16'h0001);
        feed_idle(4);
        feed_long (2'd0, DT_RAW10);
        feed_idle(4);
        feed_short(2'd0, DT_FRAME_END, 16'h0001);
        feed_idle(16);

        // decode expected TX packets (VC == cam0 == 0)
        decode_packet(2'd0, dt_fs);
        check((dt_fs == DT_FRAME_START), "first TX packet is FS");
        decode_packet(2'd0, dt_ln);
        check((dt_ln == DT_RAW10), "second TX packet is RAW10");
        decode_packet(2'd0, dt_fe);
        check((dt_fe == DT_FRAME_END), "third TX packet is FE");

        check((rx_err_seen == 0),
              $sformatf("no RX protocol errors during test (saw %0d)", rx_err_seen));

        #200;
        $display("==================================================");
        $display("[TB] packets decoded : %0d", pkt_count);
        $display("[TB] errors          : %0d", errors);
        if (errors == 0) $display("[TB] RESULT: PASS");
        else             $display("[TB] RESULT: FAIL");
        $display("==================================================");
        $finish;
    end

    // global watchdog
    initial begin
        #2_000_000;
        $error("[TB] global timeout");
        $finish;
    end

endmodule
