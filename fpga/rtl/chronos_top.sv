//==============================================================================
// Chronos Multi-Camera MIPI Synchronisation System - top level
// Target : Lattice CrossLink-NX  LIFCL-40 (ES)  HW_CHRONOS_R1
//==============================================================================
// Data plane (all CSI-2 packet token streams):
//   pads -> csi2_rx[i] (soft D-PHY, recovers byte clk) -> frame_buffer[i]
//        (async FWFT FIFO, recovered-byte-clk -> TX-byte-clk) -> tx_arbiter
//        (4:1, VC = camera) -> csi2_tx (hard D-PHY) -> Jetson
//
// Clock domains:
//   clk_sys        : control plane + soft-RX GDDR calibration ref + TX PLL ref
//   rx_byte_clk[i] : per-camera recovered byte clock (frame_buffer write side)
//   tx_byte_clk    : hard-TX byte clock (arbiter + frame_buffer read side)
//
// See fpga/DESIGN_DECISIONS.md and fpga/ip/README.md.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

import csi2_pkg::*;

module chronos_top #(
    parameter int NUM_CAMERAS    = 4,
    parameter int NUM_DATA_LANES = 2,
    parameter int MAX_FRAME_RATE = 120
)(
    input  wire clk_ref_12m,
    input  wire clk_27m,        // X2 oscillator (U3-L5); forwarded to camera XCLKs
    input  wire rst_n,

    // Camera CSI-2 RX (four 2-lane MIPI inputs; D-PHY pads are bidirectional)
    inout  wire [NUM_CAMERAS-1:0]                       csi_rx_clk_p,
    inout  wire [NUM_CAMERAS-1:0]                       csi_rx_clk_n,
    inout  wire [NUM_CAMERAS-1:0][NUM_DATA_LANES-1:0]   csi_rx_data_p,
    inout  wire [NUM_CAMERAS-1:0][NUM_DATA_LANES-1:0]   csi_rx_data_n,

    // CSI-2 TX to Jetson (single 2-lane MIPI output)
    inout  wire                            csi_tx_clk_p,
    inout  wire                            csi_tx_clk_n,
    inout  wire [NUM_DATA_LANES-1:0]       csi_tx_data_p,
    inout  wire [NUM_DATA_LANES-1:0]       csi_tx_data_n,

    output logic                           fsin_fpga,
    output logic                           cam_resetn,
    output wire  [NUM_CAMERAS-1:0]         cam_mclk,   // XCLK to each sensor (M3..M6)

    inout  wire  [NUM_CAMERAS-1:0]         cam_scl,
    inout  wire  [NUM_CAMERAS-1:0]         cam_sda,

    input  wire                            host_i2c_scl,
    inout  wire                            host_i2c_sda,

    output logic                           imu_csn,
    output logic                           imu_sclk,
    output logic                           imu_mosi,
    input  wire                            imu_miso,
    input  wire                            imu_int1,

    input  wire                            uart_rxd,
    output logic                           uart_txd,

    output logic [3:0]                     led_status
);

    //==========================================================================
    // Clocks & resets
    //==========================================================================
    logic clk_sys, clk_byte_unused, pll_locked, sys_rst_n;

    chronos_pll u_pll (
        .clk_ref (clk_ref_12m),
        .rst_n   (rst_n),
        .clk_sys (clk_sys),
        .clk_byte(clk_byte_unused),
        .locked  (pll_locked)
    );

    reset_sync u_reset_sys (
        .clk         (clk_sys),
        .rst_n_async (rst_n & pll_locked),
        .rst_n_sync  (sys_rst_n)
    );

    // Camera XCLK: forward the 27 MHz board oscillator to all four sensors.
    // The SF-AOV9281 has no on-module oscillator; without XCLK it will not even
    // respond on SCCB, so this runs unconditionally (not gated by cam_enable).
    assign cam_mclk = {NUM_CAMERAS{clk_27m}};

    //==========================================================================
    // Control plane (clk_sys): host I2C slave + register bank + trigger gen
    //==========================================================================
    logic [7:0]  cfg_addr, cfg_wdata, cfg_rdata;
    logic        cfg_wr_en, cfg_rd_en;

    i2c_slave #(.SLAVE_ADDR(7'h3C)) u_i2c_slave (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .scl(host_i2c_scl), .sda(host_i2c_sda),
        .reg_addr(cfg_addr), .reg_wdata(cfg_wdata), .reg_rdata(cfg_rdata),
        .reg_wr_en(cfg_wr_en), .reg_rd_en(cfg_rd_en)
    );

    logic        trigger_enable, soft_reset, trigger_pulse;
    logic [7:0]  frame_rate_cfg;
    logic [15:0] pulse_width_cfg;
    logic [7:0]  trigger_delay_cfg [NUM_CAMERAS];
    logic [3:0]  cam_enable;
    logic [5:0]  output_data_type;

    logic [3:0]  cam_sync_status;
    logic [3:0]  rx_error_sys, buf_overflow_sys;
    logic [31:0] frame_count_per_cam [NUM_CAMERAS];

    trigger_generator #(
        .NUM_OUTPUTS    (NUM_CAMERAS),
        .CLK_FREQ_HZ    (200_000_000),
        .MAX_FRAME_RATE (MAX_FRAME_RATE)
    ) u_trigger_gen (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .enable(trigger_enable), .frame_rate(frame_rate_cfg),
        .pulse_width(pulse_width_cfg), .trigger_delay(trigger_delay_cfg),
        .trigger_pulse(trigger_pulse), .fsin_out(fsin_fpga)
    );

    config_regs u_config_regs (
        .clk(clk_sys), .rst_n(sys_rst_n),
        .addr(cfg_addr), .wdata(cfg_wdata), .rdata(cfg_rdata),
        .wr_en(cfg_wr_en), .rd_en(cfg_rd_en),
        .trigger_enable(trigger_enable), .frame_rate(frame_rate_cfg),
        .pulse_width(pulse_width_cfg), .trigger_delay(trigger_delay_cfg),
        .soft_reset(soft_reset), .cam_enable(cam_enable),
        .output_data_type(output_data_type),
        .rx_error(rx_error_sys), .buf_overflow(buf_overflow_sys),
        .pll_locked(pll_locked), .cam_sync_status(cam_sync_status),
        .frame_count(frame_count_per_cam)
    );

    //==========================================================================
    // TX byte-clock domain reset (clock comes from the hard-TX IP)
    //==========================================================================
    wire tx_byte_clk;
    wire tx_ready;
    logic tx_rst_n;

    reset_sync u_reset_tx (
        .clk         (tx_byte_clk),
        .rst_n_async (rst_n & tx_ready),
        .rst_n_sync  (tx_rst_n)
    );

    //==========================================================================
    // CSI-2 TX (hard D-PHY) + arbiter
    //==========================================================================
    logic        sop_valid, sop_ready, sop_short;
    logic [1:0]  sop_vc;
    logic [5:0]  sop_dt;
    logic [15:0] sop_wc;
    logic [31:0] pay_data;
    logic        pay_valid, pay_ready;

    csi2_tx #(.NUM_LANES(NUM_DATA_LANES)) u_csi2_tx (
        .clk_ref(clk_sys), .rst_n(rst_n),
        .clk_byte_o(tx_byte_clk), .tx_ready(tx_ready),
        .sop_valid(sop_valid), .sop_ready(sop_ready),
        .sop_vc(sop_vc), .sop_dt(sop_dt), .sop_wc(sop_wc), .sop_short(sop_short),
        .pay_data(pay_data), .pay_valid(pay_valid), .pay_ready(pay_ready),
        .dphy_clk_p(csi_tx_clk_p), .dphy_clk_n(csi_tx_clk_n),
        .dphy_data_p(csi_tx_data_p), .dphy_data_n(csi_tx_data_n)
    );

    logic [NUM_CAMERAS-1:0]        buf_rd_valid, buf_rd_is_sop, buf_rd_last;
    logic [NUM_CAMERAS-1:0][31:0]  buf_rd_data;
    logic [NUM_CAMERAS-1:0]        buf_pkt_avail, buf_rd_en;

    tx_arbiter #(.NUM_CAMERAS(NUM_CAMERAS)) u_tx_arbiter (
        .clk(tx_byte_clk), .rst_n(tx_rst_n),
        .buf_rd_valid(buf_rd_valid), .buf_rd_is_sop(buf_rd_is_sop),
        .buf_rd_last(buf_rd_last), .buf_rd_data(buf_rd_data),
        .buf_pkt_avail(buf_pkt_avail), .buf_rd_en(buf_rd_en),
        .sop_valid(sop_valid), .sop_ready(sop_ready),
        .sop_vc(sop_vc), .sop_dt(sop_dt), .sop_wc(sop_wc), .sop_short(sop_short),
        .pay_data(pay_data), .pay_valid(pay_valid), .pay_ready(pay_ready)
    );

    //==========================================================================
    // Per-camera RX + frame buffer + OV9281 config
    //==========================================================================
    wire [NUM_CAMERAS-1:0] rx_byte_clk;
    wire [NUM_CAMERAS-1:0] rx_rst_n;
    wire [NUM_CAMERAS-1:0] frame_pulse_sys;   // synced frame_start in clk_sys
    wire [NUM_CAMERAS-1:0] cam_cfg_done;

    generate
        for (genvar i = 0; i < NUM_CAMERAS; i++) begin : gen_cam
            // RX
            logic        rx_sop_v, rx_sop_short, rx_sop_fs, rx_sop_fe, rx_sop_ls, rx_sop_le;
            logic [1:0]  rx_sop_vc;
            logic [5:0]  rx_sop_dt;
            logic [15:0] rx_sop_wc;
            logic        rx_pay_v, rx_pay_last, rx_err;
            logic [31:0] rx_pay_data;

            csi2_rx #(.NUM_LANES(NUM_DATA_LANES), .CAMERA_ID(i)) u_csi2_rx (
                .clk_sys(clk_sys), .rst_n(rst_n), .pll_locked(pll_locked),
                .enable(cam_enable[i]),
                .dphy_clk_p(csi_rx_clk_p[i]), .dphy_clk_n(csi_rx_clk_n[i]),
                .dphy_data_p(csi_rx_data_p[i]), .dphy_data_n(csi_rx_data_n[i]),
                .byte_clk_o(rx_byte_clk[i]),
                .sop_valid(rx_sop_v), .sop_short(rx_sop_short),
                .sop_vc(rx_sop_vc), .sop_dt(rx_sop_dt), .sop_wc(rx_sop_wc),
                .sop_fs(rx_sop_fs), .sop_fe(rx_sop_fe),
                .sop_ls(rx_sop_ls), .sop_le(rx_sop_le),
                .pay_valid(rx_pay_v), .pay_data(rx_pay_data), .pay_last(rx_pay_last),
                .error(rx_err)
            );

            reset_sync u_rx_rst (
                .clk(rx_byte_clk[i]), .rst_n_async(rst_n), .rst_n_sync(rx_rst_n[i])
            );

            // token write port
            wire        wr_en    = rx_sop_v | rx_pay_v;
            wire        wr_is_sop= rx_sop_v;
            wire        wr_last  = rx_sop_v ? rx_sop_short : rx_pay_last;
            wire [31:0] wr_data  = rx_sop_v ?
                pack_sop(rx_sop_short, rx_sop_vc, rx_sop_dt, rx_sop_wc,
                         rx_sop_fs, rx_sop_fe, rx_sop_ls, rx_sop_le)
                : rx_pay_data;

            wire fb_overflow;
            frame_buffer #(.ADDR_WIDTH(11), .CAMERA_ID(i)) u_fb (
                .wr_clk(rx_byte_clk[i]), .wr_rst_n(rx_rst_n[i]),
                .wr_en(wr_en), .wr_is_sop(wr_is_sop), .wr_last(wr_last),
                .wr_data(wr_data), .full(), .overflow(fb_overflow),
                .rd_clk(tx_byte_clk), .rd_rst_n(tx_rst_n),
                .rd_en(buf_rd_en[i]), .rd_valid(buf_rd_valid[i]),
                .rd_is_sop(buf_rd_is_sop[i]), .rd_last(buf_rd_last[i]),
                .rd_data(buf_rd_data[i]), .empty(), .pkt_avail(buf_pkt_avail[i])
            );

            // OV9281 SCCB configuration (per camera).
            // SCCB address follows the board's SID strapping (netlist Chronos.NET):
            // J3-17 (cam0) and J5-17 (cam2) are grounded  -> SID=0 -> 7-bit 0x60;
            // J4-17 (cam1) and J6-17 (cam3) are pulled to VCCIO6 -> SID=1 -> 0x10.
            ov9281_init #(
                .CLK_HZ  (200_000_000),
                .DEV_ADDR((i % 2 == 0) ? 7'h60 : 7'h10)
            ) u_cam_cfg (
                .clk(clk_sys), .rst_n(sys_rst_n),
                .start(cam_resetn & cam_enable[i]),
                .scl(cam_scl[i]), .sda(cam_sda[i]),
                .cfg_done(cam_cfg_done[i]), .ack_error()
            );

            // status CDC into clk_sys
            pulse_cdc u_fs_cdc (
                .src_clk(rx_byte_clk[i]), .src_rst_n(rx_rst_n[i]), .src_pulse(rx_sop_fs),
                .dst_clk(clk_sys), .dst_rst_n(sys_rst_n), .dst_pulse(frame_pulse_sys[i])
            );
            wire rx_err_sticky;
            sticky_bit u_err_sticky (
                .clk(rx_byte_clk[i]), .rst_n(rx_rst_n[i]), .set(rx_err), .q(rx_err_sticky)
            );
            bit_sync u_err_sync (.clk(clk_sys), .rst_n(sys_rst_n),
                                 .d(rx_err_sticky), .q(rx_error_sys[i]));
            bit_sync u_ovf_sync (.clk(clk_sys), .rst_n(sys_rst_n),
                                 .d(fb_overflow),  .q(buf_overflow_sys[i]));

            // frame counter (clk_sys)
            always_ff @(posedge clk_sys or negedge sys_rst_n) begin
                if (!sys_rst_n) frame_count_per_cam[i] <= '0;
                else if (frame_pulse_sys[i]) frame_count_per_cam[i] <= frame_count_per_cam[i] + 1'b1;
            end
            assign cam_sync_status[i] = ~rx_error_sys[i] & ~buf_overflow_sys[i];
        end
    endgenerate

    //==========================================================================
    // Camera reset (shared, active-low). Hold in reset until enabled; OV9281
    // wants >= 1 ms; the ov9281_init power-up delay covers SCCB start timing.
    //==========================================================================
    logic [19:0] cam_rst_timer;
    wire         any_cam_enable = |cam_enable;
    always_ff @(posedge clk_sys or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            cam_resetn    <= 1'b0;
            cam_rst_timer <= 20'd0;
        end else if (soft_reset || !any_cam_enable) begin
            cam_resetn    <= 1'b0;
            cam_rst_timer <= 20'd0;
        end else if (cam_rst_timer != 20'hFFFFF) begin
            cam_rst_timer <= cam_rst_timer + 1'b1;   // ~5 ms ramp before release
        end else begin
            cam_resetn    <= 1'b1;
        end
    end

    //==========================================================================
    // IMU SPI placeholder (real master is future work)
    //==========================================================================
    assign imu_csn  = 1'b1;
    assign imu_sclk = 1'b0;
    assign imu_mosi = 1'b0;
    logic imu_int1_sync;
    bit_sync u_imu_int (.clk(clk_sys), .rst_n(sys_rst_n), .d(imu_int1), .q(imu_int1_sync));

    //==========================================================================
    // Debug UART loopback placeholder
    //==========================================================================
    assign uart_txd = uart_rxd;

    //==========================================================================
    // Status LEDs. ACTIVE-LOW: D3..D6 hang from VCCIO1 through R174..R177 and
    // the FPGA pin sinks the cathode (netlist; same topology as D17/INITN).
    //==========================================================================
    assign led_status[0] = ~pll_locked;
    assign led_status[1] = ~|frame_pulse_sys;
    assign led_status[2] = ~trigger_pulse;
    assign led_status[3] = ~((|rx_error_sys) | (|buf_overflow_sys));

    /* verilator lint_off UNUSED */
    wire _unused = &{1'b0, clk_byte_unused, output_data_type, imu_int1_sync,
                     cam_cfg_done, 1'b0};
    /* verilator lint_on UNUSED */

endmodule

`default_nettype wire
