//==============================================================================
// Chronos Multi-Camera MIPI Synchronization System
// Top-Level Module for Lattice CrossLink-NX FPGA
//==============================================================================
//
// Copyright (C) 2025 Chronos Project
// SPDX-License-Identifier: Proprietary
//
// Description:
//   This is the top-level module for the Chronos multi-camera synchronization
//   system. It aggregates four MIPI CSI-2 camera streams into a single 2-lane
//   output using virtual channel multiplexing, enabling all cameras to share
//   one CSI port on the NVIDIA Jetson Orin NX.
//
// Features:
//   - 4x CSI-2 RX controllers with hardened D-PHY IP
//   - SRAM-based line buffers for stream aggregation
//   - Virtual channel tagging (Camera 0-3 → VC 0-3)
//   - PLL-based trigger generator with <100ns skew
//   - I2C slave for host configuration
//
// Target Device:
//   Lattice CrossLink-NX (LIFCL-40 or LIFCL-17)
//
// Tool Version:
//   Lattice Radiant 3.2 or later
//
// Revision History:
//   v1.0  2025-12-11  Initial release
//
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module chronos_top #(
    //--------------------------------------------------------------------------
    // Design Parameters
    //--------------------------------------------------------------------------
    parameter int NUM_CAMERAS     = 4,          // Number of camera inputs
    parameter int NUM_DATA_LANES  = 2,          // MIPI lanes per camera
    parameter int PIXEL_WIDTH     = 10,         // Bits per pixel (RAW10)
    parameter int VC_WIDTH        = 2,          // Virtual channel ID width
    parameter int MAX_FRAME_RATE  = 120         // Maximum supported frame rate
)(
    //--------------------------------------------------------------------------
    // System Interface
    //--------------------------------------------------------------------------
    input  wire         clk_ref,                // Reference clock input (25MHz)
    input  wire         rst_n,                  // Active-low asynchronous reset
    
    //--------------------------------------------------------------------------
    // CSI-2 Camera Inputs (4x 2-lane MIPI)
    // Each camera uses differential clock and 2 data lanes
    //--------------------------------------------------------------------------
    input  wire  [NUM_CAMERAS-1:0]                     csi_rx_clk_p,   // Diff clock +
    input  wire  [NUM_CAMERAS-1:0]                     csi_rx_clk_n,   // Diff clock -
    input  wire  [NUM_CAMERAS-1:0][NUM_DATA_LANES-1:0] csi_rx_data_p,  // Data lanes +
    input  wire  [NUM_CAMERAS-1:0][NUM_DATA_LANES-1:0] csi_rx_data_n,  // Data lanes -
    
    //--------------------------------------------------------------------------
    // CSI-2 Output to Jetson (1x 2-lane MIPI)
    // Aggregated stream with 4 virtual channels
    //--------------------------------------------------------------------------
    output logic        csi_tx_clk_p,           // Output diff clock +
    output logic        csi_tx_clk_n,           // Output diff clock -
    output logic [NUM_DATA_LANES-1:0] csi_tx_data_p,  // Output data +
    output logic [NUM_DATA_LANES-1:0] csi_tx_data_n,  // Output data -
    
    //--------------------------------------------------------------------------
    // Synchronization Trigger Outputs
    // Low-skew signals to camera FSIN pins and IMU interrupt
    //--------------------------------------------------------------------------
    output logic [NUM_CAMERAS-1:0] cam_trigger, // Camera frame sync (FSIN)
    output logic        imu_trigger,            // IMU sync interrupt
    
    //--------------------------------------------------------------------------
    // Configuration Interface (I2C Slave)
    // Host uses this to configure frame rate, exposure, etc.
    //--------------------------------------------------------------------------
    input  wire         i2c_scl,                // I2C clock
    inout  wire         i2c_sda,                // I2C data (bidirectional)
    
    //--------------------------------------------------------------------------
    // Status Indicators
    //--------------------------------------------------------------------------
    output logic [3:0]  led_status              // Debug/status LEDs
);

    //==========================================================================
    // Internal Signal Declarations
    //==========================================================================
    
    //--------------------------------------------------------------------------
    // Clock and Reset Signals
    //--------------------------------------------------------------------------
    logic clk_200m;                 // 200MHz system clock from PLL
    logic clk_byte;                 // Byte clock for CSI-2 (variable)
    logic pll_locked;               // PLL lock indicator
    logic sys_rst_n;                // Synchronized system reset
    
    //--------------------------------------------------------------------------
    // CSI-2 RX Packet Interfaces
    // One set of signals per camera
    //--------------------------------------------------------------------------
    logic [NUM_CAMERAS-1:0]        rx_valid;        // Packet data valid
    logic [NUM_CAMERAS-1:0]        rx_frame_start;  // Frame start short packet
    logic [NUM_CAMERAS-1:0]        rx_frame_end;    // Frame end short packet
    logic [NUM_CAMERAS-1:0]        rx_line_start;   // Line start marker
    logic [NUM_CAMERAS-1:0]        rx_line_end;     // Line end marker
    logic [NUM_CAMERAS-1:0][31:0]  rx_data;         // Packet payload (32-bit)
    logic [NUM_CAMERAS-1:0][5:0]   rx_data_type;    // CSI-2 data type
    logic [NUM_CAMERAS-1:0][15:0]  rx_word_count;   // Packet word count
    logic [NUM_CAMERAS-1:0]        rx_error;        // CRC/ECC error flag
    
    //--------------------------------------------------------------------------
    // Frame Buffer Interfaces
    // Connect RX controllers to TX arbiter via line buffers
    //--------------------------------------------------------------------------
    logic [NUM_CAMERAS-1:0]        buf_wr_en;       // Write enable
    logic [NUM_CAMERAS-1:0][31:0]  buf_wr_data;     // Write data
    logic [NUM_CAMERAS-1:0]        buf_rd_en;       // Read enable
    logic [NUM_CAMERAS-1:0][31:0]  buf_rd_data;     // Read data
    logic [NUM_CAMERAS-1:0]        buf_empty;       // Buffer empty flag
    logic [NUM_CAMERAS-1:0]        buf_full;        // Buffer full flag
    logic [NUM_CAMERAS-1:0]        buf_overflow;    // Overflow error (sticky)
    
    //--------------------------------------------------------------------------
    // CSI-2 TX Packet Interface
    // Aggregated output stream
    //--------------------------------------------------------------------------
    logic        tx_ready;                  // TX ready to accept data
    logic        tx_valid;                  // TX data valid
    logic [31:0] tx_data;                   // TX payload data
    logic [5:0]  tx_data_type;              // TX data type
    logic [1:0]  tx_virtual_channel;        // TX virtual channel (0-3)
    logic        tx_frame_start;            // TX frame start
    logic        tx_frame_end;              // TX frame end
    logic        tx_line_start;             // TX line start
    logic        tx_line_end;               // TX line end
    
    //--------------------------------------------------------------------------
    // Trigger Generator Signals
    //--------------------------------------------------------------------------
    logic        trigger_pulse;             // Master trigger pulse
    logic [7:0]  frame_rate_cfg;            // Configured frame rate (fps)
    logic        trigger_enable;            // Trigger enable from config
    
    //--------------------------------------------------------------------------
    // Configuration Register Interface
    //--------------------------------------------------------------------------
    logic [7:0]  cfg_addr;                  // Register address
    logic [7:0]  cfg_wdata;                 // Write data
    logic [7:0]  cfg_rdata;                 // Read data
    logic        cfg_wr_en;                 // Write enable
    logic        cfg_rd_en;                 // Read enable

    //==========================================================================
    // Clock Generation (PLL)
    //==========================================================================
    // Generate system clocks from 25MHz reference:
    //   - clk_200m: 200MHz for logic
    //   - clk_byte: Variable byte clock for CSI-2 (depends on data rate)
    //==========================================================================
    
    chronos_pll u_pll (
        .clk_ref     (clk_ref),
        .rst_n       (rst_n),
        .clk_200m    (clk_200m),
        .clk_byte    (clk_byte),
        .locked      (pll_locked)
    );
    
    //--------------------------------------------------------------------------
    // Reset Synchronizer
    // Synchronizes async reset to system clock domain
    //--------------------------------------------------------------------------
    reset_sync u_reset_sync (
        .clk         (clk_200m),
        .rst_n_async (rst_n & pll_locked),
        .rst_n_sync  (sys_rst_n)
    );
    
    //==========================================================================
    // CSI-2 RX Controllers (4 instances)
    //==========================================================================
    // Each controller handles one camera's MIPI CSI-2 input:
    //   - Deserializes data from D-PHY
    //   - Parses CSI-2 packets
    //   - Extracts frame/line boundaries
    //   - Validates CRC and ECC
    //==========================================================================
    
    generate
        for (genvar i = 0; i < NUM_CAMERAS; i++) begin : gen_csi_rx
            csi2_rx #(
                .NUM_LANES   (NUM_DATA_LANES),
                .CAMERA_ID   (i)
            ) u_csi2_rx (
                // Clock and reset
                .clk_byte    (clk_byte),
                .clk_sys     (clk_200m),
                .rst_n       (sys_rst_n),
                
                // D-PHY differential interface
                .dphy_clk_p  (csi_rx_clk_p[i]),
                .dphy_clk_n  (csi_rx_clk_n[i]),
                .dphy_data_p (csi_rx_data_p[i]),
                .dphy_data_n (csi_rx_data_n[i]),
                
                // Parsed packet interface
                .pkt_valid   (rx_valid[i]),
                .pkt_data    (rx_data[i]),
                .pkt_type    (rx_data_type[i]),
                .pkt_wc      (rx_word_count[i]),
                .frame_start (rx_frame_start[i]),
                .frame_end   (rx_frame_end[i]),
                .line_start  (rx_line_start[i]),
                .line_end    (rx_line_end[i]),
                .error       (rx_error[i])
            );
        end
    endgenerate
    
    //==========================================================================
    // Frame Buffers (4 instances)
    //==========================================================================
    // SRAM-based line buffers for each camera:
    //   - Absorbs timing variations between cameras
    //   - Provides data for TX arbiter
    //   - Tracks overflow conditions
    //==========================================================================
    
    generate
        for (genvar i = 0; i < NUM_CAMERAS; i++) begin : gen_frame_buf
            frame_buffer #(
                .DATA_WIDTH  (32),
                .DEPTH       (4096),        // 16KB per camera
                .CAMERA_ID   (i)
            ) u_frame_buffer (
                .clk         (clk_200m),
                .rst_n       (sys_rst_n),
                
                // Write interface (from CSI-2 RX)
                .wr_en       (rx_valid[i]),
                .wr_data     (rx_data[i]),
                .wr_frame_start (rx_frame_start[i]),
                .wr_frame_end   (rx_frame_end[i]),
                .wr_line_start  (rx_line_start[i]),
                .wr_line_end    (rx_line_end[i]),
                .wr_data_type   (rx_data_type[i]),
                .wr_word_count  (rx_word_count[i]),
                
                // Read interface (to TX arbiter)
                .rd_en       (buf_rd_en[i]),
                .rd_data     (buf_rd_data[i]),
                
                // Status outputs
                .empty       (buf_empty[i]),
                .full        (buf_full[i]),
                .overflow    (buf_overflow[i])
            );
        end
    endgenerate
    
    //==========================================================================
    // TX Arbiter and Virtual Channel Tagger
    //==========================================================================
    // Multiplexes 4 camera streams into single output:
    //   - Round-robin scheduling between cameras
    //   - Assigns virtual channel ID to each camera's data
    //   - Maintains frame-level packet ordering
    //==========================================================================
    
    tx_arbiter #(
        .NUM_CAMERAS (NUM_CAMERAS)
    ) u_tx_arbiter (
        .clk         (clk_200m),
        .rst_n       (sys_rst_n),
        
        // Buffer interfaces
        .buf_empty   (buf_empty),
        .buf_data    (buf_rd_data),
        .buf_rd_en   (buf_rd_en),
        
        // TX interface
        .tx_ready    (tx_ready),
        .tx_valid    (tx_valid),
        .tx_data     (tx_data),
        .tx_vc       (tx_virtual_channel),
        .tx_dt       (tx_data_type),
        .tx_fs       (tx_frame_start),
        .tx_fe       (tx_frame_end),
        .tx_ls       (tx_line_start),
        .tx_le       (tx_line_end)
    );
    
    //==========================================================================
    // CSI-2 TX Controller
    //==========================================================================
    // Generates MIPI CSI-2 output stream:
    //   - Constructs packet headers with ECC
    //   - Calculates and appends CRC
    //   - Manages D-PHY state transitions (LP ↔ HS)
    //==========================================================================
    
    csi2_tx #(
        .NUM_LANES   (NUM_DATA_LANES)
    ) u_csi2_tx (
        .clk_byte    (clk_byte),
        .clk_sys     (clk_200m),
        .rst_n       (sys_rst_n),
        
        // Packet interface from arbiter
        .pkt_valid   (tx_valid),
        .pkt_ready   (tx_ready),
        .pkt_data    (tx_data),
        .pkt_type    (tx_data_type),
        .pkt_vc      (tx_virtual_channel),
        .frame_start (tx_frame_start),
        .frame_end   (tx_frame_end),
        .line_start  (tx_line_start),
        .line_end    (tx_line_end),
        
        // D-PHY differential output
        .dphy_clk_p  (csi_tx_clk_p),
        .dphy_clk_n  (csi_tx_clk_n),
        .dphy_data_p (csi_tx_data_p),
        .dphy_data_n (csi_tx_data_n)
    );
    
    //==========================================================================
    // Trigger Generator
    //==========================================================================
    // Generates precision timing signals for synchronization:
    //   - PLL-based timing for low jitter
    //   - Configurable frame rate (1-120 fps)
    //   - Low-skew distribution to all cameras and IMU
    //   - Per-output delay adjustment for calibration
    //==========================================================================
    
    trigger_generator #(
        .NUM_OUTPUTS     (NUM_CAMERAS + 1),     // 4 cameras + 1 IMU
        .CLK_FREQ_HZ     (200_000_000),          // 200MHz system clock
        .MAX_FRAME_RATE  (MAX_FRAME_RATE)        // Max 120fps
    ) u_trigger_gen (
        .clk             (clk_200m),
        .rst_n           (sys_rst_n),
        
        // Configuration from register bank
        .enable          (trigger_enable),
        .frame_rate      (frame_rate_cfg),
        
        // Trigger outputs
        .trigger_pulse   (trigger_pulse),
        .cam_trigger     (cam_trigger),
        .imu_trigger     (imu_trigger)
    );
    
    //==========================================================================
    // Configuration Interface (I2C Slave)
    //==========================================================================
    // Provides host access to configuration registers:
    //   - Standard I2C slave at address 0x3C
    //   - 8-bit register address space
    //   - Read/write access to all config registers
    //==========================================================================
    
    i2c_slave #(
        .SLAVE_ADDR  (7'h3C)
    ) u_i2c_slave (
        .clk         (clk_200m),
        .rst_n       (sys_rst_n),
        
        // I2C pins
        .scl         (i2c_scl),
        .sda         (i2c_sda),
        
        // Register interface
        .reg_addr    (cfg_addr),
        .reg_wdata   (cfg_wdata),
        .reg_rdata   (cfg_rdata),
        .reg_wr_en   (cfg_wr_en),
        .reg_rd_en   (cfg_rd_en)
    );
    
    //==========================================================================
    // Configuration Register Bank
    //==========================================================================
    // Stores runtime configuration and provides status:
    //   - Frame rate, pulse width, trigger delays
    //   - Camera enable mask
    //   - Error flags and frame counters
    //   - Device ID and firmware version
    //==========================================================================
    
    config_regs u_config_regs (
        .clk             (clk_200m),
        .rst_n           (sys_rst_n),
        
        // Register interface from I2C slave
        .addr            (cfg_addr),
        .wdata           (cfg_wdata),
        .rdata           (cfg_rdata),
        .wr_en           (cfg_wr_en),
        .rd_en           (cfg_rd_en),
        
        // Configuration outputs to other modules
        .trigger_enable  (trigger_enable),
        .frame_rate      (frame_rate_cfg),
        
        // Status inputs from other modules
        .rx_error        (rx_error),
        .buf_overflow    (buf_overflow),
        .pll_locked      (pll_locked)
    );
    
    //==========================================================================
    // Status LED Assignment
    //==========================================================================
    // Visual indicators for debugging:
    //   LED[0]: PLL locked (should be ON after power-up)
    //   LED[1]: Data activity (blinks when receiving)
    //   LED[2]: Trigger activity (blinks at frame rate)
    //   LED[3]: Error indicator (ON if any error)
    //==========================================================================
    
    assign led_status[0] = pll_locked;
    assign led_status[1] = |rx_valid;                       // Any camera active
    assign led_status[2] = trigger_pulse;                   // Trigger activity
    assign led_status[3] = |rx_error | |buf_overflow;       // Any error

endmodule

`default_nettype wire
