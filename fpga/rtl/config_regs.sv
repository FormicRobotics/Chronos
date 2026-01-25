//==============================================================================
// Configuration Register Bank
// Host-accessible registers for runtime configuration and status monitoring
//==============================================================================
//
// Copyright (C) 2025 Chronos Project
// SPDX-License-Identifier: Proprietary
//
// Description:
//   This module implements the register bank that is accessible via the I2C
//   slave interface. It provides runtime configuration of capture parameters
//   and real-time status monitoring.
//
// Register Map Overview:
//   0x00-0x1F: Control registers (read/write)
//   0x20-0x3F: Status registers (read-only)
//   0xF0-0xFF: Identification registers (read-only)
//
// Access Patterns:
//   - Host writes configuration registers before/during capture
//   - Host reads status registers to monitor operation
//   - Device ID/version used for driver identification
//
// Usage:
//   1. After power-up, FPGA has default configuration (30fps, all cameras on)
//   2. Host reads VERSION and ID registers to verify communication
//   3. Host writes configuration registers as needed
//   4. Host asserts CTRL.enable to start capture
//   5. Host monitors STATUS and ERROR registers during operation
//   6. Host can read FRAME_CNT registers to verify capture activity
//
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module config_regs (
    //--------------------------------------------------------------------------
    // Clock and Reset
    //--------------------------------------------------------------------------
    input  wire         clk,            // System clock
    input  wire         rst_n,          // Active-low reset
    
    //--------------------------------------------------------------------------
    // Register Interface (from I2C Slave)
    //--------------------------------------------------------------------------
    input  wire  [7:0]  addr,           // Register address
    input  wire  [7:0]  wdata,          // Write data
    output logic [7:0]  rdata,          // Read data
    input  wire         wr_en,          // Write strobe
    input  wire         rd_en,          // Read strobe
    
    //--------------------------------------------------------------------------
    // Configuration Outputs (directly control hardware)
    //--------------------------------------------------------------------------
    output logic        trigger_enable, // Enable trigger generator
    output logic [7:0]  frame_rate,     // Frame rate setting (fps)
    output logic [15:0] pulse_width,    // Trigger pulse width (cycles)
    output logic [7:0]  trigger_delay [4], // Per-camera trigger delays
    output logic        soft_reset,     // Software reset request
    
    // Camera control
    output logic [3:0]  cam_enable,     // Per-camera enable bits
    output logic [5:0]  output_data_type, // CSI-2 data type for TX
    
    //--------------------------------------------------------------------------
    // Status Inputs (from hardware, read-only)
    //--------------------------------------------------------------------------
    input  wire  [3:0]  rx_error,       // RX error flags (per camera)
    input  wire  [3:0]  buf_overflow,   // Buffer overflow flags
    input  wire         pll_locked,     // PLL lock status
    input  wire  [3:0]  cam_sync_status, // Per-camera sync status
    input  wire  [31:0] frame_count [4]  // Frame counters (per camera)
);

    //==========================================================================
    // Register Address Map
    //==========================================================================
    //
    // CONTROL REGISTERS (0x00 - 0x1F) - Read/Write
    //
    
    localparam logic [7:0] REG_CTRL           = 8'h00;
    // Bit 0: Trigger enable (1=run, 0=stop)
    // Bit 1: Soft reset (self-clearing)
    // Bit 7:2: Reserved
    
    localparam logic [7:0] REG_FRAME_RATE     = 8'h01;
    // Frame rate in fps (1-120, default 30)
    
    localparam logic [7:0] REG_PULSE_WIDTH_L  = 8'h02;
    // Trigger pulse width, low byte
    
    localparam logic [7:0] REG_PULSE_WIDTH_H  = 8'h03;
    // Trigger pulse width, high byte
    
    localparam logic [7:0] REG_CAM_ENABLE     = 8'h04;
    // Camera enable mask (bits 3:0, default 0x0F = all on)
    
    localparam logic [7:0] REG_DATA_TYPE      = 8'h05;
    // Output data type (default 0x2B = RAW10)
    
    localparam logic [7:0] REG_TRIG_DELAY_0   = 8'h10;  // Camera 0 delay
    localparam logic [7:0] REG_TRIG_DELAY_1   = 8'h11;  // Camera 1 delay
    localparam logic [7:0] REG_TRIG_DELAY_2   = 8'h12;  // Camera 2 delay
    localparam logic [7:0] REG_TRIG_DELAY_3   = 8'h13;  // Camera 3 delay
    // Per-camera trigger delay in clock cycles (0-255)
    // Use for skew calibration
    
    //
    // STATUS REGISTERS (0x20 - 0x3F) - Read Only
    //
    
    localparam logic [7:0] REG_STATUS         = 8'h20;
    // Bit 0: PLL locked
    // Bit 3:1: Reserved
    // Bits 7:4: Camera sync status
    
    localparam logic [7:0] REG_ERROR          = 8'h21;
    // Bits 3:0: RX error flags (per camera)
    // Bits 7:4: Buffer overflow flags (per camera)
    
    localparam logic [7:0] REG_FRAME_CNT_0_L  = 8'h30;  // Cam 0 count [7:0]
    localparam logic [7:0] REG_FRAME_CNT_0_H  = 8'h31;  // Cam 0 count [15:8]
    localparam logic [7:0] REG_FRAME_CNT_1_L  = 8'h32;  // Cam 1 count [7:0]
    localparam logic [7:0] REG_FRAME_CNT_1_H  = 8'h33;  // Cam 1 count [15:8]
    localparam logic [7:0] REG_FRAME_CNT_2_L  = 8'h34;  // Cam 2 count [7:0]
    localparam logic [7:0] REG_FRAME_CNT_2_H  = 8'h35;  // Cam 2 count [15:8]
    localparam logic [7:0] REG_FRAME_CNT_3_L  = 8'h36;  // Cam 3 count [7:0]
    localparam logic [7:0] REG_FRAME_CNT_3_H  = 8'h37;  // Cam 3 count [15:8]
    
    //
    // IDENTIFICATION REGISTERS (0xF0 - 0xFF) - Read Only
    //
    
    localparam logic [7:0] REG_VERSION        = 8'hF0;  // Firmware version
    localparam logic [7:0] REG_ID_L           = 8'hFE;  // Device ID low
    localparam logic [7:0] REG_ID_H           = 8'hFF;  // Device ID high
    
    //==========================================================================
    // Identification Constants
    //==========================================================================
    
    // Firmware version: Major.Minor (BCD format)
    // v1.0 = 0x10
    localparam logic [7:0] FIRMWARE_VERSION = 8'h10;
    
    // Device ID: "C405" (Chronos 4-camera 05)
    localparam logic [15:0] DEVICE_ID = 16'hC405;
    
    //==========================================================================
    // Register Storage
    //==========================================================================
    
    logic [7:0]  ctrl_reg;
    logic [7:0]  frame_rate_reg;
    logic [15:0] pulse_width_reg;
    logic [7:0]  cam_enable_reg;
    logic [7:0]  data_type_reg;
    logic [7:0]  trig_delay_reg [4];
    
    //==========================================================================
    // Write Logic
    //==========================================================================
    // Handle register writes from I2C slave
    // Only control registers (0x00-0x1F) are writable
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            //------------------------------------------------------------------
            // Default Configuration After Reset
            //------------------------------------------------------------------
            ctrl_reg        <= 8'h00;           // Disabled
            frame_rate_reg  <= 8'd30;           // 30 fps
            pulse_width_reg <= 16'd2000;        // 10µs at 200MHz
            cam_enable_reg  <= 8'h0F;           // All 4 cameras enabled
            data_type_reg   <= 8'h2B;           // RAW10
            
            // Zero trigger delays (no skew compensation)
            for (int i = 0; i < 4; i++) begin
                trig_delay_reg[i] <= 8'h00;
            end
            
        end else if (wr_en) begin
            //------------------------------------------------------------------
            // Register Write Handler
            //------------------------------------------------------------------
            case (addr)
                REG_CTRL:          ctrl_reg        <= wdata;
                REG_FRAME_RATE:    frame_rate_reg  <= wdata;
                REG_PULSE_WIDTH_L: pulse_width_reg[7:0]  <= wdata;
                REG_PULSE_WIDTH_H: pulse_width_reg[15:8] <= wdata;
                REG_CAM_ENABLE:    cam_enable_reg  <= wdata;
                REG_DATA_TYPE:     data_type_reg   <= wdata;
                REG_TRIG_DELAY_0:  trig_delay_reg[0] <= wdata;
                REG_TRIG_DELAY_1:  trig_delay_reg[1] <= wdata;
                REG_TRIG_DELAY_2:  trig_delay_reg[2] <= wdata;
                REG_TRIG_DELAY_3:  trig_delay_reg[3] <= wdata;
                default: ; // Ignore writes to read-only or undefined registers
            endcase
        end
    end
    
    //==========================================================================
    // Read Logic
    //==========================================================================
    // Handle register reads from I2C slave
    // Return appropriate value based on address
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rdata <= 8'h00;
        end else if (rd_en) begin
            case (addr)
                //--------------------------------------------------------------
                // Control Registers (0x00 - 0x1F)
                //--------------------------------------------------------------
                REG_CTRL:          rdata <= ctrl_reg;
                REG_FRAME_RATE:    rdata <= frame_rate_reg;
                REG_PULSE_WIDTH_L: rdata <= pulse_width_reg[7:0];
                REG_PULSE_WIDTH_H: rdata <= pulse_width_reg[15:8];
                REG_CAM_ENABLE:    rdata <= cam_enable_reg;
                REG_DATA_TYPE:     rdata <= data_type_reg;
                REG_TRIG_DELAY_0:  rdata <= trig_delay_reg[0];
                REG_TRIG_DELAY_1:  rdata <= trig_delay_reg[1];
                REG_TRIG_DELAY_2:  rdata <= trig_delay_reg[2];
                REG_TRIG_DELAY_3:  rdata <= trig_delay_reg[3];
                
                //--------------------------------------------------------------
                // Status Registers (0x20 - 0x3F)
                //--------------------------------------------------------------
                REG_STATUS: begin
                    // Pack status bits
                    rdata <= {cam_sync_status, 3'b000, pll_locked};
                end
                
                REG_ERROR: begin
                    // Pack error bits: [7:4]=overflow, [3:0]=rx_error
                    rdata <= {buf_overflow, rx_error};
                end
                
                // Frame counters (16-bit per camera, split into L/H bytes)
                REG_FRAME_CNT_0_L: rdata <= frame_count[0][7:0];
                REG_FRAME_CNT_0_H: rdata <= frame_count[0][15:8];
                REG_FRAME_CNT_1_L: rdata <= frame_count[1][7:0];
                REG_FRAME_CNT_1_H: rdata <= frame_count[1][15:8];
                REG_FRAME_CNT_2_L: rdata <= frame_count[2][7:0];
                REG_FRAME_CNT_2_H: rdata <= frame_count[2][15:8];
                REG_FRAME_CNT_3_L: rdata <= frame_count[3][7:0];
                REG_FRAME_CNT_3_H: rdata <= frame_count[3][15:8];
                
                //--------------------------------------------------------------
                // Identification Registers (0xF0 - 0xFF)
                //--------------------------------------------------------------
                REG_VERSION: rdata <= FIRMWARE_VERSION;
                REG_ID_L:    rdata <= DEVICE_ID[7:0];
                REG_ID_H:    rdata <= DEVICE_ID[15:8];
                
                //--------------------------------------------------------------
                // Undefined Addresses
                //--------------------------------------------------------------
                default: rdata <= 8'h00;
            endcase
        end
    end
    
    //==========================================================================
    // Output Assignments
    //==========================================================================
    // Connect register values to output ports
    
    // From CTRL register
    assign trigger_enable   = ctrl_reg[0];
    assign soft_reset       = ctrl_reg[1];
    
    // From individual registers
    assign frame_rate       = frame_rate_reg;
    assign pulse_width      = pulse_width_reg;
    assign cam_enable       = cam_enable_reg[3:0];
    assign output_data_type = data_type_reg[5:0];
    
    // Trigger delays
    assign trigger_delay[0] = trig_delay_reg[0];
    assign trigger_delay[1] = trig_delay_reg[1];
    assign trigger_delay[2] = trig_delay_reg[2];
    assign trigger_delay[3] = trig_delay_reg[3];

endmodule

`default_nettype wire
