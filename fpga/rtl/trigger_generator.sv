//==============================================================================
// Precision Trigger Generator
// Generates synchronized trigger pulses for cameras and IMU
//==============================================================================
//
// Copyright (C) 2025 Chronos Project
// SPDX-License-Identifier: Proprietary
//
// Description:
//   This module generates precision timing signals for multi-camera
//   synchronization. It uses the system PLL clock for low jitter and
//   provides configurable frame rates with per-output delay adjustment.
//
// Synchronization Architecture:
//   1. Master period counter generates frame timing
//   2. Pulse generator creates trigger pulse of configurable width
//   3. Per-output delay lines compensate for cable/path differences
//   4. All outputs transition within <100ns of each other
//
// Key Features:
//   - Frame rates from 1 to 120 fps
//   - Configurable pulse width
//   - Per-output delay calibration (0-255 clock cycles)
//   - Low jitter using PLL clock
//
// Usage:
//   1. Set frame_rate register (e.g., 30 for 30fps)
//   2. Set pulse_width if needed (default 10µs)
//   3. Calibrate trigger_delay if needed
//   4. Assert enable to start triggering
//
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module trigger_generator #(
    //--------------------------------------------------------------------------
    // Module Parameters
    //--------------------------------------------------------------------------
    parameter int NUM_OUTPUTS    = 5,               // 4 cameras + 1 IMU
    parameter int CLK_FREQ_HZ    = 200_000_000,     // System clock frequency
    parameter int MAX_FRAME_RATE = 120              // Maximum frame rate
)(
    //--------------------------------------------------------------------------
    // Clock and Reset
    //--------------------------------------------------------------------------
    input  wire         clk,                // System clock (200MHz)
    input  wire         rst_n,              // Active-low reset
    
    //--------------------------------------------------------------------------
    // Configuration Inputs
    //--------------------------------------------------------------------------
    input  wire         enable,             // Enable trigger generation
    input  wire  [7:0]  frame_rate,         // Target frame rate (1-120 fps)
    input  wire  [15:0] pulse_width,        // Pulse width in clock cycles
    input  wire  [7:0]  trigger_delay [4],  // Per-output delay (cycles)
    
    //--------------------------------------------------------------------------
    // Trigger Outputs
    //--------------------------------------------------------------------------
    output logic        trigger_pulse,      // Master pulse (monitoring)
    output logic [3:0]  cam_trigger,        // Camera FSIN signals
    output logic        imu_trigger         // IMU interrupt signal
);

    //==========================================================================
    // Local Parameters
    //==========================================================================
    
    // Calculate counter width based on minimum frequency (1 fps)
    localparam int MIN_PERIOD = CLK_FREQ_HZ / 1;            // Period for 1 fps
    localparam int CNT_WIDTH  = $clog2(MIN_PERIOD) + 1;     // Counter bits needed
    
    // Default pulse width: 10 microseconds at 200MHz = 2000 cycles
    localparam int DEFAULT_PULSE_WIDTH = CLK_FREQ_HZ / 100000;  // 10µs
    
    //==========================================================================
    // Internal Signals
    //==========================================================================
    
    // Period counter and configuration
    logic [CNT_WIDTH-1:0] period_count;     // Main period counter
    logic [CNT_WIDTH-1:0] frame_period;     // Calculated period for frame rate
    logic [15:0]          pulse_count;      // Pulse width counter
    logic [15:0]          pulse_width_reg;  // Registered pulse width
    
    // Master trigger and delayed versions
    logic master_trigger;                           // Raw trigger pulse
    logic [NUM_OUTPUTS-1:0] delayed_trigger;        // Per-output delayed triggers
    
    // Delay control
    logic [7:0] delay_cnt    [NUM_OUTPUTS-1:0];     // Delay counters
    logic       delay_active [NUM_OUTPUTS-1:0];     // Delay in progress

    //==========================================================================
    // Frame Period Calculation
    //==========================================================================
    // Calculate the counter period based on requested frame rate.
    // Period = Clock Frequency / Frame Rate
    //
    // Example: 30 fps at 200MHz = 200,000,000 / 30 = 6,666,666 cycles
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_period <= CLK_FREQ_HZ / 30;       // Default: 30 fps
        end else begin
            // Validate frame rate and calculate period
            if (frame_rate > 0 && frame_rate <= MAX_FRAME_RATE) begin
                frame_period <= CLK_FREQ_HZ / frame_rate;
            end else begin
                // Invalid rate - fall back to 30 fps
                frame_period <= CLK_FREQ_HZ / 30;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Pulse Width Configuration
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_width_reg <= DEFAULT_PULSE_WIDTH[15:0];
        end else begin
            pulse_width_reg <= (pulse_width > 0) ? pulse_width : 
                                                   DEFAULT_PULSE_WIDTH[15:0];
        end
    end
    
    //==========================================================================
    // Master Trigger Generation
    //==========================================================================
    // Two-stage process:
    //   1. Period counter wraps at frame_period
    //   2. Pulse counter generates pulse of configured width
    //==========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_count   <= '0;
            pulse_count    <= 16'h0;
            master_trigger <= 1'b0;
        end else if (!enable) begin
            // Disabled - reset everything
            period_count   <= '0;
            pulse_count    <= 16'h0;
            master_trigger <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Period Counter
            // Counts up to frame_period-1, then wraps
            //------------------------------------------------------------------
            if (period_count >= frame_period - 1) begin
                period_count <= '0;             // Wrap to zero
                pulse_count  <= 16'h0;          // Reset pulse counter
            end else begin
                period_count <= period_count + 1'b1;
            end
            
            //------------------------------------------------------------------
            // Pulse Generation
            // Starts pulse when period wraps, ends after pulse_width cycles
            //------------------------------------------------------------------
            if (period_count == 0) begin
                pulse_count <= 16'h1;           // Start pulse
            end else if (pulse_count > 0 && pulse_count < pulse_width_reg) begin
                pulse_count <= pulse_count + 1'b1;
            end else if (pulse_count >= pulse_width_reg) begin
                pulse_count <= 16'h0;           // End pulse
            end
            
            // Master trigger follows pulse counter
            master_trigger <= (pulse_count > 0) && (pulse_count <= pulse_width_reg);
        end
    end
    
    // Export master trigger for monitoring
    assign trigger_pulse = master_trigger;
    
    //==========================================================================
    // Per-Output Delay Lines
    //==========================================================================
    // Each output can be delayed independently for skew compensation.
    // This allows calibration to account for cable length differences
    // and ensures all cameras receive the trigger within <100ns.
    //
    // Delay is specified in clock cycles:
    //   - 1 cycle at 200MHz = 5ns
    //   - Maximum 255 cycles = 1.275µs
    //==========================================================================
    
    generate
        for (genvar i = 0; i < NUM_OUTPUTS; i++) begin : gen_delay_line
            
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    delay_cnt[i]      <= 8'h0;
                    delay_active[i]   <= 1'b0;
                    delayed_trigger[i] <= 1'b0;
                end else begin
                    
                    //----------------------------------------------------------
                    // Detect rising edge of master trigger
                    //----------------------------------------------------------
                    if (master_trigger && !delay_active[i] && delay_cnt[i] == 0) begin
                        
                        if (trigger_delay[i] == 0) begin
                            // No delay configured - immediate output
                            delayed_trigger[i] <= 1'b1;
                        end else begin
                            // Start delay countdown
                            delay_active[i] <= 1'b1;
                            delay_cnt[i]    <= trigger_delay[i];
                        end
                        
                    end
                    
                    //----------------------------------------------------------
                    // Delay countdown
                    //----------------------------------------------------------
                    if (delay_active[i]) begin
                        if (delay_cnt[i] > 0) begin
                            delay_cnt[i] <= delay_cnt[i] - 1'b1;
                        end else begin
                            // Delay complete - assert output
                            delay_active[i]    <= 1'b0;
                            delayed_trigger[i] <= 1'b1;
                        end
                    end
                    
                    //----------------------------------------------------------
                    // Track master trigger (maintain pulse width)
                    //----------------------------------------------------------
                    if (delayed_trigger[i] && !master_trigger) begin
                        delayed_trigger[i] <= 1'b0;     // Follow master down
                    end
                    
                end
            end
            
        end
    endgenerate
    
    //==========================================================================
    // Output Assignment
    //==========================================================================
    // Map delayed triggers to physical outputs:
    //   - cam_trigger[0:3]: Camera FSIN signals
    //   - imu_trigger: IMU interrupt
    //==========================================================================
    
    assign cam_trigger = delayed_trigger[3:0];
    assign imu_trigger = delayed_trigger[4];
    
    //==========================================================================
    // Simulation/Debug Support
    //==========================================================================
    
    `ifdef SIMULATION
    
    // Jitter measurement for validation
    logic [31:0] last_trigger_time;
    logic [31:0] current_time;
    logic [31:0] measured_period;
    logic [31:0] jitter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_time      <= 32'h0;
            last_trigger_time <= 32'h0;
            measured_period   <= 32'h0;
            jitter            <= 32'h0;
        end else begin
            current_time <= current_time + 1'b1;
            
            // Measure period on rising edge of master trigger
            if (master_trigger && !delayed_trigger[0]) begin
                measured_period   <= current_time - last_trigger_time;
                last_trigger_time <= current_time;
                
                // Calculate jitter (deviation from expected period)
                if (measured_period > frame_period)
                    jitter <= measured_period - frame_period;
                else
                    jitter <= frame_period - measured_period;
            end
        end
    end
    
    // Synthesis-ignore assertions
    // synopsys translate_off
    
    // Check that trigger skew is within spec
    property trigger_skew_check;
        @(posedge clk) disable iff (!rst_n)
        $rose(master_trigger) |-> ##[0:20] (&delayed_trigger);
    endproperty
    
    assert property (trigger_skew_check)
        else $warning("Trigger skew exceeds 100ns!");
    
    // synopsys translate_on
    
    `endif

endmodule

`default_nettype wire
