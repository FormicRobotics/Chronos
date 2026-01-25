//==============================================================================
// Frame Buffer - SRAM-based Line Buffer with Flow Control
//==============================================================================
//
// Copyright (C) 2025 Chronos Project
// SPDX-License-Identifier: Proprietary
//
// Description:
//   Implements a FIFO-based line buffer between CSI-2 RX and TX stages.
//   Each camera has its own buffer instance to absorb timing variations
//   and enable round-robin arbitration by the TX arbiter.
//
// Architecture:
//   - Dual-port SRAM organized as circular FIFO
//   - Gray-coded pointers for clock domain crossing (if needed)
//   - Metadata stored alongside data for packet reconstruction
//   - Overflow detection with sticky error flag
//
// Buffer Organization:
//   +------------------+------------------+
//   |   Data (32-bit)  | Metadata (26-bit)|
//   +------------------+------------------+
//   |    Pixel data    | DT, WC, Flags    |
//   +------------------+------------------+
//
// Capacity:
//   - DEPTH entries × 32-bit data = line buffer capacity
//   - 4096 entries = enough for ~5 lines at 1280 pixels
//   - Sufficient for multi-line buffering during arbitration
//
// Flow Control:
//   - Write side: drops data when full (overflow flagged)
//   - Read side: waits when empty (controlled by arbiter)
//   - No backpressure to RX (would cause MIPI errors)
//
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module frame_buffer #(
    //--------------------------------------------------------------------------
    // Parameters
    //--------------------------------------------------------------------------
    parameter int DATA_WIDTH = 32,              // Pixel data width
    parameter int DEPTH      = 4096,            // FIFO depth (entries)
    parameter int CAMERA_ID  = 0                // Camera identifier (debug)
)(
    //--------------------------------------------------------------------------
    // Clock and Reset
    //--------------------------------------------------------------------------
    input  wire                     clk,        // System clock
    input  wire                     rst_n,      // Active-low reset
    
    //--------------------------------------------------------------------------
    // Write Interface (from CSI-2 RX)
    // Data is written every clock cycle when wr_en is high
    //--------------------------------------------------------------------------
    input  wire                     wr_en,          // Write enable
    input  wire  [DATA_WIDTH-1:0]   wr_data,        // Pixel data
    input  wire                     wr_frame_start, // Frame start flag
    input  wire                     wr_frame_end,   // Frame end flag
    input  wire                     wr_line_start,  // Line start flag
    input  wire                     wr_line_end,    // Line end flag
    input  wire  [5:0]              wr_data_type,   // CSI-2 data type
    input  wire  [15:0]             wr_word_count,  // Packet word count
    
    //--------------------------------------------------------------------------
    // Read Interface (to TX Arbiter)
    // Data appears one cycle after rd_en is asserted
    //--------------------------------------------------------------------------
    input  wire                     rd_en,          // Read enable
    output logic [DATA_WIDTH-1:0]   rd_data,        // Pixel data
    output logic                    rd_frame_start, // Frame start flag
    output logic                    rd_frame_end,   // Frame end flag
    output logic                    rd_line_start,  // Line start flag
    output logic                    rd_line_end,    // Line end flag
    output logic [5:0]              rd_data_type,   // CSI-2 data type
    output logic [15:0]             rd_word_count,  // Packet word count
    output logic                    rd_valid,       // Read data valid
    
    //--------------------------------------------------------------------------
    // Status Outputs
    //--------------------------------------------------------------------------
    output logic                    empty,          // Buffer is empty
    output logic                    full,           // Buffer is full
    output logic                    overflow,       // Overflow occurred (sticky)
    output logic [$clog2(DEPTH):0]  level           // Current fill level
);

    //==========================================================================
    // Local Parameters
    //==========================================================================
    
    localparam int ADDR_WIDTH = $clog2(DEPTH);
    
    // Metadata bit allocation:
    // [25:20] = data_type (6 bits)
    // [19:4]  = word_count (16 bits)
    // [3]     = frame_start
    // [2]     = frame_end
    // [1]     = line_start
    // [0]     = line_end
    localparam int META_WIDTH = 6 + 16 + 4;     // 26 bits total
    
    //==========================================================================
    // FIFO Pointer Registers
    //==========================================================================
    
    // Pointers are one bit wider than address to detect wrap-around
    logic [ADDR_WIDTH:0] wr_ptr, rd_ptr;
    logic [ADDR_WIDTH:0] wr_ptr_next, rd_ptr_next;
    
    //==========================================================================
    // Memory Arrays
    //==========================================================================
    // These will infer to SRAM blocks in the CrossLink-NX
    // Using separate arrays for data and metadata
    
    (* ram_style = "block" *)
    logic [DATA_WIDTH-1:0] data_mem [DEPTH-1:0];
    
    (* ram_style = "block" *)
    logic [META_WIDTH-1:0] meta_mem [DEPTH-1:0];
    
    //==========================================================================
    // Metadata Packing/Unpacking
    //==========================================================================
    
    logic [META_WIDTH-1:0] wr_meta, rd_meta;
    
    // Pack write metadata into single vector
    assign wr_meta = {
        wr_data_type,       // [25:20]
        wr_word_count,      // [19:4]
        wr_frame_start,     // [3]
        wr_frame_end,       // [2]
        wr_line_start,      // [1]
        wr_line_end         // [0]
    };
    
    //==========================================================================
    // FIFO Status Logic
    //==========================================================================
    
    // Next pointer values (for combinational status)
    assign wr_ptr_next = wr_ptr + 1'b1;
    assign rd_ptr_next = rd_ptr + 1'b1;
    
    // Empty: pointers are equal
    assign empty = (wr_ptr == rd_ptr);
    
    // Full: pointers differ only in MSB (wrapped)
    // This means write pointer has caught up to read pointer
    assign full = (wr_ptr[ADDR_WIDTH] != rd_ptr[ADDR_WIDTH]) && 
                  (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);
    
    // Fill level: difference between pointers
    assign level = wr_ptr - rd_ptr;
    
    //==========================================================================
    // Write Logic
    //==========================================================================
    // Write when enabled and not full
    // If full, data is dropped and overflow flag is set
    
    logic overflow_detect;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr         <= '0;
            overflow_detect <= 1'b0;
        end else begin
            overflow_detect <= 1'b0;
            
            if (wr_en) begin
                if (!full) begin
                    // Normal write
                    data_mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
                    meta_mem[wr_ptr[ADDR_WIDTH-1:0]] <= wr_meta;
                    wr_ptr <= wr_ptr_next;
                end else begin
                    // Buffer full - drop data and flag overflow
                    overflow_detect <= 1'b1;
                    // synthesis translate_off
                    $warning("[Frame Buffer %0d] Overflow! Data dropped.", CAMERA_ID);
                    // synthesis translate_on
                end
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Sticky Overflow Flag
    // Set on overflow, cleared only by reset
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            overflow <= 1'b0;
        else if (overflow_detect)
            overflow <= 1'b1;
        // To clear: assert reset or add clear input
    end
    
    //==========================================================================
    // Read Logic
    //==========================================================================
    // Read when enabled and not empty
    // Data appears on next clock cycle (registered output)
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr   <= '0;
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            
            if (rd_en && !empty) begin
                rd_ptr   <= rd_ptr_next;
                rd_valid <= 1'b1;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Registered Read Data Output
    // One cycle latency from rd_en to data valid
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_data <= '0;
            rd_meta <= '0;
        end else if (rd_en && !empty) begin
            rd_data <= data_mem[rd_ptr[ADDR_WIDTH-1:0]];
            rd_meta <= meta_mem[rd_ptr[ADDR_WIDTH-1:0]];
        end
    end
    
    //--------------------------------------------------------------------------
    // Unpack Read Metadata
    //--------------------------------------------------------------------------
    assign rd_data_type   = rd_meta[25:20];
    assign rd_word_count  = rd_meta[19:4];
    assign rd_frame_start = rd_meta[3];
    assign rd_frame_end   = rd_meta[2];
    assign rd_line_start  = rd_meta[1];
    assign rd_line_end    = rd_meta[0];
    
    //==========================================================================
    // Debug/Simulation Support
    //==========================================================================
    
    `ifdef SIMULATION
    
    // Track statistics
    integer total_writes = 0;
    integer total_reads = 0;
    integer total_overflows = 0;
    
    always_ff @(posedge clk) begin
        if (wr_en && !full)
            total_writes++;
        if (rd_en && !empty)
            total_reads++;
        if (overflow_detect)
            total_overflows++;
    end
    
    // Periodic status report
    initial begin
        forever begin
            #1000000;  // Every 1ms at 1ns timescale
            $display("[Frame Buffer %0d] Level=%0d Writes=%0d Reads=%0d Overflows=%0d",
                     CAMERA_ID, level, total_writes, total_reads, total_overflows);
        end
    end
    
    `endif

endmodule

`default_nettype wire
