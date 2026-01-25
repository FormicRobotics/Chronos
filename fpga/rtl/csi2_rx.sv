//------------------------------------------------------------------------------
// CSI-2 Receiver Controller
// Implements MIPI CSI-2 packet parsing with D-PHY interface
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module csi2_rx #(
    parameter NUM_LANES  = 2,
    parameter CAMERA_ID  = 0
)(
    // Clocks and reset
    input  logic        clk_byte,           // Byte clock from D-PHY
    input  logic        clk_sys,            // System clock
    input  logic        rst_n,
    
    // D-PHY Interface (directly connected to CrossLink-NX hard D-PHY)
    input  logic        dphy_clk_p,
    input  logic        dphy_clk_n,
    input  logic [NUM_LANES-1:0] dphy_data_p,
    input  logic [NUM_LANES-1:0] dphy_data_n,
    
    // Packet Output Interface
    output logic        pkt_valid,
    output logic [31:0] pkt_data,
    output logic [5:0]  pkt_type,           // Data Type
    output logic [15:0] pkt_wc,             // Word Count
    output logic        frame_start,
    output logic        frame_end,
    output logic        line_start,
    output logic        line_end,
    output logic        error               // CRC/ECC error flag
);

    //--------------------------------------------------------------------------
    // CSI-2 Data Types (subset for OV9281)
    //--------------------------------------------------------------------------
    
    localparam DT_FRAME_START  = 6'h00;
    localparam DT_FRAME_END    = 6'h01;
    localparam DT_LINE_START   = 6'h02;
    localparam DT_LINE_END     = 6'h03;
    localparam DT_RAW8         = 6'h2A;
    localparam DT_RAW10        = 6'h2B;
    localparam DT_RAW12        = 6'h2C;
    
    //--------------------------------------------------------------------------
    // State Machine States
    //--------------------------------------------------------------------------
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_SYNC,
        ST_HEADER,
        ST_PAYLOAD,
        ST_CRC,
        ST_ERROR
    } state_t;
    
    state_t state, next_state;
    
    //--------------------------------------------------------------------------
    // Internal Signals
    //--------------------------------------------------------------------------
    
    // D-PHY deserialized data (from hard IP)
    logic [7:0] lane_data [NUM_LANES-1:0];
    logic       lane_valid;
    logic       sync_detected;
    
    // Packet parsing
    logic [31:0] packet_header;
    logic [1:0]  pkt_vc;                    // Virtual Channel (from header)
    logic [5:0]  pkt_dt;                    // Data Type
    logic [15:0] word_count;
    logic [7:0]  ecc_received;
    logic [7:0]  ecc_calculated;
    logic        ecc_valid;
    
    // Payload handling
    logic [15:0] byte_count;
    logic [31:0] payload_data;
    logic        payload_valid;
    
    // CRC
    logic [15:0] crc_received;
    logic [15:0] crc_calculated;
    logic        crc_valid;
    
    // Synchronization
    logic [7:0]  sync_pattern [NUM_LANES-1:0];
    localparam   SYNC_BYTE = 8'hB8;         // CSI-2 sync byte
    
    //--------------------------------------------------------------------------
    // D-PHY Interface (simplified - actual implementation uses hard IP)
    // In CrossLink-NX, this would connect to the DPHY_RX hard IP block
    //--------------------------------------------------------------------------
    
    // This is a behavioral model - actual implementation uses Lattice DPHY IP
    dphy_rx_wrapper #(
        .NUM_LANES(NUM_LANES)
    ) u_dphy_rx (
        .clk_byte       (clk_byte),
        .rst_n          (rst_n),
        .dphy_clk_p     (dphy_clk_p),
        .dphy_clk_n     (dphy_clk_n),
        .dphy_data_p    (dphy_data_p),
        .dphy_data_n    (dphy_data_n),
        .lane_data      (lane_data),
        .lane_valid     (lane_valid),
        .sync_detected  (sync_detected)
    );
    
    //--------------------------------------------------------------------------
    // State Machine
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n)
            state <= ST_IDLE;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (sync_detected)
                    next_state = ST_SYNC;
            end
            
            ST_SYNC: begin
                if (lane_valid)
                    next_state = ST_HEADER;
            end
            
            ST_HEADER: begin
                if (lane_valid) begin
                    if (ecc_valid)
                        next_state = (word_count > 0) ? ST_PAYLOAD : ST_IDLE;
                    else
                        next_state = ST_ERROR;
                end
            end
            
            ST_PAYLOAD: begin
                if (byte_count >= word_count)
                    next_state = ST_CRC;
            end
            
            ST_CRC: begin
                if (crc_valid)
                    next_state = ST_IDLE;
                else
                    next_state = ST_ERROR;
            end
            
            ST_ERROR: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //--------------------------------------------------------------------------
    // Packet Header Parsing
    //--------------------------------------------------------------------------
    
    // Assemble 32-bit header from lane data
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            packet_header <= 32'h0;
        end else if (state == ST_HEADER && lane_valid) begin
            // CSI-2 header: [VC:2][DT:6][WC:16][ECC:8]
            packet_header <= {lane_data[1], lane_data[0], 
                              lane_data[1], lane_data[0]};
        end
    end
    
    // Extract fields from header
    assign pkt_vc        = packet_header[7:6];
    assign pkt_dt        = packet_header[5:0];
    assign word_count    = packet_header[23:8];
    assign ecc_received  = packet_header[31:24];
    
    //--------------------------------------------------------------------------
    // ECC Calculation (Hamming code for CSI-2)
    //--------------------------------------------------------------------------
    
    function automatic logic [7:0] calc_ecc(input logic [23:0] data);
        logic [7:0] ecc;
        // CSI-2 ECC polynomial calculation
        ecc[0] = data[0]  ^ data[1]  ^ data[2]  ^ data[4]  ^ data[5]  ^ 
                 data[7]  ^ data[10] ^ data[11] ^ data[13] ^ data[16] ^ 
                 data[20] ^ data[21] ^ data[22] ^ data[23];
        ecc[1] = data[0]  ^ data[1]  ^ data[3]  ^ data[4]  ^ data[6]  ^ 
                 data[8]  ^ data[10] ^ data[12] ^ data[14] ^ data[17] ^ 
                 data[20] ^ data[21] ^ data[22] ^ data[23];
        ecc[2] = data[0]  ^ data[2]  ^ data[3]  ^ data[5]  ^ data[6]  ^ 
                 data[9]  ^ data[11] ^ data[12] ^ data[15] ^ data[18] ^ 
                 data[20] ^ data[21] ^ data[22];
        ecc[3] = data[1]  ^ data[2]  ^ data[3]  ^ data[7]  ^ data[8]  ^ 
                 data[9]  ^ data[13] ^ data[14] ^ data[15] ^ data[19] ^ 
                 data[20] ^ data[21] ^ data[23];
        ecc[4] = data[4]  ^ data[5]  ^ data[6]  ^ data[7]  ^ data[8]  ^ 
                 data[9]  ^ data[16] ^ data[17] ^ data[18] ^ data[19] ^ 
                 data[20] ^ data[22] ^ data[23];
        ecc[5] = data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ 
                 data[15] ^ data[16] ^ data[17] ^ data[18] ^ data[19] ^ 
                 data[21] ^ data[22] ^ data[23];
        ecc[6] = 1'b0;
        ecc[7] = 1'b0;
        return ecc;
    endfunction
    
    assign ecc_calculated = calc_ecc(packet_header[23:0]);
    assign ecc_valid      = (ecc_received == ecc_calculated);
    
    //--------------------------------------------------------------------------
    // Payload Processing
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            byte_count    <= 16'h0;
            payload_data  <= 32'h0;
            payload_valid <= 1'b0;
        end else begin
            payload_valid <= 1'b0;
            
            if (state == ST_HEADER) begin
                byte_count <= 16'h0;
            end else if (state == ST_PAYLOAD && lane_valid) begin
                // Accumulate bytes into 32-bit word
                byte_count <= byte_count + NUM_LANES;
                payload_data <= {lane_data[1], lane_data[0], 
                                 payload_data[31:16]};
                
                if (byte_count[1:0] == 2'b10)
                    payload_valid <= 1'b1;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // CRC-16 Calculation
    //--------------------------------------------------------------------------
    
    logic [15:0] crc_reg;
    
    function automatic logic [15:0] crc16_ccitt(
        input logic [15:0] crc_in,
        input logic [7:0]  data
    );
        logic [15:0] crc_out;
        logic [7:0] d;
        d = data;
        
        crc_out[0]  = crc_in[8]  ^ crc_in[12] ^ d[0] ^ d[4];
        crc_out[1]  = crc_in[9]  ^ crc_in[13] ^ d[1] ^ d[5];
        crc_out[2]  = crc_in[10] ^ crc_in[14] ^ d[2] ^ d[6];
        crc_out[3]  = crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        crc_out[4]  = crc_in[12] ^ d[4];
        crc_out[5]  = crc_in[8]  ^ crc_in[12] ^ crc_in[13] ^ d[0] ^ d[4] ^ d[5];
        crc_out[6]  = crc_in[9]  ^ crc_in[13] ^ crc_in[14] ^ d[1] ^ d[5] ^ d[6];
        crc_out[7]  = crc_in[10] ^ crc_in[14] ^ crc_in[15] ^ d[2] ^ d[6] ^ d[7];
        crc_out[8]  = crc_in[0]  ^ crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        crc_out[9]  = crc_in[1]  ^ crc_in[12] ^ d[4];
        crc_out[10] = crc_in[2]  ^ crc_in[13] ^ d[5];
        crc_out[11] = crc_in[3]  ^ crc_in[14] ^ d[6];
        crc_out[12] = crc_in[4]  ^ crc_in[8]  ^ crc_in[12] ^ crc_in[15] ^ 
                      d[0] ^ d[4] ^ d[7];
        crc_out[13] = crc_in[5]  ^ crc_in[9]  ^ crc_in[13] ^ d[1] ^ d[5];
        crc_out[14] = crc_in[6]  ^ crc_in[10] ^ crc_in[14] ^ d[2] ^ d[6];
        crc_out[15] = crc_in[7]  ^ crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        
        return crc_out;
    endfunction
    
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= 16'hFFFF;
        end else begin
            if (state == ST_HEADER)
                crc_reg <= 16'hFFFF;
            else if (state == ST_PAYLOAD && lane_valid) begin
                for (int i = 0; i < NUM_LANES; i++)
                    crc_reg <= crc16_ccitt(crc_reg, lane_data[i]);
            end
        end
    end
    
    assign crc_calculated = crc_reg;
    assign crc_valid = (state == ST_CRC) && (crc_received == crc_calculated);
    
    //--------------------------------------------------------------------------
    // Output Generation
    //--------------------------------------------------------------------------
    
    // Clock domain crossing from clk_byte to clk_sys
    logic        pkt_valid_sync;
    logic [31:0] pkt_data_sync;
    logic        fs_sync, fe_sync, ls_sync, le_sync;
    logic        error_sync;
    
    // Detect short packet types
    logic is_frame_start, is_frame_end, is_line_start, is_line_end;
    
    assign is_frame_start = (pkt_dt == DT_FRAME_START);
    assign is_frame_end   = (pkt_dt == DT_FRAME_END);
    assign is_line_start  = (pkt_dt == DT_LINE_START);
    assign is_line_end    = (pkt_dt == DT_LINE_END);
    
    always_ff @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            pkt_valid   <= 1'b0;
            pkt_data    <= 32'h0;
            pkt_type    <= 6'h0;
            pkt_wc      <= 16'h0;
            frame_start <= 1'b0;
            frame_end   <= 1'b0;
            line_start  <= 1'b0;
            line_end    <= 1'b0;
            error       <= 1'b0;
        end else begin
            // Default: deassert pulses
            frame_start <= 1'b0;
            frame_end   <= 1'b0;
            line_start  <= 1'b0;
            line_end    <= 1'b0;
            error       <= 1'b0;
            
            // Payload data output
            pkt_valid   <= payload_valid;
            pkt_data    <= payload_data;
            pkt_type    <= pkt_dt;
            pkt_wc      <= word_count;
            
            // Short packet indicators
            if (state == ST_HEADER && lane_valid && ecc_valid) begin
                frame_start <= is_frame_start;
                frame_end   <= is_frame_end;
                line_start  <= is_line_start;
                line_end    <= is_line_end;
            end
            
            // Error indication
            if (state == ST_ERROR)
                error <= 1'b1;
        end
    end

endmodule

//------------------------------------------------------------------------------
// D-PHY RX Wrapper (placeholder for CrossLink-NX hard IP)
//------------------------------------------------------------------------------

module dphy_rx_wrapper #(
    parameter NUM_LANES = 2
)(
    input  logic        clk_byte,
    input  logic        rst_n,
    input  logic        dphy_clk_p,
    input  logic        dphy_clk_n,
    input  logic [NUM_LANES-1:0] dphy_data_p,
    input  logic [NUM_LANES-1:0] dphy_data_n,
    output logic [7:0]  lane_data [NUM_LANES-1:0],
    output logic        lane_valid,
    output logic        sync_detected
);

    // This module wraps the Lattice CrossLink-NX DPHY_RX hard IP
    // Actual implementation instantiates the IP generated by Radiant
    
    // Placeholder behavioral model
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            lane_valid    <= 1'b0;
            sync_detected <= 1'b0;
            for (int i = 0; i < NUM_LANES; i++)
                lane_data[i] <= 8'h0;
        end else begin
            // Actual implementation connects to hard D-PHY IP
            // This is just a placeholder
        end
    end

endmodule
