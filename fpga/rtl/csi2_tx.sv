//------------------------------------------------------------------------------
// CSI-2 Transmitter Controller
// Generates MIPI CSI-2 packets with Virtual Channel tagging
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module csi2_tx #(
    parameter NUM_LANES = 2
)(
    // Clocks and reset
    input  logic        clk_byte,           // Byte clock for D-PHY
    input  logic        clk_sys,            // System clock
    input  logic        rst_n,
    
    // Packet Input Interface
    input  logic        pkt_valid,
    output logic        pkt_ready,
    input  logic [31:0] pkt_data,
    input  logic [5:0]  pkt_type,           // Data Type
    input  logic [1:0]  pkt_vc,             // Virtual Channel
    input  logic [15:0] pkt_wc,             // Word Count
    input  logic        frame_start,
    input  logic        frame_end,
    input  logic        line_start,
    input  logic        line_end,
    
    // D-PHY Interface (to CrossLink-NX hard D-PHY)
    output logic        dphy_clk_p,
    output logic        dphy_clk_n,
    output logic [NUM_LANES-1:0] dphy_data_p,
    output logic [NUM_LANES-1:0] dphy_data_n
);

    //--------------------------------------------------------------------------
    // CSI-2 Data Types
    //--------------------------------------------------------------------------
    
    localparam DT_FRAME_START  = 6'h00;
    localparam DT_FRAME_END    = 6'h01;
    localparam DT_LINE_START   = 6'h02;
    localparam DT_LINE_END     = 6'h03;
    
    //--------------------------------------------------------------------------
    // State Machine
    //--------------------------------------------------------------------------
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_LP,              // Low Power state
        ST_HS_REQUEST,      // Request HS mode
        ST_HS_PREPARE,      // HS preparation
        ST_SYNC,            // Send sync byte
        ST_HEADER,          // Send packet header
        ST_PAYLOAD,         // Send payload data
        ST_CRC,             // Send CRC
        ST_HS_TRAIL,        // HS trail
        ST_HS_EXIT          // Exit HS mode
    } state_t;
    
    state_t state, next_state;
    
    //--------------------------------------------------------------------------
    // Internal Signals
    //--------------------------------------------------------------------------
    
    // Packet construction
    logic [31:0] packet_header;
    logic [7:0]  ecc;
    logic [15:0] crc_reg;
    logic [15:0] crc_out;
    
    // Lane data
    logic [7:0] lane_data [NUM_LANES-1:0];
    logic       lane_valid;
    
    // Counters
    logic [15:0] byte_count;
    logic [15:0] word_count_reg;
    logic [5:0]  dt_reg;
    logic [1:0]  vc_reg;
    
    // Timing counters
    logic [7:0]  timing_cnt;
    localparam   LP_TIME  = 8'd10;
    localparam   HS_PREP  = 8'd5;
    localparam   HS_TRAIL = 8'd8;
    
    // Short packet detection
    logic is_short_packet;
    
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
                if (pkt_valid)
                    next_state = ST_LP;
            end
            
            ST_LP: begin
                if (timing_cnt >= LP_TIME)
                    next_state = ST_HS_REQUEST;
            end
            
            ST_HS_REQUEST: begin
                next_state = ST_HS_PREPARE;
            end
            
            ST_HS_PREPARE: begin
                if (timing_cnt >= HS_PREP)
                    next_state = ST_SYNC;
            end
            
            ST_SYNC: begin
                next_state = ST_HEADER;
            end
            
            ST_HEADER: begin
                if (byte_count >= 4) begin
                    if (is_short_packet)
                        next_state = ST_HS_TRAIL;
                    else
                        next_state = ST_PAYLOAD;
                end
            end
            
            ST_PAYLOAD: begin
                if (byte_count >= word_count_reg)
                    next_state = ST_CRC;
            end
            
            ST_CRC: begin
                if (byte_count >= 2)
                    next_state = ST_HS_TRAIL;
            end
            
            ST_HS_TRAIL: begin
                if (timing_cnt >= HS_TRAIL)
                    next_state = ST_HS_EXIT;
            end
            
            ST_HS_EXIT: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //--------------------------------------------------------------------------
    // Packet Header Construction
    //--------------------------------------------------------------------------
    
    // Short packet types (no payload)
    assign is_short_packet = (dt_reg == DT_FRAME_START) || 
                             (dt_reg == DT_FRAME_END) ||
                             (dt_reg == DT_LINE_START) || 
                             (dt_reg == DT_LINE_END);
    
    // Capture packet parameters
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            dt_reg         <= 6'h0;
            vc_reg         <= 2'b00;
            word_count_reg <= 16'h0;
        end else if (state == ST_IDLE && pkt_valid) begin
            dt_reg         <= pkt_type;
            vc_reg         <= pkt_vc;
            word_count_reg <= pkt_wc;
        end
    end
    
    // Build header: [VC:2][DT:6][WC:16][ECC:8]
    assign packet_header = {ecc, word_count_reg, vc_reg, dt_reg};
    
    //--------------------------------------------------------------------------
    // ECC Calculation
    //--------------------------------------------------------------------------
    
    function automatic logic [7:0] calc_ecc(input logic [23:0] data);
        logic [7:0] e;
        e[0] = data[0]  ^ data[1]  ^ data[2]  ^ data[4]  ^ data[5]  ^ 
               data[7]  ^ data[10] ^ data[11] ^ data[13] ^ data[16] ^ 
               data[20] ^ data[21] ^ data[22] ^ data[23];
        e[1] = data[0]  ^ data[1]  ^ data[3]  ^ data[4]  ^ data[6]  ^ 
               data[8]  ^ data[10] ^ data[12] ^ data[14] ^ data[17] ^ 
               data[20] ^ data[21] ^ data[22] ^ data[23];
        e[2] = data[0]  ^ data[2]  ^ data[3]  ^ data[5]  ^ data[6]  ^ 
               data[9]  ^ data[11] ^ data[12] ^ data[15] ^ data[18] ^ 
               data[20] ^ data[21] ^ data[22];
        e[3] = data[1]  ^ data[2]  ^ data[3]  ^ data[7]  ^ data[8]  ^ 
               data[9]  ^ data[13] ^ data[14] ^ data[15] ^ data[19] ^ 
               data[20] ^ data[21] ^ data[23];
        e[4] = data[4]  ^ data[5]  ^ data[6]  ^ data[7]  ^ data[8]  ^ 
               data[9]  ^ data[16] ^ data[17] ^ data[18] ^ data[19] ^ 
               data[20] ^ data[22] ^ data[23];
        e[5] = data[10] ^ data[11] ^ data[12] ^ data[13] ^ data[14] ^ 
               data[15] ^ data[16] ^ data[17] ^ data[18] ^ data[19] ^ 
               data[21] ^ data[22] ^ data[23];
        e[6] = 1'b0;
        e[7] = 1'b0;
        return e;
    endfunction
    
    assign ecc = calc_ecc({word_count_reg, vc_reg, dt_reg});
    
    //--------------------------------------------------------------------------
    // CRC-16 Calculation
    //--------------------------------------------------------------------------
    
    function automatic logic [15:0] crc16_ccitt(
        input logic [15:0] crc_in,
        input logic [7:0]  data
    );
        logic [15:0] c;
        logic [7:0] d;
        d = data;
        
        c[0]  = crc_in[8]  ^ crc_in[12] ^ d[0] ^ d[4];
        c[1]  = crc_in[9]  ^ crc_in[13] ^ d[1] ^ d[5];
        c[2]  = crc_in[10] ^ crc_in[14] ^ d[2] ^ d[6];
        c[3]  = crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        c[4]  = crc_in[12] ^ d[4];
        c[5]  = crc_in[8]  ^ crc_in[12] ^ crc_in[13] ^ d[0] ^ d[4] ^ d[5];
        c[6]  = crc_in[9]  ^ crc_in[13] ^ crc_in[14] ^ d[1] ^ d[5] ^ d[6];
        c[7]  = crc_in[10] ^ crc_in[14] ^ crc_in[15] ^ d[2] ^ d[6] ^ d[7];
        c[8]  = crc_in[0]  ^ crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        c[9]  = crc_in[1]  ^ crc_in[12] ^ d[4];
        c[10] = crc_in[2]  ^ crc_in[13] ^ d[5];
        c[11] = crc_in[3]  ^ crc_in[14] ^ d[6];
        c[12] = crc_in[4]  ^ crc_in[8]  ^ crc_in[12] ^ crc_in[15] ^ 
                d[0] ^ d[4] ^ d[7];
        c[13] = crc_in[5]  ^ crc_in[9]  ^ crc_in[13] ^ d[1] ^ d[5];
        c[14] = crc_in[6]  ^ crc_in[10] ^ crc_in[14] ^ d[2] ^ d[6];
        c[15] = crc_in[7]  ^ crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        
        return c;
    endfunction
    
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= 16'hFFFF;
        end else begin
            case (state)
                ST_HEADER: crc_reg <= 16'hFFFF;
                ST_PAYLOAD: begin
                    if (lane_valid) begin
                        for (int i = 0; i < NUM_LANES; i++)
                            crc_reg <= crc16_ccitt(crc_reg, lane_data[i]);
                    end
                end
                default: ;
            endcase
        end
    end
    
    assign crc_out = crc_reg;
    
    //--------------------------------------------------------------------------
    // Byte Counter and Timing
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            byte_count  <= 16'h0;
            timing_cnt  <= 8'h0;
        end else begin
            case (state)
                ST_IDLE: begin
                    byte_count <= 16'h0;
                    timing_cnt <= 8'h0;
                end
                
                ST_LP, ST_HS_PREPARE, ST_HS_TRAIL: begin
                    timing_cnt <= timing_cnt + 1'b1;
                end
                
                ST_HEADER, ST_PAYLOAD, ST_CRC: begin
                    byte_count <= byte_count + NUM_LANES;
                end
                
                ST_SYNC: begin
                    byte_count <= 16'h0;
                    timing_cnt <= 8'h0;
                end
                
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // Lane Data Generation
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            lane_data[0] <= 8'h0;
            lane_data[1] <= 8'h0;
            lane_valid   <= 1'b0;
        end else begin
            lane_valid <= 1'b0;
            
            case (state)
                ST_SYNC: begin
                    // Sync byte on all lanes
                    lane_data[0] <= 8'hB8;
                    lane_data[1] <= 8'hB8;
                    lane_valid   <= 1'b1;
                end
                
                ST_HEADER: begin
                    // Send header bytes
                    case (byte_count[2:0])
                        3'd0: begin
                            lane_data[0] <= packet_header[7:0];
                            lane_data[1] <= packet_header[15:8];
                        end
                        3'd2: begin
                            lane_data[0] <= packet_header[23:16];
                            lane_data[1] <= packet_header[31:24];
                        end
                        default: ;
                    endcase
                    lane_valid <= 1'b1;
                end
                
                ST_PAYLOAD: begin
                    // Send payload bytes
                    lane_data[0] <= pkt_data[7:0];
                    lane_data[1] <= pkt_data[15:8];
                    lane_valid   <= 1'b1;
                end
                
                ST_CRC: begin
                    // Send CRC bytes
                    if (byte_count == 0) begin
                        lane_data[0] <= crc_out[7:0];
                        lane_data[1] <= crc_out[15:8];
                        lane_valid   <= 1'b1;
                    end
                end
                
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // Ready Signal
    //--------------------------------------------------------------------------
    
    assign pkt_ready = (state == ST_IDLE) || 
                       (state == ST_PAYLOAD && byte_count < word_count_reg);
    
    //--------------------------------------------------------------------------
    // D-PHY TX Wrapper
    //--------------------------------------------------------------------------
    
    dphy_tx_wrapper #(
        .NUM_LANES(NUM_LANES)
    ) u_dphy_tx (
        .clk_byte       (clk_byte),
        .rst_n          (rst_n),
        .lane_data      (lane_data),
        .lane_valid     (lane_valid),
        .hs_mode        (state != ST_IDLE && state != ST_LP),
        .dphy_clk_p     (dphy_clk_p),
        .dphy_clk_n     (dphy_clk_n),
        .dphy_data_p    (dphy_data_p),
        .dphy_data_n    (dphy_data_n)
    );

endmodule

//------------------------------------------------------------------------------
// D-PHY TX Wrapper (placeholder for CrossLink-NX hard IP)
//------------------------------------------------------------------------------

module dphy_tx_wrapper #(
    parameter NUM_LANES = 2
)(
    input  logic        clk_byte,
    input  logic        rst_n,
    input  logic [7:0]  lane_data [NUM_LANES-1:0],
    input  logic        lane_valid,
    input  logic        hs_mode,
    output logic        dphy_clk_p,
    output logic        dphy_clk_n,
    output logic [NUM_LANES-1:0] dphy_data_p,
    output logic [NUM_LANES-1:0] dphy_data_n
);

    // This module wraps the Lattice CrossLink-NX DPHY_TX hard IP
    // Actual implementation instantiates the IP generated by Radiant
    
    // Placeholder - differential output generation
    always_ff @(posedge clk_byte or negedge rst_n) begin
        if (!rst_n) begin
            dphy_clk_p  <= 1'b0;
            dphy_clk_n  <= 1'b1;
            dphy_data_p <= '0;
            dphy_data_n <= '1;
        end else if (hs_mode) begin
            dphy_clk_p  <= ~dphy_clk_p;
            dphy_clk_n  <= dphy_clk_p;
            // Actual serialization happens in hard IP
        end
    end

endmodule
