//------------------------------------------------------------------------------
// TX Arbiter - Multiplexes 4 camera streams with Virtual Channel tagging
// Round-robin arbitration with frame-level priority
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module tx_arbiter #(
    parameter NUM_CAMERAS = 4,
    parameter DATA_WIDTH  = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // Buffer interfaces (from frame_buffers)
    input  logic [NUM_CAMERAS-1:0]  buf_empty,
    input  logic [NUM_CAMERAS-1:0][DATA_WIDTH-1:0] buf_data,
    input  logic [NUM_CAMERAS-1:0]  buf_frame_start,
    input  logic [NUM_CAMERAS-1:0]  buf_frame_end,
    input  logic [NUM_CAMERAS-1:0]  buf_line_start,
    input  logic [NUM_CAMERAS-1:0]  buf_line_end,
    input  logic [NUM_CAMERAS-1:0][5:0]  buf_data_type,
    input  logic [NUM_CAMERAS-1:0][15:0] buf_word_count,
    input  logic [NUM_CAMERAS-1:0]  buf_valid,
    output logic [NUM_CAMERAS-1:0]  buf_rd_en,
    
    // TX interface (to CSI-2 TX)
    input  logic                    tx_ready,
    output logic                    tx_valid,
    output logic [DATA_WIDTH-1:0]   tx_data,
    output logic [1:0]              tx_vc,          // Virtual Channel (0-3)
    output logic [5:0]              tx_dt,          // Data Type
    output logic [15:0]             tx_wc,          // Word Count
    output logic                    tx_fs,          // Frame Start
    output logic                    tx_fe,          // Frame End
    output logic                    tx_ls,          // Line Start
    output logic                    tx_le           // Line End
);

    //--------------------------------------------------------------------------
    // State Machine
    //--------------------------------------------------------------------------
    
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_SELECT,
        ST_SEND_HEADER,
        ST_SEND_PAYLOAD,
        ST_SEND_FOOTER,
        ST_NEXT
    } state_t;
    
    state_t state, next_state;
    
    //--------------------------------------------------------------------------
    // Internal Signals
    //--------------------------------------------------------------------------
    
    // Arbiter state
    logic [1:0] current_cam;            // Currently selected camera (0-3)
    logic [1:0] next_cam;
    logic [NUM_CAMERAS-1:0] cam_ready;  // Cameras with data available
    logic [NUM_CAMERAS-1:0] cam_grant;
    
    // Packet tracking
    logic in_frame [NUM_CAMERAS-1:0];   // Currently receiving frame
    logic [15:0] remaining_bytes;
    
    // Selected camera data (registered)
    logic [DATA_WIDTH-1:0] sel_data;
    logic [5:0]  sel_dt;
    logic [15:0] sel_wc;
    logic        sel_fs, sel_fe, sel_ls, sel_le;
    logic        sel_valid;
    
    //--------------------------------------------------------------------------
    // Camera Ready Detection
    //--------------------------------------------------------------------------
    
    assign cam_ready = ~buf_empty;
    
    //--------------------------------------------------------------------------
    // Round-Robin Arbitration
    //--------------------------------------------------------------------------
    
    // Priority encoder with round-robin wrap
    always_comb begin
        next_cam = current_cam;
        
        // Check cameras starting from next one after current
        for (int i = 0; i < NUM_CAMERAS; i++) begin
            logic [1:0] check_cam;
            check_cam = (current_cam + 1 + i) % NUM_CAMERAS;
            if (cam_ready[check_cam]) begin
                next_cam = check_cam;
                break;
            end
        end
    end
    
    // Grant generation
    always_comb begin
        cam_grant = '0;
        cam_grant[current_cam] = 1'b1;
    end
    
    //--------------------------------------------------------------------------
    // State Machine
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= ST_IDLE;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        
        case (state)
            ST_IDLE: begin
                if (|cam_ready)
                    next_state = ST_SELECT;
            end
            
            ST_SELECT: begin
                next_state = ST_SEND_HEADER;
            end
            
            ST_SEND_HEADER: begin
                if (tx_ready)
                    next_state = (sel_wc > 0) ? ST_SEND_PAYLOAD : ST_NEXT;
            end
            
            ST_SEND_PAYLOAD: begin
                if (tx_ready && remaining_bytes == 0)
                    next_state = ST_SEND_FOOTER;
            end
            
            ST_SEND_FOOTER: begin
                if (tx_ready)
                    next_state = ST_NEXT;
            end
            
            ST_NEXT: begin
                if (|cam_ready)
                    next_state = ST_SELECT;
                else
                    next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
    end
    
    //--------------------------------------------------------------------------
    // Camera Selection and Data Path
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_cam <= 2'b00;
        end else if (state == ST_SELECT) begin
            current_cam <= next_cam;
        end
    end
    
    // Buffer read enable generation
    always_comb begin
        buf_rd_en = '0;
        if (state == ST_SEND_PAYLOAD && tx_ready && !buf_empty[current_cam])
            buf_rd_en[current_cam] = 1'b1;
    end
    
    // Capture selected camera metadata
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sel_data <= '0;
            sel_dt   <= '0;
            sel_wc   <= '0;
            sel_fs   <= 1'b0;
            sel_fe   <= 1'b0;
            sel_ls   <= 1'b0;
            sel_le   <= 1'b0;
        end else if (state == ST_SELECT) begin
            sel_dt <= buf_data_type[next_cam];
            sel_wc <= buf_word_count[next_cam];
            sel_fs <= buf_frame_start[next_cam];
            sel_fe <= buf_frame_end[next_cam];
            sel_ls <= buf_line_start[next_cam];
            sel_le <= buf_line_end[next_cam];
        end else if (state == ST_SEND_PAYLOAD && tx_ready) begin
            sel_data <= buf_data[current_cam];
        end
    end
    
    // Byte counter for payload
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            remaining_bytes <= 16'h0;
        end else begin
            case (state)
                ST_SELECT: remaining_bytes <= buf_word_count[next_cam];
                ST_SEND_PAYLOAD: begin
                    if (tx_ready && remaining_bytes > 4)
                        remaining_bytes <= remaining_bytes - 4;
                    else if (tx_ready)
                        remaining_bytes <= 16'h0;
                end
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // TX Output Generation
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_valid <= 1'b0;
            tx_data  <= '0;
            tx_vc    <= 2'b00;
            tx_dt    <= 6'h00;
            tx_wc    <= 16'h0;
            tx_fs    <= 1'b0;
            tx_fe    <= 1'b0;
            tx_ls    <= 1'b0;
            tx_le    <= 1'b0;
        end else begin
            // Default: deassert
            tx_valid <= 1'b0;
            tx_fs    <= 1'b0;
            tx_fe    <= 1'b0;
            tx_ls    <= 1'b0;
            tx_le    <= 1'b0;
            
            case (state)
                ST_SEND_HEADER: begin
                    if (tx_ready) begin
                        tx_valid <= 1'b1;
                        tx_vc    <= current_cam;  // Camera ID = Virtual Channel
                        tx_dt    <= sel_dt;
                        tx_wc    <= sel_wc;
                        tx_fs    <= sel_fs;
                        tx_fe    <= sel_fe;
                        tx_ls    <= sel_ls;
                        tx_le    <= sel_le;
                    end
                end
                
                ST_SEND_PAYLOAD: begin
                    if (tx_ready && buf_valid[current_cam]) begin
                        tx_valid <= 1'b1;
                        tx_data  <= buf_data[current_cam];
                        tx_vc    <= current_cam;
                    end
                end
                
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // Frame Tracking (for debugging/status)
    //--------------------------------------------------------------------------
    
    generate
        for (genvar i = 0; i < NUM_CAMERAS; i++) begin : gen_frame_track
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n)
                    in_frame[i] <= 1'b0;
                else begin
                    if (buf_frame_start[i] && !buf_empty[i])
                        in_frame[i] <= 1'b1;
                    else if (buf_frame_end[i] && !buf_empty[i])
                        in_frame[i] <= 1'b0;
                end
            end
        end
    endgenerate

endmodule
