//------------------------------------------------------------------------------
// I2C Slave Controller
// Provides configuration interface for host (Jetson)
//------------------------------------------------------------------------------

`timescale 1ns / 1ps

module i2c_slave #(
    parameter SLAVE_ADDR = 7'h3C
)(
    input  logic        clk,
    input  logic        rst_n,
    
    // I2C interface
    input  logic        scl,
    inout  wire         sda,
    
    // Register interface
    output logic [7:0]  reg_addr,
    output logic [7:0]  reg_wdata,
    input  logic [7:0]  reg_rdata,
    output logic        reg_wr_en,
    output logic        reg_rd_en
);

    //--------------------------------------------------------------------------
    // State Machine
    //--------------------------------------------------------------------------
    
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_START,
        ST_ADDR,
        ST_ADDR_ACK,
        ST_REG_ADDR,
        ST_REG_ACK,
        ST_WRITE_DATA,
        ST_WRITE_ACK,
        ST_READ_DATA,
        ST_READ_ACK,
        ST_STOP
    } state_t;
    
    state_t state, next_state;
    
    //--------------------------------------------------------------------------
    // Internal Signals
    //--------------------------------------------------------------------------
    
    // Synchronizers for SCL and SDA
    logic scl_sync, scl_prev;
    logic sda_sync, sda_prev;
    logic scl_rise, scl_fall;
    logic start_cond, stop_cond;
    
    // Bit counter
    logic [3:0] bit_cnt;
    
    // Shift registers
    logic [7:0] shift_reg;
    logic [6:0] addr_received;
    logic       rw_bit;
    
    // SDA control
    logic sda_out;
    logic sda_oe;
    
    // Address match
    logic addr_match;
    
    //--------------------------------------------------------------------------
    // Input Synchronization
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 1'b1;
            scl_prev <= 1'b1;
            sda_sync <= 1'b1;
            sda_prev <= 1'b1;
        end else begin
            scl_prev <= scl_sync;
            scl_sync <= scl;
            sda_prev <= sda_sync;
            sda_sync <= sda;
        end
    end
    
    // Edge detection
    assign scl_rise = scl_sync && !scl_prev;
    assign scl_fall = !scl_sync && scl_prev;
    
    // Start: SDA falls while SCL is high
    assign start_cond = scl_sync && scl_prev && !sda_sync && sda_prev;
    
    // Stop: SDA rises while SCL is high
    assign stop_cond = scl_sync && scl_prev && sda_sync && !sda_prev;
    
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
                if (start_cond)
                    next_state = ST_START;
            end
            
            ST_START: begin
                next_state = ST_ADDR;
            end
            
            ST_ADDR: begin
                if (scl_rise && bit_cnt == 4'd7)
                    next_state = ST_ADDR_ACK;
            end
            
            ST_ADDR_ACK: begin
                if (scl_fall) begin
                    if (addr_match)
                        next_state = rw_bit ? ST_READ_DATA : ST_REG_ADDR;
                    else
                        next_state = ST_IDLE;
                end
            end
            
            ST_REG_ADDR: begin
                if (scl_rise && bit_cnt == 4'd7)
                    next_state = ST_REG_ACK;
            end
            
            ST_REG_ACK: begin
                if (scl_fall)
                    next_state = ST_WRITE_DATA;
            end
            
            ST_WRITE_DATA: begin
                if (scl_rise && bit_cnt == 4'd7)
                    next_state = ST_WRITE_ACK;
            end
            
            ST_WRITE_ACK: begin
                if (scl_fall)
                    next_state = ST_WRITE_DATA;
            end
            
            ST_READ_DATA: begin
                if (scl_rise && bit_cnt == 4'd7)
                    next_state = ST_READ_ACK;
            end
            
            ST_READ_ACK: begin
                if (scl_fall) begin
                    if (!sda_sync)  // ACK received
                        next_state = ST_READ_DATA;
                    else            // NACK - end transfer
                        next_state = ST_IDLE;
                end
            end
            
            ST_STOP: begin
                next_state = ST_IDLE;
            end
            
            default: next_state = ST_IDLE;
        endcase
        
        // Handle repeated start or stop
        if (start_cond && state != ST_IDLE)
            next_state = ST_START;
        if (stop_cond)
            next_state = ST_STOP;
    end
    
    //--------------------------------------------------------------------------
    // Bit Counter
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt <= 4'd0;
        end else begin
            case (state)
                ST_START, ST_ADDR_ACK, ST_REG_ACK, 
                ST_WRITE_ACK, ST_READ_ACK: begin
                    bit_cnt <= 4'd0;
                end
                
                ST_ADDR, ST_REG_ADDR, ST_WRITE_DATA, ST_READ_DATA: begin
                    if (scl_rise)
                        bit_cnt <= bit_cnt + 1'b1;
                end
                
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // Shift Register
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 8'h0;
        end else begin
            case (state)
                ST_ADDR, ST_REG_ADDR, ST_WRITE_DATA: begin
                    if (scl_rise)
                        shift_reg <= {shift_reg[6:0], sda_sync};
                end
                
                ST_READ_DATA: begin
                    if (scl_fall)
                        shift_reg <= {shift_reg[6:0], 1'b0};
                end
                
                ST_ADDR_ACK: begin
                    if (rw_bit && addr_match)
                        shift_reg <= reg_rdata;  // Load read data
                end
                
                ST_READ_ACK: begin
                    if (!sda_sync)  // ACK received, load next byte
                        shift_reg <= reg_rdata;
                end
                
                default: ;
            endcase
        end
    end
    
    //--------------------------------------------------------------------------
    // Address and R/W Handling
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_received <= 7'h0;
            rw_bit        <= 1'b0;
        end else begin
            if (state == ST_ADDR && scl_rise && bit_cnt == 4'd7) begin
                addr_received <= shift_reg[6:0];
                rw_bit        <= sda_sync;
            end
        end
    end
    
    assign addr_match = (addr_received == SLAVE_ADDR);
    
    //--------------------------------------------------------------------------
    // Register Address
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_addr <= 8'h0;
        end else begin
            if (state == ST_REG_ADDR && scl_rise && bit_cnt == 4'd7)
                reg_addr <= {shift_reg[6:0], sda_sync};
            else if (state == ST_WRITE_ACK && scl_fall)
                reg_addr <= reg_addr + 1'b1;  // Auto-increment
            else if (state == ST_READ_ACK && scl_fall && !sda_sync)
                reg_addr <= reg_addr + 1'b1;  // Auto-increment
        end
    end
    
    //--------------------------------------------------------------------------
    // Register Interface
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reg_wdata <= 8'h0;
            reg_wr_en <= 1'b0;
            reg_rd_en <= 1'b0;
        end else begin
            reg_wr_en <= 1'b0;
            reg_rd_en <= 1'b0;
            
            // Write operation
            if (state == ST_WRITE_DATA && scl_rise && bit_cnt == 4'd7) begin
                reg_wdata <= {shift_reg[6:0], sda_sync};
                reg_wr_en <= 1'b1;
            end
            
            // Read operation
            if (state == ST_ADDR_ACK && rw_bit && addr_match)
                reg_rd_en <= 1'b1;
            if (state == ST_READ_ACK && scl_fall && !sda_sync)
                reg_rd_en <= 1'b1;
        end
    end
    
    //--------------------------------------------------------------------------
    // SDA Output Control
    //--------------------------------------------------------------------------
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sda_out <= 1'b1;
            sda_oe  <= 1'b0;
        end else begin
            sda_oe  <= 1'b0;
            sda_out <= 1'b1;
            
            case (state)
                ST_ADDR_ACK, ST_REG_ACK, ST_WRITE_ACK: begin
                    if (addr_match) begin
                        sda_oe  <= 1'b1;
                        sda_out <= 1'b0;  // ACK
                    end
                end
                
                ST_READ_DATA: begin
                    sda_oe  <= 1'b1;
                    sda_out <= shift_reg[7];  // MSB first
                end
                
                default: ;
            endcase
        end
    end
    
    // Bidirectional SDA
    assign sda = sda_oe ? sda_out : 1'bz;

endmodule
