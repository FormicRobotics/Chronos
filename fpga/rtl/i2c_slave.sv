//------------------------------------------------------------------------------
// I2C Slave Controller (host/Jetson register access)
//------------------------------------------------------------------------------
// Byte-level protocol (register pointer auto-increments):
//   write: START [addr+W] A [reg] A [data] A [data] A ... STOP
//   read : START [addr+W] A [reg] A Sr [addr+R] A [data] A ... [data] N STOP
//
// Bit-timing rules implemented here (the previous FSM asserted the ACK during
// the 8th clock's HIGH phase - SDA falling while SCL is high is a spurious
// START to every device on the bus - and released it before the 9th clock, so
// a master always sampled NACK and the byte framing shifted; it could never
// complete a transaction):
//   * incoming bits are sampled on SCL rising edges;
//   * outgoing bits (ACK, read data) change only while SCL is LOW;
//   * the slave ACK occupies the full 9th clock: asserted after the 8th
//     falling edge, held through the 9th high phase, released after the 9th
//     falling edge.
//
// reg_rd_en is pulsed one byte ahead (prefetch), so the registered reg_rdata
// from config_regs is stable microseconds before it is loaded into the shift
// register - this also fixes the old stale-first-read-byte bug.
//------------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module i2c_slave #(
    parameter logic [6:0] SLAVE_ADDR = 7'h3C
)(
    input  wire         clk,
    input  wire         rst_n,

    // I2C pads
    input  wire         scl,
    inout  wire         sda,

    // Register interface
    output logic [7:0]  reg_addr,
    output logic [7:0]  reg_wdata,
    input  wire  [7:0]  reg_rdata,
    output logic        reg_wr_en,      // 1-cycle pulse, reg_wdata valid
    output logic        reg_rd_en       // 1-cycle pulse, reg_rdata expected next
);

    //--------------------------------------------------------------------------
    // Input synchronisers + edge / bus-condition detection
    //--------------------------------------------------------------------------
    logic scl_meta, scl_sync, scl_prev;
    logic sda_meta, sda_sync, sda_prev;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            {scl_meta, scl_sync, scl_prev} <= '1;
            {sda_meta, sda_sync, sda_prev} <= '1;
        end else begin
            scl_meta <= scl;  scl_sync <= scl_meta;  scl_prev <= scl_sync;
            sda_meta <= sda;  sda_sync <= sda_meta;  sda_prev <= sda_sync;
        end
    end

    wire scl_rise   =  scl_sync & ~scl_prev;
    wire scl_fall   = ~scl_sync &  scl_prev;
    wire start_cond =  scl_sync &  scl_prev & ~sda_sync &  sda_prev; // SDA falls, SCL high
    wire stop_cond  =  scl_sync &  scl_prev &  sda_sync & ~sda_prev; // SDA rises, SCL high

    //--------------------------------------------------------------------------
    // FSM
    //--------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ST_IDLE,        // bus idle or transaction not addressed to us
        ST_RX,          // receiving a byte (device address, reg pointer, write data)
        ST_ACK_DRV,     // driving the slave ACK through the 9th clock
        ST_TX,          // transmitting a read byte
        ST_ACK_SMP      // sampling the master ACK/NACK after a read byte
    } state_t;

    typedef enum logic [1:0] { BY_ADDR, BY_REG, BY_WDATA, BY_RDATA } kind_t;

    state_t     state;
    kind_t      kind;           // what the current byte is
    logic [3:0] bit_cnt;
    logic [7:0] sh;
    logic       rw_bit;         // 1 = master reads
    logic       ack_smp;        // sampled master ACK (1 = ACK)
    logic       sda_oe;         // 1 = pull SDA low (open drain)

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            kind      <= BY_ADDR;
            bit_cnt   <= 4'd0;
            sh        <= 8'd0;
            rw_bit    <= 1'b0;
            ack_smp   <= 1'b0;
            sda_oe    <= 1'b0;
            reg_addr  <= 8'd0;
            reg_wdata <= 8'd0;
            reg_wr_en <= 1'b0;
            reg_rd_en <= 1'b0;
        end else begin
            reg_wr_en <= 1'b0;
            reg_rd_en <= 1'b0;

            unique case (state)
                //--------------------------------------------------------------
                ST_IDLE: sda_oe <= 1'b0;
                //--------------------------------------------------------------
                ST_RX: begin
                    if (scl_rise) begin
                        sh      <= {sh[6:0], sda_sync};
                        bit_cnt <= bit_cnt + 4'd1;
                    end
                    if (scl_fall && bit_cnt == 4'd8) begin
                        bit_cnt <= 4'd0;
                        case (kind)
                            BY_ADDR: begin
                                rw_bit <= sh[0];
                                if (sh[7:1] == SLAVE_ADDR) begin
                                    if (sh[0]) reg_rd_en <= 1'b1; // prefetch first read byte
                                    sda_oe <= 1'b1;               // ACK (SCL is low)
                                    state  <= ST_ACK_DRV;
                                end else begin
                                    state <= ST_IDLE;             // not addressed to us
                                end
                            end
                            BY_REG: begin
                                reg_addr <= sh;
                                sda_oe   <= 1'b1;
                                state    <= ST_ACK_DRV;
                            end
                            default: begin                        // BY_WDATA
                                reg_wdata <= sh;
                                reg_wr_en <= 1'b1;
                                sda_oe    <= 1'b1;
                                state     <= ST_ACK_DRV;
                            end
                        endcase
                    end
                end
                //--------------------------------------------------------------
                ST_ACK_DRV: begin
                    // SDA held low; master samples the ACK on the 9th rising
                    // edge; release / present first data bit after the fall.
                    if (scl_fall) begin
                        if (kind == BY_ADDR && rw_bit) begin
                            sh      <= reg_rdata;         // prefetched at fall #8
                            sda_oe  <= ~reg_rdata[7];     // present MSB (SCL low)
                            kind    <= BY_RDATA;
                            bit_cnt <= 4'd0;
                            state   <= ST_TX;
                        end else begin
                            sda_oe <= 1'b0;
                            if (kind == BY_WDATA) reg_addr <= reg_addr + 8'd1;
                            if (kind == BY_ADDR) kind <= BY_REG;
                            else                 kind <= BY_WDATA;
                            state <= ST_RX;
                        end
                    end
                end
                //--------------------------------------------------------------
                ST_TX: begin
                    if (scl_rise) bit_cnt <= bit_cnt + 4'd1;
                    if (scl_fall) begin
                        if (bit_cnt == 4'd8) begin
                            // byte clocked out; release SDA for the master ACK
                            // slot and prefetch the next byte (auto-increment)
                            sda_oe    <= 1'b0;
                            reg_addr  <= reg_addr + 8'd1;
                            reg_rd_en <= 1'b1;
                            bit_cnt   <= 4'd0;
                            state     <= ST_ACK_SMP;
                        end else begin
                            sh     <= {sh[6:0], 1'b0};
                            sda_oe <= ~sh[6];             // next bit, SCL low
                        end
                    end
                end
                //--------------------------------------------------------------
                ST_ACK_SMP: begin
                    if (scl_rise) ack_smp <= ~sda_sync;   // low = master ACK
                    if (scl_fall) begin
                        if (ack_smp) begin
                            sh     <= reg_rdata;          // prefetched byte
                            sda_oe <= ~reg_rdata[7];
                            state  <= ST_TX;
                        end else begin
                            state <= ST_IDLE;             // NACK: master is done
                        end
                    end
                end
                //--------------------------------------------------------------
                default: state <= ST_IDLE;
            endcase

            // (Repeated) START and STOP override any state. We never change
            // sda_oe while SCL is high, so we cannot trigger these ourselves.
            if (start_cond) begin
                state   <= ST_RX;
                kind    <= BY_ADDR;
                bit_cnt <= 4'd0;
                sda_oe  <= 1'b0;
            end
            if (stop_cond) begin
                state  <= ST_IDLE;
                kind   <= BY_ADDR;
                sda_oe <= 1'b0;
            end
        end
    end

    assign sda = sda_oe ? 1'b0 : 1'bz;

endmodule

`default_nettype wire
