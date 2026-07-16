//==============================================================================
// i2c_master - byte-oriented open-drain I2C/SCCB master (write-only path)
//==============================================================================
// Minimal master used to program the OV9281 over SCCB. Streaming byte interface:
//   - push a byte with tx_first=1 to emit START + (dev_addr<<1 | 0) + byte
//   - push subsequent bytes (tx_first=0) to continue the same transaction
//   - set tx_last=1 on the final byte to emit STOP afterwards
// tx_ready pulses when the pushed byte (and trailing STOP, if last) has been
// shifted out. ack_error latches if any addressed/data byte was NAKed.
//
// SCCB note: the OV9281 ignores the 9th (ACK) bit for some phases; we drive SDA
// released during ACK and merely sample it, which is compatible.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module i2c_master #(
    parameter int CLK_HZ = 192_000_000,
    parameter int SCL_HZ = 400_000
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [6:0] dev_addr,

    // open-drain pads
    inout  wire       scl,
    inout  wire       sda,

    // byte stream
    input  wire       tx_valid,
    input  wire [7:0] tx_data,
    input  wire       tx_first,   // emit START + address before this byte
    input  wire       tx_last,    // emit STOP after this byte
    output logic      tx_ready,   // 1-cycle pulse: byte consumed/sent
    output logic      busy,
    output logic      ack_error
);

    localparam int QDIV = (CLK_HZ / SCL_HZ) / 4;   // cycles per quarter-bit

    // open-drain drivers
    logic scl_oe, sda_oe;
    assign scl = scl_oe ? 1'b0 : 1'bz;
    assign sda = sda_oe ? 1'b0 : 1'bz;
    wire   sda_in = sda;

    typedef enum logic [3:0] {
        S_IDLE, S_START, S_ADDR, S_ADDR_ACK, S_DATA, S_DATA_ACK, S_STOP, S_DONE
    } st_t;
    st_t st;

    logic [15:0] qcnt;       // quarter-bit timer
    logic [1:0]  q;          // quarter phase 0..3
    logic [3:0]  bit_idx;    // 0..8
    logic [7:0]  sh;         // shift register
    logic [7:0]  data_l;
    logic        last_l;
    logic        bus_open;    // START emitted, STOP not yet: hold SCL low in idle

    wire qtick = (qcnt == QDIV[15:0]-1);

    // advance quarter timer
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin qcnt <= '0; end
        else if (st == S_IDLE) qcnt <= '0;
        else qcnt <= qtick ? '0 : qcnt + 1'b1;
    end

    task automatic drive_bit(input logic b);
        // drive SDA for a data bit across the quarter phases of one SCL period
        case (q)
            2'd0: begin scl_oe <= 1'b1; sda_oe <= ~b; end // SCL low, set SDA
            2'd1: begin scl_oe <= 1'b0;               end // SCL high
            2'd2: begin scl_oe <= 1'b0;               end // SCL high (valid)
            2'd3: begin scl_oe <= 1'b1;               end // SCL low
        endcase
    endtask

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_IDLE;
            q         <= 2'd0;
            bit_idx   <= 4'd0;
            sh        <= 8'd0;
            data_l    <= 8'd0;
            last_l    <= 1'b0;
            scl_oe    <= 1'b0;
            sda_oe    <= 1'b0;
            tx_ready  <= 1'b0;
            busy      <= 1'b0;
            ack_error <= 1'b0;
            bus_open  <= 1'b0;
        end else begin
            tx_ready <= 1'b0;

            unique case (st)
                //--------------------------------------------------------------
                S_IDLE: begin
                    // Between bytes of an open transaction keep SCL driven low
                    // (clock stretching); releasing both lines mid-transaction
                    // would look like a STOP to the slave once the bus RC lets
                    // the lines rise.
                    scl_oe <= bus_open;
                    sda_oe <= 1'b0;
                    q      <= 2'd0;
                    if (tx_valid) begin
                        busy   <= 1'b1;
                        data_l <= tx_data;
                        last_l <= tx_last;
                        if (tx_first) begin
                            sh <= {dev_addr, 1'b0};   // address + write
                            st <= S_START;
                        end else begin
                            sh <= tx_data;
                            bit_idx <= 4'd0;
                            st <= S_DATA;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end
                //--------------------------------------------------------------
                S_START: begin
                    // SDA high->low while SCL high
                    case (q)
                        2'd0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end
                        2'd1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; end // SDA low, SCL high
                        2'd2: begin scl_oe <= 1'b0; sda_oe <= 1'b1; end
                        2'd3: begin scl_oe <= 1'b1; sda_oe <= 1'b1; end // SCL low
                    endcase
                    if (qtick) begin
                        q <= q + 2'd1;
                        if (q == 2'd3) begin
                            bus_open <= 1'b1;
                            bit_idx  <= 4'd0;
                            st       <= S_ADDR;
                        end
                    end
                end
                //--------------------------------------------------------------
                S_ADDR: begin
                    drive_bit(sh[7]);
                    if (qtick) begin
                        q <= q + 2'd1;
                        if (q == 2'd3) begin
                            sh <= {sh[6:0], 1'b0};
                            if (bit_idx == 4'd7) begin bit_idx <= 4'd0; st <= S_ADDR_ACK; end
                            else bit_idx <= bit_idx + 4'd1;
                        end
                    end
                end
                //--------------------------------------------------------------
                S_ADDR_ACK: begin
                    case (q)
                        2'd0: begin scl_oe <= 1'b1; sda_oe <= 1'b0; end // release SDA
                        2'd1: begin scl_oe <= 1'b0;               end
                        2'd2: begin scl_oe <= 1'b0; if (sda_in) ack_error <= 1'b1; end
                        2'd3: begin scl_oe <= 1'b1;               end
                    endcase
                    if (qtick) begin
                        q <= q + 2'd1;
                        if (q == 2'd3) begin sh <= data_l; bit_idx <= 4'd0; st <= S_DATA; end
                    end
                end
                //--------------------------------------------------------------
                S_DATA: begin
                    drive_bit(sh[7]);
                    if (qtick) begin
                        q <= q + 2'd1;
                        if (q == 2'd3) begin
                            sh <= {sh[6:0], 1'b0};
                            if (bit_idx == 4'd7) begin bit_idx <= 4'd0; st <= S_DATA_ACK; end
                            else bit_idx <= bit_idx + 4'd1;
                        end
                    end
                end
                //--------------------------------------------------------------
                S_DATA_ACK: begin
                    case (q)
                        2'd0: begin scl_oe <= 1'b1; sda_oe <= 1'b0; end
                        2'd1: begin scl_oe <= 1'b0;               end
                        2'd2: begin scl_oe <= 1'b0; if (sda_in) ack_error <= 1'b1; end
                        2'd3: begin scl_oe <= 1'b1;               end
                    endcase
                    if (qtick) begin
                        q <= q + 2'd1;
                        if (q == 2'd3) begin
                            if (last_l) st <= S_STOP;
                            else        st <= S_DONE;
                        end
                    end
                end
                //--------------------------------------------------------------
                S_STOP: begin
                    // SDA low->high while SCL high
                    case (q)
                        2'd0: begin scl_oe <= 1'b1; sda_oe <= 1'b1; end
                        2'd1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; end // SCL high, SDA low
                        2'd2: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end // SDA high -> STOP
                        2'd3: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end
                    endcase
                    if (qtick) begin
                        q <= q + 2'd1;
                        if (q == 2'd3) begin
                            bus_open <= 1'b0;
                            st       <= S_DONE;
                        end
                    end
                end
                //--------------------------------------------------------------
                S_DONE: begin
                    tx_ready <= 1'b1;
                    q        <= 2'd0;
                    st       <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
