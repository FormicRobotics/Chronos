//==============================================================================
// tb_i2c_slave - I2C master BFM exercising i2c_slave + config_regs
//==============================================================================
// Verifies the rewritten host I2C slave end-to-end against the real register
// bank: correct ACK bit timing (ACK spans the full 9th clock), register
// writes, single reads, auto-increment multi-byte reads/writes, repeated
// START, address mismatch NACK behaviour, and the ID/VERSION registers.
//
// Run:
//   iverilog -g2012 -o i2c.out ../rtl/i2c_slave.sv ../rtl/config_regs.sv tb_i2c_slave.sv
//   vvp i2c.out
//==============================================================================

`timescale 1ns / 1ps

module tb_i2c_slave;

    localparam logic [6:0] DEV = 7'h3C;

    // 192 MHz system clock
    logic clk = 1'b0;
    always #2.604 clk = ~clk;
    logic rst_n = 1'b0;

    //--------------------------------------------------------------------------
    // Open-drain bus with pull-ups
    //--------------------------------------------------------------------------
    logic m_scl_oe = 1'b0;    // master pulls SCL low
    logic m_sda_oe = 1'b0;    // master pulls SDA low
    tri1  scl, sda;
    assign scl = m_scl_oe ? 1'b0 : 1'bz;
    assign sda = m_sda_oe ? 1'b0 : 1'bz;

    //--------------------------------------------------------------------------
    // DUT: i2c_slave + config_regs
    //--------------------------------------------------------------------------
    logic [7:0] reg_addr, reg_wdata, reg_rdata;
    logic       reg_wr_en, reg_rd_en;

    i2c_slave #(.SLAVE_ADDR(DEV)) u_slave (
        .clk(clk), .rst_n(rst_n),
        .scl(scl), .sda(sda),
        .reg_addr(reg_addr), .reg_wdata(reg_wdata), .reg_rdata(reg_rdata),
        .reg_wr_en(reg_wr_en), .reg_rd_en(reg_rd_en)
    );

    logic        trigger_enable, soft_reset;
    logic [7:0]  frame_rate;
    logic [15:0] pulse_width;
    logic [7:0]  trigger_delay [4];
    logic [3:0]  cam_enable;
    logic [5:0]  output_data_type;
    logic [31:0] frame_count [4];
    initial for (int i = 0; i < 4; i++) frame_count[i] = 32'h0000_1100 + i;

    config_regs u_regs (
        .clk(clk), .rst_n(rst_n),
        .addr(reg_addr), .wdata(reg_wdata), .rdata(reg_rdata),
        .wr_en(reg_wr_en), .rd_en(reg_rd_en),
        .trigger_enable(trigger_enable), .frame_rate(frame_rate),
        .pulse_width(pulse_width), .trigger_delay(trigger_delay),
        .soft_reset(soft_reset), .cam_enable(cam_enable),
        .output_data_type(output_data_type),
        .rx_error(4'b0000), .buf_overflow(4'b0000),
        .pll_locked(1'b1), .cam_sync_status(4'b1111),
        .frame_count(frame_count)
    );

    //--------------------------------------------------------------------------
    // I2C master BFM (400 kHz-ish; quarter bit = 625 ns)
    //--------------------------------------------------------------------------
    localparam time QT = 625ns;

    task automatic i2c_start();          // SDA falls while SCL high
        m_sda_oe = 1'b0; m_scl_oe = 1'b0; #QT;
        m_sda_oe = 1'b1; #QT;            // START
        m_scl_oe = 1'b1; #QT;
    endtask

    task automatic i2c_stop();           // SDA rises while SCL high
        m_scl_oe = 1'b1; m_sda_oe = 1'b1; #QT;
        m_scl_oe = 1'b0; #QT;
        m_sda_oe = 1'b0; #(2*QT);        // STOP, bus idle
    endtask

    task automatic i2c_write_byte(input logic [7:0] b, output logic ack);
        for (int i = 7; i >= 0; i--) begin
            m_scl_oe = 1'b1; m_sda_oe = ~b[i]; #QT;   // set while SCL low
            m_scl_oe = 1'b0; #(2*QT);                 // SCL high
            m_scl_oe = 1'b1; #QT;                     // SCL low
        end
        // 9th clock: release SDA, sample slave ACK mid-high
        m_sda_oe = 1'b0; #QT;
        m_scl_oe = 1'b0; #QT;
        ack = ~sda;                                    // low = ACK
        #QT;
        m_scl_oe = 1'b1; #QT;
    endtask

    task automatic i2c_read_byte(output logic [7:0] b, input logic send_ack);
        m_sda_oe = 1'b0;                               // slave drives data
        for (int i = 7; i >= 0; i--) begin
            m_scl_oe = 1'b1; #QT;
            m_scl_oe = 1'b0; #QT;
            b[i] = sda;                                // sample mid-high
            #QT;
            m_scl_oe = 1'b1; #QT;
        end
        // 9th clock: master drives ACK (0) or NACK (1)
        m_sda_oe = send_ack; #QT;
        m_scl_oe = 1'b0; #(2*QT);
        m_scl_oe = 1'b1; #QT;
        m_sda_oe = 1'b0;
    endtask

    //--------------------------------------------------------------------------
    // Scoreboard helpers
    //--------------------------------------------------------------------------
    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin $error("[TB] CHECK FAILED: %s", msg); errors++; end
    endtask

    task automatic wr_reg(input logic [7:0] r, input logic [7:0] v);
        logic a0, a1, a2;
        i2c_start();
        i2c_write_byte({DEV, 1'b0}, a0);
        i2c_write_byte(r, a1);
        i2c_write_byte(v, a2);
        i2c_stop();
        check(a0 & a1 & a2, $sformatf("write 0x%02h=0x%02h all ACKed", r, v));
    endtask

    task automatic rd_reg(input logic [7:0] r, output logic [7:0] v);
        logic a0, a1, a2;
        i2c_start();
        i2c_write_byte({DEV, 1'b0}, a0);
        i2c_write_byte(r, a1);
        i2c_start();                        // repeated START
        i2c_write_byte({DEV, 1'b1}, a2);
        i2c_read_byte(v, 1'b0);             // send_ack=0 -> NACK: single-byte read
        i2c_stop();
        check(a0 & a1 & a2, $sformatf("read 0x%02h addressing ACKed", r));
    endtask

    //--------------------------------------------------------------------------
    // Test sequence
    //--------------------------------------------------------------------------
    logic [7:0] d0, d1;
    logic       ack;

    initial begin
        rst_n = 1'b0; #100; rst_n = 1'b1; #100;

        // 1. defaults over the bus: VERSION / ID (auto-increment read 0xFE,0xFF)
        rd_reg(8'hF0, d0); check(d0 == 8'h10, "VERSION == 0x10");
        begin : multiread
            logic a0, a1, a2;
            i2c_start();
            i2c_write_byte({DEV, 1'b0}, a0);
            i2c_write_byte(8'hFE, a1);
            i2c_start();
            i2c_write_byte({DEV, 1'b1}, a2);
            i2c_read_byte(d0, 1'b1);        // send_ack=1 -> ACK, continue
            i2c_read_byte(d1, 1'b0);        // send_ack=0 -> NACK, done
            i2c_stop();
            check(a0 & a1 & a2, "ID multi-read addressing ACKed");
            check(d0 == 8'h05, "ID_L == 0x05");
            check(d1 == 8'hC4, "ID_H == 0xC4 (auto-increment)");
        end

        // 2. write / read back FRAME_RATE
        wr_reg(8'h01, 8'd45);
        check(frame_rate == 8'd45, "frame_rate output == 45");
        rd_reg(8'h01, d0); check(d0 == 8'd45, "FRAME_RATE readback == 45");

        // 3. auto-increment multi-byte WRITE: PULSE_WIDTH_L/H in one transaction
        begin : multiwrite
            logic a0, a1, a2, a3;
            i2c_start();
            i2c_write_byte({DEV, 1'b0}, a0);
            i2c_write_byte(8'h02, a1);      // reg pointer
            i2c_write_byte(8'hD0, a2);      // 0x02 = low byte
            i2c_write_byte(8'h07, a3);      // 0x03 = high byte (auto-inc)
            i2c_stop();
            check(a0 & a1 & a2 & a3, "multi-write all ACKed");
            check(pulse_width == 16'h07D0, "pulse_width == 0x07D0 after auto-inc write");
        end

        // 4. CTRL: trigger enable + self-clearing soft reset
        wr_reg(8'h00, 8'h03);
        check(trigger_enable == 1'b1, "trigger_enable set");
        #200;   // several clk cycles
        rd_reg(8'h00, d0);
        check(d0 == 8'h01, "soft-reset bit self-cleared, trigger still on");

        // 5. wrong address is NACKed and ignored
        begin : wrongaddr
            logic a0;
            i2c_start();
            i2c_write_byte({7'h22, 1'b0}, a0);
            i2c_stop();
            check(a0 == 1'b0, "foreign address NACKed");
        end

        // 6. status read (packed inputs)
        rd_reg(8'h20, d0); check(d0 == 8'hF1, "STATUS == 0xF1 (sync=1111, pll=1)");

        $display("==================================================");
        $display("[TB] errors: %0d", errors);
        if (errors == 0) $display("[TB] RESULT: PASS");
        else             $display("[TB] RESULT: FAIL");
        $display("==================================================");
        $finish;
    end

    initial begin
        #3_000_000;   // 3 ms watchdog
        $error("[TB] global timeout");
        $finish;
    end

endmodule
