//==============================================================================
// tb_trigger_generator - validates the phase-accumulator rate engine
//==============================================================================
// Uses a scaled-down CLK_FREQ_HZ (192 kHz) so one "virtual second" is only
// 192k cycles: at fps=30 exactly 30 pulses/virtual-second must fire, and at
// fps=7 (a rate that does NOT divide the clock) exactly 7 - proving the
// accumulator's average rate is exact. Also checks pulse width and the
// out-of-range fps fallback to 30.
//
// Run:
//   iverilog -g2012 -o trig.out ../rtl/trigger_generator.sv tb_trigger_generator.sv
//   vvp trig.out
//==============================================================================

`timescale 1ns / 1ps

module tb_trigger_generator;

    localparam int CLK_HZ = 192_000;     // scaled: 1 virtual second = 192k cycles

    logic clk = 1'b0;
    always #5 clk = ~clk;
    logic rst_n = 1'b0;

    logic        enable = 1'b0;
    logic [7:0]  frame_rate  = 8'd30;
    logic [15:0] pulse_width = 16'd10;
    logic [7:0]  trigger_delay [4];
    logic        trigger_pulse, fsin_out;

    initial for (int i = 0; i < 4; i++) trigger_delay[i] = 8'd0;

    trigger_generator #(
        .NUM_OUTPUTS(4), .CLK_FREQ_HZ(CLK_HZ), .MAX_FRAME_RATE(120)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .enable(enable), .frame_rate(frame_rate),
        .pulse_width(pulse_width), .trigger_delay(trigger_delay),
        .trigger_pulse(trigger_pulse), .fsin_out(fsin_out)
    );

    int errors = 0;
    task automatic check(input bit cond, input string msg);
        if (!cond) begin $error("[TB] CHECK FAILED: %s", msg); errors++; end
    endtask

    // count rising edges of trigger_pulse over exactly one virtual second
    task automatic count_pulses(output int n);
        logic prev;
        n = 0; prev = trigger_pulse;
        repeat (CLK_HZ) begin
            @(posedge clk);
            if (trigger_pulse && !prev) n++;
            prev = trigger_pulse;
        end
    endtask

    int n;
    int width;

    initial begin
        rst_n = 1'b0; #100; rst_n = 1'b1; #100;

        // 30 fps: 30 pulses per virtual second. Discard the first window: it
        // contains the intentional immediate pulse on enable (prompt start).
        frame_rate = 8'd30; enable = 1'b1;
        @(posedge clk);
        count_pulses(n);   // settle (includes the prompt-start pulse)
        count_pulses(n);
        check(n == 30, $sformatf("30 fps -> got %0d pulses/venv-sec", n));

        // 7 fps: does not divide 192000 (192000/7 = 27428.57) - accumulator
        // must still average exactly 7 per second
        frame_rate = 8'd7;
        count_pulses(n);   // let the new rate latch through one period
        count_pulses(n);
        check(n == 7, $sformatf("7 fps -> got %0d pulses/venv-sec", n));

        // pulse width: measure high time of one pulse (10 cycles programmed)
        frame_rate = 8'd30;
        count_pulses(n);   // settle
        @(posedge trigger_pulse);
        width = 0;
        // sample on negedge so we never race the DUT's non-blocking update
        forever begin @(negedge clk); if (!trigger_pulse) break; width++; end
        check(width == 10, $sformatf("pulse width %0d cycles (exp 10)", width));

        // out-of-range fps falls back to 30
        frame_rate = 8'd200;   // > MAX_FRAME_RATE
        count_pulses(n);       // settle
        count_pulses(n);
        check(n == 30, $sformatf("fps=200 clamps to 30 -> got %0d", n));

        // disable stops pulses
        enable = 1'b0;
        count_pulses(n);
        check(n == 0, "disabled -> no pulses");

        $display("==================================================");
        $display("[TB] errors: %0d", errors);
        if (errors == 0) $display("[TB] RESULT: PASS");
        else             $display("[TB] RESULT: FAIL");
        $display("==================================================");
        $finish;
    end

    initial begin
        #20_000_000;
        $error("[TB] global timeout");
        $finish;
    end

endmodule
