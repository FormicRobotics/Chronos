//==============================================================================
// cdc - small clock-domain-crossing primitives
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

// 2-flop level synchronizer
module bit_sync (
    input  wire  clk,
    input  wire  rst_n,
    input  wire  d,
    output logic q
);
    logic m;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin m <= 1'b0; q <= 1'b0; end
        else        begin m <= d;    q <= m;    end
    end
endmodule

// Pulse crossing via toggle: a 1-cycle pulse in the source domain becomes a
// 1-cycle pulse in the destination domain.
module pulse_cdc (
    input  wire  src_clk,
    input  wire  src_rst_n,
    input  wire  src_pulse,
    input  wire  dst_clk,
    input  wire  dst_rst_n,
    output logic dst_pulse
);
    logic tgl;
    always_ff @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) tgl <= 1'b0;
        else if (src_pulse) tgl <= ~tgl;
    end
    logic s0, s1, s2;
    always_ff @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin s0 <= 1'b0; s1 <= 1'b0; s2 <= 1'b0; end
        else            begin s0 <= tgl;  s1 <= s0;   s2 <= s1;   end
    end
    assign dst_pulse = s1 ^ s2;
endmodule

// sticky-OR a pulse in its own domain (cleared by reset)
module sticky_bit (
    input  wire  clk,
    input  wire  rst_n,
    input  wire  set,
    output logic q
);
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)    q <= 1'b0;
        else if (set)  q <= 1'b1;
    end
endmodule

`default_nettype wire
