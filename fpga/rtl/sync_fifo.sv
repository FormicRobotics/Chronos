//==============================================================================
// sync_fifo - single-clock first-word-fall-through FIFO
//==============================================================================
// Small register/BRAM FIFO used for the TX payload prefetch. FWFT: rd_data is
// valid whenever !empty; assert rd_en to pop.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sync_fifo #(
    parameter int WIDTH = 32,
    parameter int DEPTH = 16
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              wr_en,
    input  wire [WIDTH-1:0]  wr_data,
    output wire              full,
    input  wire              rd_en,
    output wire [WIDTH-1:0]  rd_data,
    output wire              empty
);
    localparam int AW = (DEPTH <= 1) ? 1 : $clog2(DEPTH);

    logic [WIDTH-1:0]   mem [DEPTH-1:0];
    logic [AW:0]        wr_ptr, rd_ptr;

    wire [AW:0] cnt = wr_ptr - rd_ptr;
    assign full  = (cnt == DEPTH[AW:0]);
    assign empty = (wr_ptr == rd_ptr);
    assign rd_data = mem[rd_ptr[AW-1:0]];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr[AW-1:0]] <= wr_data;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (rd_en && !empty)
                rd_ptr <= rd_ptr + 1'b1;
        end
    end
endmodule

`default_nettype wire
