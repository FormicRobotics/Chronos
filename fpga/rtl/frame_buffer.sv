//==============================================================================
// frame_buffer - per-camera asynchronous FWFT token FIFO
//==============================================================================
// Bridges a camera's recovered byte-clock domain (write) to the TX byte-clock
// domain (read), and stores the CSI-2 packet token stream from csi2_rx:
//   token = {is_sop, last, data[31:0]}
// where a SOP token carries packed packet metadata (csi2_pkg::pack_sop) and
// data tokens carry payload words; 'last' marks the final token of a packet
// (the SOP itself for short packets).
//
// Rewrite vs old design:
//   * True dual-clock async FIFO (Gray-coded pointers) instead of a single-clock
//     buffer that assumed RX and TX shared one clock.
//   * First-word-fall-through: rd_data/rd_is_sop/rd_last are valid whenever
//     rd_valid is high; rd_en pops. (Old design needed a read before data
//     appeared, which deadlocked the arbiter handshake.)
//   * pkt_avail: asserted only when at least one COMPLETE packet is buffered, so
//     the arbiter never starts a packet that would underflow mid-HS.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module frame_buffer #(
    parameter int ADDR_WIDTH = 11,          // 2048 tokens (~5 lines of RAW10)
    parameter int CAMERA_ID  = 0
)(
    // Write side (camera recovered byte clock)
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        wr_en,
    input  wire        wr_is_sop,
    input  wire        wr_last,
    input  wire [31:0] wr_data,
    output wire        full,
    output logic       overflow,

    // Read side (TX byte clock)
    input  wire        rd_clk,
    input  wire        rd_rst_n,
    input  wire        rd_en,
    output wire        rd_valid,
    output wire        rd_is_sop,
    output wire        rd_last,
    output wire [31:0] rd_data,
    output wire        empty,
    output wire        pkt_avail
);

    localparam int DEPTH = (1 << ADDR_WIDTH);
    localparam int TW    = 34;              // {is_sop,last,data[31:0]}

    (* ram_style = "block" *)
    logic [TW-1:0] mem [DEPTH-1:0];

    //--------------------------------------------------------------------------
    // helpers
    //--------------------------------------------------------------------------
    function automatic logic [ADDR_WIDTH:0] bin2gray(input logic [ADDR_WIDTH:0] b);
        return b ^ (b >> 1);
    endfunction

    //--------------------------------------------------------------------------
    // Write domain
    //--------------------------------------------------------------------------
    logic [ADDR_WIDTH:0] wr_bin, wr_gray;
    logic [ADDR_WIDTH:0] rd_gray_w1, rd_gray_w2;   // rd_gray synced to wr
    logic [7:0]          pkt_wr;                    // complete-packet count (wr)
    logic [7:0]          pkt_wr_gray;

    wire [ADDR_WIDTH:0] wr_bin_next = wr_bin + 1'b1;
    assign full = (bin2gray(wr_bin_next) ==
                   {~rd_gray_w2[ADDR_WIDTH:ADDR_WIDTH-1], rd_gray_w2[ADDR_WIDTH-2:0]});

    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_bin   <= '0;
            wr_gray  <= '0;
            overflow <= 1'b0;
            pkt_wr   <= 8'd0;
        end else begin
            if (wr_en) begin
                if (!full) begin
                    mem[wr_bin[ADDR_WIDTH-1:0]] <= {wr_is_sop, wr_last, wr_data};
                    wr_bin  <= wr_bin_next;
                    wr_gray <= bin2gray(wr_bin_next);
                    if (wr_last) pkt_wr <= pkt_wr + 8'd1;
                end else begin
                    overflow <= 1'b1;
                end
            end
        end
    end
    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) pkt_wr_gray <= 8'd0;
        else           pkt_wr_gray <= pkt_wr ^ (pkt_wr >> 1);
    end

    // rd_gray -> wr domain sync
    logic [ADDR_WIDTH:0] rd_gray_r;
    always_ff @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin rd_gray_w1 <= '0; rd_gray_w2 <= '0; end
        else           begin rd_gray_w1 <= rd_gray_r; rd_gray_w2 <= rd_gray_w1; end
    end

    //--------------------------------------------------------------------------
    // Read domain (FWFT)
    //--------------------------------------------------------------------------
    logic [ADDR_WIDTH:0] rd_bin;
    logic [ADDR_WIDTH:0] wr_gray_r1, wr_gray_r2;   // wr_gray synced to rd
    logic [7:0]          pkt_wr_g1, pkt_wr_g2;      // pkt_wr_gray synced to rd
    logic [7:0]          pkt_rd;                    // packets fully popped (rd)
    logic                dout_valid;
    logic [TW-1:0]       dout;

    assign rd_gray_r = bin2gray(rd_bin);
    wire mem_empty = (bin2gray(rd_bin) == wr_gray_r2);

    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin wr_gray_r1 <= '0; wr_gray_r2 <= '0; end
        else           begin wr_gray_r1 <= wr_gray; wr_gray_r2 <= wr_gray_r1; end
    end
    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin pkt_wr_g1 <= 8'd0; pkt_wr_g2 <= 8'd0; end
        else           begin pkt_wr_g1 <= pkt_wr_gray; pkt_wr_g2 <= pkt_wr_g1; end
    end

    // gray->bin of synced complete-packet count
    logic [7:0] pkt_wr_bin_rd;
    always_comb begin
        pkt_wr_bin_rd[7] = pkt_wr_g2[7];
        for (int i = 6; i >= 0; i--)
            pkt_wr_bin_rd[i] = pkt_wr_bin_rd[i+1] ^ pkt_wr_g2[i];
    end

    always_ff @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_bin     <= '0;
            dout_valid <= 1'b0;
            dout       <= '0;
            pkt_rd     <= 8'd0;
        end else begin
            if (!mem_empty && (!dout_valid || rd_en)) begin
                dout       <= mem[rd_bin[ADDR_WIDTH-1:0]];
                rd_bin     <= rd_bin + 1'b1;
                dout_valid <= 1'b1;
            end else if (rd_en) begin
                dout_valid <= 1'b0;
            end
            if (rd_en && dout_valid && dout[TW-2] /*last*/)
                pkt_rd <= pkt_rd + 8'd1;
        end
    end

    assign rd_valid  = dout_valid;
    assign rd_is_sop = dout[TW-1];
    assign rd_last   = dout[TW-2];
    assign rd_data   = dout[31:0];
    assign empty     = !dout_valid && mem_empty;
    assign pkt_avail = (pkt_wr_bin_rd != pkt_rd);

    /* verilator lint_off UNUSED */
    wire _unused = &{1'b0, CAMERA_ID[0], 1'b0};
    /* verilator lint_on UNUSED */

endmodule

`default_nettype wire
