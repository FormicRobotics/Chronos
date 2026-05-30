//==============================================================================
// csi2_pkg - shared CSI-2 constants and combinational helpers
//==============================================================================
// Single source of truth for the CSI-2 data types, the packet-header ECC
// (Hamming) and the payload CRC-16. Both csi2_rx and csi2_tx use these so the
// RX and TX sides can never drift apart (the old design hand-copied the ECC/CRC
// into each module, and they had already diverged).
//==============================================================================

`ifndef CSI2_PKG_SV
`define CSI2_PKG_SV

package csi2_pkg;

    // CSI-2 data types (subset relevant to OV9281 RAW mono)
    localparam logic [5:0] DT_FRAME_START = 6'h00;
    localparam logic [5:0] DT_FRAME_END   = 6'h01;
    localparam logic [5:0] DT_LINE_START  = 6'h02;
    localparam logic [5:0] DT_LINE_END    = 6'h03;
    localparam logic [5:0] DT_EMBEDDED     = 6'h12;
    localparam logic [5:0] DT_RAW8        = 6'h2A;
    localparam logic [5:0] DT_RAW10       = 6'h2B;
    localparam logic [5:0] DT_RAW12       = 6'h2C;

    // CSI-2 HS sync byte (sent LSB-first on the wire after SoT)
    localparam logic [7:0] SYNC_BYTE = 8'hB8;

    // True for short packets (data ID is the packet, no payload/CRC).
    function automatic logic is_short_packet(input logic [5:0] dt);
        return (dt == DT_FRAME_START) || (dt == DT_FRAME_END) ||
               (dt == DT_LINE_START)  || (dt == DT_LINE_END);
    endfunction

    //--------------------------------------------------------------------------
    // Start-of-packet metadata packing into a 32-bit token field. Shared by
    // csi2_rx (writer), frame_buffer (storage) and tx_arbiter (reader) so the
    // layout can never drift.
    //   [28]    short     [27:26] vc      [25:20] dt
    //   [19:4]  wc        [3] fs  [2] fe  [1] ls  [0] le
    //--------------------------------------------------------------------------
    function automatic logic [31:0] pack_sop(
        input logic        short, input logic [1:0] vc, input logic [5:0] dt,
        input logic [15:0] wc,
        input logic        fs, input logic fe, input logic ls, input logic le);
        return {3'b000, short, vc, dt, wc, fs, fe, ls, le};
    endfunction
    function automatic logic        sop_short_f(input logic [31:0] s); return s[28];      endfunction
    function automatic logic [1:0]  sop_vc_f   (input logic [31:0] s); return s[27:26];   endfunction
    function automatic logic [5:0]  sop_dt_f   (input logic [31:0] s); return s[25:20];   endfunction
    function automatic logic [15:0] sop_wc_f   (input logic [31:0] s); return s[19:4];    endfunction
    function automatic logic        sop_fs_f   (input logic [31:0] s); return s[3];       endfunction
    function automatic logic        sop_fe_f   (input logic [31:0] s); return s[2];       endfunction
    function automatic logic        sop_ls_f   (input logic [31:0] s); return s[1];       endfunction
    function automatic logic        sop_le_f   (input logic [31:0] s); return s[0];       endfunction

    //--------------------------------------------------------------------------
    // Packet-header ECC (MIPI CSI-2 spec, Hamming-modified over 24 data bits).
    // Input: the 24 header bits = {WC[15:0], VC[1:0], DT[5:0]} (i.e. byte0 is
    // {VC,DT}, then WC low byte, then WC high byte). Returns the 8-bit ECC with
    // ecc[7:6]=0.
    //--------------------------------------------------------------------------
    function automatic logic [7:0] calc_ecc(input logic [23:0] d);
        logic [7:0] e;
        e[0] = d[0]^d[1]^d[2]^d[4]^d[5]^d[7]^d[10]^d[11]^d[13]^d[16]^d[20]^d[21]^d[22]^d[23];
        e[1] = d[0]^d[1]^d[3]^d[4]^d[6]^d[8]^d[10]^d[12]^d[14]^d[17]^d[20]^d[21]^d[22]^d[23];
        e[2] = d[0]^d[2]^d[3]^d[5]^d[6]^d[9]^d[11]^d[12]^d[15]^d[18]^d[20]^d[21]^d[22];
        e[3] = d[1]^d[2]^d[3]^d[7]^d[8]^d[9]^d[13]^d[14]^d[15]^d[19]^d[20]^d[21]^d[23];
        e[4] = d[4]^d[5]^d[6]^d[7]^d[8]^d[9]^d[16]^d[17]^d[18]^d[19]^d[20]^d[22]^d[23];
        e[5] = d[10]^d[11]^d[12]^d[13]^d[14]^d[15]^d[16]^d[17]^d[18]^d[19]^d[21]^d[22]^d[23];
        e[6] = 1'b0;
        e[7] = 1'b0;
        return e;
    endfunction

    //--------------------------------------------------------------------------
    // CSI-2 payload CRC-16 (poly x^16+x^12+x^5+1, init 0xFFFF), processed one
    // byte at a time, LSB-first as required by the spec. Chain calls per byte
    // in transmission order.
    //--------------------------------------------------------------------------
    function automatic logic [15:0] crc16_step(input logic [15:0] crc_in,
                                               input logic [7:0]  data);
        logic [15:0] c;
        logic [7:0]  d;
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
        c[12] = crc_in[4]  ^ crc_in[8]  ^ crc_in[12] ^ crc_in[15] ^ d[0] ^ d[4] ^ d[7];
        c[13] = crc_in[5]  ^ crc_in[9]  ^ crc_in[13] ^ d[1] ^ d[5];
        c[14] = crc_in[6]  ^ crc_in[10] ^ crc_in[14] ^ d[2] ^ d[6];
        c[15] = crc_in[7]  ^ crc_in[11] ^ crc_in[15] ^ d[3] ^ d[7];
        return c;
    endfunction

endpackage

`endif
