`ifndef COMMON_SV
`define COMMON_SV
`ifdef VERILATOR
`include "include/config.sv"
`endif

import config_pkg::*;
package common;
    // parameters
    import config_pkg::*;
    parameter XLEN = 64;
    parameter MXLEN = XLEN;
    parameter LINK_REG_ID = 1;
    parameter logic [63:0] PCINIT = 64'h00000000_80000000;

    // typedefs
    typedef logic [127:0] u128;
    typedef logic [63:0] u64;
    typedef logic [43:0] u44;
    typedef logic [31:0] u32;
    typedef logic [26:0] u27;
    typedef logic [19:0] u20;
    typedef logic [15:0] u16;
    typedef logic [14:0] u15;
    typedef logic [13:0] u14;
    typedef logic [12:0] u13;
    typedef logic [11:0] u12;
    typedef logic [10:0] u11;
    typedef logic [9:0] u10;
    typedef logic [8:0] u9;
    typedef logic [7:0] u8;
    typedef logic [6:0] u7;
    typedef logic [5:0] u6;
    typedef logic [4:0] u5;
    typedef logic [3:0] u4;
    typedef logic [2:0] u3;
    typedef logic [1:0] u2;
    typedef logic u1;

    typedef u5 creg_addr_t;
    // typedef u64 word_t;
    typedef u8 strobe_t;
    typedef u12 csr_addr_t;

    /**
 * this file contains basic definitions and typedefs for general designs.
 */

    // Vivado does not support string parameters.
`ifdef VERILATOR
    `define STRING string
`else
    `define STRING /* f**k vivado */
`endif

    /**
 * Vivado does not support that members of a packed union
 * have different sizes. Therefore, we have to use struct
 * instead of union in Vivado.
 */
`ifdef VERILATOR
    `define PACKED_UNION union packed
`else
    `define PACKED_UNION struct packed
`endif

    // simple compile-time assertion
    `define ASSERTS(expr, message) \
    if (!(expr)) $error(message);
    `define ASSERT(expr) `ASSERTS(expr, "Assertion failed.");

    // to ignore some signals
    `define UNUSED_OK(list) \
    logic _unused_ok = &{1'b0, {list}, 1'b0};

    // basic data types
    `define BITS(x) logic[(x)-1:0]

    typedef int unsigned uint;

    typedef logic i1;
    typedef `BITS(2) i2;
    typedef `BITS(3) i3;
    typedef `BITS(4) i4;
    typedef `BITS(5) i5;
    typedef `BITS(6) i6;
    typedef `BITS(7) i7;
    typedef `BITS(8) i8;
    typedef `BITS(9) i9;
    typedef `BITS(16) i16;
    typedef `BITS(19) i19;
    typedef `BITS(26) i26;
    typedef `BITS(32) i32;
    typedef `BITS(33) i33;
    typedef `BITS(34) i34;
    typedef `BITS(35) i35;
    typedef `BITS(36) i36;
    typedef `BITS(37) i37;
    typedef `BITS(38) i38;
    typedef `BITS(39) i39;
    typedef `BITS(40) i40;
    typedef `BITS(41) i41;
    typedef `BITS(42) i42;
    typedef `BITS(64) i64;
    typedef `BITS(65) i65;
    typedef `BITS(66) i66;
    typedef `BITS(67) i67;
    typedef `BITS(68) i68;

    // for arithmetic overflow detection
    typedef i65 arith_t;

    // all addresses and words are 64-bit
    typedef i64 addr_t;
    typedef i64 word_t;

    // number of bytes transferred in one memory r/w
    typedef enum i3 {
        MSIZE1 = 3'b000,
        MSIZE2 = 3'b001,
        MSIZE4 = 3'b010,
        MSIZE8 = 3'b011
    } msize_t;

    // length of a burst transaction
    // NOTE: WRAP mode in AXI3 only supports power-of-2 length.
    typedef enum i8 {
        MLEN1   = 8'h00,
        MLEN2   = 8'h01,
        MLEN4   = 8'h03,
        MLEN8   = 8'h07,
        MLEN16  = 8'h0f,
        MLEN32  = 8'h1f,
        MLEN64  = 8'h3f,
        MLEN128 = 8'h7f,
        MLEN256 = 8'hff
    } mlen_t;

    parameter mlen_t AXI_BURST_LEN = AXI_BURST_NUM == 16 ? MLEN16 : AXI_BURST_NUM == 32 ? MLEN32 : AXI_BURST_NUM == 64 ? MLEN64 : AXI_BURST_NUM == 128 ? MLEN128 : AXI_BURST_NUM == 256 ? MLEN256 : MLEN1;

    /**
 * SOME NOTES ON BUSES
 *
 * bus naming convention:
 *  * CPU -> cache: xxx_req_t
 *  * cache -> CPU: xxx_resp_t
 *
 * in other words, caches are masters and CPU is the worker,
 * and CPU must wait for caches to complete memory transactions.
 * handshake signals are synchronized at positive edge of the clock.
 *
 * we guarantee that IBus is a subset of DBus, so that data cache can
 * be used as a instruction cache.
 * powerful students are free to design their own bus interfaces to
 * enable superscalar pipelines and other advanced techniques.
 *
 * a request on cache bus can bypass a cache instance if the address
 * is in uncached memory regions.
 */

    /**
 * NOTE on strobe:
 *
 * strobe is used to mask out unused bytes in data, and
 * data are always assumed be placed at addresses aligned to
 * 4 bytes, no matter the lowest 2 bits of addr says.
 * for example, if you want to write one byte "0xcd" at 0x1f2,
 * the addr is "0x000001f2", but the data should be "0x00cd0000"
 * and the strobe should be "0b0100", rather than "0x000000cd"
 * and "0b0001".
 */

    /**
 * instruction cache bus
 * addr must be aligned to 4 bytes.
 *
 * basically, ibus_resp_t is the same as dbus_resp_t.
 */
    typedef struct packed {
        logic  valid;  // in request?
        addr_t addr;   // target address
    } ibus_req_t;

    typedef struct packed {
        logic addr_ok;  // is the address accepted by cache?
        logic data_ok;  // is the field "data" valid?
        u32   data;     // the data read from cache
    } ibus_resp_t;

    /**
 * data cache bus
 */
    typedef struct packed {
        logic    valid;   // in request?
        addr_t   addr;    // target address
        msize_t  size;    // number of bytes
        strobe_t strobe;  // which bytes are enabled? set to zeros for read request
        word_t   data;    // the data to write
    } dbus_req_t;

    typedef struct packed {
        logic  addr_ok;  // is the address accepted by cache?
        logic  data_ok;  // is the field "data" valid?
        word_t data;     // the data read from cache
    } dbus_resp_t;

    `define IREQ_TO_DREQ(ireq) \
    {ireq, MSIZE4, 8'b0, 64'b0}

    `define DRESP_TO_IRESP(dresp, ireq) \
    {dresp.addr_ok, dresp.data_ok, ireq.addr[2] ? dresp.data[63:32] : dresp.data[31:0]}

    /**
 * cache bus: simplified burst AXI transaction interface
 */
    typedef enum i2 {
        AXI_BURST_FIXED = '0,
        AXI_BURST_INCR,
        AXI_BURST_WRAP,
        AXI_BURST_RESERVED
    } axi_burst_type_t;

    typedef struct packed {
        logic            valid;     // in request?
        logic            is_write;  // is it a write transaction?
        msize_t          size;      // number of bytes in one burst
        addr_t           addr;      // start address
        strobe_t         strobe;    // which bytes are enabled?
        word_t           data;      // the data to write
        mlen_t           len;       // number of bursts
        axi_burst_type_t burst;
    } cbus_req_t;

    typedef struct packed {
        logic  ready;  // is data arrived in this cycle?
        logic  last;   // is it the last word?
        word_t data;   // the data from AXI bus
    } cbus_resp_t;


    typedef struct packed {
        u64 x0;
        u64 ra;
        u64 sp;
        u64 gp;
        u64 tp;
        u64 t0;
        u64 t1;
        u64 t2;
        u64 s0;
        u64 s1;

        u64 a0;
        u64 a1;
        u64 a2;
        u64 a3;
        u64 a4;
        u64 a5;
        u64 a6;
        u64 a7;

        u64 s2;
        u64 s3;
        u64 s4;
        u64 s5;
        u64 s6;
        u64 s7;
        u64 s8;
        u64 s9;
        u64 s10;
        u64 s11;

        u64 t3;
        u64 t4;
        u64 t5;
        u64 t6;
    } regs_t;


    typedef struct packed {
        u1 sd;  // [63]
        u27 xx;  // [62:36]
        u2 sxl;  // [35:34]
        u2 uxl;  // [33:32]
        u9 xxx;  // [31:23]
        u1 tsr;  // [22]
        u1 tw;  // [21]
        u1 tvm;  // [20]
        u1 mxr;  // [19]
        u1 sum;  // [18]
        u1 mprv;  // [17]
        u2 xs;  // [16:15]
        u2 fs;  // [14:13]
        u2 mpp;  // [12:11] machine previous privilege
        u2 xxxx;  // [10:9]
        u1 spp;  // [8] supervisor previous privilege
        u1 mpie;  // [7] machine previous interrupt enable
        u1 xxxxx;  // [6]
        u1 spie;  // [5] supervisor previous interrupt enable
        u1 upie;  // [4] user previous interrupt enable
        u1 mie;  // [3] machine interrupt enable
        u1 xxxxxx;  // [2]
        u1 sie;  // [1] supervisor interrupt enable
        u1 uie;  // [0] user interrupt enable
    } mstatus_t;

    typedef struct packed {
        u4  mode;  // [63:60] (0:bare 8:Sv39 9:Sv48)
        u16 asid;  // [59:44] address space identifier
        u44 ppn;   //  [43:0] physical page number
    } satp_t;

    parameter logic [3:0] SATP_bare = 4'b0000;
    parameter logic [3:0] SATP_sv39 = 4'b1000;
    parameter logic [3:0] SATP_sv48 = 4'b1001;

    typedef struct packed {
        u10 reserved;  // [63:54]
        u44 ppn;       // [53:10] physical page number
        u2  rsw;       // [9:8] reserved for supervisor software
        u1  d;         // [7] dirty
        u1  a;         // [6] accessed
        u1  g;         // [5] global
        u1  u;         // [4] user
        u1  x;         // [3] executable
        u1  w;         // [2] writable
        u1  r;         // [1] readable
        u1  v;         // [0] valid
    } Sv39_entry_t;

    parameter logic [1:0] U_Mode = 2'b00;
    parameter logic [1:0] S_Mode = 2'b01;
    parameter logic [1:0] M_Mode = 2'b11;
endpackage
`endif
