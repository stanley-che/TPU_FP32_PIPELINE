// ============================================================
// feature_sram_burst_reader.sv  (IVERILOG-SAFE)
// - Send mem_rd_* commands to SRAM, receive mem_rdata beats,
//   pack into a full feature vector, then output feat_*.
//
// Assumptions / Notes:
// - Single outstanding burst at a time (simplest + robust tag alignment)
// - mem_rd_* is per-beat command (addr increments by +1 word each beat)
// - mem_rvalid may have bubbles; we count beats by mem_rvalid
// - Output feat_valid holds until feat_ready (backpressure safe)
//
// Parameters:
//   FEAT_W        : feature vector width in bits (e.g. 256)
//   MEM_W         : SRAM read data width per beat (e.g. 64)
//   TAG_W         : tag width
//   READ_LATENCY  : SRAM read latency in cycles (0/1/2/..). In this
//                  single-outstanding design, tag alignment is still correct.
// ============================================================

`ifndef FEATURE_SRAM_BURST_READER_SV
`define FEATURE_SRAM_BURST_READER_SV

`timescale 1ns/1ps
`default_nettype none

module feature_sram_burst_reader #(
  parameter int unsigned FEAT_W       = 256,
  parameter int unsigned MEM_W        = 64,
  parameter int unsigned TAG_W        = 16,
  parameter int unsigned ADDR_W       = 32,
  parameter int unsigned READ_LATENCY = 1,   // 0/1/2...
  parameter bit          USE_RTAG     = 0    // 1: use mem_rtag as returned tag (optional)
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 en,

  // ----------------------------
  // Upstream burst request
  // ----------------------------
  input  logic                 cmd_valid,
  output logic                 cmd_ready,
  input  logic [ADDR_W-1:0]    cmd_addr,     // word address
  input  logic [15:0]          cmd_beats,    // number of MEM_W beats
  input  logic [TAG_W-1:0]     cmd_tag,

  // ----------------------------
  // SRAM read command (per beat)
  // ----------------------------
  output logic                 mem_rd_valid,
  input  logic                 mem_rd_ready,
  output logic [ADDR_W-1:0]    mem_addr,     // word address for this beat
  output logic [TAG_W-1:0]     mem_tag,      // tag for this beat (optional usage)

  // ----------------------------
  // SRAM read data return (per beat)
  // ----------------------------
  input  logic                 mem_rvalid,
  input  logic [MEM_W-1:0]     mem_rdata,
  input  logic [TAG_W-1:0]     mem_rtag,     // optional (valid when mem_rvalid=1)

  // ----------------------------
  // Packed feature output
  // ----------------------------
  output logic                 feat_valid,
  input  logic                 feat_ready,
  output logic [FEAT_W-1:0]    feat_vec,
  output logic [TAG_W-1:0]     feat_tag
);

  // ----------------------------
  // Derived
  // ----------------------------
  localparam int unsigned BEATS_PER_FEAT =
    (MEM_W == 0) ? 1 :
    ((FEAT_W + MEM_W - 1) / MEM_W);

  wire go = en && !rst;

  // ----------------------------
  // State machine
  // ----------------------------
  typedef enum logic [1:0] {S_IDLE, S_ISSUE, S_WAITDATA, S_OUT} state_t;
  state_t st, st_n;

  // Latched command
  logic [ADDR_W-1:0]  base_addr_q;
  logic [15:0]        beats_q;
  logic [TAG_W-1:0]   tag_q;

  // Issue counters
  logic [15:0] issue_cnt_q;   // how many beats already issued (handshaked on mem_rd)
  logic [15:0] data_cnt_q;    // how many beats already received (mem_rvalid)

  // Current issuing address
  logic [ADDR_W-1:0] issue_addr_q;

  // Packing buffer
  logic [FEAT_W-1:0] feat_buf_q;

  // Tag pipeline (for completeness; single outstanding => tag_q is enough)
  // We keep a small shift register to demonstrate alignment if you want to extend later.
  localparam int unsigned LAT = (READ_LATENCY < 0) ? 0 : READ_LATENCY;
  logic [TAG_W-1:0] tag_pipe [0:LAT];
  integer ii;

  // Handshakes
  wire mem_fire  = go && mem_rd_valid && mem_rd_ready;
  wire out_fire  = go && feat_valid   && feat_ready;

  // ----------------------------
  // cmd_ready: accept only when idle and not holding output
  // ----------------------------
  always @* begin
    if (!go) cmd_ready = 1'b0;
    else     cmd_ready = (st == S_IDLE);
  end

  wire cmd_fire = go && cmd_valid && cmd_ready;

  // ----------------------------
  // Next-state logic
  // ----------------------------
  always @* begin
    st_n = st;

    case (st)
      S_IDLE: begin
        if (cmd_fire) st_n = S_ISSUE;
      end

      S_ISSUE: begin
        // After issuing all beats, move to WAITDATA (data may still be in flight)
        if (mem_fire && (issue_cnt_q + 16'd1 >= beats_q)) st_n = S_WAITDATA;
      end

      S_WAITDATA: begin
        // Once received all beats => produce output
        if (go && mem_rvalid && (data_cnt_q + 16'd1 >= beats_q)) st_n = S_OUT;
      end

      S_OUT: begin
        if (out_fire) st_n = S_IDLE;
      end

      default: st_n = S_IDLE;
    endcase
  end

  // ----------------------------
  // mem_rd_valid/mem_addr/mem_tag (per beat)
  // ----------------------------
  always @* begin
    if (!go) begin
      mem_rd_valid = 1'b0;
      mem_addr     = '0;
      mem_tag      = '0;
    end else begin
      mem_rd_valid = (st == S_ISSUE);

      // issue current address
      mem_addr = issue_addr_q;

      // send same tag for all beats (single outstanding)
      mem_tag  = tag_q;
    end
  end

  // ----------------------------
  // feat_valid/feat_vec/feat_tag outputs (hold under backpressure)
  // ----------------------------
  always @* begin
    if (!go) begin
      feat_valid = 1'b0;
      feat_vec   = '0;
      feat_tag   = '0;
    end else begin
      feat_valid = (st == S_OUT);
      feat_vec   = feat_buf_q;

      // If USE_RTAG=1, we use captured return tag (tag_pipe[LAT]) at completion;
      // else use cmd tag.
      feat_tag   = tag_q;
      if (USE_RTAG) feat_tag = tag_pipe[LAT];
    end
  end

  // ----------------------------
  // Sequential: state and registers
  // ----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      st          <= S_IDLE;

      base_addr_q <= '0;
      beats_q     <= 16'd0;
      tag_q       <= '0;

      issue_cnt_q <= 16'd0;
      data_cnt_q  <= 16'd0;
      issue_addr_q<= '0;

      feat_buf_q  <= '0;

      for (ii = 0; ii <= LAT; ii = ii + 1) begin
        tag_pipe[ii] <= '0;
      end

    end else if (en) begin
      st <= st_n;

      // Accept a new command
      if (cmd_fire) begin
        base_addr_q  <= cmd_addr;
        beats_q      <= (cmd_beats == 16'd0) ? 16'd1 : cmd_beats; // guard 0 => 1
        tag_q        <= cmd_tag;

        issue_cnt_q  <= 16'd0;
        data_cnt_q   <= 16'd0;
        issue_addr_q <= cmd_addr;

        feat_buf_q   <= '0;

        // init tag pipe with cmd_tag
        tag_pipe[0]  <= cmd_tag;
        for (ii = 1; ii <= LAT; ii = ii + 1) begin
          tag_pipe[ii] <= cmd_tag;
        end
      end

      // Issue beats (addr increments on each mem_fire)
      if (mem_fire) begin
        issue_cnt_q  <= issue_cnt_q + 16'd1;
        issue_addr_q <= issue_addr_q + {{(ADDR_W-1){1'b0}},1'b1}; // +1 word

        // advance tag pipeline on issue (optional)
        tag_pipe[0] <= tag_q;
        for (ii = 1; ii <= LAT; ii = ii + 1) begin
          tag_pipe[ii] <= tag_pipe[ii-1];
        end
      end

      // Receive data beats and pack
      if (go && mem_rvalid) begin
        // place mem_rdata into feat_buf_q at index data_cnt_q
        // lower beats go to lower bits (little-endian packing)
        if (data_cnt_q < BEATS_PER_FEAT[15:0]) begin
          feat_buf_q[data_cnt_q*MEM_W +: MEM_W] <= mem_rdata;
        end

        data_cnt_q <= data_cnt_q + 16'd1;

        // optionally capture returned tag
        if (USE_RTAG) begin
          tag_pipe[LAT] <= mem_rtag;
        end
      end

      // Consume output
      if (out_fire) begin
        // nothing special; st_n returns to IDLE and cmd_ready opens
      end
    end
  end

  // ----------------------------
  // Sanity checks
  // ----------------------------
  initial begin
    if (MEM_W == 0) begin
      $display("FATAL: MEM_W must be > 0");
      $fatal(1);
    end
    if (FEAT_W == 0) begin
      $display("FATAL: FEAT_W must be > 0");
      $fatal(1);
    end
  end

endmodule

`default_nettype wire
`endif
