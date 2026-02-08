`ifndef RD_ASSEMBLE_FIFO_SV
`define RD_ASSEMBLE_FIFO_SV

`timescale 1ns/1ps
`default_nettype none

module rd_assemble_fifo #(
  parameter int unsigned SRAM_BUS_W     = 32,
  parameter int unsigned FEAT_W         = 256,
  parameter int unsigned WORDS_PER_FEAT = 8,
  parameter int unsigned TAG_W          = 16,
  parameter int unsigned FIFO_DEPTH     = 2   // 1..N
)(
  input  logic clk,
  input  logic rst,

  // ---------- Beats in ----------
  input  logic                           beat_valid,
  output logic                           beat_ready,
  input  logic [TAG_W-1:0]               beat_tag,
  input  logic [$clog2(WORDS_PER_FEAT)-1:0] beat_idx,
  input  logic [SRAM_BUS_W-1:0]          beat_data,

  // ---------- Feature out ----------
  output logic                           feat_out_valid,
  input  logic                           feat_out_ready,
  output logic [TAG_W-1:0]               feat_out_tag,
  output logic [FEAT_W-1:0]              feat_out_data
);

  // -----------------------------
  // localparams / widths
  // -----------------------------
  localparam int unsigned IDX_W   = (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);
  localparam int unsigned FIFO_AW = (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH);
  localparam logic [WORDS_PER_FEAT-1:0] ALL_ONES = {WORDS_PER_FEAT{1'b1}};

  // -----------------------------
  // put_word helper
  // -----------------------------
  function automatic [FEAT_W-1:0] put_word(
    input [FEAT_W-1:0]       feat,
    input [IDX_W-1:0]        idx,
    input [SRAM_BUS_W-1:0]   w
  );
    reg [FEAT_W-1:0] tmp;
    int unsigned sh;
    reg [FEAT_W-1:0] mask_word;
    begin
      tmp = feat;
      sh  = idx * SRAM_BUS_W;
      mask_word = ({{(FEAT_W-SRAM_BUS_W){1'b0}}, {SRAM_BUS_W{1'b1}}} << sh);
      tmp = (tmp & ~mask_word) | (({{(FEAT_W-SRAM_BUS_W){1'b0}}, w}) << sh);
      put_word = tmp;
    end
  endfunction

  // -----------------------------
  // Assembler state
  // -----------------------------
  logic                     assembling;
  logic [TAG_W-1:0]          cur_tag;
  logic [FEAT_W-1:0]         cur_feat;
  logic [WORDS_PER_FEAT-1:0] seen_mask;

  // -----------------------------
  // Output FIFO
  // -----------------------------
  logic [TAG_W-1:0]  fifo_tag  [0:FIFO_DEPTH-1];
  logic [FEAT_W-1:0] fifo_data [0:FIFO_DEPTH-1];

  logic [FIFO_AW-1:0] wptr, rptr;
  logic [$clog2(FIFO_DEPTH+1)-1:0] cnt;

  wire fifo_full  = (cnt == FIFO_DEPTH);
  wire fifo_empty = (cnt == 0);

  assign feat_out_valid = !fifo_empty;
  assign feat_out_tag   = fifo_tag[rptr];
  assign feat_out_data  = fifo_data[rptr];

  wire fifo_pop = feat_out_valid && feat_out_ready;

  function automatic [FIFO_AW-1:0] inc_ptr(input [FIFO_AW-1:0] p);
    if (FIFO_DEPTH <= 1)        inc_ptr = '0;
    else if (p == FIFO_DEPTH-1) inc_ptr = '0;
    else                        inc_ptr = p + 1'b1;
  endfunction

  // -----------------------------
  // X-safe guards
  // -----------------------------
  wire idx_known = ((^beat_idx) !== 1'bX);  // beat_idx 有任一bit X -> XOR 會是 X

  // -----------------------------
  // Combinational "next" computations (IVERILOG SAFE)
  // -----------------------------
  logic do_restart;
  logic [TAG_W-1:0]          nxt_tag;
  logic [FEAT_W-1:0]         feat_before;
  logic [WORDS_PER_FEAT-1:0] mask_before;

  logic idx_new;
  logic [FEAT_W-1:0]         feat_after;
  logic [WORDS_PER_FEAT-1:0] mask_after;

  logic will_complete;
  logic will_push;

  always @* begin
    // defaults
    do_restart  = 1'b0;
    nxt_tag     = beat_tag;
    feat_before = cur_feat;
    mask_before = seen_mask;

    idx_new     = 1'b0;
    feat_after  = cur_feat;
    mask_after  = seen_mask;

    will_complete = 1'b0;
    will_push     = 1'b0;

    if (!assembling) begin
      do_restart  = 1'b1;
      feat_before = '0;
      mask_before = '0;
    end else if (beat_tag !== cur_tag) begin
      // tag changed mid-assemble -> flush + restart
      do_restart  = 1'b1;
      feat_before = '0;
      mask_before = '0;
    end

    if (idx_known) begin
      idx_new = !mask_before[beat_idx];

      feat_after = idx_new ? put_word(feat_before, beat_idx, beat_data) : feat_before;
      mask_after = mask_before | ({{(WORDS_PER_FEAT-1){1'b0}},1'b1} << beat_idx);

      if (WORDS_PER_FEAT == 1) begin
        will_complete = 1'b1;
      end else begin
        will_complete = (mask_after == ALL_ONES);
      end

      will_push = will_complete && !fifo_full;
    end else begin
      // idx unknown -> don't touch anything
      idx_new       = 1'b0;
      feat_after    = feat_before;
      mask_after    = mask_before;
      will_complete = 1'b0;
      will_push     = 1'b0;
    end
  end

  // Backpressure:
  // - idx must be known
  // - stall only when accepting this beat would complete AND fifo_full
  assign beat_ready = (!rst) && idx_known && !(beat_valid && fifo_full && will_complete);

  wire beat_fire = beat_valid && beat_ready;

  // -----------------------------
  // Sequential
  // -----------------------------
  integer i;
  always_ff @(posedge clk) begin
    if (rst) begin
      assembling <= 1'b0;
      cur_tag    <= '0;
      cur_feat   <= '0;
      seen_mask  <= '0;

      wptr <= '0;
      rptr <= '0;
      cnt  <= '0;

      for (i = 0; i < FIFO_DEPTH; i = i + 1) begin
        fifo_tag[i]  <= '0;
        fifo_data[i] <= '0;
      end
    end else begin
      // -------------------------
      // FIFO pop
      // -------------------------
      if (fifo_pop) begin
        rptr <= inc_ptr(rptr);
      end

      // -------------------------
      // FIFO push (only when beat accepted AND completes AND space)
      // -------------------------
      if (beat_fire && will_push) begin
        fifo_tag[wptr]  <= nxt_tag;
        fifo_data[wptr] <= feat_after;
        wptr            <= inc_ptr(wptr);
      end

      // -------------------------
      // cnt update (push/pop)
      // -------------------------
      case ({(beat_fire && will_push), fifo_pop})
        2'b10: cnt <= cnt + 1'b1;
        2'b01: cnt <= cnt - 1'b1;
        default: cnt <= cnt;
      endcase

      // -------------------------
      // Assembler state update
      // -------------------------
      if (beat_fire) begin
        if (will_push) begin
          // pushed -> clear assembler
          assembling <= 1'b0;
          cur_tag    <= '0;
          cur_feat   <= '0;
          seen_mask  <= '0;
        end else begin
          // still assembling
          assembling <= 1'b1;
          cur_tag    <= nxt_tag;
          cur_feat   <= feat_after;
          seen_mask  <= mask_after;
        end
      end
    end
  end

endmodule

`default_nettype wire
`endif
