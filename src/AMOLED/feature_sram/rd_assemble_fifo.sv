`ifndef RD_ASSEMBLE_FIFO_SV
`define RD_ASSEMBLE_FIFO_SV

`timescale 1ns/1ps
`default_nettype none

module rd_assemble_fifo #(
  parameter int unsigned SRAM_BUS_W   = 32,
  parameter int unsigned FEAT_W       = 256,
  parameter int unsigned WORDS_PER_FEAT= 8,
  parameter int unsigned TAG_W        = 16,
  parameter int unsigned FIFO_DEPTH   = 2   // 1..N
)(
  input  logic clk,
  input  logic rst,

  // ---------- Beats in ----------
  input  logic               beat_valid,
  output logic               beat_ready,
  input  logic [TAG_W-1:0]   beat_tag,
  input  logic [$clog2(WORDS_PER_FEAT)-1:0] beat_idx,
  input  logic [SRAM_BUS_W-1:0] beat_data,

  // ---------- Feature out ----------
  output logic               feat_out_valid,
  input  logic               feat_out_ready,
  output logic [TAG_W-1:0]   feat_out_tag,
  output logic [FEAT_W-1:0]  feat_out_data
);

  // -----------------------------
  // localparams / types
  // -----------------------------
  localparam int unsigned IDX_W = (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);
  localparam int unsigned FIFO_AW = (FIFO_DEPTH <= 1) ? 1 : $clog2(FIFO_DEPTH);

  // sanity (optional)
  // synthesis translate_off
  initial begin
    if (FEAT_W != WORDS_PER_FEAT * SRAM_BUS_W) begin
      $display("WARN: FEAT_W (%0d) != WORDS_PER_FEAT*SRAM_BUS_W (%0d). Packing will still shift by SRAM_BUS_W.",
               FEAT_W, WORDS_PER_FEAT*SRAM_BUS_W);
    end
    if (FIFO_DEPTH < 1) begin
      $display("ERROR: FIFO_DEPTH must be >= 1");
      $finish;
    end
  end
  // synthesis translate_on

  // -----------------------------
  // 1) Assembler state
  // -----------------------------
  logic assembling;
  logic [TAG_W-1:0]  cur_tag;
  logic [FEAT_W-1:0] cur_feat;
  logic [WORDS_PER_FEAT-1:0] seen_mask;

  wire all_seen = &seen_mask;

  // write one beat into feature vector
  function automatic [FEAT_W-1:0] put_word(
    input [FEAT_W-1:0] feat,
    input [IDX_W-1:0]  idx,
    input [SRAM_BUS_W-1:0] w
  );
    reg [FEAT_W-1:0] tmp;
    int unsigned sh;
    begin
      tmp = feat;
      sh  = idx * SRAM_BUS_W;

      // place SRAM_BUS_W bits at position sh
      // (works when FEAT_W >= WORDS_PER_FEAT*SRAM_BUS_W; if smaller, upper bits truncate)
      tmp = tmp & ~({{(FEAT_W-SRAM_BUS_W){1'b0}}, {SRAM_BUS_W{1'b1}}} << sh);
      tmp = tmp | ({{(FEAT_W-SRAM_BUS_W){1'b0}}, w} << sh);

      put_word = tmp;
    end
  endfunction

  // -----------------------------
  // 2) Small output FIFO
  // -----------------------------
  logic [TAG_W-1:0]  fifo_tag  [0:FIFO_DEPTH-1];
  logic [FEAT_W-1:0] fifo_data [0:FIFO_DEPTH-1];

  logic [FIFO_AW-1:0] wptr, rptr;
  logic [$clog2(FIFO_DEPTH+1)-1:0] cnt;

  wire fifo_full  = (cnt == FIFO_DEPTH);
  wire fifo_empty = (cnt == 0);

  // output side
  assign feat_out_valid = !fifo_empty;
  assign feat_out_tag   = fifo_tag[rptr];
  assign feat_out_data  = fifo_data[rptr];

  wire fifo_pop  = feat_out_valid && feat_out_ready;
  wire fifo_push; // defined later when assembled done && space available

  // pointer increment helper (avoid % issues)
  function automatic [FIFO_AW-1:0] inc_ptr(input [FIFO_AW-1:0] p);
    if (FIFO_DEPTH <= 1) inc_ptr = '0;
    else if (p == FIFO_DEPTH-1) inc_ptr = '0;
    else inc_ptr = p + 1'b1;
  endfunction

  // -----------------------------
  // Beat backpressure rule
  // -----------------------------
  // We only need to stall beats when:
  // - we just completed a feature (all_seen after accepting this beat)
  //   AND fifo is full (can't push assembled feature)
  //
  // Otherwise, accept beats freely to avoid stalling SRAM returns.
  wire will_complete = assembling
                       && (beat_valid)
                       && (seen_mask[beat_idx] == 1'b0) // new idx
                       && ((seen_mask | ({{(WORDS_PER_FEAT-1){1'b0}},1'b1} << beat_idx)) == {WORDS_PER_FEAT{1'b1}});

  assign beat_ready = !rst && !(will_complete && fifo_full);

  // define fifo_push when we actually accept a beat that completes the feature
  wire beat_fire = beat_valid && beat_ready;

  assign fifo_push = beat_fire && assembling
                     && ( (seen_mask | ({{(WORDS_PER_FEAT-1){1'b0}},1'b1} << beat_idx)) == {WORDS_PER_FEAT{1'b1}} )
                     && !fifo_full;

  // -----------------------------
  // 3) Main sequential logic
  // -----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      assembling <= 1'b0;
      cur_tag    <= '0;
      cur_feat   <= '0;
      seen_mask  <= '0;

      wptr <= '0;
      rptr <= '0;
      cnt  <= '0;
    end else begin
      // -------------------------
      // FIFO pop
      // -------------------------
      if (fifo_pop) begin
        rptr <= inc_ptr(rptr);
      end

      // cnt update for pop/push (single assignment)
      case ({fifo_push, fifo_pop})
        2'b10: cnt <= cnt + 1'b1;
        2'b01: cnt <= cnt - 1'b1;
        default: cnt <= cnt;
      endcase

      // -------------------------
      // Assembler: start / accept beats
      // -------------------------
      if (!assembling) begin
        // start assembling on first accepted beat
        if (beat_fire) begin
          assembling <= 1'b1;
          cur_tag    <= beat_tag;
          cur_feat   <= put_word('0, beat_idx, beat_data);
          seen_mask  <= ({{(WORDS_PER_FEAT-1){1'b0}},1'b1} << beat_idx);

          // if single-beat feature (WORDS_PER_FEAT==1), push immediately
          if (WORDS_PER_FEAT == 1) begin
            // push happens via fifo_push combinational; but WORDS_PER_FEAT==1 => will_complete always true on first beat
            // after push, we should clear assembling next cycle
            if (!fifo_full) begin
              // will push below in FIFO push block
              assembling <= 1'b0;
              seen_mask  <= '0;
            end
          end
        end
      end else begin
        // assembling in progress
        if (beat_fire) begin
          // We assume beats belong to same tag; if not, you can choose to drop/flush or assert.
          // synthesis translate_off
          if (beat_tag !== cur_tag) begin
            $display("WARN: beat_tag changed mid-assemble (cur=0x%h new=0x%h) @%0t", cur_tag, beat_tag, $time);
          end
          // synthesis translate_on

          // only record unseen indices (ignore duplicates)
          if (!seen_mask[beat_idx]) begin
            cur_feat  <= put_word(cur_feat, beat_idx, beat_data);
            seen_mask <= seen_mask | ({{(WORDS_PER_FEAT-1){1'b0}},1'b1} << beat_idx);
          end

          // if this beat completes the feature and we can push, clear assembler
          if ( ( (seen_mask | ({{(WORDS_PER_FEAT-1){1'b0}},1'b1} << beat_idx)) == {WORDS_PER_FEAT{1'b1}} )
               && !fifo_full ) begin
            assembling <= 1'b0;
            seen_mask  <= '0;
            // cur_tag/cur_feat don't matter after push
          end
        end
      end

      // -------------------------
      // FIFO push (write assembled feature)
      // Note: we push the assembled data *after* incorporating current beat.
      // -------------------------
      if (fifo_push) begin
        fifo_tag[wptr]  <= cur_tag;
        // Use updated cur_feat if beat_idx was new; but cur_feat updates are nonblocking.
        // So compute the "would-be" feat for the push here:
        if (!seen_mask[beat_idx]) fifo_data[wptr] <= put_word(cur_feat, beat_idx, beat_data);
        else                      fifo_data[wptr] <= cur_feat;

        wptr <= inc_ptr(wptr);
      end
    end
  end

endmodule

`default_nettype wire
`endif
