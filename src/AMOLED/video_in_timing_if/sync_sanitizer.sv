`ifndef SYNC_SANITIZER_SV
`define SYNC_SANITIZER_SV

`timescale 1ns/1ps
`default_nettype none

// ------------------------------------------------------------
// sync_sanitizer (redesigned)
// - Polarity normalize (optional invert)
// - 2FF synchronizer stage (vs_ff2/hs_ff2/de_ff2)
// - Optional deglitch:
//     output updates only after the 2FF signal has remained
//     constant for STABLE_CYCLES consecutive clk edges.
//   * No off-by-one: uses next_cnt to decide update immediately.
// ------------------------------------------------------------
module sync_sanitizer #(
  parameter bit VS_INV = 1'b0,
  parameter bit HS_INV = 1'b0,
  parameter bit DE_INV = 1'b0,

  parameter bit USE_DEGLITCH = 1'b0,
  parameter int unsigned STABLE_CYCLES = 2  // >=2 when USE_DEGLITCH=1
)(
  input  logic clk,
  input  logic rst,

  input  logic vsync_i,
  input  logic hsync_i,
  input  logic de_i,

  output logic vsync,
  output logic hsync,
  output logic de
);

  // -----------------------------
  // 1) Optional polarity normalize
  // -----------------------------
  logic vs_in, hs_in, de_in;
  always_comb begin
    vs_in = VS_INV ? ~vsync_i : vsync_i;
    hs_in = HS_INV ? ~hsync_i : hsync_i;
    de_in = DE_INV ? ~de_i    : de_i;
  end

  // -----------------------------
  // 2) 2FF synchronizer / pipeline
  // -----------------------------
  (* ASYNC_REG = "TRUE" *) logic vs_ff1, hs_ff1, de_ff1;
  (* ASYNC_REG = "TRUE" *) logic vs_ff2, hs_ff2, de_ff2;

  always_ff @(posedge clk) begin
    if (rst) begin
      vs_ff1 <= 1'b0;  vs_ff2 <= 1'b0;
      hs_ff1 <= 1'b0;  hs_ff2 <= 1'b0;
      de_ff1 <= 1'b0;  de_ff2 <= 1'b0;
    end else begin
      vs_ff1 <= vs_in;  vs_ff2 <= vs_ff1;
      hs_ff1 <= hs_in;  hs_ff2 <= hs_ff1;
      de_ff1 <= de_in;  de_ff2 <= de_ff1;
    end
  end

  // -----------------------------
  // 3) Output stage
  //    - nofilter: pure 2FF (combinational)
  //    - filter  : deglitch FSM/counters (registered)
  // -----------------------------
  generate
    if (!USE_DEGLITCH) begin : g_nofilter
      // IMPORTANT: no extra FF here → exact 2FF latency
      always_comb begin
        vsync = vs_ff2;
        hsync = hs_ff2;
        de    = de_ff2;
      end
    end else begin : g_filter

      // counter width enough to count to STABLE_CYCLES
      localparam int unsigned CW =
        (STABLE_CYCLES <= 1) ? 1 : $clog2(STABLE_CYCLES+1);

      logic [CW-1:0] vs_cnt, hs_cnt, de_cnt;
      logic          vs_last, hs_last, de_last;

      // saturating increment
      function automatic [CW-1:0] sat_inc(input [CW-1:0] v);
        if (v >= STABLE_CYCLES[CW-1:0]) sat_inc = STABLE_CYCLES[CW-1:0];
        else                            sat_inc = v + {{(CW-1){1'b0}},1'b1};
      endfunction

      // compute next counter (no off-by-one)
      function automatic [CW-1:0] next_cnt_val(
        input logic        same,
        input [CW-1:0]     cur
      );
        if (same) next_cnt_val = sat_inc(cur);
        else      next_cnt_val = '0;
      endfunction

      always_ff @(posedge clk) begin
        if (rst) begin
          vsync   <= 1'b0;
          hsync   <= 1'b0;
          de      <= 1'b0;

          vs_last <= 1'b0;  hs_last <= 1'b0;  de_last <= 1'b0;
          vs_cnt  <= '0;    hs_cnt  <= '0;    de_cnt  <= '0;
        end else begin
          // ---------------- VS ----------------
          begin
            logic same_vs;
            logic [CW-1:0] vs_cnt_n;

            same_vs  = (vs_ff2 == vs_last);
            vs_cnt_n = next_cnt_val(same_vs, vs_cnt);

            // update last/cnt
            if (!same_vs) vs_last <= vs_ff2;
            vs_cnt <= vs_cnt_n;

            // update output when stable reached (use NEXT counter!)
            if (vs_cnt_n >= STABLE_CYCLES[CW-1:0])
              vsync <= (same_vs ? vs_last : vs_ff2);
          end

          // ---------------- HS ----------------
          begin
            logic same_hs;
            logic [CW-1:0] hs_cnt_n;

            same_hs  = (hs_ff2 == hs_last);
            hs_cnt_n = next_cnt_val(same_hs, hs_cnt);

            if (!same_hs) hs_last <= hs_ff2;
            hs_cnt <= hs_cnt_n;

            if (hs_cnt_n >= STABLE_CYCLES[CW-1:0])
              hsync <= (same_hs ? hs_last : hs_ff2);
          end

          // ---------------- DE ----------------
          begin
            logic same_de;
            logic [CW-1:0] de_cnt_n;

            same_de  = (de_ff2 == de_last);
            de_cnt_n = next_cnt_val(same_de, de_cnt);

            if (!same_de) de_last <= de_ff2;
            de_cnt <= de_cnt_n;

            if (de_cnt_n >= STABLE_CYCLES[CW-1:0])
              de <= (same_de ? de_last : de_ff2);
          end
        end
      end

      // parameter guard (generate-time behavior still fine)
      initial begin
        if (STABLE_CYCLES < 2) begin
          $fatal(1, "sync_sanitizer: STABLE_CYCLES must be >= 2 when USE_DEGLITCH=1");
        end
      end

    end
  endgenerate

endmodule

`default_nettype wire
`endif
