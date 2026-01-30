// tb_rgb2y_luma_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb2y_luma_top.vvp \
  ./test/tb_rgb2y_luma_top.sv

vvp ./vvp/tb_rgb2y_luma_top.vvp
gtkwave ./vvp/tb_rgb2y_luma_top.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb2y_luma_top.sv" // your file name may differ


`timescale 1ns/1ps
`default_nettype none

module tb_rgb2y_luma_top;

  // ============================================================
  // clock / reset
  // ============================================================
  reg clk = 1'b0;
  always #5 clk = ~clk;
  reg rst;

  // ============================================================
  // DUT params (match your top defaults or override here)
  // ============================================================
  localparam int unsigned FORMAT = 0;  // 0 RGB888

  localparam int unsigned LAT_U  = 1;
  localparam int unsigned LAT_M  = 2;
  localparam int unsigned LAT_SUM = 2;
  localparam int unsigned LAT_RND = 2;

  localparam bit ZERO_WHEN_INVALID = 1'b1;
  localparam bit USE_DSP = 1'b0;
  localparam bit BYPASS  = 1'b0;

  localparam bit HOLD_VALID_WHEN_STALL = 1'b1;
  localparam int unsigned ROUND_MODE = 1;
  localparam int unsigned Y8_MODE = 0;

  // out_pack
  localparam int unsigned PACK_IN_W  = 10;
  localparam int unsigned PACK_OUT_W = 10;
  localparam int unsigned SB_W       = 8;   // test sideband
  localparam int unsigned PACK_LAT   = 2;   // elastic depth=3

  localparam bit PACK_GATE_SIDEBAND_WITH_VALID = 1'b1;
  localparam bit PACK_USE_ROUND = 1'b0;
  localparam bit PACK_USE_SAT   = 1'b1;
  localparam bit PACK_USE_CLIP  = 1'b1;     // enable clip logic in DUT
  localparam bit PACK_COUNT_DBG = 1'b1;

  // ============================================================
  // DUT I/O
  // ============================================================
  reg                    en;

  reg                    pix_valid;
  reg [35:0]             rgb_in;
  reg [SB_W-1:0]         sb_in;
  wire                   in_ready;

  wire                   out_valid;
  reg                    out_ready;
  wire [PACK_OUT_W-1:0]  Y;
  wire [SB_W-1:0]        sb_out;

  reg                    clip_en;
  reg [PACK_OUT_W-1:0]   clip_min;
  reg [PACK_OUT_W-1:0]   clip_max;

  // debug taps
  wire                   v1_valid;
  wire [7:0]             r8,g8,b8;
  wire                   v2_valid;
  wire [15:0]            r_term,g_term,b_term;
  wire                   v4_valid;
  wire [16:0]            sum17;
  wire [9:0]             y10;
  wire [7:0]             y8;
  wire                   ovf_sum, ovf_y;

  wire                   drop_pulse, stall_pulse;
  wire [31:0]            drop_cnt, stall_cnt;

  // ============================================================
  // DUT
  // ============================================================
  rgb2y_luma_top #(
    .FORMAT(FORMAT),
    .LAT_U(LAT_U),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),

    .LAT_M(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS),

    .LAT_SUM(LAT_SUM),
    .HOLD_VALID_WHEN_STALL(HOLD_VALID_WHEN_STALL),

    .LAT_RND(LAT_RND),
    .ROUND_MODE(ROUND_MODE),
    .Y8_MODE(Y8_MODE),

    .PACK_IN_W(PACK_IN_W),
    .PACK_OUT_W(PACK_OUT_W),
    .SB_W(SB_W),
    .PACK_LAT(PACK_LAT),

    .PACK_GATE_SIDEBAND_WITH_VALID(PACK_GATE_SIDEBAND_WITH_VALID),
    .PACK_USE_ROUND(PACK_USE_ROUND),
    .PACK_USE_SAT(PACK_USE_SAT),
    .PACK_USE_CLIP(PACK_USE_CLIP),
    .PACK_COUNT_DBG(PACK_COUNT_DBG)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .pix_valid(pix_valid),
    .rgb_in(rgb_in),
    .sb_in(sb_in),
    .in_ready(in_ready),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .Y(Y),
    .sb_out(sb_out),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .v1_valid(v1_valid),
    .r8(r8), .g8(g8), .b8(b8),

    .v2_valid(v2_valid),
    .r_term(r_term), .g_term(g_term), .b_term(b_term),

    .v4_valid(v4_valid),
    .sum17(sum17),
    .y10(y10),
    .y8(y8),
    .ovf_sum(ovf_sum),
    .ovf_y(ovf_y),

    .drop_pulse(drop_pulse),
    .stall_pulse(stall_pulse),
    .drop_cnt(drop_cnt),
    .stall_cnt(stall_cnt)
  );

  // ============================================================
  // helpers: pack by FORMAT (same style as your previous TB)
  // ============================================================
  task automatic pack_rgb888(
    input  [7:0] rr, input [7:0] gg, input [7:0] bb,
    output [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[23:16] = rr;
      bus[15:8]  = gg;
      bus[7:0]   = bb;
    end
  endtask

  // ============================================================
  // expected models (full_pipe + out_pack)
  // ============================================================
  function automatic [31:0] lfsr32_next(input [31:0] s);
    reg fb;
    begin
      fb = s[31] ^ s[21] ^ s[1] ^ s[0];
      lfsr32_next = {s[30:0], fb};
    end
  endfunction

  // coeffs for Y = (77R + 150G + 29B)>>8 with rounding mode
  function automatic [15:0] mul77 (input [7:0] x);  begin mul77  = x * 16'd77;  end endfunction
  function automatic [15:0] mul150(input [7:0] x);  begin mul150 = x * 16'd150; end endfunction
  function automatic [15:0] mul29 (input [7:0] x);  begin mul29  = x * 16'd29;  end endfunction

  localparam integer SUM_MAX = 65280;

  function automatic [16:0] f_sum17(input [15:0] r, input [15:0] g, input [15:0] b);
    reg [16:0] s;
    begin
      s = {1'b0,r} + {1'b0,g} + {1'b0,b};
      f_sum17 = s;
    end
  endfunction

  function automatic f_ovf_sum(input [16:0] s);
    begin
      f_ovf_sum = (s > SUM_MAX);
    end
  endfunction

  function automatic [9:0] f_y10_round(input [16:0] s);
    reg [9:0] q;
    reg [7:0] r;
    reg inc;
    reg [10:0] qinc;
    begin
      q = s[16:8];
      r = s[7:0];

      // ROUND_MODE=1 => inc when r>=0x80
      inc  = (r >= 8'h80);
      qinc = {1'b0,q} + {10'd0,inc};
      if (qinc[10]) f_y10_round = 10'h3FF;
      else         f_y10_round = qinc[9:0];
    end
  endfunction

  // out_pack reference: unsigned trunc+sat+clip (match your rgb2y_out_pack default)
  function automatic [PACK_OUT_W-1:0] ref_pack_y(
    input logic [PACK_IN_W-1:0] yin,
    input logic                 clip_en_i,
    input logic [PACK_OUT_W-1:0] cmin,
    input logic [PACK_OUT_W-1:0] cmax
  );
    logic [PACK_OUT_W-1:0] ytrunc;
    logic [PACK_OUT_W-1:0] ysat;
    logic [PACK_OUT_W-1:0] yclip;
    begin
      ytrunc = yin[PACK_OUT_W-1:0];

      if (PACK_USE_SAT) begin
        if (PACK_IN_W > PACK_OUT_W) begin
          if (|yin[PACK_IN_W-1:PACK_OUT_W]) ysat = {PACK_OUT_W{1'b1}};
          else                               ysat = ytrunc;
        end else begin
          ysat = ytrunc;
        end
      end else begin
        ysat = ytrunc;
      end

      if (PACK_USE_CLIP && clip_en_i) begin
        if (ysat < cmin)      yclip = cmin;
        else if (ysat > cmax) yclip = cmax;
        else                  yclip = ysat;
      end else begin
        yclip = ysat;
      end

      ref_pack_y = yclip;
    end
  endfunction

  // ============================================================
  // scoreboard queue for expected (push on input handshake, pop on output handshake)
  // ============================================================
  localparam int QDEPTH = 4096;

  reg [PACK_OUT_W-1:0] exp_y  [0:QDEPTH-1];
  reg [SB_W-1:0]       exp_sb [0:QDEPTH-1];
  integer q_w, q_r, q_count;

  task automatic q_reset;
    integer k;
    begin
      q_w=0; q_r=0; q_count=0;
      for (k=0;k<QDEPTH;k=k+1) begin
        exp_y[k]='0;
        exp_sb[k]='0;
      end
    end
  endtask

  task automatic q_push(input [PACK_OUT_W-1:0] yv, input [SB_W-1:0] sbv);
    begin
      if (q_count >= QDEPTH) begin
        $display("[TB] ERROR: queue overflow");
        $fatal;
      end
      exp_y[q_w]  = yv;
      exp_sb[q_w] = sbv;
      q_w = (q_w + 1) % QDEPTH;
      q_count = q_count + 1;
    end
  endtask

  task automatic q_pop_check(input [PACK_OUT_W-1:0] yv, input [SB_W-1:0] sbv);
    begin
      if (q_count == 0) begin
        $display("[TB] ERROR: unexpected output while queue empty y=%0d sb=0x%0h", yv, sbv);
        $fatal;
      end
      if (yv !== exp_y[q_r] || sbv !== exp_sb[q_r]) begin
        $display("[TB] ERROR: mismatch @%0t", $time);
        $display("  got  y=%0d sb=0x%0h", yv, sbv);
        $display("  exp  y=%0d sb=0x%0h", exp_y[q_r], exp_sb[q_r]);
        $fatal;
      end
      q_r = (q_r + 1) % QDEPTH;
      q_count = q_count - 1;
    end
  endtask

  // ============================================================
  // stall hold check at output
  // ============================================================
  reg [PACK_OUT_W-1:0] Y_hold;
  reg [SB_W-1:0]       SB_hold;
  reg                  hold_armed;

  always @(posedge clk) begin
    if (rst) begin
      Y_hold     <= '0;
      SB_hold    <= '0;
      hold_armed <= 1'b0;
    end else begin
      if (en && out_valid && !out_ready) begin
        if (!hold_armed) begin
          Y_hold     <= Y;
          SB_hold    <= sb_out;
          hold_armed <= 1'b1;
        end else begin
          if (Y !== Y_hold || sb_out !== SB_hold) begin
            $display("[TB] ERROR: output changed while stalled @%0t", $time);
            $display("  was y=%0d sb=0x%0h, now y=%0d sb=0x%0h", Y_hold, SB_hold, Y, sb_out);
            $fatal;
          end
        end
      end else begin
        hold_armed <= 1'b0;
      end
    end
  end

  // ============================================================
  // step helper
  // ============================================================
  task automatic step;
    begin
      @(posedge clk);
      #1;
    end
  endtask

  // ============================================================
  // drive one cycle with upstream policy: only send when in_ready
  // and compute expected on the SAME handshake event:
  //   input handshake: en && pix_valid && in_ready
  //   output handshake: en && out_valid && out_ready
  // ============================================================
  task automatic drive_one(
    input integer idx,
    input logic    want_valid,     // "I want to send" (but TB will only actually send when in_ready)
    input [7:0]    rr,
    input [7:0]    gg,
    input [7:0]    bb,
    input [SB_W-1:0] sbv,
    input logic    en_i,
    input logic    out_rdy_i,
    input logic    clip_en_i,
    input [PACK_OUT_W-1:0] cmin,
    input [PACK_OUT_W-1:0] cmax
  );
    reg do_send;
    reg [35:0] bus;
    reg [15:0] rt, gt, bt;
    reg [16:0] s17;
    reg [9:0]  y10e;
    reg [PACK_OUT_W-1:0] yout_e;
    reg [SB_W-1:0] sb_e;
    begin
      en        = en_i;
      out_ready = out_rdy_i;
      clip_en   = clip_en_i;
      clip_min  = cmin;
      clip_max  = cmax;

      pack_rgb888(rr,gg,bb,bus);

      // upstream rule: only assert pix_valid when in_ready
      do_send  = want_valid && in_ready && en_i;

      pix_valid = do_send;
      rgb_in    = do_send ? bus : 36'h0; // keep clean
      sb_in     = sbv;

      // input handshake => push expected (NOTE: full_pipe computes Y10 from current RGB)
      if (do_send) begin
        if (BYPASS) begin
          rt = {8'h00, rr};
          gt = {8'h00, gg};
          bt = {8'h00, bb};
        end else begin
          rt = mul77(rr);
          gt = mul150(gg);
          bt = mul29(bb);
        end
        s17  = f_sum17(rt,gt,bt);
        y10e = f_y10_round(s17);

        yout_e = ref_pack_y(y10e[PACK_IN_W-1:0], clip_en_i, cmin, cmax);

        if (SB_W == 0) sb_e = '0;
        else begin
          if (PACK_GATE_SIDEBAND_WITH_VALID) sb_e = sbv; // valid=1 so pass-through
          else                               sb_e = sbv;
        end

        q_push(yout_e, sb_e);
      end

      step();

      // output handshake => pop/check
      if (en_i && out_valid && out_ready) begin
        q_pop_check(Y, sb_out);
      end
    end
  endtask

  // ============================================================
  // tests
  // ============================================================
  task automatic warmup(input integer n);
    integer k;
    begin
      for (k=0;k<n;k=k+1)
        drive_one(-1000+k, 1'b0, 8'h0,8'h0,8'h0, '0, 1'b1, 1'b1, 1'b1, 10'd16, 10'd900);
    end
  endtask

  task automatic drain(input integer n);
    integer k;
    begin
      for (k=0;k<n;k=k+1)
        drive_one(900000+k, 1'b0, 8'h0,8'h0,8'h0, '0, 1'b1, 1'b1, clip_en, clip_min, clip_max);
    end
  endtask

  task automatic test_mix;
    integer t;
    reg [31:0] s;
    reg [7:0] rr,gg,bb;
    reg [SB_W-1:0] sbv;
    reg en_pat;
    reg rdy_pat;
    reg clip_pat;
    begin
      $display("[RUN ] MIX + backpressure + en gating + clip toggles");
      warmup(10);
      s = 32'h1ACE_B00C;

      for (t=0;t<2000;t=t+1) begin
        s  = lfsr32_next(s);
        rr = s[7:0];
        gg = s[15:8];
        bb = s[23:16];
        sbv = (8'hA0 ^ t[7:0]);

        // en pattern: holds
        if ((t%17)==8 || (t%17)==9 || (t%17)==10) en_pat = 1'b0;
        else if ((t%9)==4)                         en_pat = 1'b0;
        else                                        en_pat = 1'b1;

        // downstream backpressure bursts
        if ((t%23)>=16 && (t%23)<=20) rdy_pat = 1'b0;
        else                          rdy_pat = 1'b1;

        // clip toggle
        clip_pat = (t%31) < 20;

        // want_valid bubbles (~30%)
        if (s[31] & s[28]) begin
          drive_one(100+t, 1'b0, rr,gg,bb, sbv, en_pat, rdy_pat, clip_pat, 10'd16, 10'd900);
        end else begin
          drive_one(100+t, 1'b1, rr,gg,bb, sbv, en_pat, rdy_pat, clip_pat, 10'd16, 10'd900);
        end
      end

      drain(200);
      if (q_count != 0) begin
        $display("[TB] ERROR: queue not empty after MIX, remaining=%0d", q_count);
        $fatal;
      end
      $display("[PASS] MIX");
    end
  endtask

  task automatic test_known_vectors;
    reg [SB_W-1:0] sbv;
    begin
      $display("[RUN ] KNOWN VECTORS (no stall)");
      warmup(8);

      sbv = 8'h11; drive_one(20000, 1'b1, 8'hFF,8'h00,8'h00, sbv, 1'b1, 1'b1, 1'b1, 10'd0, 10'd1023);
      sbv = 8'h22; drive_one(20001, 1'b1, 8'h00,8'hFF,8'h00, sbv, 1'b1, 1'b1, 1'b1, 10'd0, 10'd1023);
      sbv = 8'h33; drive_one(20002, 1'b1, 8'h00,8'h00,8'hFF, sbv, 1'b1, 1'b1, 1'b1, 10'd0, 10'd1023);
      sbv = 8'h44; drive_one(20003, 1'b1, 8'hFF,8'hFF,8'hFF, sbv, 1'b1, 1'b1, 1'b1, 10'd0, 10'd1023);

      drain(120);
      if (q_count != 0) begin
        $display("[TB] ERROR: queue not empty after KNOWN, remaining=%0d", q_count);
        $fatal;
      end
      $display("[PASS] KNOWN");
    end
  endtask

  task automatic test_midstream_reset;
    integer t;
    reg [7:0] rr,gg,bb;
    begin
      $display("[RUN ] MIDSTREAM RESET");
      warmup(8);

      for (t=0;t<60;t=t+1) begin
        rr = t[7:0];
        gg = (t*3);
        bb = 8'hFF - t[7:0];
        drive_one(30000+t, 1'b1, rr,gg,bb, t[7:0], 1'b1, 1'b1, 1'b1, 10'd16, 10'd900);
      end

      // reset asserted: expected queue should be cleared because pipeline flushes
      rst = 1'b1;
      step();
      step();
      rst = 1'b0;

      // IMPORTANT: clear scoreboard too (since DUT state cleared)
      q_reset();

      warmup(12);

      for (t=0;t<60;t=t+1) begin
        rr = (t+8);
        gg = (t*5);
        bb = (t*7);
        drive_one(31000+t, 1'b1, rr,gg,bb, (8'h80^t[7:0]), 1'b1, 1'b1, 1'b1, 10'd16, 10'd900);
      end

      drain(200);
      if (q_count != 0) begin
        $display("[TB] ERROR: queue not empty after RESET, remaining=%0d", q_count);
        $fatal;
      end
      $display("[PASS] MIDSTREAM RESET");
    end
  endtask

  // ============================================================
  // main
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_rgb2y_luma_top.vcd");
    $dumpvars(0, tb_rgb2y_luma_top);

    rst = 1'b1;

    en = 1'b1;
    pix_valid = 1'b0;
    rgb_in    = 36'h0;
    sb_in     = '0;

    out_ready = 1'b1;

    clip_en  = 1'b1;
    clip_min = 10'd16;
    clip_max = 10'd900;

    q_reset();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    // settle 1 cycle
    step();

    test_mix();
    test_known_vectors();
    test_midstream_reset();

    $display("[TB] PASS ALL. drop_cnt=%0d stall_cnt=%0d", drop_cnt, stall_cnt);
    $finish;
  end

endmodule

`default_nettype wire
