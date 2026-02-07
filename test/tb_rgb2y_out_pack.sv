// tb_rgb2y_out_pack.sv  (FIXED: push expected ONLY on real handshake)
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb2y_out_pack.vvp \
  ./test/tb_rgb2y_out_pack.sv

vvp ./vvp/tb_rgb2y_out_pack.vvp
*/

`include "./src/AMOLED/rgb2y_luma/rgb2y_out_pack.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb2y_out_pack;

  // -----------------------------
  // clock / reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // -----------------------------
  // DUT params
  // -----------------------------
  localparam int unsigned IN_W  = 12;
  localparam int unsigned OUT_W = 10;
  localparam int unsigned SB_W  = 8;
  localparam int unsigned LAT   = 2;

  localparam bit ZERO_WHEN_INVALID         = 1'b1;
  localparam bit GATE_SIDEBAND_WITH_VALID  = 1'b1;
  localparam bit USE_ROUND                 = 1'b0;
  localparam bit USE_SAT                   = 1'b1;
  localparam bit USE_CLIP                  = 1'b1;
  localparam bit COUNT_DBG                 = 1'b1;

  // -----------------------------
  // DUT I/O
  // -----------------------------
  logic en;
  logic bypass;

  logic                 in_valid;
  logic                 in_ready;
  logic [IN_W-1:0]       y_in;
  logic [SB_W-1:0]       sb_in;

  logic                 clip_en;
  logic [OUT_W-1:0]      clip_min;
  logic [OUT_W-1:0]      clip_max;

  logic                 out_valid;
  logic                 out_ready;
  logic [OUT_W-1:0]      Y;
  logic [SB_W-1:0]       sb_out;

  logic                 drop_pulse;
  logic                 stall_pulse;
  logic [31:0]           drop_cnt;
  logic [31:0]           stall_cnt;

  // -----------------------------
  // DUT
  // -----------------------------
  rgb2y_out_pack #(
    .IN_W(IN_W),
    .OUT_W(OUT_W),
    .SB_W(SB_W),
    .LAT(LAT),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .GATE_SIDEBAND_WITH_VALID(GATE_SIDEBAND_WITH_VALID),
    .USE_ROUND(USE_ROUND),
    .USE_SAT(USE_SAT),
    .USE_CLIP(USE_CLIP),
    .COUNT_DBG(COUNT_DBG)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),
    .bypass(bypass),

    .in_valid(in_valid),
    .in_ready(in_ready),
    .y_in(y_in),
    .sb_in(sb_in),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .Y(Y),
    .sb_out(sb_out),

    .drop_pulse(drop_pulse),
    .stall_pulse(stall_pulse),
    .drop_cnt(drop_cnt),
    .stall_cnt(stall_cnt)
  );

  // -----------------------------
  // reference (match current DUT config: unsigned, SAT on, CLIP optional)
  // -----------------------------
  function automatic [OUT_W-1:0] ref_pack(
  input logic [IN_W-1:0]  yin,
  input logic             bypass_i,
  input logic             clip_en_i,
  input logic [OUT_W-1:0] cmin,
  input logic [OUT_W-1:0] cmax
);
  logic [OUT_W-1:0] ytrunc;
  logic [OUT_W-1:0] ysat;
  logic [OUT_W-1:0] yclip;
  begin
    // bypass => pure truncation, no sat/clip
    if (bypass_i) begin
      ref_pack = yin[OUT_W-1:0];
    end else begin
      ytrunc = yin[OUT_W-1:0];

      // saturation (unsigned)
      if (IN_W > OUT_W) begin
        if (|yin[IN_W-1:OUT_W]) ysat = {OUT_W{1'b1}};
        else                    ysat = ytrunc;
      end else begin
        ysat = ytrunc;
      end

      // clip
      if (clip_en_i) begin
        if (ysat < cmin)      yclip = cmin;
        else if (ysat > cmax) yclip = cmax;
        else                  yclip = ysat;
      end else begin
        yclip = ysat;
      end

      ref_pack = yclip;
    end
  end
endfunction


  // -----------------------------
  // queue
  // -----------------------------
  localparam int QDEPTH = 256;

  logic [OUT_W-1:0] exp_y   [0:QDEPTH-1];
  logic [SB_W-1:0]  exp_sb  [0:QDEPTH-1];
  int unsigned q_w, q_r, q_count;

  task automatic q_reset();
    int k;
    begin
      q_w = 0; q_r = 0; q_count = 0;
      for (k=0; k<QDEPTH; k=k+1) begin
        exp_y[k]  = '0;
        exp_sb[k] = '0;
      end
    end
  endtask

  task automatic q_push(input logic [OUT_W-1:0] yv,
                        input logic [SB_W-1:0]  sbv);
    begin
      if (q_count >= QDEPTH) begin
        $display("[TB] ERROR: queue overflow");
        $fatal;
      end
      exp_y[q_w]  = yv;
      exp_sb[q_w] = sbv;
      q_w = (q_w + 1) % QDEPTH;
      q_count++;
    end
  endtask

  task automatic q_pop_check(input logic [OUT_W-1:0] yv,
                             input logic [SB_W-1:0]  sbv);
    begin
      if (q_count == 0) begin
        $display("[TB] ERROR: unexpected output (queue empty) y=%0d sb=0x%0h", yv, sbv);
        $fatal;
      end
      if (yv !== exp_y[q_r] || sbv !== exp_sb[q_r]) begin
        $display("[TB] ERROR: mismatch @%0t", $time);
        $display("  got  y=%0d sb=0x%0h", yv, sbv);
        $display("  exp  y=%0d sb=0x%0h", exp_y[q_r], exp_sb[q_r]);
        $fatal;
      end
      q_r = (q_r + 1) % QDEPTH;
      q_count--;
    end
  endtask

  // -----------------------------
  // step
  // -----------------------------
  task automatic step();
    begin
      @(posedge clk);
      #1;
    end
  endtask

  // -----------------------------
  // output hold check under stall
  // -----------------------------
  logic [OUT_W-1:0] Y_hold;
  logic [SB_W-1:0]  SB_hold;
  logic             hold_armed;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      Y_hold     <= '0;
      SB_hold    <= '0;
      hold_armed <= 1'b0;
    end else begin
      if (out_valid && !out_ready) begin
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

  // -----------------------------
  // FIX: push expected ONLY on real input handshake (posedge)
  // -----------------------------
  logic [OUT_W-1:0] yexp;
  logic [SB_W-1:0]  sbexp;

  always_ff @(posedge clk) begin
    if (!rst && en) begin
      if (in_valid && in_ready) begin
        yexp = ref_pack(y_in, bypass, clip_en, clip_min, clip_max);
        if (SB_W == 0) sbexp = '0;
        else begin
          if (GATE_SIDEBAND_WITH_VALID) sbexp = sb_in; // in_valid=1 so passes
          else                          sbexp = sb_in;
        end
        q_push(yexp, sbexp);
      end

      if (out_valid && out_ready) begin
        q_pop_check(Y, sb_out);
      end
    end
  end

  // -----------------------------
  // stimulus
  // -----------------------------
  int unsigned n;
  int unsigned seed = 32'h1234;

  initial begin
    rst = 1'b1;
    en  = 1'b0;
    bypass = 1'b0;

    in_valid = 1'b0;
    y_in     = '0;
    sb_in    = '0;

    clip_en  = 1'b1;
    clip_min = 10'd16;
    clip_max = 10'd900;

    out_ready = 1'b1;

    q_reset();

    repeat (5) step();
    rst = 1'b0;
    en  = 1'b1;

    // IMPORTANT: give 1 cycle to settle post-reset
    step();

    $display("[TB] Phase1: no stall, bypass=0 clip_en=1");
    for (n=0; n<40; n=n+1) begin
      in_valid = 1'b1;
      y_in     = $random(seed);
      sb_in    = n[SB_W-1:0];
      step();
    end
    in_valid = 1'b0;
    y_in     = '0;
    sb_in    = '0;
    repeat (80) step();

    if (q_count != 0) begin
      $display("[TB] ERROR: queue not empty after Phase1, remaining=%0d", q_count);
      $fatal;
    end

    $display("[TB] Phase2: downstream backpressure (toggle out_ready)");
    q_reset();

    for (n=0; n<120; n=n+1) begin
      if ((n % 9) == 0) out_ready = 1'b0;
      else if ((n % 9) == 3) out_ready = 1'b1;

      if (($random(seed) & 32'h3) != 0) begin
        in_valid = 1'b1;
        y_in     = $random(seed);
        sb_in    = (8'hA0 ^ n[7:0]);
      end else begin
        in_valid = 1'b0;
        y_in     = '0;
        sb_in    = '0;
      end

      step();
    end

    out_ready = 1'b1;
    in_valid  = 1'b0;
    repeat (160) step();

    if (q_count != 0) begin
      $display("[TB] ERROR: queue not empty after Phase2, remaining=%0d", q_count);
      $fatal;
    end

    $display("[TB] Phase3: bypass=1");
    q_reset();
    bypass = 1'b1;
    out_ready = 1'b1;

    for (n=0; n<40; n=n+1) begin
      in_valid = 1'b1;
      y_in     = $random(seed);
      sb_in    = n[SB_W-1:0];
      step();
    end
    in_valid = 1'b0;
    repeat (120) step();

    if (q_count != 0) begin
      $display("[TB] ERROR: queue not empty after Phase3, remaining=%0d", q_count);
      $fatal;
    end

    $display("[TB] Phase4: en=0 freeze check");
    q_reset();
    bypass = 1'b0;
    out_ready = 1'b1;

    // send a few
    for (n=0; n<10; n=n+1) begin
      in_valid = 1'b1;
      y_in     = $random(seed);
      sb_in    = (8'h55 + n[7:0]);
      step();
    end

    // freeze
    en = 1'b0;
    repeat (10) begin
      out_ready = ~out_ready;
      step();
    end

    en = 1'b1;
    out_ready = 1'b1;
    in_valid  = 1'b0;
    repeat (200) step();

    if (q_count != 0) begin
      $display("[TB] ERROR: queue not empty after Phase4, remaining=%0d", q_count);
      $fatal;
    end

    $display("[TB] PASS. drop_cnt=%0d stall_cnt=%0d", drop_cnt, stall_cnt);
    $finish;
  end

endmodule

`default_nettype wire
