// tb_video_rgb2y_integrated_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_video_rgb2y_integrated_top.vvp \
  ./test/tb_video_rgb2y_integrated_top.sv

vvp ./vvp/tb_video_rgb2y_integrated_top.vvp
gtkwave ./vvp/tb_video_rgb2y_integrated_top.vcd
*/
`include "./src/AMOLED/video_rgb2y_integrated_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_video_rgb2y_integrated_top;

  // ============================================================
  // clock/reset
  // ============================================================
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  // ============================================================
  // params (縮小解析度加速)
  // ============================================================
  localparam int unsigned ACTIVE_W = 16;
  localparam int unsigned ACTIVE_H = 6;

  localparam int unsigned X_W = 11;
  localparam int unsigned Y_W = 10;

  // sb layout: {eof,eol,sol,sof, de,hs,vs, y, x}
  localparam int unsigned SB_W = (4+3+Y_W+X_W);

  // rgb2y out
  localparam int unsigned PACK_IN_W  = 10;
  localparam int unsigned PACK_OUT_W = 10;
  localparam int unsigned PACK_LAT   = 2;

  // ============================================================
  // DUT I/O
  // ============================================================
  logic        en;

  logic        vsync_i, hsync_i, de_i;
  logic [23:0] rgb_i;

  logic        clip_en;
  logic [PACK_OUT_W-1:0] clip_min, clip_max;

  logic        out_valid;
  logic        out_ready;
  logic [PACK_OUT_W-1:0] Y;
  logic [SB_W-1:0]       sb_out;

  logic        in_frame, in_line, pix_valid;
  logic [X_W-1:0] x;
  logic [Y_W-1:0] y;

  logic        skid_drop_pulse;
  logic [31:0] skid_drop_cnt;

  // ============================================================
  // DUT ✅
  // ============================================================
  video_rgb2y_integrated_top #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(ACTIVE_W),
    .ACTIVE_H(ACTIVE_H),

    .SB_W(SB_W),

    .PACK_IN_W(PACK_IN_W),
    .PACK_OUT_W(PACK_OUT_W),
    .PACK_LAT(PACK_LAT),
    .PACK_USE_SAT(1'b1),
    .PACK_USE_CLIP(1'b1)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),

    .vsync_i(vsync_i),
    .hsync_i(hsync_i),
    .de_i(de_i),
    .rgb_i(rgb_i),

    .clip_en(clip_en),
    .clip_min(clip_min),
    .clip_max(clip_max),

    .out_valid(out_valid),
    .out_ready(out_ready),
    .Y(Y),
    .sb_out(sb_out),

    .in_frame(in_frame),
    .in_line(in_line),
    .pix_valid(pix_valid),
    .x(x),
    .y(y),

    .skid_drop_pulse(skid_drop_pulse),
    .skid_drop_cnt(skid_drop_cnt)
  );

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
  // RGB->Y reference (77/150/29 + round>=0x80)
  // ============================================================
  function automatic [15:0] mul77 (input [7:0] v);  begin mul77  = v * 16'd77;  end endfunction
  function automatic [15:0] mul150(input [7:0] v);  begin mul150 = v * 16'd150; end endfunction
  function automatic [15:0] mul29 (input [7:0] v);  begin mul29  = v * 16'd29;  end endfunction

  function automatic [16:0] f_sum17(input [15:0] r, input [15:0] g, input [15:0] b);
    reg [16:0] s;
    begin
      s = {1'b0,r} + {1'b0,g} + {1'b0,b};
      f_sum17 = s;
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
      inc  = (r >= 8'h80);
      qinc = {1'b0,q} + {10'd0,inc};
      if (qinc[10]) f_y10_round = 10'h3FF;
      else         f_y10_round = qinc[9:0];
    end
  endfunction

  function automatic [PACK_OUT_W-1:0] ref_pack_y(
    input logic [PACK_IN_W-1:0]  yin,
    input logic                  clip_en_i,
    input logic [PACK_OUT_W-1:0]  cmin,
    input logic [PACK_OUT_W-1:0]  cmax
  );
    logic [PACK_OUT_W-1:0] ysat;
    begin
      ysat = yin[PACK_OUT_W-1:0]; // IN_W==OUT_W => pass
      if (clip_en_i) begin
        if (ysat < cmin)      ref_pack_y = cmin;
        else if (ysat > cmax) ref_pack_y = cmax;
        else                  ref_pack_y = ysat;
      end else begin
        ref_pack_y = ysat;
      end
    end
  endfunction

  // ============================================================
  // Scoreboard FIFO (push on internal input handshake, pop on output handshake)
  // ============================================================
  localparam int QDEPTH = 8192;
  logic [PACK_OUT_W-1:0] exp_y  [0:QDEPTH-1];
  logic [SB_W-1:0]       exp_sb [0:QDEPTH-1];
  int q_w, q_r, q_count;

  task automatic q_reset;
    int k;
    begin
      q_w=0; q_r=0; q_count=0;
      for (k=0;k<QDEPTH;k=k+1) begin
        exp_y[k]='0;
        exp_sb[k]='0;
      end
    end
  endtask

  task automatic q_push(input logic [PACK_OUT_W-1:0] yv, input logic [SB_W-1:0] sbv);
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

  task automatic q_pop_check(input logic [PACK_OUT_W-1:0] yv, input logic [SB_W-1:0] sbv);
    begin
      if (q_count == 0) begin
        $display("[TB] ERROR: unexpected output while queue empty y=%0d sb=0x%0h", yv, sbv);
        $fatal;
      end
      if (yv !== exp_y[q_r] || sbv !== exp_sb[q_r]) begin
        $display("[TB] ERROR: mismatch @t=%0t", $time);
        $display("  got y=%0d sb=0x%0h", yv, sbv);
        $display("  exp y=%0d sb=0x%0h", exp_y[q_r], exp_sb[q_r]);
        $fatal;
      end
      q_r = (q_r + 1) % QDEPTH;
      q_count = q_count - 1;
    end
  endtask

  // ============================================================
  // output hold check during stall (AXIS rule)
  // ============================================================
  logic [PACK_OUT_W-1:0] Y_hold;
  logic [SB_W-1:0]       SB_hold;
  logic                  hold_armed;

  always_ff @(posedge clk) begin
    if (rst) begin
      Y_hold     <= '0;
      SB_hold    <= '0;
      hold_armed <= 1'b0;
    end else if (en) begin
      if (out_valid && !out_ready) begin
        if (!hold_armed) begin
          Y_hold     <= Y;
          SB_hold    <= sb_out;
          hold_armed <= 1'b1;
        end else begin
          if (Y !== Y_hold || sb_out !== SB_hold) begin
            $display("[TB] ERROR: output changed while stalled @t=%0t", $time);
            $fatal;
          end
        end
      end else begin
        hold_armed <= 1'b0;
      end
    end
  end

  // ============================================================
  // drive raw 1 cycle + scoreboard
  // ============================================================
  task automatic drive_raw(
    input bit          v,
    input bit          h,
    input bit          d,
    input logic [23:0]  rgb
  );
    logic in_hs;
    logic [7:0] rr,gg,bb;
    logic [15:0] rt,gt,bt;
    logic [16:0] s17;
    logic [9:0]  y10e;
    logic [PACK_OUT_W-1:0] yout_e;
    logic [SB_W-1:0] sb_e;
    begin
      vsync_i <= v;
      hsync_i <= h;
      de_i    <= d;
      rgb_i   <= rgb;

      step();

      // push when rgb2y truly accepted an item
      in_hs = en && dut.skid_out_valid && dut.rgb2y_in_ready;
      if (in_hs) begin
        rr = dut.rgb2y_rgb_in[23:16];
        gg = dut.rgb2y_rgb_in[15:8];
        bb = dut.rgb2y_rgb_in[7:0];

        rt  = mul77(rr);
        gt  = mul150(gg);
        bt  = mul29(bb);
        s17 = f_sum17(rt,gt,bt);

        y10e   = f_y10_round(s17);
        yout_e = ref_pack_y(y10e[PACK_IN_W-1:0], clip_en, clip_min, clip_max);

        sb_e = dut.rgb2y_sb_in;
        q_push(yout_e, sb_e);
      end

      // pop/check on output handshake
      if (en && out_valid && out_ready) begin
        q_pop_check(Y, sb_out);
      end
    end
  endtask

  // ============================================================
  // pattern generators
  // ============================================================
  task automatic vsync_pulse;
    begin
      drive_raw(1'b1, 1'b0, 1'b0, 24'h0);
      drive_raw(1'b0, 1'b0, 1'b0, 24'h0);
    end
  endtask

  task automatic hsync_pulse;
    begin
      drive_raw(vsync_i, 1'b1, 1'b0, 24'h0);
      drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end
  endtask

  task automatic drive_active_line(input int unsigned w, input int unsigned row_id);
    int unsigned i;
    logic [23:0] rgbp;
    begin
      hsync_pulse();
      for (i=0; i<w; i=i+1) begin
        rgbp = {row_id[7:0], i[7:0], (row_id[7:0]^i[7:0])};
        drive_raw(vsync_i, 1'b0, 1'b1, rgbp);
      end
      for (i=0; i<4; i=i+1) drive_raw(vsync_i, 1'b0, 1'b0, 24'h0);
    end
  endtask

  task automatic drive_blank(input int unsigned n);
    int unsigned i;
    begin
      for (i=0; i<n; i=i+1) drive_raw(vsync_i, hsync_i, de_i, rgb_i);
    end
  endtask

  // ============================================================
  // Test: one frame + burst backpressure
  // ============================================================
  task automatic run_one_frame_with_backpressure;
    int unsigned r;
    begin
      $display("[RUN ] one frame + burst backpressure");

      // frame start
      vsync_pulse();

      for (r=0; r<ACTIVE_H; r=r+1) begin
        out_ready = 1'b1;
        drive_blank(3);

        // short stall (1-deep skid should absorb)
        out_ready = 1'b0;
        drive_blank(2);

        out_ready = 1'b1;
        drive_active_line(ACTIVE_W, r);
      end

      drive_blank(40);
      vsync_pulse();

      out_ready = 1'b1;
      drive_blank(200);

      $display("[INFO] q_count=%0d skid_drop_cnt=%0d", q_count, skid_drop_cnt);

      if (q_count != 0) begin
        $display("[TB] ERROR: queue not empty after drain, remain=%0d", q_count);
        $fatal;
      end
      if (skid_drop_cnt != 0) begin
        $display("[TB] ERROR: skid dropped data unexpectedly! drop_cnt=%0d", skid_drop_cnt);
        $fatal;
      end

      $display("[PASS] one frame + backpressure");
    end
  endtask

  // ============================================================
  // main
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_video_rgb2y_integrated_top.vcd");
    $dumpvars(0, tb_video_rgb2y_integrated_top);

    en        = 1'b1;
    out_ready = 1'b1;

    vsync_i   = 1'b0;
    hsync_i   = 1'b0;
    de_i      = 1'b0;
    rgb_i     = 24'h0;

    clip_en   = 1'b1;
    clip_min  = 10'd16;
    clip_max  = 10'd900;

    q_reset();

    rst = 1'b1;
    repeat (10) @(posedge clk);
    rst = 1'b0;

    drive_blank(10);
    run_one_frame_with_backpressure();

    $display("[TB] PASS ALL");
    $finish;
  end

endmodule

`default_nettype wire
