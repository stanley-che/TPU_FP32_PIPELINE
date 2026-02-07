// tb_linebuf_y_1line.sv  (ready/valid aware)
// ------------------------------------------------------------
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_linebuf_y_1line.vvp ./test/tb_linebuf_y_1line.sv
// Run :
// vvp ./vvp/tb_linebuf_y_1line.vvp
// Wave:
// gtkwave ./vvp/tb_linebuf_y_1line.vcd
// ------------------------------------------------------------
`include "./src/AMOLED/tile feature_extracter/linebuf_y_1line.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_linebuf_y_1line;

  // -----------------------------
  // params
  // -----------------------------
  localparam int unsigned X_W   = 6;
  localparam int unsigned Y_W   = 8;
  localparam int unsigned W     = 16;
  localparam int unsigned LINES = 2;

  // -----------------------------
  // clock/reset
  // -----------------------------
  logic clk = 1'b0; always #5 clk = ~clk;
  logic rst, en;

  // -----------------------------
  // DUT I/O (ready/valid)
  // -----------------------------
  logic                 v1_valid;
  logic                 v1_ready;
  logic [X_W-1:0]        x;
  logic [Y_W-1:0]        y_in;

  logic                 v2_valid;
  logic                 v2_ready;
  logic [Y_W-1:0]        y_cur, y_left, y_up;

  linebuf_y_1line #(
    .X_W(X_W),
    .Y_W(Y_W),
    .ACTIVE_W(W)
  ) dut (
    .clk(clk), .rst(rst), .en(en),

    .v1_valid(v1_valid),
    .v1_ready(v1_ready),
    .x(x),
    .y_in(y_in),

    .v2_valid(v2_valid),
    .v2_ready(v2_ready),
    .y_cur(y_cur),
    .y_left(y_left),
    .y_up(y_up)
  );

  task automatic step; begin @(posedge clk); #1; end endtask

  // -----------------------------
  // downstream ready pattern (create backpressure)
  // -----------------------------
  int cyc;
  always_comb begin
    // stall every 5th cycle
    if ((cyc % 5) == 4) v2_ready = 1'b0;
    else                v2_ready = 1'b1;
  end

  // -----------------------------
  // Golden model (updated ONLY on accept at input)
  // -----------------------------
  logic [Y_W-1:0] gmem [0:W-1];
  logic [Y_W-1:0] gleft;
  integer i;

  task automatic gold_reset;
    begin
      for (i=0; i<W; i=i+1) gmem[i] = '0;
      gleft = '0;
    end
  endtask

  // -----------------------------
  // Scoreboard queue (store expected output per accepted input beat)
  // -----------------------------
  localparam int unsigned QDEPTH = 1024;
  logic [Y_W-1:0] q_exp_cur [0:QDEPTH-1];
  logic [Y_W-1:0] q_exp_left[0:QDEPTH-1];
  logic [Y_W-1:0] q_exp_up  [0:QDEPTH-1];
  int q_wptr, q_rptr;

  task automatic q_reset;
    begin
      q_wptr = 0;
      q_rptr = 0;
    end
  endtask

  task automatic q_push(input logic [Y_W-1:0] ec,
                        input logic [Y_W-1:0] el,
                        input logic [Y_W-1:0] eu);
    begin
      if (q_wptr >= QDEPTH) $fatal(1, "[TB] queue overflow");
      q_exp_cur [q_wptr] = ec;
      q_exp_left[q_wptr] = el;
      q_exp_up  [q_wptr] = eu;
      q_wptr++;
    end
  endtask

  task automatic q_pop(output logic [Y_W-1:0] ec,
                       output logic [Y_W-1:0] el,
                       output logic [Y_W-1:0] eu);
    begin
      if (q_rptr >= q_wptr) $fatal(1, "[TB] queue underflow");
      ec = q_exp_cur [q_rptr];
      el = q_exp_left[q_rptr];
      eu = q_exp_up  [q_rptr];
      q_rptr++;
    end
  endtask

  // -----------------------------
  // Helper: drive one pixel with proper input handshake
  // - hold v1_valid until v1_ready
  // - compute expected based on golden BEFORE update
  // - push expected into queue at acceptance
  // - update golden at acceptance (read-before-write model)
  // -----------------------------
  task automatic send_pix(input int xx, input byte val);
  logic [Y_W-1:0] exp_cur, exp_left, exp_up;
  int guard;
  bit accepted;
begin
  // drive input
  v1_valid = 1'b1;
  x        = xx[X_W-1:0];
  y_in     = val[Y_W-1:0];

  guard    = 0;
  accepted = 1'b0;

  // wait until input handshake happens
  while (!accepted) begin
    step();
    guard++;

    if (guard > 200) begin
      $fatal(1, "[TB] TIMEOUT waiting v1_ready (xx=%0d)", xx);
    end

    if (v1_valid && v1_ready) begin
      // ---------- accepted this cycle ----------
      accepted = 1'b1;

      // expected values based on golden BEFORE update
      exp_cur  = val[Y_W-1:0];
      exp_up   = gmem[xx];
      exp_left = (xx==0) ? val[Y_W-1:0] : gleft;

      // enqueue expected output (may be stalled later)
      q_push(exp_cur, exp_left, exp_up);

      // update golden AFTER enqueue (read-before-write)
      gmem[xx] = val[Y_W-1:0];
      gleft    = val[Y_W-1:0];

      // deassert valid AFTER accept
      v1_valid = 1'b0;
    end
  end
end
endtask


  // -----------------------------
  // Monitor output handshake
  // - only check when v2_valid && v2_ready (consumed)
  // - also check "hold under stall": if v2_valid && !v2_ready, outputs must not change
  // -----------------------------
  logic [Y_W-1:0] hold_cur, hold_left, hold_up;
  logic           hold_active;

  always_ff @(posedge clk) begin
    if (rst || !en) begin
      hold_active <= 1'b0;
      hold_cur    <= '0;
      hold_left   <= '0;
      hold_up     <= '0;
    end else begin
      // when output becomes valid and downstream NOT ready: start/continue holding check
      if (v2_valid && !v2_ready) begin
        if (!hold_active) begin
          hold_active <= 1'b1;
          hold_cur    <= y_cur;
          hold_left   <= y_left;
          hold_up     <= y_up;
        end else begin
          if (y_cur  !== hold_cur)  $fatal(1, "[TB] HOLD FAIL: y_cur changed under stall");
          if (y_left !== hold_left) $fatal(1, "[TB] HOLD FAIL: y_left changed under stall");
          if (y_up   !== hold_up)   $fatal(1, "[TB] HOLD FAIL: y_up changed under stall");
        end
      end

      // when downstream consumes: compare against queue, and clear hold
      if (v2_valid && v2_ready) begin
        logic [Y_W-1:0] ec, el, eu;
        q_pop(ec, el, eu);

        if (y_cur !== ec) begin
          $display("[TB] y_cur mismatch t=%0t got=%0d exp=%0d", $time, y_cur, ec);
          $fatal(1, "FAIL");
        end
        if (y_left !== el) begin
          $display("[TB] y_left mismatch t=%0t got=%0d exp=%0d", $time, y_left, el);
          $fatal(1, "FAIL");
        end
        if (y_up !== eu) begin
          $display("[TB] y_up mismatch t=%0t got=%0d exp=%0d", $time, y_up, eu);
          $fatal(1, "FAIL");
        end

        hold_active <= 1'b0;
      end
    end
  end

  // -----------------------------
  // main stimulus
  // -----------------------------
  integer xx, yy;

  initial begin
    $dumpfile("./vvp/tb_linebuf_y_1line.vcd");
    $dumpvars(0, tb_linebuf_y_1line);

    // init
    en       = 1'b1;
    rst      = 1'b1;
    v1_valid = 1'b0;
    x        = '0;
    y_in     = '0;
    cyc      = 0;

    gold_reset();
    q_reset();

    // reset
    repeat (5) begin
      step();
      cyc++;
    end
    rst = 1'b0;

    // run multiple lines
    for (yy=0; yy<LINES; yy=yy+1) begin
      // reset golden left at start of each line (this matches your original TB assumption)
      gleft = '0;
      for (xx=0; xx<W; xx=xx+1) begin
        send_pix(xx, byte'(yy*40 + xx));
      end
    end

    // drain any pending outputs
    begin : DRAIN
      int guard;
      guard = 0;
      while (q_rptr != q_wptr) begin
        step(); cyc++; guard++;
        if (guard > 2000) $fatal(1, "[TB] TIMEOUT draining outputs q_rptr=%0d q_wptr=%0d", q_rptr, q_wptr);
      end
    end

    $display("[PASS] tb_linebuf_y_1line (ready/valid)");
    $finish;
  end

  // advance cyc counter
  always_ff @(posedge clk) begin
    if (!rst) cyc <= cyc + 1;
  end

endmodule

`default_nettype wire
