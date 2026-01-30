// tb_linebuf_y_1line.sv
/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_linebuf_y_1line.vvp ./test/tb_linebuf_y_1line.sv
vvp ./vvp/tb_linebuf_y_1line.vvp
gtkwave ./vvp/tb_linebuf_y_1line.vcd
*/
`timescale 1ns/1ps
`default_nettype none

`include "./src/AMOLED/tile feature_extracter/linebuf_y_1line.sv"

module tb_linebuf_y_1line;

  localparam int unsigned X_W    = 6;
  localparam int unsigned Y_W    = 6;
  localparam int unsigned YPIX_W = 8;
  localparam int unsigned W      = 16;   // small
  localparam int unsigned H      = 6;

  logic clk=0; always #5 clk=~clk;
  logic rst, en;

  logic v1_valid;
  logic v1_ready;
  logic [X_W-1:0] x;
  logic [Y_W-1:0] y;
  logic [YPIX_W-1:0] y_in;

  logic v2_valid;
  logic v2_ready;
  logic [YPIX_W-1:0] y_cur, y_left, y_up;

  linebuf_y_1line #(
    .X_W(X_W), .Y_W(Y_W), .YPIX_W(YPIX_W), .DEPTH(W), .INIT_ZERO(1'b1)
  ) dut (
    .clk(clk), .rst(rst), .en(en),
    .v1_valid(v1_valid), .v1_ready(v1_ready),
    .x(x), .y(y), .y_in(y_in),
    .v2_valid(v2_valid), .v2_ready(v2_ready),
    .y_cur(y_cur), .y_left(y_left), .y_up(y_up)
  );

  task automatic step; begin @(posedge clk); #1; end endtask

  // golden model memory + left
  logic [YPIX_W-1:0] gmem [0:W-1];
  logic [YPIX_W-1:0] gleft;
  integer i;

  task automatic gold_reset;
    begin
      for (i=0;i<W;i=i+1) gmem[i]='0;
      gleft='0;
    end
  endtask

  // ready pattern
  int cyc;
  always_comb begin
    if ((cyc % 5) == 4) v2_ready = 1'b0;
    else                v2_ready = 1'b1;
  end

  // drive one pixel, hold until accepted (no break)
  task automatic send_pix(input int xx, input int yy, input byte val);
    bit accepted;
    int guard;
    logic [YPIX_W-1:0] exp_up, exp_left;
    begin
      v1_valid = 1'b1;
      x = xx[X_W-1:0];
      y = yy[Y_W-1:0];
      y_in = val;

      accepted = 0;
      guard = 0;

      while (!accepted) begin
        step();
        cyc++;

        // check output when valid+ready (consume)
        if (v2_valid && v2_ready) begin
          // compare against what was computed at accept time (stored in regs)
          // Here we validate invariants:
          // y_cur equals input val at that beat is checked via expected buffering below.
        end

        // did this beat accept at input?
        if (v1_valid && v1_ready) begin
          accepted = 1;

          // compute expected for THIS accepted beat
          exp_up   = gmem[xx];
          exp_left = (xx==0) ? val : gleft;

          // After accept, DUT will present outputs (registered) and v2_valid asserted.
          // We check immediately in next cycles when output fires.
          // Update golden state now:
          gmem[xx] = val;
          gleft    = val;

          // Wait until output actually fires to compare
          while (!(v2_valid && v2_ready)) begin
            step(); cyc++;
          end

          if (y_cur !== val) begin
            $display("[TB] y_cur mismatch got=%0d exp=%0d at (x,y)=(%0d,%0d)", y_cur, val, xx, yy);
            $fatal;
          end
          if (y_up !== exp_up) begin
            $display("[TB] y_up mismatch got=%0d exp=%0d at (x,y)=(%0d,%0d)", y_up, exp_up, xx, yy);
            $fatal;
          end
          if (y_left !== exp_left) begin
            $display("[TB] y_left mismatch got=%0d exp=%0d at (x,y)=(%0d,%0d)", y_left, exp_left, xx, yy);
            $fatal;
          end
        end

        guard++;
        if (guard > 2000) begin
          $fatal(1, "[TB] TIMEOUT waiting accept");
        end
      end

      // deassert
      v1_valid = 1'b0;
      step(); cyc++;
    end
  endtask

  integer xx, yy;

  initial begin
    $dumpfile("./vvp/tb_linebuf_y_1line.vcd");
    $dumpvars(0, tb_linebuf_y_1line);

    en=1;
    v1_valid=0; x='0; y='0; y_in='0;
    cyc=0;

    rst=1;
    repeat(5) step();
    rst=0;

    gold_reset();

    // run 2 lines to validate y_up becomes previous line
    for (yy=0; yy<2; yy=yy+1) begin
      // reset left at line start in golden too
      gleft = '0;
      for (xx=0; xx<W; xx=xx+1) begin
        send_pix(xx, yy, byte'(yy*40 + xx));
      end
    end

    $display("[PASS] linebuf_y_1line");
    $finish;
  end

endmodule

`default_nettype wire
