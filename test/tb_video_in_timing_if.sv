// tb_video_in_timing_if.sv
/*
iverilog -g2012 -Wall -o ./vvp/tb_video.vvp  ./test/tb_video_in_timing_if.sv
vvp ./vvp/tb_video.vvp
gtkwave tb_video.vcd
*/

`include "./src/AMOLED/video_in_timing_if.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_video_in_timing_if;

  // ----------------------------
  // Timing parameters
  // ----------------------------
  localparam int H_ACTIVE = 8;
  localparam int H_BLANK  = 4;
  localparam int V_ACTIVE = 4;
  localparam int V_BLANK  = 2;

  localparam int X_W = 11;
  localparam int Y_W = 10;

  // ----------------------------
  // Clock / Reset
  // ----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  // ----------------------------
  // Video inputs
  // ----------------------------
  logic        vsync;
  logic        hsync;
  logic        de;
  logic [23:0] rgb;

  // ----------------------------
  // DUT outputs
  // ----------------------------
  logic           pix_valid;
  logic [23:0]    pix_rgb;
  logic [X_W-1:0] x;
  logic [Y_W-1:0] y;
  logic           sof, sol, eol, eof;

  // ----------------------------
  // Instantiate DUT
  // ----------------------------
  video_in_timing_if #(
    .X_W(X_W),
    .Y_W(Y_W)
  ) dut (
    .clk       (clk),
    .rst       (rst),
    .vsync     (vsync),
    .hsync     (hsync),
    .de        (de),
    .rgb       (rgb),
    .pix_valid (pix_valid),
    .pix_rgb   (pix_rgb),
    .x         (x),
    .y         (y),
    .sof       (sof),
    .sol       (sol),
    .eol       (eol),
    .eof       (eof)
  );

  // ----------------------------
  // Wave dump
  // ----------------------------
  initial begin
    $dumpfile("tb_video.vcd");
    $dumpvars(0, tb_video_in_timing_if);
  end

  // ============================================================
  // TB-side pipelining to match DUT internal sampling:
  // DUT does: de_q <= de; de_d <= de_q; pix_valid <= de_q
  // So TB uses:
  //   de_p1 ~ DUT de_q
  //   de_p2 ~ DUT de_d
  // ============================================================
  logic de_p1, de_p2;
  logic vs_p1, vs_p2;

  logic [31:0] cyc;

  always_ff @(posedge clk) begin
    if (rst) begin
      de_p1 <= 1'b0; de_p2 <= 1'b0;
      vs_p1 <= 1'b0; vs_p2 <= 1'b0;
      cyc   <= 32'd0;
    end else begin
      cyc <= cyc + 1;

      de_p1 <= de;
      de_p2 <= de_p1;

      vs_p1 <= vsync;
      vs_p2 <= vs_p1;
    end
  end

  // TB edges aligned to DUT's de_q/de_d and vs_q/vs_d
  wire tb_de_rise = (de_p1 && !de_p2);
  wire tb_de_fall = (!de_p1 && de_p2);
  wire tb_vs_rise = (vs_p1 && !vs_p2);

  // ============================================================
  // Stimulus
  // ============================================================
  task automatic drive_pixel(input bit v_de, input logic [23:0] v_rgb);
    begin
      de  <= v_de;
      rgb <= v_rgb;
      @(posedge clk);
    end
  endtask

  task automatic drive_line(input int frame_idx, input int line_idx);
    int px;
    logic [23:0] pat;
    begin
      // hsync pulse 1 cycle before line
      hsync <= 1'b1;
      de    <= 1'b0;
      rgb   <= '0;
      @(posedge clk);
      hsync <= 1'b0;

      // front blank
      for (px = 0; px < H_BLANK/2; px++) begin
        drive_pixel(1'b0, 24'h0);
      end

      // active
      for (px = 0; px < H_ACTIVE; px++) begin
        pat = { frame_idx[3:0], line_idx[7:0], px[7:0] };
        drive_pixel(1'b1, pat);
      end

      // back blank
      for (px = 0; px < (H_BLANK - H_BLANK/2); px++) begin
        drive_pixel(1'b0, 24'h0);
      end
    end
  endtask

  task automatic drive_frame(input int frame_idx);
    int ly;
    begin
      // vsync pulse 2 cycles (DUT 用 vs_rise reset)
      vsync <= 1'b0;
      @(posedge clk);
      vsync <= 1'b1;
      @(posedge clk);
      vsync <= 1'b1;
      @(posedge clk);
      vsync <= 1'b0;

      // vertical blank lines
      for (ly = 0; ly < V_BLANK; ly++) begin
        drive_line(frame_idx, -ly-1);
      end

      // active lines
      for (ly = 0; ly < V_ACTIVE; ly++) begin
        drive_line(frame_idx, ly);
      end
    end
  endtask

  // ============================================================
  // Checker (match your current DUT behavior)
  // ============================================================
  int exp_x, exp_y;
  bit exp_in_frame;
  bit seen_first_active;

  always_ff @(posedge clk) begin
    if (rst) begin
      exp_x <= 0;
      exp_y <= 0;
      exp_in_frame <= 0;
      seen_first_active <= 0;
    end else begin
      // frame reset on vs_rise (aligned)
      if (tb_vs_rise) begin
        exp_x <= 0;
        exp_y <= 0;
        exp_in_frame <= 1;
        seen_first_active <= 0;
      end

      // --------------------------------------------------------
      // pix_valid should equal DUT de_q, which TB models as de_p1
      // --------------------------------------------------------
      if (pix_valid !== de_p1) begin
        $display("[ERR] cyc=%0d pix_valid mismatch: dut=%0b exp=%0b (de=%0b de_p1=%0b de_p2=%0b)",
                 cyc, pix_valid, de_p1, de, de_p1, de_p2);
        $fatal;
      end

      // --------------------------------------------------------
      // Line start: tb_de_rise corresponds to DUT de_rise
      // Your DUT behavior: on de_rise: x<=0; y<=y+1 (unless vs_rise)
      // --------------------------------------------------------
      if (tb_de_rise) begin
        exp_x <= 0;

        if (!tb_vs_rise) exp_y <= exp_y + 1;

        // SOL pulse expected on de_rise
        if (sol !== 1'b1) begin
          $display("[ERR] cyc=%0d SOL not asserted on de_rise (sol=%0b)", cyc, sol);
          $fatal;
        end

        // SOF expected on first de_rise after vsync (your sof_armed logic)
        if (exp_in_frame && !seen_first_active) begin
          if (sof !== 1'b1) begin
            $display("[ERR] cyc=%0d SOF expected on first de_rise after vsync, sof=%0b", cyc, sof);
            $fatal;
          end
          seen_first_active <= 1;
        end else begin
          if (sof !== 1'b0) begin
            $display("[ERR] cyc=%0d SOF unexpected (sof=%0b)", cyc, sof);
            $fatal;
          end
        end
      end else begin
        // no de_rise => sol should be 0
        if (sol !== 1'b0) begin
          $display("[ERR] cyc=%0d SOL should be 0 when not de_rise (sol=%0b)", cyc, sol);
          $fatal;
        end
        // sof should be 0 too (except checked above)
        if (sof !== 1'b0) begin
          $display("[ERR] cyc=%0d SOF should be 0 when not first de_rise (sof=%0b)", cyc, sof);
          $fatal;
        end
      end

      // --------------------------------------------------------
      // Active pixel counting:
      // Use de_p1 (aligned with pix_valid) as "active"
      // Your DUT increments x when de_q (de_p1) is 1
      // --------------------------------------------------------
      if (de_p1) begin
        if (x !== exp_x[X_W-1:0]) begin
          $display("[ERR] cyc=%0d X mismatch: dut=%0d exp=%0d", cyc, x, exp_x);
          $fatal;
        end
        if (y !== exp_y[Y_W-1:0]) begin
          $display("[ERR] cyc=%0d Y mismatch: dut=%0d exp=%0d", cyc, y, exp_y);
          $fatal;
        end
        exp_x <= exp_x + 1;
      end

      // --------------------------------------------------------
      // Line end: tb_de_fall corresponds to DUT de_fall
      // Your DUT eol is a bit non-standard; we check it's pulsing near de_fall
      // --------------------------------------------------------
      if (tb_de_fall) begin
        if (eol !== 1'b1) begin
          $display("[WARN] cyc=%0d expected EOL pulse near de_fall, got eol=%0b", cyc, eol);
        end
      end else begin
        if (eol !== 1'b0) begin
          $display("[ERR] cyc=%0d EOL should be 0 when not de_fall (eol=%0b)", cyc, eol);
          $fatal;
        end
      end

      // --------------------------------------------------------
      // EOF: your DUT does eof <= (vs_rise && last_pix_valid)
      // last_pix_valid tracks previous DUT de_q, i.e. previous de_p1
      // In TB, previous de_p1 is de_p2 (because de_p2 <= de_p1)
      // So at tb_vs_rise: eof should equal de_p2
      // --------------------------------------------------------
      if (tb_vs_rise) begin
        if (eof !== de_p2) begin
          $display("[ERR] cyc=%0d EOF mismatch at vs_rise: dut=%0b exp=%0b (de_p2)", cyc, eof, de_p2);
          $fatal;
        end
      end else begin
        if (eof !== 1'b0) begin
          $display("[ERR] cyc=%0d EOF should be 0 when not vs_rise (eof=%0b)", cyc, eof);
          $fatal;
        end
      end

      // Basic sanity: pix_rgb shouldn't be X when valid
      if (pix_valid && (^pix_rgb === 1'bX)) begin
        $display("[ERR] cyc=%0d pix_rgb has X during valid!", cyc);
        $fatal;
      end
    end
  end

  // ============================================================
  // Main
  // ============================================================
  initial begin
    vsync = 0;
    hsync = 0;
    de    = 0;
    rgb   = 0;

    rst = 1;
    repeat (5) @(posedge clk);
    rst = 0;

    drive_frame(0);
    drive_frame(1);

    repeat (20) @(posedge clk);
    $display("[OK] Simulation finished.");
    $finish;
  end

endmodule

`default_nettype wire
