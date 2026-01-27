// tb_frame_event_gen.sv
/*
iverilog -g2012 -Wall -o ./vvp/tb_frame_event_gen.vvp ./test/tb_frame_event_gen.sv

vvp ./vvp/tb_frame_event_gen.vvp
gtkwave ./vvp/tb_frame_event_gen.vcd
*/
`include"./src/AMOLED/video_in_timing_if/frame_event_gen.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_frame_event_gen;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz

  logic rst;

  // ------------------------------------------------------------
  // inputs to DUT
  // ------------------------------------------------------------
  logic        pix_valid;
  logic [23:0] pix_rgb_in;
  logic [10:0] x;
  logic [9:0]  y;

  logic vs_rise, vs_fall, de_rise, de_fall;
  logic in_frame;

  // ------------------------------------------------------------
  // DUT0: eof inferred by next vs_rise (USE_VS_FALL=0)
  // ------------------------------------------------------------
  logic        sof0, sol0, eol0, eof0;
  logic        pix_valid_o0;
  logic [23:0] pix_rgb_out0;
  logic [10:0] x_o0;
  logic [9:0]  y_o0;

  frame_event_gen #(
    .USE_VS_FALL(1'b0),
    .GATE_BY_FRAME(1'b1)
  ) dut0 (
    .clk(clk),
    .rst(rst),
    .pix_valid(pix_valid),
    .pix_rgb_in(pix_rgb_in),
    .x(x),
    .y(y),
    .vs_rise(vs_rise),
    .vs_fall(vs_fall),   // ignored in this mode
    .de_rise(de_rise),
    .de_fall(de_fall),
    .in_frame(in_frame),
    .sof(sof0),
    .sol(sol0),
    .eol(eol0),
    .eof(eof0),
    .pix_valid_o(pix_valid_o0),
    .pix_rgb_out(pix_rgb_out0),
    .x_o(x_o0),
    .y_o(y_o0)
  );

  // ------------------------------------------------------------
  // DUT1: eof = vs_fall (USE_VS_FALL=1)
  // ------------------------------------------------------------
  logic        sof1, sol1, eol1, eof1;
  logic        pix_valid_o1;
  logic [23:0] pix_rgb_out1;
  logic [10:0] x_o1;
  logic [9:0]  y_o1;

  frame_event_gen #(
    .USE_VS_FALL(1'b1),
    .GATE_BY_FRAME(1'b1)
  ) dut1 (
    .clk(clk),
    .rst(rst),
    .pix_valid(pix_valid),
    .pix_rgb_in(pix_rgb_in),
    .x(x),
    .y(y),
    .vs_rise(vs_rise),
    .vs_fall(vs_fall),
    .de_rise(de_rise),
    .de_fall(de_fall),
    .in_frame(in_frame),
    .sof(sof1),
    .sol(sol1),
    .eol(eol1),
    .eof(eof1),
    .pix_valid_o(pix_valid_o1),
    .pix_rgb_out(pix_rgb_out1),
    .x_o(x_o1),
    .y_o(y_o1)
  );

  // ------------------------------------------------------------
  // TB helpers
  // ------------------------------------------------------------
  int unsigned err_cnt = 0;

  task automatic tb_err(input string msg);
    begin
      $display("[ERR] t=%0t %s", $time, msg);
      err_cnt++;
    end
  endtask

  task automatic tb_ok(input string msg);
    begin
      $display("[ OK ] t=%0t %s", $time, msg);
    end
  endtask

  task automatic chk_pulse(input string tag, input logic got, input logic exp);
    begin
      if (got !== exp) tb_err($sformatf("%s got=%0b exp=%0b", tag, got, exp));
    end
  endtask

  task automatic chk_bus_eq24(input string tag, input logic [23:0] got, input logic [23:0] exp);
    begin
      if (got !== exp) tb_err($sformatf("%s got=0x%06x exp=0x%06x", tag, got, exp));
    end
  endtask

  task automatic chk_bus_eq11(input string tag, input logic [10:0] got, input logic [10:0] exp);
    begin
      if (got !== exp) tb_err($sformatf("%s got=%0d exp=%0d", tag, got, exp));
    end
  endtask

  task automatic chk_bus_eq10(input string tag, input logic [9:0] got, input logic [9:0] exp);
    begin
      if (got !== exp) tb_err($sformatf("%s got=%0d exp=%0d", tag, got, exp));
    end
  endtask

  // Drive one cycle; pulses are sampled/registered on posedge
  task automatic drive_cycle(
    input logic        pv,
    input logic [23:0] rgb,
    input logic [10:0] xi,
    input logic [9:0]  yi,
    input logic        v_rise,
    input logic        v_fall,
    input logic        d_rise,
    input logic        d_fall,
    input logic        inF
  );
    begin
      pix_valid  <= pv;
      pix_rgb_in <= rgb;
      x          <= xi;
      y          <= yi;
      vs_rise    <= v_rise;
      vs_fall    <= v_fall;
      de_rise    <= d_rise;
      de_fall    <= d_fall;
      in_frame   <= inF;
      @(posedge clk);
      #1;
    end
  endtask

  task automatic idle_cycles(input int unsigned n);
    int unsigned i;
    begin
      for (i=0; i<n; i++) begin
        drive_cycle(1'b0, 24'h0, '0, '0, 1'b0, 1'b0, 1'b0, 1'b0, in_frame);
      end
    end
  endtask

  // ------------------------------------------------------------
  // Testcases
  // ------------------------------------------------------------

  // 1) sof must follow vs_rise (both DUTs)
  task automatic tc_sof_vs_rise;
    begin
      $display("\n--- TC1: sof = vs_rise ---");

      // hold low
      drive_cycle(0, 24'h0, 0, 0, 0, 0, 0, 0, 0);
      chk_pulse("DUT0 sof idle", sof0, 0);
      chk_pulse("DUT1 sof idle", sof1, 0);

      // vs_rise pulse
      drive_cycle(0, 24'h0, 0, 0, 1, 0, 0, 0, 0);
      chk_pulse("DUT0 sof on vs_rise", sof0, 1);
      chk_pulse("DUT1 sof on vs_rise", sof1, 1);

      // back to 0
      drive_cycle(0, 24'h0, 0, 0, 0, 0, 0, 0, 0);
      chk_pulse("DUT0 sof back", sof0, 0);
      chk_pulse("DUT1 sof back", sof1, 0);

      tb_ok("TC1 done");
    end
  endtask

  // 2) sol/eol must be de_rise/de_fall AND gated by in_frame (GATE_BY_FRAME=1)
  task automatic tc_sol_eol_gated;
    begin
      $display("\n--- TC2: sol/eol = de_rise/de_fall gated by in_frame ---");

      // de_rise while out of frame => should be 0
      drive_cycle(0, 24'h0, 0, 0, 0, 0, 1, 0, 0);
      chk_pulse("DUT0 sol gated off", sol0, 0);
      chk_pulse("DUT1 sol gated off", sol1, 0);

      // enter frame
      drive_cycle(0, 24'h0, 0, 0, 1, 0, 0, 0, 1); // sof too, ok
      // de_rise inside frame => sol=1
      drive_cycle(0, 24'h0, 0, 0, 0, 0, 1, 0, 1);
      chk_pulse("DUT0 sol on", sol0, 1);
      chk_pulse("DUT1 sol on", sol1, 1);

      // de_fall inside frame => eol=1
      drive_cycle(0, 24'h0, 0, 0, 0, 0, 0, 1, 1);
      chk_pulse("DUT0 eol on", eol0, 1);
      chk_pulse("DUT1 eol on", eol1, 1);

      // out of frame again
      drive_cycle(0, 24'h0, 0, 0, 0, 0, 0, 0, 0);
      tb_ok("TC2 done");
    end
  endtask

  // 3) eof behavior difference
  //    DUT0 (no vs_fall): eof asserted when vs_rise arrives AND previous frame was seen (in_frame_seen=1)
  //    DUT1 (vs_fall):    eof asserted on vs_fall
  task automatic tc_eof_modes;
    begin
      $display("\n--- TC3: eof generation (mode compare) ---");

      // reset to known state: start frame once so in_frame_seen becomes 1
      drive_cycle(0, 24'h0, 0, 0, 1, 0, 0, 0, 1); // vs_rise
      chk_pulse("DUT0 sof", sof0, 1);
      chk_pulse("DUT1 sof", sof1, 1);

      // Now: DUT0 eof will ALSO assert on next vs_rise (because in_frame_seen=1)
      // DUT1 eof should be 0 unless vs_fall is pulsed.
      drive_cycle(0, 24'h0, 0, 0, 1, 0, 0, 0, 1); // next vs_rise
      chk_pulse("DUT0 eof on next vs_rise", eof0, 1);
      chk_pulse("DUT1 eof NOT on vs_rise", eof1, 0);

      // DUT1 eof on vs_fall
      drive_cycle(0, 24'h0, 0, 0, 0, 1, 0, 0, 1);
      chk_pulse("DUT1 eof on vs_fall", eof1, 1);

      // DUT0 ignore vs_fall
      chk_pulse("DUT0 eof ignore vs_fall", eof0, 0);

      tb_ok("TC3 done");
    end
  endtask

  // 4) pixel bus forwarding: pix_valid_o is 1-cycle delayed; rgb updates only when pix_valid=1
  task automatic tc_pixel_forward;
  logic [23:0] rgbA, rgbB;
  begin
    $display("\n--- TC4: pixel forwarding / register behavior ---");
    rgbA = 24'h12_34_56;
    rgbB = 24'hAB_CD_EF;

    // cycle0: valid pixel
    drive_cycle(1, rgbA, 11'd3, 10'd2, 0,0,0,0, 1);

    // pix_valid_o is SAME cycle registered
    chk_pulse("DUT0 pix_valid_o same", pix_valid_o0, 1);
    chk_pulse("DUT1 pix_valid_o same", pix_valid_o1, 1);

    chk_bus_eq24("DUT0 rgb_out=A", pix_rgb_out0, rgbA);
    chk_bus_eq11("DUT0 x_o=3", x_o0, 11'd3);
    chk_bus_eq10("DUT0 y_o=2", y_o0, 10'd2);

    // cycle1: pv=0, new rgb should NOT overwrite
    drive_cycle(0, rgbB, 11'd4, 10'd2, 0,0,0,0, 1);

    chk_pulse("DUT0 pix_valid_o back0", pix_valid_o0, 0);
    chk_bus_eq24("DUT0 rgb_out holds A", pix_rgb_out0, rgbA);

    tb_ok("TC4 done");
  end
  endtask


  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_frame_event_gen.vcd");
    $dumpvars(0, tb_frame_event_gen);

    // init
    pix_valid  = 0;
    pix_rgb_in = 0;
    x = 0; y = 0;
    vs_rise = 0; vs_fall = 0; de_rise = 0; de_fall = 0;
    in_frame = 0;

    // reset
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);
    #1;

    // run tests
    tc_sof_vs_rise();
    tc_sol_eol_gated();
    tc_eof_modes();
    tc_pixel_forward();

    // done
    if (err_cnt == 0) $display("\n[TB] PASS: no errors");
    else              $display("\n[TB] FAIL: err_cnt=%0d", err_cnt);

    $finish;
  end

endmodule

`default_nettype wire
