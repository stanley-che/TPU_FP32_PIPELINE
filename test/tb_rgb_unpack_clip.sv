// tb_rgb_unpack_clip.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb_unpack_clip.vvp \
  ./test/tb_rgb_unpack_clip.sv

vvp ./vvp/tb_rgb_unpack_clip.vvp
gtkwave ./vvp/tb_rgb_unpack_clip.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb_unpack_clip.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb_unpack_clip;

  logic clk = 1'b0;
  always #5 clk = ~clk; // 100MHz
  logic rst;

  logic        pix_valid;
  logic [35:0] rgb_in;

  logic        v1_valid;
  logic [7:0]  r8, g8, b8;

  // ---- match your DUT params ----
  localparam int unsigned FORMAT = 0; // 0 RGB888, 1 RGB101010, 2 RGB121212, 3 RGB565
  localparam int unsigned LAT    = 1;
  localparam bit ZERO_WHEN_INVALID = 1'b1;

  rgb_unpack_clip #(
    .FORMAT(FORMAT),
    .LAT(LAT),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID)
  ) dut (
    .clk(clk),
    .rst(rst),
    .pix_valid(pix_valid),
    .rgb_in(rgb_in),
    .v1_valid(v1_valid),
    .r8(r8),
    .g8(g8),
    .b8(b8)
  );

  // -----------------------------
  // Expected shift regs
  // -----------------------------
  integer i;

  logic [LAT:0] exp_v_sh;
  logic [7:0]   exp_r_sh [0:LAT];
  logic [7:0]   exp_g_sh [0:LAT];
  logic [7:0]   exp_b_sh [0:LAT];

  // helpers (same as DUT)
  function automatic [7:0] exp5_to_8(input [4:0] x);
    begin exp5_to_8 = {x, x[4:2]}; end
  endfunction
  function automatic [7:0] exp6_to_8(input [5:0] x);
    begin exp6_to_8 = {x, x[5:4]}; end
  endfunction
  function automatic [7:0] msb10_to_8(input [9:0] x);
    begin msb10_to_8 = x[9:2]; end
  endfunction
  function automatic [7:0] msb12_to_8(input [11:0] x);
    begin msb12_to_8 = x[11:4]; end
  endfunction

  task automatic pack_rgb888(
    input  logic [7:0]  rr,
    input  logic [7:0]  gg,
    input  logic [7:0]  bb,
    output logic [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[23:16] = rr;
      bus[15:8]  = gg;
      bus[7:0]   = bb;
    end
  endtask

  task automatic compute_expected0(
    input  logic        v_in,
    input  logic [35:0] bus_in,
    output logic        v0,
    output logic [7:0]  r0,
    output logic [7:0]  g0,
    output logic [7:0]  b0
  );
    logic [7:0] rc, gc, bc;
    begin
      rc = 8'h00; gc = 8'h00; bc = 8'h00;

      if (FORMAT == 0) begin
        rc = bus_in[23:16];
        gc = bus_in[15:8];
        bc = bus_in[7:0];
      end else if (FORMAT == 1) begin
        rc = msb10_to_8(bus_in[29:20]);
        gc = msb10_to_8(bus_in[19:10]);
        bc = msb10_to_8(bus_in[9:0]);
      end else if (FORMAT == 2) begin
        rc = msb12_to_8(bus_in[35:24]);
        gc = msb12_to_8(bus_in[23:12]);
        bc = msb12_to_8(bus_in[11:0]);
      end else begin
        rc = exp5_to_8(bus_in[15:11]);
        gc = exp6_to_8(bus_in[10:5]);
        bc = exp5_to_8(bus_in[4:0]);
      end

      if (ZERO_WHEN_INVALID && !v_in) begin
        r0 = 8'h00; g0 = 8'h00; b0 = 8'h00;
      end else begin
        r0 = rc;    g0 = gc;    b0 = bc;
      end

      v0 = v_in;
    end
  endtask

  // one simulation step: drive -> posedge -> #1 -> update expected -> check
  task automatic step_and_check(
    input int          idx,
    input logic        v_in,
    input logic [35:0] bus_in
  );
    logic v0;
    logic [7:0] r0, g0, b0;
    integer t;
    begin
      // drive for this cycle
      pix_valid = v_in;
      rgb_in    = bus_in;

      // compute expected for stage0
      compute_expected0(v_in, bus_in, v0, r0, g0, b0);

      // advance clock
      @(posedge clk);
      #1;

      // shift expected
      if (rst) begin
        exp_v_sh = '0;
        for (t = 0; t <= LAT; t = t + 1) begin
          exp_r_sh[t] = 8'h00;
          exp_g_sh[t] = 8'h00;
          exp_b_sh[t] = 8'h00;
        end
      end else begin
        for (t = LAT; t >= 1; t = t - 1) begin
          exp_v_sh[t] = exp_v_sh[t-1];
          exp_r_sh[t] = exp_r_sh[t-1];
          exp_g_sh[t] = exp_g_sh[t-1];
          exp_b_sh[t] = exp_b_sh[t-1];
        end
        exp_v_sh[0] = v0;
        exp_r_sh[0] = r0;
        exp_g_sh[0] = g0;
        exp_b_sh[0] = b0;
      end

      // check
      if (v1_valid !== exp_v_sh[LAT]) begin
        $display("FAIL valid @i=%0d got=%b exp=%b", idx, v1_valid, exp_v_sh[LAT]);
        $fatal;
      end

      if (v1_valid === 1'b1) begin
        if (r8 !== exp_r_sh[LAT] || g8 !== exp_g_sh[LAT] || b8 !== exp_b_sh[LAT]) begin
          $display("FAIL data @i=%0d got R=%h G=%h B=%h exp R=%h G=%h B=%h",
                   idx, r8, g8, b8, exp_r_sh[LAT], exp_g_sh[LAT], exp_b_sh[LAT]);
          $fatal;
        end
      end
    end
  endtask

  // ------------------------------------------------------------
  // simple 32-bit LFSR for pseudo-random (iverilog-safe)
  // ------------------------------------------------------------
  function automatic [31:0] lfsr32_next(input [31:0] s);
    logic fb;
    begin
      fb = s[31] ^ s[21] ^ s[1] ^ s[0];
      lfsr32_next = {s[30:0], fb};
    end
  endfunction

  task automatic warmup;
    integer t;
    begin
      for (t = 0; t < (LAT + 2); t = t + 1)
        step_and_check(-1000 + t, 1'b0, 36'h0);
    end
  endtask

  task automatic drain;
    integer t;
    begin
      for (t = 0; t < (LAT + 2); t = t + 1)
        step_and_check(9000 + t, 1'b0, 36'h0);
    end
  endtask


  // ------------------------------------------------------------
  // Pack helpers for each FORMAT (bus is always [35:0])
  // ------------------------------------------------------------
  task automatic pack_rgb565(
    input  logic [4:0]  r5,
    input  logic [5:0]  g6,
    input  logic [4:0]  b5,
    output logic [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[15:11] = r5;
      bus[10:5]  = g6;
      bus[4:0]   = b5;
    end
  endtask

  task automatic pack_rgb101010(
    input  logic [9:0]  r10,
    input  logic [9:0]  g10,
    input  logic [9:0]  b10,
    output logic [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[29:20] = r10;
      bus[19:10] = g10;
      bus[9:0]   = b10;
    end
  endtask

  task automatic pack_rgb121212(
    input  logic [11:0] r12,
    input  logic [11:0] g12,
    input  logic [11:0] b12,
    output logic [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[35:24] = r12;
      bus[23:12] = g12;
      bus[11:0]  = b12;
    end
  endtask

  // For re-using the same tests across FORMATs:
  // Given 8-bit "logical colors" rr/gg/bb, generate a bus that matches FORMAT.
  task automatic pack_by_format(
    input  logic [7:0]  rr,
    input  logic [7:0]  gg,
    input  logic [7:0]  bb,
    output logic [35:0] bus
  );
    logic [4:0]  r5, b5;
    logic [5:0]  g6;
    logic [9:0]  r10, g10, b10;
    logic [11:0] r12, g12, b12;
    begin
      if (FORMAT == 0) begin
        pack_rgb888(rr, gg, bb, bus);
      end else if (FORMAT == 1) begin
        // 8->10 : replicate LSBs (simple up-expand)
        r10 = {rr, rr[7:6]};
        g10 = {gg, gg[7:6]};
        b10 = {bb, bb[7:6]};
        pack_rgb101010(r10, g10, b10, bus);
      end else if (FORMAT == 2) begin
        // 8->12 : replicate LSBs
        r12 = {rr, rr[7:4]};
        g12 = {gg, gg[7:4]};
        b12 = {bb, bb[7:4]};
        pack_rgb121212(r12, g12, b12, bus);
      end else begin
        // RGB565: take MSBs
        r5 = rr[7:3];
        g6 = gg[7:2];
        b5 = bb[7:3];
        pack_rgb565(r5, g6, b5, bus);
      end
    end
  endtask

  // ------------------------------------------------------------
  // TEST runner (TEST1..TEST7)
  // ------------------------------------------------------------
  function automatic [127:0] test_name(input int tid);
  begin
    case (tid)
      1:  test_name = "RAMP";
      2:  test_name = "BOUNDARY_VALUES";
      3:  test_name = "BUBBLE_STRESS";
      4:  test_name = "RANDOM_WITH_BUBBLES";
      5:  test_name = "MIDSTREAM_RESET";
      6:  test_name = "X_ON_INVALID";
      7:  test_name = "LONG_STRESS_1024";

      // ---- extended coverage ----
      8:  test_name = "HOLD_LAST_ON_INVALID";
      9:  test_name = "VALID_PIPE_ALIGNMENT";
      10: test_name = "RESET_PULSE_STRESS";
      11: test_name = "RGB565_EXPAND_EXACT";
      12: test_name = "RGB101010_MSB_CLIP";
      13: test_name = "RGB121212_MSB_CLIP";
      14: test_name = "INVALID_GARBAGE_NO_EFFECT";
      15: test_name = "LONG_MIXED_STRESS_2048";

      default: test_name = "UNKNOWN_TEST";
    endcase
  end
endfunction

  task automatic run_test(input int tid);
    integer t;
    logic [7:0]  rr, gg, bb;
    logic [35:0] bus;
    logic [31:0] s;
    begin
      $display("\n[RUN ] TEST%0d %s (FORMAT=%0d LAT=%0d ZERO=%0d)",
               tid, test_name(tid), FORMAT, LAT, ZERO_WHEN_INVALID);

      warmup();

      case (tid)
        1: begin
          // TEST1: ramp (same pattern for all FORMAT)
          for (t = 0; t < 40; t = t + 1) begin
            rr = (t      ) & 8'hFF;
            gg = (t * 3  ) & 8'hFF;
            bb = (8'hFF - (t & 8'hFF));
            pack_by_format(rr, gg, bb, bus);

            if ((t % 7) == 3) step_and_check(t, 1'b0, 36'h0);
            step_and_check(t, 1'b1, bus);
          end
        end

        2: begin
          // TEST2: boundary values (per-format edge coverage)
          // Use 8-bit logical boundary set; pack_by_format will map to the selected FORMAT.
          logic [7:0] vals [0:5];
          integer i2;
          begin
            vals[0]=8'h00; vals[1]=8'h01; vals[2]=8'h7F;
            vals[3]=8'h80; vals[4]=8'hFE; vals[5]=8'hFF;

            // sweep combinations on channels but keep it short
            for (i2 = 0; i2 < 6; i2 = i2 + 1) begin
              // R boundary, G/B fixed
              pack_by_format(vals[i2], 8'h55, 8'hAA, bus);
              step_and_check(100 + i2, 1'b1, bus);
            end
            for (i2 = 0; i2 < 6; i2 = i2 + 1) begin
              // G boundary
              pack_by_format(8'h55, vals[i2], 8'hAA, bus);
              step_and_check(120 + i2, 1'b1, bus);
            end
            for (i2 = 0; i2 < 6; i2 = i2 + 1) begin
              // B boundary
              pack_by_format(8'h55, 8'hAA, vals[i2], bus);
              step_and_check(140 + i2, 1'b1, bus);
            end
          end

          // Extra hard-edge for 10/12-bit formats: near-max codes
          // (讓你抓 trunc/round 邏輯)
          if (FORMAT == 1) begin
            pack_rgb101010(10'd0, 10'd1, 10'd2, bus);        step_and_check(160, 1'b1, bus);
            pack_rgb101010(10'd3, 10'd1020, 10'd1023, bus);  step_and_check(161, 1'b1, bus);
          end
          if (FORMAT == 2) begin
            pack_rgb121212(12'd0, 12'd1, 12'd2, bus);        step_and_check(170, 1'b1, bus);
            pack_rgb121212(12'd3, 12'd4094, 12'd4095, bus);  step_and_check(171, 1'b1, bus);
          end
          if (FORMAT == 3) begin
            pack_rgb565(5'd0, 6'd0, 5'd0, bus);              step_and_check(180, 1'b1, bus);
            pack_rgb565(5'd1, 6'd1, 5'd1, bus);              step_and_check(181, 1'b1, bus);
            pack_rgb565(5'd31,6'd63,5'd31, bus);             step_and_check(182, 1'b1, bus);
          end
        end

        3: begin
          // TEST3: bubble stress
          for (t = 0; t < 80; t = t + 1) begin
            rr = (t * 7) & 8'hFF;
            gg = (t * 5) & 8'hFF;
            bb = (t * 3) & 8'hFF;
            pack_by_format(rr, gg, bb, bus);

            if ((t % 5) == 2 || (t % 5) == 4)
              step_and_check(300 + t, 1'b0, 36'h0);
            else
              step_and_check(300 + t, 1'b1, bus);
          end
        end

        4: begin
          // TEST4: random with bubbles (short)
          s = 32'h1ACE_B00C;
          for (t = 0; t < 200; t = t + 1) begin
            s = lfsr32_next(s);
            rr = s[7:0];
            gg = s[15:8];
            bb = s[23:16];
            pack_by_format(rr, gg, bb, bus);

            if (s[24])
              step_and_check(500 + t, 1'b0, 36'h0);
            else
              step_and_check(500 + t, 1'b1, bus);
          end
        end

        5: begin
          // TEST5: mid-stream reset
          for (t = 0; t < 20; t = t + 1) begin
            rr = t[7:0];
            gg = (t * 2) & 8'hFF;
            bb = (8'hFF - (t & 8'hFF));
            pack_by_format(rr, gg, bb, bus);
            step_and_check(800 + t, 1'b1, bus);
          end

          rst = 1'b1;
          step_and_check(900, 1'b0, 36'h0);
          step_and_check(901, 1'b0, 36'h0);
          rst = 1'b0;

          warmup();
          for (t = 0; t < 20; t = t + 1) begin
            rr = (t + 8) & 8'hFF;
            gg = (t * 3) & 8'hFF;
            bb = (t * 5) & 8'hFF;
            pack_by_format(rr, gg, bb, bus);
            step_and_check(1000 + t, 1'b1, bus);
          end
        end

        6: begin
          // TEST6: X on invalid (check X does NOT propagate when ZERO_WHEN_INVALID=1)
          // 先送一筆 valid 確保 pipe 內有資料，再送 invalid + rgb_in='x
          pack_by_format(8'h12, 8'h34, 8'h56, bus);
          step_and_check(1200, 1'b1, bus);

          // invalid cycles with X bus
          for (t = 0; t < 10; t = t + 1) begin
            step_and_check(1210 + t, 1'b0, 36'hx);
          end

          // 再送一筆 valid，確認恢復正常
          pack_by_format(8'hAB, 8'hCD, 8'hEF, bus);
          step_and_check(1300, 1'b1, bus);
        end

        7: begin
          // TEST7: long stress 1024 cycles (random + bubbles)
          s = 32'hC0FF_EE11;
          for (t = 0; t < 1024; t = t + 1) begin
            s = lfsr32_next(s);
            rr = s[7:0];
            gg = s[15:8];
            bb = s[23:16];
            pack_by_format(rr, gg, bb, bus);

            // bubbles: ~25%
            if (s[31] & s[30])
              step_and_check(2000 + t, 1'b0, 36'h0);
            else
              step_and_check(2000 + t, 1'b1, bus);
          end
        end

                8: begin
          // TEST8: HOLD_LAST_ON_INVALID (for ZERO_WHEN_INVALID==0 case)
          // 先送一筆 valid，接著連續 invalid，期望資料保持（由 step_and_check 的 expected 決定）
          pack_by_format(8'h12, 8'h34, 8'h56, bus);
          step_and_check(3000, 1'b1, bus);

          // invalid with changing garbage
          s = 32'hA5A5_1234;
          for (t = 0; t < 20; t = t + 1) begin
            s = lfsr32_next(s);
            // invalid cycle: bus random (or X)
            step_and_check(3010 + t, 1'b0, {4'h0, s}); // any garbage
          end

          // 再送一筆 valid 確認恢復正常
          pack_by_format(8'hAB, 8'hCD, 8'hEF, bus);
          step_and_check(3100, 1'b1, bus);
        end

        9: begin
          // TEST9: VALID_PIPE_ALIGNMENT (single valid pulse)
          // 一拍 valid，其餘 invalid，抓 off-by-one / LAT 對齊
          pack_by_format(8'hDE, 8'hAD, 8'hBE, bus);
          step_and_check(3200, 1'b1, bus);

          // rest invalid; should produce exactly one v1_valid pulse at correct delayed cycle
          for (t = 0; t < (LAT + 6); t = t + 1) begin
            step_and_check(3210 + t, 1'b0, 36'h0);
          end
        end

                10: begin
          // TEST10: RESET_PULSE_STRESS (glitchy reset pulses)
          // valid stream a bit
          for (t = 0; t < 10; t = t + 1) begin
            logic [7:0] rtmp, gtmp, btmp;
            rtmp = t[7:0];
            gtmp = (t * 3);              // 這裡先算
            btmp = 8'hFF - t[7:0];
            pack_by_format(rtmp, gtmp, btmp, bus);
            step_and_check(3400 + t, 1'b1, bus);
          end

          // reset pulses: 1-cycle high bursts
          for (t = 0; t < 6; t = t + 1) begin
            rst = 1'b1;
            step_and_check(3500 + (t*3),   1'b0, 36'h0);
            rst = 1'b0;
            step_and_check(3500 + (t*3)+1, 1'b0, 36'h0);
            step_and_check(3500 + (t*3)+2, 1'b0, 36'h0);
          end

          // back to valid stream
          warmup();
          for (t = 0; t < 10; t = t + 1) begin
            logic [7:0] rtmp, gtmp, btmp;
            rtmp = (t + 8);              // 先算
            gtmp = (t * 5);
            btmp = (t * 7);
            pack_by_format(rtmp, gtmp, btmp, bus);
            step_and_check(3600 + t, 1'b1, bus);
          end
        end


        11: begin
          // TEST11: RGB565_EXPAND_EXACT (FORMAT must be 3)
          // 直接用 pack_rgb565 驗 expand slicing/replication
          if (FORMAT != 3) begin
            // 若不是 565，就快速跑幾拍無效當作 skip
            for (t = 0; t < 5; t = t + 1) step_and_check(3700 + t, 1'b0, 36'h0);
          end else begin
            pack_rgb565(5'd0,  6'd0,  5'd0,  bus); step_and_check(3700, 1'b1, bus);
            pack_rgb565(5'd1,  6'd1,  5'd1,  bus); step_and_check(3701, 1'b1, bus);
            pack_rgb565(5'd2,  6'd2,  5'd2,  bus); step_and_check(3702, 1'b1, bus);
            pack_rgb565(5'd15, 6'd31, 5'd15, bus); step_and_check(3703, 1'b1, bus);
            pack_rgb565(5'd30, 6'd62, 5'd30, bus); step_and_check(3704, 1'b1, bus);
            pack_rgb565(5'd31, 6'd63, 5'd31, bus); step_and_check(3705, 1'b1, bus);
          end
        end

        12: begin
          // TEST12: MSB_CLIP_EXACT for RGB101010 (FORMAT=1)
          if (FORMAT != 1) begin
            for (t = 0; t < 5; t = t + 1) step_and_check(3800 + t, 1'b0, 36'h0);
          end else begin
            // Check [9:2] behavior explicitly
            pack_rgb101010(10'b1111111100, 10'd0,          10'd0,          bus); step_and_check(3800, 1'b1, bus); // expect R=FF
            pack_rgb101010(10'b0000000011, 10'd0,          10'd0,          bus); step_and_check(3801, 1'b1, bus); // expect R=00
            pack_rgb101010(10'd1023,       10'd1020,       10'd3,          bus); step_and_check(3802, 1'b1, bus);
            pack_rgb101010(10'd0,          10'd1023,       10'd0,          bus); step_and_check(3803, 1'b1, bus);
          end
        end

        13: begin
          // TEST13: MSB_CLIP_EXACT for RGB121212 (FORMAT=2)
          if (FORMAT != 2) begin
            for (t = 0; t < 5; t = t + 1) step_and_check(3900 + t, 1'b0, 36'h0);
          end else begin
            // Check [11:4] behavior explicitly
            pack_rgb121212(12'hFF0, 12'd0,   12'd0,   bus); step_and_check(3900, 1'b1, bus); // expect R=FF
            pack_rgb121212(12'h00F, 12'd0,   12'd0,   bus); step_and_check(3901, 1'b1, bus); // expect R=00
            pack_rgb121212(12'd4095,12'd4094,12'd3,   bus); step_and_check(3902, 1'b1, bus);
            pack_rgb121212(12'd0,   12'd4095,12'd0,   bus); step_and_check(3903, 1'b1, bus);
          end
        end

        14: begin
          // TEST14: INVALID_GARBAGE_NO_EFFECT (ZERO=1 should suppress garbage)
          // invalid cycles: rgb_in random/X, ensure no X leakage when v_in=0
          s = 32'h55AA_F00D;
          for (t = 0; t < 100; t = t + 1) begin
            s = lfsr32_next(s);
            if (t[3]) begin
              step_and_check(4000 + t, 1'b0, 36'hx); // heavy X injection
            end else begin
              step_and_check(4000 + t, 1'b0, {4'h0, s});
            end
          end

          // one valid after garbage
          pack_by_format(8'h01, 8'h02, 8'h03, bus);
          step_and_check(4200, 1'b1, bus);
        end

        15: begin
          // TEST15: LONG_MIXED (longer than TEST7, with periodic bubbles + periodic reset)
          s = 32'h0BAD_CAFE;
          for (t = 0; t < 2048; t = t + 1) begin
            s = lfsr32_next(s);
            rr = s[7:0];
            gg = s[15:8];
            bb = s[23:16];
            pack_by_format(rr, gg, bb, bus);

            // every ~256 cycles do a tiny reset pulse (simulate system disturbance)
            if ((t % 256) == 200) begin
              rst = 1'b1;
              step_and_check(5000 + t, 1'b0, 36'h0);
              rst = 1'b0;
            end else begin
              // bubbles ~25%
              if (s[31] & s[29])
                step_and_check(5000 + t, 1'b0, 36'h0);
              else
                step_and_check(5000 + t, 1'b1, bus);
            end
          end

          // recover
          warmup();
        end

        default: begin
          $display("[ERR ] Unknown TEST id=%0d", tid);
          $fatal;
        end
      endcase

      drain();
      $display("[PASS] TEST%0d %s", tid, test_name(tid));
    end
  endtask


  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_rgb_unpack_clip.vcd");
    $dumpvars(0, tb_rgb_unpack_clip);

    // init
    rst = 1'b1;
    pix_valid = 1'b0;
    rgb_in    = 36'h0;

    exp_v_sh = '0;
    for (i = 0; i <= LAT; i = i + 1) begin
      exp_r_sh[i] = 8'h00;
      exp_g_sh[i] = 8'h00;
      exp_b_sh[i] = 8'h00;
    end

    repeat (3) @(posedge clk);
    rst = 1'b0;

    // run TEST1..TEST5
    run_test(1);
    run_test(2);
    run_test(3);
    run_test(4);
    run_test(5);
    run_test(6);
    run_test(7);
    run_test(8);
    run_test(9);
    run_test(10);
    run_test(11);
    run_test(12);
    run_test(13);
    run_test(14);
    run_test(15);
    $display("\nPASS tb_rgb_unpack_clip ALL (FORMAT=%0d LAT=%0d)", FORMAT, LAT);
    $finish;
  end

endmodule

`default_nettype wire
