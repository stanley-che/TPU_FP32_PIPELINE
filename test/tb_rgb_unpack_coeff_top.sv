// tb_rgb_unpack_coeff_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_rgb_unpack_coeff_top.vvp \
  ./test/tb_rgb_unpack_coeff_top.sv

vvp ./vvp/tb_rgb_unpack_coeff_top.vvp
gtkwave ./vvp/tb_rgb_unpack_coeff_top.vcd
*/

`include "./src/AMOLED/rgb2y_luma/rgb_unpack_coeff_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rgb_unpack_coeff_top;

  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  logic        en;
  logic        pix_valid;
  logic [35:0] rgb_in;

  logic        v2_valid;
  logic [15:0] r_term, g_term, b_term;

  logic        v1_valid;
  logic [7:0]  r8, g8, b8;

  // ---- match your params ----
  localparam int unsigned FORMAT = 0;   // 0/1/2/3
  localparam int unsigned LAT_U  = 1;
  localparam int unsigned LAT_M  = 2;
  localparam bit ZERO_WHEN_INVALID = 1'b1;
  localparam bit USE_DSP = 1'b0;
  localparam bit BYPASS  = 1'b0;

  rgb_unpack_coeff_top #(
    .FORMAT(FORMAT),
    .LAT_U(LAT_U),
    .ZERO_WHEN_INVALID(ZERO_WHEN_INVALID),
    .LAT_M(LAT_M),
    .USE_DSP(USE_DSP),
    .BYPASS(BYPASS)
  ) dut (
    .clk(clk),
    .rst(rst),
    .en(en),
    .pix_valid(pix_valid),
    .rgb_in(rgb_in),
    .v2_valid(v2_valid),
    .r_term(r_term),
    .g_term(g_term),
    .b_term(b_term),
    .v1_valid(v1_valid),
    .r8(r8),
    .g8(g8),
    .b8(b8)
  );

  // ============================================================
  // helpers: pack by FORMAT (same idea as your tb_rgb_unpack_clip)
  // ============================================================
  task automatic drain(input int n);
      integer t;
      begin
          for (t=0; t<n; t=t+1)
              step_drive_update_check(900000+t, 1'b0, 36'h0, 1'b0);
      end
  endtask

  task automatic pack_rgb888(
    input  logic [7:0] rr, input logic [7:0] gg, input logic [7:0] bb,
    output logic [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[23:16] = rr;
      bus[15:8]  = gg;
      bus[7:0]   = bb;
    end
  endtask

  task automatic pack_rgb565(
    input logic [4:0] r5, input logic [5:0] g6, input logic [4:0] b5,
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
    input logic [9:0] r10, input logic [9:0] g10, input logic [9:0] b10,
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
    input logic [11:0] r12, input logic [11:0] g12, input logic [11:0] b12,
    output logic [35:0] bus
  );
    begin
      bus = 36'h0;
      bus[35:24] = r12;
      bus[23:12] = g12;
      bus[11:0]  = b12;
    end
  endtask

  task automatic pack_by_format(
    input  logic [7:0] rr, input logic [7:0] gg, input logic [7:0] bb,
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
        r10 = {rr, rr[7:6]};
        g10 = {gg, gg[7:6]};
        b10 = {bb, bb[7:6]};
        pack_rgb101010(r10, g10, b10, bus);
      end else if (FORMAT == 2) begin
        r12 = {rr, rr[7:4]};
        g12 = {gg, gg[7:4]};
        b12 = {bb, bb[7:4]};
        pack_rgb121212(r12, g12, b12, bus);
      end else begin
        r5 = rr[7:3];
        g6 = gg[7:2];
        b5 = bb[7:3];
        pack_rgb565(r5, g6, b5, bus);
      end
    end
  endtask

  // ============================================================
  // expected model
  // ============================================================
  function automatic [31:0] lfsr32_next(input [31:0] s);
    logic fb;
    begin
      fb = s[31] ^ s[21] ^ s[1] ^ s[0];
      lfsr32_next = {s[30:0], fb};
    end
  endfunction

  // unpack expected: we reuse your same mapping behavior
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

  // mult expected
  function automatic [15:0] mul77(input [7:0] x);
    begin mul77 = x * 16'd77; end
  endfunction
  function automatic [15:0] mul150(input [7:0] x);
    begin mul150 = x * 16'd150; end
  endfunction
  function automatic [15:0] mul29(input [7:0] x);
    begin mul29 = x * 16'd29; end
  endfunction

  // ---- stage0 expected unpack (current cycle)
  task automatic unpack_expected0(
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

  integer i;

  // unpack pipeline state (always advances)
  logic [LAT_U:0] u_vsh;
  logic [7:0]     u_rsh [0:LAT_U];
  logic [7:0]     u_gsh [0:LAT_U];
  logic [7:0]     u_bsh [0:LAT_U];

  // mult pipeline state (advances only when en=1)
  logic [LAT_M:0] m_vsh;
  logic [15:0]    m_rsh [0:LAT_M];
  logic [15:0]    m_gsh [0:LAT_M];
  logic [15:0]    m_bsh [0:LAT_M];

  task automatic clear_expected;
    integer t;
    begin
      u_vsh = '0;
      for (t=0; t<=LAT_U; t=t+1) begin
        u_rsh[t]=8'h00; u_gsh[t]=8'h00; u_bsh[t]=8'h00;
      end

      m_vsh = '0;
      for (t=0; t<=LAT_M; t=t+1) begin
        m_rsh[t]=16'h0000; m_gsh[t]=16'h0000; m_bsh[t]=16'h0000;
      end
    end
  endtask

  task automatic step_drive_update_check(
  input int idx,
  input logic vin,
  input logic [35:0] bus_in,
  input bit inject_x_when_invalid
);
  // unpack stage0 expected (from current inputs)
  logic u0_v;
  logic [7:0] u0_r, u0_g, u0_b;

  // ---- PRE-EDGE snapshot: what mult will see at this posedge
  logic mv0_pre;
  logic [7:0]  mr8_pre, mg8_pre, mb8_pre;
  logic [15:0] mr0_pre, mg0_pre, mb0_pre;

  integer t;
  begin
    // drive
    pix_valid = vin;
    if (!vin && inject_x_when_invalid) rgb_in = 36'hx;
    else                              rgb_in = bus_in;

    // expected unpack stage0 from bus_in (not X)
    unpack_expected0(vin, bus_in, u0_v, u0_r, u0_g, u0_b);

    // ============================================================
    // IMPORTANT: mult samples v1_valid/r8/g8/b8 using PRE-edge values
    // so we must compute mv0_pre from current stored unpack pipeline tail
    // BEFORE we update unpack expected shift regs.
    // ============================================================
    mv0_pre = u_vsh[LAT_U];
    mr8_pre = u_rsh[LAT_U];
    mg8_pre = u_gsh[LAT_U];
    mb8_pre = u_bsh[LAT_U];

    if (BYPASS) begin
      mr0_pre = {8'h00, mr8_pre};
      mg0_pre = {8'h00, mg8_pre};
      mb0_pre = {8'h00, mb8_pre};
    end else begin
      mr0_pre = mul77(mr8_pre);
      mg0_pre = mul150(mg8_pre);
      mb0_pre = mul29(mb8_pre);
    end

    // tick
    @(posedge clk);
    #1;

    if (rst) begin
      clear_expected();
    end else begin
      // ---- advance unpack expected ALWAYS (this creates "new" v1_valid after edge)
      for (t=LAT_U; t>=1; t=t-1) begin
        u_vsh[t] = u_vsh[t-1];
        u_rsh[t] = u_rsh[t-1];
        u_gsh[t] = u_gsh[t-1];
        u_bsh[t] = u_bsh[t-1];
      end
      u_vsh[0] = u0_v;
      u_rsh[0] = u0_r;
      u_gsh[0] = u0_g;
      u_bsh[0] = u0_b;

      // ---- advance mult expected ONLY when en=1, using PRE-edge snapshot
      if (en) begin
        for (t=LAT_M; t>=1; t=t-1) begin
          m_vsh[t] = m_vsh[t-1];
          m_rsh[t] = m_rsh[t-1];
          m_gsh[t] = m_gsh[t-1];
          m_bsh[t] = m_bsh[t-1];
        end
        m_vsh[0] = mv0_pre;
        m_rsh[0] = mr0_pre;
        m_gsh[0] = mg0_pre;
        m_bsh[0] = mb0_pre;
      end
    end

    // ---- check unpack outputs (posedge AFTER update)
    if (v1_valid !== u_vsh[LAT_U]) begin
      $display("FAIL v1_valid idx=%0d got=%b exp=%b", idx, v1_valid, u_vsh[LAT_U]);
      $fatal;
    end
    if (v1_valid === 1'b1) begin
      if (r8 !== u_rsh[LAT_U] || g8 !== u_gsh[LAT_U] || b8 !== u_bsh[LAT_U]) begin
        $display("FAIL unpack data idx=%0d got R=%h G=%h B=%h exp R=%h G=%h B=%h",
                 idx, r8, g8, b8, u_rsh[LAT_U], u_gsh[LAT_U], u_bsh[LAT_U]);
        $fatal;
      end
    end

    // ---- check mult outputs (posedge AFTER update)
    if (v2_valid !== m_vsh[LAT_M]) begin
      $display("FAIL v2_valid idx=%0d got=%b exp=%b", idx, v2_valid, m_vsh[LAT_M]);
      $fatal;
    end
    if (v2_valid === 1'b1) begin
      if (r_term !== m_rsh[LAT_M] || g_term !== m_gsh[LAT_M] || b_term !== m_bsh[LAT_M]) begin
        $display("FAIL mult data idx=%0d got r=%0d g=%0d b=%0d exp r=%0d g=%0d b=%0d",
                 idx, r_term, g_term, b_term, m_rsh[LAT_M], m_gsh[LAT_M], m_bsh[LAT_M]);
        $fatal;
      end
    end
  end
  endtask

  task automatic warmup(input int n);
    integer t;
    begin
      for (t=0; t<n; t=t+1)
        step_drive_update_check(-1000+t, 1'b0, 36'h0, 1'b0);
    end
  endtask

  // ============================================================
  // tests
  // ============================================================
  task automatic test_mix;
    integer t;
    logic [31:0] s;
    logic [7:0] rr, gg, bb;
    logic [35:0] bus;
    begin
      $display("[RUN ] INTEG_MIX (FORMAT=%0d LAT_U=%0d LAT_M=%0d)", FORMAT, LAT_U, LAT_M);
      warmup(8);
      s = 32'h1ACE_B00C;

      for (t=0; t<800; t=t+1) begin
        s = lfsr32_next(s);
        rr = s[7:0];
        gg = s[15:8];
        bb = s[23:16];
        pack_by_format(rr, gg, bb, bus);

        // en hold pattern: every ~13 cycles hold 3 cycles
        if ((t % 13) == 7 || (t % 13) == 8 || (t % 13) == 9) en = 1'b0;
        else                                                 en = 1'b1;

        // bubbles ~25%
        if (s[24]) begin
          step_drive_update_check(100+t, 1'b0, 36'h0, 1'b1); // inject X on invalid
        end else begin
          step_drive_update_check(100+t, 1'b1, bus, 1'b0);
        end
      end

      $display("[PASS] INTEG_MIX");
    end
  endtask
  task automatic test_single_pulse_alignment;
  logic [35:0] bus;
  integer t;
  begin
    $display("[RUN ] SINGLE_PULSE_ALIGNMENT");
    en = 1'b1;
    warmup(8);

    // 只打一筆 valid
    pack_by_format(8'hDE, 8'hAD, 8'hBE, bus);
    step_drive_update_check(10000, 1'b1, bus, 1'b0);

    // 後面全 invalid（pipeline flush）
    for (t=0; t<(LAT_U+LAT_M+10); t=t+1)
      step_drive_update_check(10010+t, 1'b0, 36'h0, 1'b0);

    $display("[PASS] SINGLE_PULSE_ALIGNMENT");
  end
endtask
task automatic test_midstream_reset;
  logic [35:0] bus;
  integer t;
  logic [7:0] rr, gg, bb;
  begin
    $display("[RUN ] MIDSTREAM_RESET");
    en = 1'b1;
    warmup(8);

    // 先灌一些 valid
    for (t = 0; t < 20; t = t + 1) begin
      rr = t[7:0];
      gg = (t * 3);               // 先算到 8-bit
      bb = 8'hFF - t[7:0];
      pack_by_format(rr, gg, bb, bus);
      step_drive_update_check(11000 + t, 1'b1, bus, 1'b0);
    end

    // reset pulse 2 cycles
    rst = 1'b1;
    step_drive_update_check(12000, 1'b0, 36'h0, 1'b0);
    step_drive_update_check(12001, 1'b0, 36'h0, 1'b0);
    rst = 1'b0;

    warmup(4);

    // reset 後再灌 valid
    for (t = 0; t < 20; t = t + 1) begin
      rr = (t + 8);               // 先算
      gg = (t * 5);
      bb = (t * 7);
      pack_by_format(rr, gg, bb, bus);
      step_drive_update_check(12100 + t, 1'b1, bus, 1'b0);
    end

    drain(LAT_U + LAT_M + 6);
    $display("[PASS] MIDSTREAM_RESET");
  end
endtask
task automatic test_en_glitch_with_bubbles;
  logic [31:0] s;
  logic [7:0] rr, gg, bb;
  logic [35:0] bus;
  integer t;
  begin
    $display("[RUN ] EN_GLITCH_WITH_BUBBLES");
    warmup(8);

    s = 32'hC0FF_EE11;
    for (t = 0; t < 600; t = t + 1) begin
      // en pattern：長短 hold 混合
      if ((t % 17) == 8 || (t % 17) == 9 || (t % 17) == 10 || (t % 17) == 11) en = 1'b0;
      else if ((t % 9) == 4)                                                    en = 1'b0;
      else                                                                      en = 1'b1;

      s  = lfsr32_next(s);
      rr = s[7:0];
      gg = s[15:8];
      bb = s[23:16];
      pack_by_format(rr, gg, bb, bus);

      // bubbles: 約 30%
      if (s[31] & s[28]) begin
        step_drive_update_check(13000 + t, 1'b0, 36'h0, 1'b1); // invalid + X
      end else begin
        step_drive_update_check(13000 + t, 1'b1, bus, 1'b0);
      end
    end

    en = 1'b1;
    drain(LAT_U + LAT_M + 6);
    $display("[PASS] EN_GLITCH_WITH_BUBBLES");
  end
endtask

task automatic test_known_vectors_end2end;
  logic [35:0] bus;
  begin
    $display("[RUN ] KNOWN_VECTORS_E2E");
    en = 1'b1;
    warmup(8);

    // (R,G,B) = (255,0,0) => r_term=77*255=19635
    pack_by_format(8'hFF, 8'h00, 8'h00, bus);
    step_drive_update_check(20000, 1'b1, bus, 1'b0);

    // (0,255,0) => g_term=150*255=38250
    pack_by_format(8'h00, 8'hFF, 8'h00, bus);
    step_drive_update_check(20001, 1'b1, bus, 1'b0);

    // (0,0,255) => b_term=29*255=7395
    pack_by_format(8'h00, 8'h00, 8'hFF, bus);
    step_drive_update_check(20002, 1'b1, bus, 1'b0);

    // (255,255,255) => terms max (確認 bitwidth)
    pack_by_format(8'hFF, 8'hFF, 8'hFF, bus);
    step_drive_update_check(20003, 1'b1, bus, 1'b0);

    drain(LAT_U + LAT_M + 6);
    $display("[PASS] KNOWN_VECTORS_E2E");
  end
endtask
task automatic test_invalid_x_no_poison;
  logic [35:0] bus;
  integer t;
  begin
    $display("[RUN ] INVALID_X_NO_POISON");
    en = 1'b1;
    warmup(8);

    // 先送一筆 valid
    pack_by_format(8'h12, 8'h34, 8'h56, bus);
    step_drive_update_check(30000, 1'b1, bus, 1'b0);

    // 連續 invalid + X
    for (t=0; t<20; t=t+1)
      step_drive_update_check(30010+t, 1'b0, 36'h0, 1'b1);

    // 再送 valid，確認恢復正常
    pack_by_format(8'hAB, 8'hCD, 8'hEF, bus);
    step_drive_update_check(30100, 1'b1, bus, 1'b0);

    drain(LAT_U + LAT_M + 6);
    $display("[PASS] INVALID_X_NO_POISON");
  end
endtask
task automatic test_format_exactness;
  logic [35:0] bus;
  begin
    $display("[RUN ] FORMAT_EXACTNESS (FORMAT=%0d)", FORMAT);
    en = 1'b1;
    warmup(8);

    if (FORMAT == 3) begin
      // RGB565 corner codes
      pack_rgb565(5'd0,  6'd0,  5'd0,  bus); step_drive_update_check(40000, 1'b1, bus, 1'b0);
      pack_rgb565(5'd1,  6'd1,  5'd1,  bus); step_drive_update_check(40001, 1'b1, bus, 1'b0);
      pack_rgb565(5'd31, 6'd63, 5'd31, bus); step_drive_update_check(40002, 1'b1, bus, 1'b0);
    end else if (FORMAT == 1) begin
      // RGB101010 MSB clip exactness
      pack_rgb101010(10'd1023, 10'd0,    10'd0,   bus); step_drive_update_check(41000, 1'b1, bus, 1'b0);
      pack_rgb101010(10'd0,    10'd1023, 10'd0,   bus); step_drive_update_check(41001, 1'b1, bus, 1'b0);
      pack_rgb101010(10'd0,    10'd0,    10'd1023,bus); step_drive_update_check(41002, 1'b1, bus, 1'b0);
      pack_rgb101010(10'd3,    10'd2,    10'd1,   bus); step_drive_update_check(41003, 1'b1, bus, 1'b0);
    end else if (FORMAT == 2) begin
      // RGB121212 MSB clip exactness
      pack_rgb121212(12'd4095, 12'd0,    12'd0,   bus); step_drive_update_check(42000, 1'b1, bus, 1'b0);
      pack_rgb121212(12'd0,    12'd4095, 12'd0,   bus); step_drive_update_check(42001, 1'b1, bus, 1'b0);
      pack_rgb121212(12'd0,    12'd0,    12'd4095,bus); step_drive_update_check(42002, 1'b1, bus, 1'b0);
      pack_rgb121212(12'd15,   12'd14,   12'd1,   bus); step_drive_update_check(42003, 1'b1, bus, 1'b0);
    end else begin
      // RGB888 basic sanity
      pack_by_format(8'h01, 8'h02, 8'h03, bus); step_drive_update_check(43000, 1'b1, bus, 1'b0);
      pack_by_format(8'hFF, 8'h00, 8'h80, bus); step_drive_update_check(43001, 1'b1, bus, 1'b0);
    end

    drain(LAT_U + LAT_M + 6);
    $display("[PASS] FORMAT_EXACTNESS");
  end
endtask

  // ============================================================
  // main
  // ============================================================
  initial begin
    $dumpfile("./vvp/tb_rgb_unpack_coeff_top.vcd");
    $dumpvars(0, tb_rgb_unpack_coeff_top);

    rst = 1'b1;
    en  = 1'b1;
    pix_valid = 1'b0;
    rgb_in    = 36'h0;
    clear_expected();

    repeat (3) @(posedge clk);
    rst = 1'b0;

    test_mix();
    test_mix();
    test_single_pulse_alignment();
    test_midstream_reset();
    test_en_glitch_with_bubbles();
    test_known_vectors_end2end();
    test_invalid_x_no_poison();
    test_format_exactness();

    $display("PASS tb_rgb_unpack_coeff_top ALL");
    $finish;
  end

endmodule

`default_nettype wire
