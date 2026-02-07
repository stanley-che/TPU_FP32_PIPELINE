// tb_attention_mask_readout.sv
// ------------------------------------------------------------
// Icarus:
//   iverilog -g2012 -Wall -o ./vvp/tb_mask.vvp ./test/tb_attention_mask_readout.sv
//   vvp ./vvp/tb_mask.vvp
//
// This TB verifies:
//  1) no mask
//  2) padding mask (mask by key index)
//  3) causal mask (mask k>q)
//  4) padding + causal together
// ------------------------------------------------------------

`include "./src/EPU/attention_score/attention_mask_readout.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_attention_mask_readout;

  localparam int unsigned T      = 4;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned T_W    = (T<=1)?1:$clog2(T);

  // clock/reset
  logic clk, rst_n;
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
  end

  // DUT I/O
  logic [T-1:0]         pad_valid;
  logic                 causal_en;

  logic                 in_rvalid;
  logic [DATA_W-1:0]    in_rdata;
  logic [T_W-1:0]       in_q;
  logic [T_W-1:0]       in_k;

  logic                 out_rvalid;
  logic [DATA_W-1:0]    out_rdata;

  attention_mask_readout #(
    .T(T),
    .DATA_W(DATA_W)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    .pad_valid(pad_valid),
    .causal_en(causal_en),

    .in_rvalid(in_rvalid),
    .in_rdata(in_rdata),
    .in_q(in_q),
    .in_k(in_k),

    .out_rvalid(out_rvalid),
    .out_rdata(out_rdata)
  );

  // ------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------
  localparam logic [31:0] FP32_NINF = 32'hFF800000;

  function automatic bit is_ninf(input logic [31:0] b);
    begin
      is_ninf = (b === FP32_NINF);
    end
  endfunction

  // drive one "read sample" and check expected
  task automatic drive_and_check(
    input int q,
    input int k,
    input logic [31:0] din,
    input bit expect_mask
  );
    logic [31:0] exp;
    begin
      exp = expect_mask ? FP32_NINF : din;

      // present inputs (combinational module, but we sync on clk for clean waveform)
      @(posedge clk);
      in_q      <= q[T_W-1:0];
      in_k      <= k[T_W-1:0];
      in_rdata  <= din;
      in_rvalid <= 1'b1;

      // allow comb to settle within same cycle, sample at #1
      #1;
      if (out_rvalid !== 1'b1) begin
        $display("[ERR] out_rvalid not asserted at q=%0d k=%0d", q, k);
        $finish;
      end

      if (out_rdata !== exp) begin
        $display("[ERR] mismatch at q=%0d k=%0d  din=%08h  got=%08h  exp=%08h  pad_valid=%b causal=%0d",
                 q, k, din, out_rdata, exp, pad_valid, causal_en);
        $finish;
      end

      // deassert valid
      @(posedge clk);
      in_rvalid <= 1'b0;
      in_rdata  <= '0;
    end
  endtask

  // pretty print matrix by sweeping q,k
  task automatic dump_matrix(input string title);
    int q, k;
    logic [31:0] din;
    begin
      $display("=== %s ===", title);
      for (q = 0; q < T; q++) begin
        $write("row%0d:", q);
        for (k = 0; k < T; k++) begin
          // give each cell a unique value: 0x3f800000 + (q*T+k)*0x00100000
          din = 32'h3F800000 + ((q*T+k) * 32'h0010_0000);
          @(posedge clk);
          in_q      <= q[T_W-1:0];
          in_k      <= k[T_W-1:0];
          in_rdata  <= din;
          in_rvalid <= 1'b1;
          #1;
          $write("  %08h", out_rdata);
        end
        $write("\n");
      end
      @(posedge clk);
      in_rvalid <= 1'b0;
      $display("");
    end
  endtask

  // ------------------------------------------------------------
  // test sequences
  // ------------------------------------------------------------
  int q, k;
  logic [31:0] din;

  initial begin
    // defaults
    pad_valid = {T{1'b1}};
    causal_en = 1'b0;

    in_rvalid = 1'b0;
    in_rdata  = '0;
    in_q      = '0;
    in_k      = '0;

    @(posedge rst_n);
    repeat (2) @(posedge clk);

    // ----------------------------
    // RUN #1: no mask
    // ----------------------------
    $display("RUN #1: no mask (pad_valid=1111, causal=0)");
    pad_valid = 4'b1111;
    causal_en = 1'b0;

    for (q = 0; q < T; q++) begin
      for (k = 0; k < T; k++) begin
        din = 32'h3F800000 + ((q*T+k) * 32'h0010_0000);
        drive_and_check(q, k, din, /*expect_mask*/ 0);
      end
    end
    dump_matrix("DUMP no-mask (expect all pass-through)");

    // ----------------------------
    // RUN #2: padding mask (mask key=2,3)
    // pad_valid bit=1 => valid
    // pad_valid=0011 => token0,1 valid; token2,3 padding
    // expected: any column k=2 or 3 -> -INF
    // ----------------------------
    $display("RUN #2: padding mask (pad_valid=0011, causal=0)");
    pad_valid = 4'b0011;
    causal_en = 1'b0;

    for (q = 0; q < T; q++) begin
      for (k = 0; k < T; k++) begin
        din = 32'h3F800000 + ((q*T+k) * 32'h0010_0000);
        drive_and_check(q, k, din, /*expect_mask*/ (k >= 2));
      end
    end
    dump_matrix("DUMP padding-mask (expect col2/3 = -INF)");

    // ----------------------------
    // RUN #3: causal mask only
    // expected: upper triangle (k>q) -> -INF
    // ----------------------------
    $display("RUN #3: causal mask (pad_valid=1111, causal=1)");
    pad_valid = 4'b1111;
    causal_en = 1'b1;

    for (q = 0; q < T; q++) begin
      for (k = 0; k < T; k++) begin
        din = 32'h3F800000 + ((q*T+k) * 32'h0010_0000);
        drive_and_check(q, k, din, /*expect_mask*/ (k > q));
      end
    end
    dump_matrix("DUMP causal-mask (expect upper triangle = -INF)");

    // ----------------------------
    // RUN #4: padding + causal
    // pad_valid=0111 => token0..2 valid, token3 padding
    // expected: col3 always -INF, and also k>q -INF
    // ----------------------------
    $display("RUN #4: padding+causal (pad_valid=0111, causal=1)");
    pad_valid = 4'b0111;
    causal_en = 1'b1;

    for (q = 0; q < T; q++) begin
      for (k = 0; k < T; k++) begin
        din = 32'h3F800000 + ((q*T+k) * 32'h0010_0000);
        drive_and_check(q, k, din, /*expect_mask*/ ((k == 3) || (k > q)));
      end
    end
    dump_matrix("DUMP padding+causal (col3 and upper triangle = -INF)");

    $display("ALL TESTS PASS.");
    #20;
    $finish;
  end

endmodule

`default_nettype wire
