// tb_attention_score_top_scaling.sv
// ------------------------------------------------------------
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_att_scale.vvp ./test/tb_attention_score_top_scaling.sv
// vvp ./vvp/tb_att_scale.vvp
// ------------------------------------------------------------
`include "./src/EPU/attention_score/attention_scaling_tile.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_attention_scaling_tile;

  localparam int unsigned T      = 4;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);

  localparam int unsigned ROW_W = (T<=1)?1:$clog2(T);
  localparam int unsigned COL_W = (T<=1)?1:$clog2(T);

  // clock/reset
  logic clk, rst;
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
  end

  // DUT I/O
  logic        start;
  logic [15:0] D_len;
  logic        busy, done;

  logic              in_score_re;
  logic [ROW_W-1:0]  in_score_tq;
  logic [COL_W-1:0]  in_score_tk;
  logic [DATA_W-1:0] in_score_rdata;
  logic              in_score_rvalid;

  logic              c_rd_en;
  logic              c_rd_re;
  logic [ROW_W-1:0]  c_rd_row;
  logic [COL_W-1:0]  c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic              c_rd_rvalid;

  logic              C_valid;

  attention_scaling_tile #(
    .T(T),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start(start),
    .D_len(D_len),
    .busy(busy),
    .done(done),

    .in_score_re(in_score_re),
    .in_score_tq(in_score_tq),
    .in_score_tk(in_score_tk),
    .in_score_rdata(in_score_rdata),
    .in_score_rvalid(in_score_rvalid),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid),

    .C_valid(C_valid)
  );

  // ------------------------------------------------------------
  // Score memory model (1-cycle latency)
  // ------------------------------------------------------------
  logic req_d;
  logic [ROW_W-1:0] tq_d;
  logic [COL_W-1:0] tk_d;

  logic [DATA_W-1:0] score_mem [0:T-1][0:T-1];

  integer ii, jj;

  // FP32 bits -> real (iverilog-safe)
  function automatic real fp32_to_real(input logic [31:0] b);
    int  s;
    int  e;
    int  frac;
    real mant;
    real val;
    begin
      s    = b[31];
      e    = b[30:23];
      frac = b[22:0];

      if (e == 0) begin
        if (frac == 0) begin
          val = 0.0;
        end else begin
          mant = frac / 8388608.0;     // 2^23
          val  = mant * (2.0 ** (-126));
        end
      end else if (e == 255) begin
        if (frac == 0) val = 1.0/0.0;
        else           val = 0.0/0.0;
      end else begin
        mant = 1.0 + (frac / 8388608.0);
        val  = mant * (2.0 ** (e - 127));
      end

      if (s) fp32_to_real = -val;
      else   fp32_to_real =  val;
    end
  endfunction

  // 初始化 score_mem：填 1.0~16.0 的 FP32 hex
  // (對 T=4 足夠；你要更大再擴充 table)
  initial begin
    // row 0: 1 2 3 4
    score_mem[0][0] = 32'h3F800000; // 1.0
    score_mem[0][1] = 32'h40000000; // 2.0
    score_mem[0][2] = 32'h40400000; // 3.0
    score_mem[0][3] = 32'h40800000; // 4.0
    // row 1: 5 6 7 8
    score_mem[1][0] = 32'h40A00000; // 5.0
    score_mem[1][1] = 32'h40C00000; // 6.0
    score_mem[1][2] = 32'h40E00000; // 7.0
    score_mem[1][3] = 32'h41000000; // 8.0
    // row 2: 9 10 11 12
    score_mem[2][0] = 32'h41100000; // 9.0
    score_mem[2][1] = 32'h41200000; // 10.0
    score_mem[2][2] = 32'h41300000; // 11.0
    score_mem[2][3] = 32'h41400000; // 12.0
    // row 3: 13 14 15 16
    score_mem[3][0] = 32'h41500000; // 13.0
    score_mem[3][1] = 32'h41600000; // 14.0
    score_mem[3][2] = 32'h41700000; // 15.0
    score_mem[3][3] = 32'h41800000; // 16.0
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      req_d           <= 1'b0;
      tq_d            <= '0;
      tk_d            <= '0;
      in_score_rvalid <= 1'b0;
      in_score_rdata  <= '0;
    end else begin
      req_d <= in_score_re;
      tq_d  <= in_score_tq;
      tk_d  <= in_score_tk;

      in_score_rvalid <= req_d;
      if (req_d) in_score_rdata <= score_mem[tq_d][tk_d];
      else       in_score_rdata <= '0;
    end
  end

  // C read helper
  task automatic c_read_one(input int r, input int c, output logic [31:0] bits);
    begin
      @(posedge clk);
      c_rd_en  <= 1'b1;
      c_rd_re  <= 1'b1;
      c_rd_row <= r[ROW_W-1:0];
      c_rd_col <= c[COL_W-1:0];

      @(posedge clk);
      c_rd_re <= 1'b0;

      while (c_rd_rvalid !== 1'b1) @(posedge clk);
      bits = c_rd_rdata;

      @(posedge clk);
      c_rd_en <= 1'b0;
    end
  endtask

  // iverilog-safe print
  task automatic print_f32(input string tag, input logic [31:0] bits);
    real v;
    begin
      v = fp32_to_real(bits);
      $display("%s  bits=0x%08h  val=%f", tag, bits, v);
    end
  endtask

  logic [31:0] c_bits;

  initial begin
    start   = 1'b0;
    D_len   = 16'd4;   // alpha=0.5
    c_rd_en = 1'b0;
    c_rd_re = 1'b0;
    c_rd_row= '0;
    c_rd_col= '0;

    @(negedge rst);
    repeat (2) @(posedge clk);

    $display("=== SCORE (input) ===");
    for (ii = 0; ii < T; ii++) begin
      for (jj = 0; jj < T; jj++) begin
        print_f32($sformatf("S[%0d][%0d]", ii, jj), score_mem[ii][jj]);
      end
    end

    // start pulse
    @(posedge clk);
    start <= 1'b1;
    @(posedge clk);
    start <= 1'b0;

    $display("Waiting for C_valid...");
    while (C_valid !== 1'b1) @(posedge clk);
    $display("C_valid asserted!");

    $display("=== C (scaled score) readback ===");
    for (ii = 0; ii < T; ii++) begin
      for (jj = 0; jj < T; jj++) begin
        c_read_one(ii, jj, c_bits);
        print_f32($sformatf("C[%0d][%0d]", ii, jj), c_bits);
      end
    end

    $display("TEST DONE");
    #50;
    $finish;
  end

endmodule

`default_nettype wire
