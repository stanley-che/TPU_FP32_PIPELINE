// tb_systolic_wrapper_fixed_expect.sv
// Wrapper TB: fixed 2D A/B + fixed expected C (NO calculation in TB)
// M=8, N=8, K=4
//
// iverilog -g2012 -Wall -I. -o ./vvp/tb_wrap.vvp ./test/tb_systolic_wrapper_fixed_expect.sv
// vvp ./vvp/tb_wrap.vvp

`include "./src/systolic_wrapper.sv"
`timescale 1ns/1ps

module tb_systolic_wrapper_debug;

  localparam int M = 8;
  localparam int N = 8;
  localparam int K = 4;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  logic        start;
  logic        done_clear;
  logic [15:0] K_in;

  logic [31:0] a_row_in [M];
  logic [31:0] b_col_in [N];

  logic        busy;
  logic        done;

  logic [31:0] c_out   [M][N];
  logic        c_valid [M][N];

  systolic_wrapper #(.M(M), .N(N)) dut (
    .clk(clk), .rst(rst),
    .start(start),
    .done_clear(done_clear),
    .K(K_in),
    .a_row_in(a_row_in),
    .b_col_in(b_col_in),
    .busy(busy),
    .done(done),
    .c_out(c_out),
    .c_valid(c_valid)
  );

  // stimulus tables
  logic [31:0] A_mat [0:M-1][0:K-1];
  logic [31:0] B_mat [0:K-1][0:N-1];

  task automatic init_vectors;
    int i,j,kk;
    logic [31:0] aval[0:3], bval[0:3];
    begin
      aval[0]=32'h3f800000; aval[1]=32'h40000000; aval[2]=32'h3f000000; aval[3]=32'h40400000;
      bval[0]=32'h3f800000; bval[1]=32'h3f000000; bval[2]=32'h40000000; bval[3]=32'h40400000;

      for (kk=0; kk<K; kk++) begin
        for (i=0; i<M; i++) A_mat[i][kk] = aval[(kk+i)%4];
        for (j=0; j<N; j++) B_mat[kk][j] = bval[(kk+j)%4];
      end
    end
  endtask

  task automatic drive_k(input int kk);
    int i,j;
    begin
      for (i=0;i<M;i++) a_row_in[i] = A_mat[i][kk];
      for (j=0;j<N;j++) b_col_in[j] = B_mat[kk][j];
    end
  endtask

  // ----------------------------
  // DEBUG monitors
  // ----------------------------

  // 1) wrapper launch monitor
  always_ff @(posedge clk) begin
    if (!rst && dut.step_valid) begin
      $display("[MON] LAUNCH: k_idx=%0d  a_lat0=%08h b_lat0=%08h  k_first=%0b k_last=%0b  (t=%0t)",
               dut.k_idx, dut.u_sa.a_lat[0], dut.u_sa.b_lat[0],
               dut.u_sa.k_first_lat, dut.u_sa.k_last_lat, $time);
    end
  end

  // 2) PE[0][0] output monitor
  // (hier access: u_sa.pe_out_valid / pe_psum_in / pe_psum_out)
  always_ff @(posedge clk) begin
    if (!rst && dut.u_sa.pe_out_valid[0][0]) begin
      $display("[MON] PE00 out_valid: psum_in=%08h psum_out=%08h  (t=%0t)",
               dut.u_sa.pe_psum_in[0][0], dut.u_sa.pe_psum_out[0][0], $time);
    end
  end

  // 3) step_ready transitions
  always_ff @(posedge clk) begin
    if (!rst) begin
      if (dut.u_sa.step_ready) $display("[MON] u_sa.step_ready=1 (t=%0t)", $time);
    end
  end

  // ----------------------------
  // main
  // ----------------------------
  initial begin
    int i,j;
    int kk_sent;
    int unsigned timeout;

    rst=1; start=0; done_clear=0; K_in=K;
    for (i=0;i<M;i++) a_row_in[i]=32'h0;
    for (j=0;j<N;j++) b_col_in[j]=32'h0;

    init_vectors();

    repeat(6) @(posedge clk);
    rst=0;
    repeat(2) @(posedge clk);

    done_clear=1; @(posedge clk); done_clear=0;

    @(posedge clk); start=1; @(posedge clk); start=0;

    // wait busy
    timeout=0;
    while (busy!==1'b1) begin
      @(posedge clk);
      timeout++;
      if (timeout>1_000_000) $fatal(1,"TIMEOUT wait busy");
    end

    kk_sent=0;
    timeout=0;

    while (done!==1'b1) begin
      // only feed when array idle
      @(negedge clk);
      if (busy && (kk_sent<K) && dut.u_sa.step_ready) begin
        drive_k(kk_sent);
        $display("[TB] drive k=%0d  A00=%08h B00=%08h", kk_sent, A_mat[0][kk_sent], B_mat[kk_sent][0]);
      end

      @(posedge clk);
      timeout++;
      if (timeout>100_000_000) $fatal(1,"TIMEOUT wait done");

      if (dut.step_valid) kk_sent++;
    end

    $display("[TB] DONE. kk_sent=%0d  C00=%08h", kk_sent, dut.u_sa.psum_reg[0][0]);

    // dump a few cells
    $display("[TB] C00=%08h C01=%08h C10=%08h", dut.u_sa.psum_reg[0][0], dut.u_sa.psum_reg[0][1], dut.u_sa.psum_reg[1][0]);

    $finish;
  end

endmodule
