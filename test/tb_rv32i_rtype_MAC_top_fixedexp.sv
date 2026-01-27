// tb_rv32i_rtype_MAC_top_fixedexp.sv
/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_rv32i_mac.vvp ./test/tb_rv32i_rtype_MAC_top_fixedexp.sv
vvp ./vvp/tb_rv32i_mac.vvp
gtkwave ./vvp/tb_rv32i_mac.vcd
*/

`include "./src/rv32i_rtype_MAC_top.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_rv32i_rtype_MAC_top_fixedexp;

  localparam int unsigned M    = 8;
  localparam int unsigned N    = 8;
  localparam int unsigned K    = 4;
  localparam int unsigned KMAX = 1024;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // custom instruction IF
  logic        instr_valid;
  wire         instr_ready;
  logic [31:0] instr;
  logic [31:0] rs1_val, rs2_val;
  logic [4:0]  rd_addr;

  wire         rd_we;
  wire [4:0]   rd_waddr;
  wire [31:0]  rd_wdata;

  wire accel_busy, accel_done, accel_C_valid;

  // DUT
  rv32i_rtype_MAC_top #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(1),
    .ROW_W(ROW_W), .COL_W(COL_W), .N_W(N_W), .K_W(K_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .instr_valid(instr_valid),
    .instr_ready(instr_ready),
    .instr(instr),
    .rs1_val(rs1_val),
    .rs2_val(rs2_val),
    .rd_addr(rd_addr),

    .rd_we(rd_we),
    .rd_waddr(rd_waddr),
    .rd_wdata(rd_wdata),

    .accel_busy(accel_busy),
    .accel_done(accel_done),
    .accel_C_valid(accel_C_valid)
  );

  // Fixed pattern
  logic [31:0] A_mat [0:M-1][0:K-1];
  logic [31:0] B_mat [0:K-1][0:N-1];
  logic [31:0] C_exp_flat [0:M*N-1];

  // IMPORTANT: C_got in module scope (NOT task arg)
  logic [31:0] C_got [0:M*N-1];

  function automatic int C_idx(input int i, input int j);
    return i*N + j;
  endfunction

  task automatic build_fixed_A_B_and_expected;
    int i, j, kk;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    begin
      aval[0] = 32'h3f800000; // 1.0
      aval[1] = 32'h40000000; // 2.0
      aval[2] = 32'h3f000000; // 0.5
      aval[3] = 32'h40400000; // 3.0

      bval[0] = 32'h3f800000; // 1.0
      bval[1] = 32'h3f000000; // 0.5
      bval[2] = 32'h40000000; // 2.0
      bval[3] = 32'h40400000; // 3.0

      for (kk = 0; kk < K; kk++) begin
        for (i = 0; i < M; i++) A_mat[i][kk] = aval[(kk + i) % 4];
        for (j = 0; j < N; j++) B_mat[kk][j] = bval[(kk + j) % 4];
      end

      // expected C
      C_exp_flat[0]  = 32'h41400000; C_exp_flat[1]  = 32'h41100000; C_exp_flat[2]  = 32'h41200000; C_exp_flat[3]  = 32'h41340000;
      C_exp_flat[4]  = 32'h41400000; C_exp_flat[5]  = 32'h41100000; C_exp_flat[6]  = 32'h41200000; C_exp_flat[7]  = 32'h41340000;
      C_exp_flat[8]  = 32'h41340000; C_exp_flat[9]  = 32'h41400000; C_exp_flat[10] = 32'h41100000; C_exp_flat[11] = 32'h41200000;
      C_exp_flat[12] = 32'h41340000; C_exp_flat[13] = 32'h41400000; C_exp_flat[14] = 32'h41100000; C_exp_flat[15] = 32'h41200000;
      C_exp_flat[16] = 32'h41200000; C_exp_flat[17] = 32'h41340000; C_exp_flat[18] = 32'h41400000; C_exp_flat[19] = 32'h41100000;
      C_exp_flat[20] = 32'h41200000; C_exp_flat[21] = 32'h41340000; C_exp_flat[22] = 32'h41400000; C_exp_flat[23] = 32'h41100000;
      C_exp_flat[24] = 32'h41100000; C_exp_flat[25] = 32'h41200000; C_exp_flat[26] = 32'h41340000; C_exp_flat[27] = 32'h41400000;
      C_exp_flat[28] = 32'h41100000; C_exp_flat[29] = 32'h41200000; C_exp_flat[30] = 32'h41340000; C_exp_flat[31] = 32'h41400000;
      C_exp_flat[32] = 32'h41400000; C_exp_flat[33] = 32'h41100000; C_exp_flat[34] = 32'h41200000; C_exp_flat[35] = 32'h41340000;
      C_exp_flat[36] = 32'h41400000; C_exp_flat[37] = 32'h41100000; C_exp_flat[38] = 32'h41200000; C_exp_flat[39] = 32'h41340000;
      C_exp_flat[40] = 32'h41340000; C_exp_flat[41] = 32'h41400000; C_exp_flat[42] = 32'h41100000; C_exp_flat[43] = 32'h41200000;
      C_exp_flat[44] = 32'h41340000; C_exp_flat[45] = 32'h41400000; C_exp_flat[46] = 32'h41100000; C_exp_flat[47] = 32'h41200000;
      C_exp_flat[48] = 32'h41200000; C_exp_flat[49] = 32'h41340000; C_exp_flat[50] = 32'h41400000; C_exp_flat[51] = 32'h41100000;
      C_exp_flat[52] = 32'h41200000; C_exp_flat[53] = 32'h41340000; C_exp_flat[54] = 32'h41400000; C_exp_flat[55] = 32'h41100000;
      C_exp_flat[56] = 32'h41100000; C_exp_flat[57] = 32'h41200000; C_exp_flat[58] = 32'h41340000; C_exp_flat[59] = 32'h41400000;
      C_exp_flat[60] = 32'h41100000; C_exp_flat[61] = 32'h41200000; C_exp_flat[62] = 32'h41340000; C_exp_flat[63] = 32'h41400000;
    end
  endtask

  // IEEE754 bits -> real
  function automatic real f32_to_real(input logic [31:0] b);
    int sign;
    int exp;
    int frac;
    real mant;
    real val;
    begin
      sign = b[31];
      exp  = b[30:23];
      frac = b[22:0];

      if (exp == 0 && frac == 0) begin
        val = 0.0;
      end else if (exp == 0) begin
        mant = frac / (2.0**23);
        val  = mant * (2.0**(-126));
      end else if (exp == 255) begin
        val = 1.0/0.0;
      end else begin
        mant = 1.0 + (frac / (2.0**23));
        val  = mant * (2.0**(exp-127));
      end

      if (sign) val = -val;
      return val;
    end
  endfunction

  // R-type encoding
  localparam logic [6:0] OPC_RTYPE = 7'b0110011;
  localparam logic [6:0] F7_ACCEL  = 7'b0000001;

  localparam logic [2:0] F3_WWR    = 3'b000;
  localparam logic [2:0] F3_XWR    = 3'b001;
  localparam logic [2:0] F3_START  = 3'b010;
  localparam logic [2:0] F3_CRD    = 3'b011;
  localparam logic [2:0] F3_STAT   = 3'b100;

  function automatic [31:0] mk_rtype(
    input logic [6:0] funct7,
    input logic [4:0] rs2,
    input logic [4:0] rs1,
    input logic [2:0] funct3,
    input logic [4:0] rd,
    input logic [6:0] opcode
  );
    mk_rtype = {funct7, rs2, rs1, funct3, rd, opcode};
  endfunction

  task automatic exec_insn(
    input logic [2:0]  f3,
    input logic [31:0] rs1v,
    input logic [31:0] rs2v,
    input logic [4:0]  rd
  );
    begin
      while (instr_ready !== 1'b1) @(posedge clk);

      @(negedge clk);
      instr       <= mk_rtype(F7_ACCEL, 5'd0, 5'd0, f3, rd, OPC_RTYPE);
      rs1_val     <= rs1v;
      rs2_val     <= rs2v;
      rd_addr     <= rd;
      instr_valid <= 1'b1;

      @(negedge clk);
      instr_valid <= 1'b0;
      instr       <= '0;
      rs1_val     <= '0;
      rs2_val     <= '0;
      rd_addr     <= '0;
    end
  endtask

  function automatic [31:0] pack_W(input int row, input int kk);
    pack_W = '0;
    pack_W[ROW_W-1:0]    = row[ROW_W-1:0];
    pack_W[ROW_W +: K_W] = kk[K_W-1:0];
  endfunction

  function automatic [31:0] pack_X(input int kk, input int nn);
    pack_X = '0;
    pack_X[K_W-1:0]    = kk[K_W-1:0];
    pack_X[K_W +: N_W] = nn[N_W-1:0];
  endfunction

  function automatic [31:0] pack_C(input int row, input int col);
    pack_C = '0;
    pack_C[ROW_W-1:0]     = row[ROW_W-1:0];
    pack_C[ROW_W +: COL_W]= col[COL_W-1:0];
  endfunction

  task automatic wait_rd_write(output logic [31:0] data);
    begin
      while (rd_we !== 1'b1) @(posedge clk);
      data = rd_wdata;
      @(posedge clk);
    end
  endtask

  task automatic dump_A_B_decimal;
    int i, j, kk;
    real v;
    begin
      $display("\n================ A (M x K) decimal ================");
      for (i = 0; i < M; i++) begin
        $write("A[%0d,:] : ", i);
        for (kk = 0; kk < K; kk++) begin
          v = f32_to_real(A_mat[i][kk]);
          $write("%0.6f ", v);
        end
        $write("\n");
      end

      $display("\n================ B (K x N) decimal ================");
      for (kk = 0; kk < K; kk++) begin
        $write("B[%0d,:] : ", kk);
        for (j = 0; j < N; j++) begin
          v = f32_to_real(B_mat[kk][j]);
          $write("%0.6f ", v);
        end
        $write("\n");
      end
    end
  endtask

  // NO unpacked array argument here
  task automatic dump_C_decimal_from_C_got;
    int i, j;
    real v;
    begin
      $display("\n================ C (M x N) decimal (from ACC_CRD readback) ================");
      for (i = 0; i < M; i++) begin
        $write("C[%0d,:] : ", i);
        for (j = 0; j < N; j++) begin
          v = f32_to_real(C_got[C_idx(i,j)]);
          $write("%0.6f ", v);
        end
        $write("\n");
      end
      $display("=============================================================================\n");
    end
  endtask

  // wave
  initial begin
    $dumpfile("./vvp/tb_rv32i_mac.vcd");
    $dumpvars(0, tb_rv32i_rtype_MAC_top_fixedexp);
  end

  initial begin
    logic [31:0] stat, tmp;
    int i, j, kk;
    int unsigned timeout;
    bit seen_cvalid;

    // init
    rst = 1'b1;
    instr_valid = 1'b0;
    instr = '0;
    rs1_val = '0;
    rs2_val = '0;
    rd_addr = '0;

    build_fixed_A_B_and_expected();

    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    $display("=== TB start: rv32i_rtype_MAC_top fixed-exp (M=%0d N=%0d K=%0d) ===", M, N, K);

    // preload
    for (kk = 0; kk < K; kk++) begin
      for (i = 0; i < M; i++) exec_insn(F3_WWR, pack_W(i, kk), A_mat[i][kk], 5'd0);
      for (j = 0; j < N; j++) exec_insn(F3_XWR, pack_X(kk, j), B_mat[kk][j], 5'd0);
    end
    $display("[TB] preload done.");

    // start
    exec_insn(F3_START, {16'd0, K[15:0]}, 32'd0, 5'd0);
    $display("[TB] start issued (K_len=%0d).", K);

    // poll status until C_valid
    seen_cvalid = 1'b0;
    timeout = 0;
    while (!seen_cvalid) begin
      exec_insn(F3_STAT, 32'd0, 32'd0, 5'd1);
      wait_rd_write(stat);

      if (stat[2] === 1'b1) begin
        seen_cvalid = 1'b1;
        $display("[TB] C_valid seen. stat=%08h (C_valid=%0b done=%0b busy=%0b)",
                 stat, stat[2], stat[1], stat[0]);
      end

      timeout++;
      if (timeout > 50_000_000)
        $fatal(1, "[TB] TIMEOUT waiting C_valid. last stat=%08h", stat);
    end

    dump_A_B_decimal();

    // readback C
    for (i = 0; i < M; i++) begin
      for (j = 0; j < N; j++) begin
        exec_insn(F3_CRD, pack_C(i, j), 32'd0, 5'd2);
        wait_rd_write(tmp);
        C_got[C_idx(i,j)] = tmp;

        if (tmp !== C_exp_flat[C_idx(i,j)]) begin
          $display("[TB] FAIL C[%0d][%0d] got=%08h exp=%08h", i, j, tmp, C_exp_flat[C_idx(i,j)]);
          $fatal(1);
        end
      end
    end

    dump_C_decimal_from_C_got();

    $display("[TB] PASS ✅ all C matched expected.");
    $finish;
  end

endmodule

`default_nettype wire
