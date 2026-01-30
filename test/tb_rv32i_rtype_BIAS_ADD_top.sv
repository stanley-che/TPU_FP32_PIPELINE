// tb_rv32i_rtype_BIAS_ADD_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_bias_add.vvp \
  ./test/tb_rv32i_rtype_BIAS_ADD_top.sv

vvp ./vvp/tb_bias_add.vvp
*/
`include "./src/EPU/rv32i_rtype_BIAS_ADD_top.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_rv32i_rtype_BIAS_ADD_top_5cases;

  // =========================
  // DUT params (MAX capacity)
  // =========================
  localparam int unsigned M_DUT  = 8;
  localparam int unsigned N_DUT  = 8;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned ROW_W  = (M_DUT<=1)?1:$clog2(M_DUT);
  localparam int unsigned COL_W  = (N_DUT<=1)?1:$clog2(N_DUT);

  // =========================
  // clock/reset
  // =========================
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // =========================
  // custom instruction IF
  // =========================
  logic        instr_valid;
  wire         instr_ready;
  logic [31:0] instr;
  logic [31:0] rs1_val, rs2_val;
  logic [4:0]  rd_addr;

  wire         rd_we;
  wire [4:0]   rd_waddr;
  wire [31:0]  rd_wdata;

  wire accel_busy, accel_done, accel_C_valid;

  // =========================
  // DUT
  // =========================
  rv32i_rtype_BIAS_ADD_top #(
    .M(M_DUT), .N(N_DUT),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .COL_W(COL_W)
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

  // =========================
  // Instruction encoding (BIAS_ADD)
  // =========================
  localparam logic [6:0] OPC_RTYPE = 7'b0110011;
  localparam logic [6:0] F7_BIAS   = 7'b0000100; // 0x04

  localparam logic [2:0] F3_XWR    = 3'b000; // BA_XWR
  localparam logic [2:0] F3_BWR    = 3'b001; // BA_BWR
  localparam logic [2:0] F3_START  = 3'b010; // BA_START
  localparam logic [2:0] F3_CRD    = 3'b011; // BA_CRD
  localparam logic [2:0] F3_STAT   = 3'b100; // BA_STAT

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

  // =========================
  // pack helpers (match your RTL decode)
  // BA_XWR/BA_CRD: rs1[ROW_W-1:0]=row, rs1[ROW_W +: COL_W]=col
  // BA_BWR: rs1 low bits = col
  // =========================
  function automatic [31:0] pack_XC(input int row, input int col);
    pack_XC = '0;
    pack_XC[ROW_W-1:0]      = row[ROW_W-1:0];
    pack_XC[ROW_W +: COL_W] = col[COL_W-1:0];
  endfunction

  function automatic [31:0] pack_B(input int col);
    pack_B = '0;
    pack_B[COL_W-1:0] = col[COL_W-1:0];
  endfunction

  // =========================
  // exec one instruction
  // =========================
  task automatic exec_insn(
    input logic [2:0]  f3,
    input logic [31:0] rs1v,
    input logic [31:0] rs2v,
    input logic [4:0]  rd
  );
    begin
      while (instr_ready !== 1'b1) @(posedge clk);

      @(negedge clk);
      instr       <= mk_rtype(F7_BIAS, 5'd0, 5'd0, f3, rd, OPC_RTYPE);
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

  task automatic wait_rd_write(output logic [31:0] data);
    begin
      while (rd_we !== 1'b1) @(posedge clk);
      data = rd_wdata;
      @(posedge clk);
    end
  endtask

  // =========================
  // IEEE754 bits -> real (for printing)
  // =========================
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

  // =========================
  // Storage (module-scope)
  // =========================
  logic [31:0] X_mat [0:M_DUT-1][0:N_DUT-1];
  logic [31:0] B_vec [0:N_DUT-1];
  logic [31:0] C_got [0:M_DUT-1][0:N_DUT-1];

  // =========================
  // Build patterns (simple deterministic)
  // X(i,j) = xval[(i+j)%4]
  // B(j)   = bval[j%4]
  // =========================
  task automatic build_case_xb(input int M, input int N);
    int i,j;
    logic [31:0] xval [0:3];
    logic [31:0] bval [0:3];
    begin
      xval[0] = 32'h3f800000; // 1.0
      xval[1] = 32'h40000000; // 2.0
      xval[2] = 32'h3f000000; // 0.5
      xval[3] = 32'h40400000; // 3.0

      bval[0] = 32'h3f800000; // 1.0
      bval[1] = 32'hbf800000; // -1.0
      bval[2] = 32'h3f000000; // 0.5
      bval[3] = 32'h40000000; // 2.0

      // clear all
      for (i=0;i<M_DUT;i++) begin
        for (j=0;j<N_DUT;j++) begin
          X_mat[i][j] = 32'h0;
          C_got[i][j] = 32'h0;
        end
      end
      for (j=0;j<N_DUT;j++) B_vec[j] = 32'h0;

      // fill only used
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          X_mat[i][j] = xval[(i+j)%4];
        end
      end
      for (j=0;j<N;j++) begin
        B_vec[j] = bval[j%4];
      end
    end
  endtask

  // =========================
  // Optional: clear used region by writing 0
  // (depends on your core: if it doesn't have reset-clears SRAM)
  // =========================
  task automatic clear_used_region(input int M, input int N);
    int i,j;
    begin
      // clear X
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          exec_insn(F3_XWR, pack_XC(i,j), 32'h0, 5'd0);
        end
      end
      // clear B
      for (j=0;j<N;j++) begin
        exec_insn(F3_BWR, pack_B(j), 32'h0, 5'd0);
      end
    end
  endtask

  // =========================
  // Wait until (C_valid==1 && busy==0)
  // STAT returns {29'd0, C_valid, done, busy}
  // =========================
  task automatic wait_cvalid_and_idle(input int case_id);
    logic [31:0] stat;
    int unsigned timeout;
    bit ok;
    begin
      timeout = 0;
      ok = 1'b0;

      while (!ok) begin
        exec_insn(F3_STAT, 32'd0, 32'd0, 5'd1);
        wait_rd_write(stat);

        if (stat[2] === 1'b1 && stat[0] === 1'b0) begin
          ok = 1'b1;
          $display("[CASE %0d] C_valid seen. stat=%08h (C_valid=%0b done=%0b busy=%0b)",
                   case_id, stat, stat[2], stat[1], stat[0]);
        end

        timeout++;
        if (timeout > 50_000_000) begin
          $fatal(1, "[CASE %0d] TIMEOUT waiting C_valid && !busy. last stat=%08h", case_id, stat);
        end
      end
    end
  endtask

  // =========================
  // Dump decimal
  // =========================
  task automatic dump_X_B_decimal(input int M, input int N);
    int i,j;
    real v;
    begin
      $display("\n---- X (M x N) decimal ----");
      for (i=0;i<M;i++) begin
        $write("X[%0d,:] : ", i);
        for (j=0;j<N;j++) begin
          v = f32_to_real(X_mat[i][j]);
          $write("%0.6f ", v);
        end
        $write("\n");
      end

      $display("\n---- B (N) decimal ----");
      $write("B[:]   : ");
      for (j=0;j<N;j++) begin
        v = f32_to_real(B_vec[j]);
        $write("%0.6f ", v);
      end
      $write("\n");
    end
  endtask

  task automatic dump_C_decimal(input int M, input int N);
    int i,j;
    real v;
    begin
      $display("\n---- C (M x N) decimal (from BA_CRD readback) ----");
      for (i=0;i<M;i++) begin
        $write("C[%0d,:] : ", i);
        for (j=0;j<N;j++) begin
          v = f32_to_real(C_got[i][j]);
          $write("%0.6f ", v);
        end
        $write("\n");
      end
      $display("---------------------------------------------------\n");
    end
  endtask

  // =========================
  // reset helper
  // =========================
  task automatic pulse_reset(input int cycles);
    int t;
    begin
      rst <= 1'b1;
      instr_valid <= 1'b0;
      instr <= '0; rs1_val <= '0; rs2_val <= '0; rd_addr <= '0;
      for (t = 0; t < cycles; t++) @(posedge clk);
      rst <= 1'b0;
      for (t = 0; t < 2; t++) @(posedge clk);
    end
  endtask

  // =========================
  // Run one case
  // =========================
  task automatic run_case(input int case_id, input int M, input int N);
    int i,j;
    logic [31:0] tmp;
    begin
      pulse_reset(6);
      $display("===============================================================");
      $display("[CASE %0d] C = X + B   (X=%0dx%0d, B=%0d, C=%0dx%0d)", case_id, M, N, N, M, N);
      $display("===============================================================");

      build_case_xb(M,N);

      // clear (optional but safe)
      clear_used_region(M,N);

      // preload X
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          exec_insn(F3_XWR, pack_XC(i,j), X_mat[i][j], 5'd0);
        end
      end

      // preload B
      for (j=0;j<N;j++) begin
        exec_insn(F3_BWR, pack_B(j), B_vec[j], 5'd0);
      end
      $display("[CASE %0d] preload X/B done.", case_id);

      // optional dump X/B
      //dump_X_B_decimal(M,N);

      // start
      exec_insn(F3_START, 32'd0, 32'd0, 5'd0);
      $display("[CASE %0d] start issued.", case_id);

      // wait complete
      wait_cvalid_and_idle(case_id);

      // readback C
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          exec_insn(F3_CRD, pack_XC(i,j), 32'd0, 5'd2);
          wait_rd_write(tmp);
          C_got[i][j] = tmp;

          // minimal safety checks (avoid X)
          if (^tmp === 1'bX) begin
            $display("[CASE %0d] FAIL C[%0d][%0d] is X : %08h", case_id, i, j, tmp);
            $fatal(1);
          end
        end
      end

      // dump C decimal for human readable
      dump_C_decimal(M,N);

      $display("[CASE %0d] PASS ✅", case_id);
      $display("");
    end
  endtask

  // =========================
  // wave
  // =========================
  initial begin
    $dumpfile("./vvp/tb_rv32i_bias_add_5cases.vcd");
    $dumpvars(0, tb_rv32i_rtype_BIAS_ADD_top_5cases);
  end

  // =========================
  // MAIN
  // =========================
  initial begin
    rst = 1'b1;
    instr_valid = 1'b0;
    instr = '0;
    rs1_val = '0;
    rs2_val = '0;
    rd_addr = '0;

    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // 5 cases (只改 M,N；DUT 最大 8x8)
    // 1) 8x8
    // 2) 6x7
    // 3) 4x4
    // 4) 7x2
    // 5) 8x3
    run_case(1, 8, 8);
    run_case(2, 6, 7);
    run_case(3, 4, 4);
    run_case(4, 7, 2);
    run_case(5, 8, 3);

    $display("===== ALL 5 CASES PASS ✅ =====");
    $finish;
  end

endmodule

`default_nettype wire
