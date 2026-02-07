/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_relu.vvp \
  ./test/tb_relu_custom_rtype.sv

vvp ./vvp/tb_relu.vvp
*/

`include "./src/EPU/relu_custom_rtype.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_relu_custom_rtype;

  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk;
  logic rst;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
  end

  // ------------------------------------------------------------
  // DUT interface
  // ------------------------------------------------------------
  logic        instr_valid;
  logic        instr_ready;
  logic [31:0] instr;
  logic [31:0] rs1_val;
  logic [31:0] rs2_val;
  logic [4:0]  rd_addr;

  logic        rd_we;
  logic [4:0]  rd_waddr;
  logic [31:0] rd_wdata;

  logic accel_busy;
  logic accel_done;
  logic accel_C_valid;

  relu_custom_rtype #(
    .M(M),
    .N(N),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .NB(NB)
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

  // ------------------------------------------------------------
  // Instruction encoding helpers
  // ------------------------------------------------------------
  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_RELU   = 7'h03;

  function automatic [31:0] make_relu_inst(input [2:0] funct3);
    make_relu_inst = {F7_RELU, 5'd0, 5'd0, funct3, 5'd0, OPC_RTYPE};
  endfunction

  localparam logic [2:0] F3_AWR   = 3'b000;
  localparam logic [2:0] F3_START = 3'b001;
  localparam logic [2:0] F3_STAT  = 3'b010;
  localparam logic [2:0] F3_YRD   = 3'b011;

  // ------------------------------------------------------------
  // FP32 patterns
  // ------------------------------------------------------------
  localparam logic [31:0] FP32_P0  = 32'h0000_0000;
  localparam logic [31:0] FP32_M0  = 32'h8000_0000;
  localparam logic [31:0] FP32_P1  = 32'h3F80_0000;
  localparam logic [31:0] FP32_M1  = 32'hBF80_0000;
  localparam logic [31:0] FP32_P2  = 32'h4000_0000;
  localparam logic [31:0] FP32_NEG = 32'hC120_0000;
  localparam logic [31:0] FP32_P3  = 32'h4040_0000;
  localparam logic [31:0] FP32_M2  = 32'hC000_0000;

  function automatic logic [31:0] relu_golden(input logic [31:0] x);
    relu_golden = x[31] ? 32'h0 : x;
  endfunction

  // ------------------------------------------------------------
  // FP32 decode (NO $bitstoshortreal)
  // ------------------------------------------------------------
  
  function automatic real pow2_int(input int e);
    real p;
    int i;
  begin
    p = 1.0;
    if (e >= 0) begin
      for (i = 0; i < e; i++) p = p * 2.0;
    end else begin
      for (i = 0; i < (-e); i++) p = p * 0.5;
    end
    pow2_int = p;
  end
  endfunction
   
  function automatic real fp32_to_real_no_b2sr(input logic [31:0] x);
    int  sign;
    int  exp;
    int  frac;
    real m;
    int  e;
    real v;
  begin
    sign = x[31];
    exp  = x[30:23];
    frac = x[22:0];

    // Special cases
    if (exp == 8'hFF) begin
      if (frac == 0) begin
        v = (sign ? -1.0 : 1.0) * (1.0/0.0); // +/- inf
      end else begin
        v = 0.0/0.0; // NaN
      end
      fp32_to_real_no_b2sr = v;
    end
    else if (exp == 8'h00) begin
      // Zero or denormal
      if (frac == 0) begin
        // +0 / -0
        fp32_to_real_no_b2sr = (sign ? -0.0 : 0.0);
      end else begin
        // denormal: m = frac / 2^23, e = -126
        m = $itor(frac) / pow2_int(23);
        e = -126;
        v = (sign ? -1.0 : 1.0) * m * pow2_int(e);
        fp32_to_real_no_b2sr = v;
      end
    end
    else begin
      // normal: m = 1 + frac/2^23, e = exp-127
      m = 1.0 + ($itor(frac) / pow2_int(23));
      e = exp - 127;
      v = (sign ? -1.0 : 1.0) * m * pow2_int(e);
      fp32_to_real_no_b2sr = v;
    end
  end
  endfunction

  // ------------------------------------------------------------
  // Tasks: issue instruction
  // ------------------------------------------------------------
  task automatic issue_inst(
    input [31:0] inst,
    input [31:0] rs1,
    input [31:0] rs2,
    input [4:0]  rd
  );
  begin
    @(negedge clk);
    instr_valid = 1'b1;
    instr       = inst;
    rs1_val     = rs1;
    rs2_val     = rs2;
    rd_addr     = rd;

    @(posedge clk);
    @(negedge clk);
    instr_valid = 1'b0;
  end
  endtask

  task automatic wait_rd_write(output logic [31:0] data);
    int i;
  begin
    data = 'x;
    for (i = 0; i < 500; i++) begin
      @(posedge clk);
      if (rd_we) begin
        data = rd_wdata;
        disable wait_rd_write;
      end
    end
    $display("[TB][TIMEOUT] rd_we not seen");
    $finish;
  end
  endtask

  // ------------------------------------------------------------
  // Test storage
  // ------------------------------------------------------------
  logic [31:0] X [0:M-1][0:N-1];
  logic [31:0] Y [0:M-1][0:N-1];
  logic [31:0] G [0:M-1][0:N-1];
  logic [31:0] stat;
    // ------------------------------------------------------------
  // FP32 decode (NO $bitstoshortreal, Icarus friendly)
  // ------------------------------------------------------------
  

  task automatic print_X();
    integer r, c;
  begin
    $display("----- X input (hex + approx real) -----");
    for (r = 0; r < M; r = r + 1) begin
      $write("[%0d] ", r);
      for (c = 0; c < N; c = c + 1) begin
        $write("%08h(%0.6f) ", X[r][c], fp32_to_real_no_b2sr(X[r][c]));
      end
      $write("\n");
    end
    $display("--------------------------------------");
  end
  endtask

  task automatic print_Y();
    integer r, c;
  begin
    $display("----- Y output (hex + approx real) -----");
    for (r = 0; r < M; r = r + 1) begin
      $write("[%0d] ", r);
      for (c = 0; c < N; c = c + 1) begin
        $write("%08h(%0.6f) ", Y[r][c], fp32_to_real_no_b2sr(Y[r][c]));
      end
      $write("\n");
    end
    $display("---------------------------------------");
  end
  endtask

  task automatic print_G();
    integer r, c;
  begin
    $display("----- Golden (hex + approx real) -----");
    for (r = 0; r < M; r = r + 1) begin
      $write("[%0d] ", r);
      for (c = 0; c < N; c = c + 1) begin
        $write("%08h(%0.6f) ", G[r][c], fp32_to_real_no_b2sr(G[r][c]));
      end
      $write("\n");
    end
    $display("-------------------------------------");
  end
  endtask

  
  // ------------------------------------------------------------
  // Main test
  // ------------------------------------------------------------
  initial begin
    instr_valid = 0;
    instr       = '0;
    rs1_val     = '0;
    rs2_val     = '0;
    rd_addr     = '0;

    // init arrays
    for (int r = 0; r < M; r++) begin
      for (int c = 0; c < N; c++) begin
        X[r][c] = 32'h0;
        Y[r][c] = 32'h0;
        G[r][c] = 32'h0;
      end
    end

    @(negedge rst);
    repeat (2) @(posedge clk);

    $display("[TB] ==== RELU custom R-type test begin ====");

    // -------------------------
    // Prepare X matrix
    // -------------------------
    for (int r = 0; r < M; r++) begin
      for (int c = 0; c < N; c++) begin
        case ((r*N + c) % 8)
          0: X[r][c] = FP32_M1;
          1: X[r][c] = FP32_P1;
          2: X[r][c] = FP32_M0;
          3: X[r][c] = FP32_P0;
          4: X[r][c] = FP32_NEG;
          5: X[r][c] = FP32_P2;
          6: X[r][c] = FP32_M2;
          7: X[r][c] = FP32_P3;
        endcase
      end
    end

    // 先把 input 印出來
    print_X();
    // -------------------------
    // RELU_AWR
    // -------------------------
    $display("[TB] Writing X matrix via RELU_AWR...");
    for (int r = 0; r < M; r++) begin
      for (int c = 0; c < N; c++) begin
        issue_inst(
          make_relu_inst(F3_AWR),
          {r[15:0], c[15:0]},
          X[r][c],
          5'd0
        );
        @(posedge clk);
      end
    end

    // -------------------------
    // RELU_START
    // -------------------------
    $display("[TB] Start ReLU engine...");
    issue_inst(make_relu_inst(F3_START), 32'h0, 32'h0, 5'd0);

    // -------------------------
    // Poll RELU_STAT
    // -------------------------
    $display("[TB] Polling RELU_STAT...");
    do begin
      issue_inst(make_relu_inst(F3_STAT), 32'h0, 32'h0, 5'd1);
      wait_rd_write(stat);
    end while (stat[1] == 1'b1); // busy

    $display("[TB] Engine done.");

    // -------------------------
    // RELU_YRD + check
    // -------------------------
    $display("[TB] Reading Y matrix via RELU_YRD...");
    for (int r = 0; r < M; r++) begin
      for (int c = 0; c < N; c++) begin
        logic [31:0] y, exp;
        exp = relu_golden(X[r][c]);

        issue_inst(
          make_relu_inst(F3_YRD),
          {r[15:0], c[15:0]},
          32'h0,
          5'd2
        );
        wait_rd_write(y);

        Y[r][c] = y;
        G[r][c] = exp;


        if (y !== exp) begin
          $display("[TB][FAIL] (%0d,%0d) X=0x%08h expY=0x%08h gotY=0x%08h",
                   r, c, X[r][c], exp, y);
          $finish;
        end
      end
    end
    
    print_Y();
    print_G();

    $display("[TB][PASS] RELU custom instruction test passed.");
    repeat (10) @(posedge clk);
    $finish;
  end

endmodule

`default_nettype wire
