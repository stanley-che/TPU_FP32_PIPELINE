// tb_rv32i_rtype_TRANSPOSE_top.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_tp1.vvp \
  ./test/tb_rv32i_rtype_TRANSPOSE_to.sv

vvp ./vvp/tb_tp1.vvp
*/


`include "./src/EPU/rv32i_rtype_TRANSPOSE_top.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_tp_custom_rtype_fp32;

  localparam int unsigned NRows  = 8;
  localparam int unsigned NCols  = 8;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;

  // clock/reset
  logic clk;
  logic rst;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // ----------------------------
  // Custom-instr interface signals
  // ----------------------------
  logic        instr_valid;
  logic        instr_ready;
  logic [31:0] instr;
  logic [31:0] rs1_val;
  logic [31:0] rs2_val;
  logic [4:0]  rd_addr;

  logic        rd_we;
  logic [4:0]  rd_waddr;
  logic [31:0] rd_wdata;

  logic accel_busy, accel_done, accel_C_valid;

  // DUT: wrapper
  transpose_custom_rtype #(
    .M(NRows),
    .N(NCols),
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

  // ----------------------------
  // helpers: fp32 bits
  // ----------------------------
  function automatic [31:0] fp32_bits(input int r, input int c);
    begin
      fp32_bits = 32'h3f800000 + (r << 8) + c;
    end
  endfunction

  // ----------------------------
  // R-type encode helper
  // instr[31:25]=funct7, [24:20]=rs2, [19:15]=rs1, [14:12]=funct3,
  // [11:7]=rd, [6:0]=opcode
  // ----------------------------
  function automatic [31:0] enc_rtype(
    input [6:0] funct7,
    input [4:0] rs2,
    input [4:0] rs1,
    input [2:0] funct3,
    input [4:0] rd,
    input [6:0] opcode
  );
    begin
      enc_rtype = {funct7, rs2, rs1, funct3, rd, opcode};
    end
  endfunction

  // constants (match your table)
  localparam [6:0] OPC_RTYPE = 7'h33;
  localparam [6:0] F7_TP     = 7'h02;

  localparam [2:0] F3_AWR    = 3'b000;
  localparam [2:0] F3_START  = 3'b001;
  localparam [2:0] F3_STAT   = 3'b010;
  localparam [2:0] F3_BRD    = 3'b011;

  // ----------------------------
  // drive protocol tasks
  // ----------------------------

  task automatic wait_cycles(input int n);
    for (int i=0; i<n; i++) @(posedge clk);
  endtask

  // One instruction issue (handshake). Leaves instr_valid low after handshake.
  task automatic issue_instr(
    input [31:0] instr_i,
    input [31:0] rs1_i,
    input [31:0] rs2_i,
    input [4:0]  rd_i
  );
    begin
      // wait until ready
      while (!instr_ready) @(posedge clk);

      @(negedge clk);
      instr       <= instr_i;
      rs1_val      <= rs1_i;
      rs2_val      <= rs2_i;
      rd_addr      <= rd_i;
      instr_valid  <= 1'b1;

      // hold until accepted (usually 1 cycle)
      do @(posedge clk); while (!(instr_valid && instr_ready));

      @(negedge clk);
      instr_valid <= 1'b0;
    end
  endtask

  // AWR: write A[row,col] = data
  task automatic TP_AWR(input int r, input int c, input [31:0] data);
    reg [31:0] rs1_pack;
    reg [31:0] inst;
    begin
      rs1_pack = {r[15:0], c[15:0]}; // rs1={row[15:0],col[15:0]}
      inst     = enc_rtype(F7_TP, 5'd0, 5'd0, F3_AWR, 5'd0, OPC_RTYPE);
      issue_instr(inst, rs1_pack, data, 5'd0);
    end
  endtask

  // START
  task automatic TP_START();
    reg [31:0] inst;
    begin
      inst = enc_rtype(F7_TP, 5'd0, 5'd0, F3_START, 5'd0, OPC_RTYPE);
      issue_instr(inst, 32'b0, 32'b0, 5'd0);
    end
  endtask

  // STAT: return {done,busy} in [1:0]
  task automatic TP_STAT(output logic done_o, output logic busy_o);
  reg [31:0] inst;
  int guard;
  begin
    inst = enc_rtype(F7_TP, 5'd0, 5'd0, F3_STAT, 5'd1, OPC_RTYPE);
    issue_instr(inst, 32'b0, 32'b0, 5'd1);

    guard = 0;
    while (!rd_we) begin
      @(posedge clk);
      guard++;
      if (guard > 1000) begin
        $display("[TB] TP_STAT TIMEOUT");
        $fatal(1);
      end
    end
    done_o = rd_wdata[0];
    busy_o = rd_wdata[1];
  end
  endtask


  // BRD: read B[row,col] -> returns data (wait for rd_we)
  task automatic TP_BRD(input int r, input int c, output logic [31:0] data_o);
    reg [31:0] rs1_pack;
    reg [31:0] inst;
    int guard;
    begin
      rs1_pack = {r[15:0], c[15:0]};
      inst     = enc_rtype(F7_TP, 5'd0, 5'd0, F3_BRD, 5'd2, OPC_RTYPE);

      issue_instr(inst, rs1_pack, 32'b0, 5'd2);

      // wrapper會 stall，等 cpu_b_rvalid 回來那拍 rd_we=1
      guard = 0;
      data_o = 32'h0;
      while (!rd_we) begin
        @(posedge clk);
        guard++;
        if (guard > 5000) begin
          $display("[TB] TP_BRD TIMEOUT r=%0d c=%0d", r, c);
          $fatal(1);
        end
      end
      data_o = rd_wdata;
    end
  endtask

  // ----------------------------
  // main test
  // ----------------------------
  int r,c;
  logic [31:0] got, exp;
  logic stat_done, stat_busy;

  initial begin
    $dumpfile("./vvp/tb_tp_custom.vcd");
    $dumpvars(0, tb_tp_custom_rtype_fp32);

    // init
    rst = 1'b1;
    instr_valid = 1'b0;
    instr = 32'b0;
    rs1_val = 32'b0;
    rs2_val = 32'b0;
    rd_addr = 5'd0;

    wait_cycles(5);
    rst = 1'b0;
    wait_cycles(5);

    // preload A via TP_AWR
    $display("[TB] preload A FP32 via TP_AWR...");
    for (r=0; r<NRows; r++) begin
      for (c=0; c<NCols; c++) begin
        TP_AWR(r, c, fp32_bits(r,c));
      end
    end

    // start transpose
    $display("[TB] TP_START...");
    TP_START();

    // poll status until done
    $display("[TB] polling TP_STAT...");
    do begin
      TP_STAT(stat_done, stat_busy);
      @(posedge clk);
    end while (!stat_done);

    $display("[TB] done!");

    // read back B[col,row] and compare to A[row,col]
    $display("[TB] check B transpose via TP_BRD...");
    for (r=0; r<NRows; r++) begin
      for (c=0; c<NCols; c++) begin
        TP_BRD(c, r, got);       // B[rowp=col][colp=row]
        exp = fp32_bits(r,c);
        if (got !== exp) begin
          $display("[TB][FAIL] A(%0d,%0d)=0x%08x but B(%0d,%0d)=0x%08x",
                   r, c, exp, c, r, got);
          $fatal(1);
        end
      end
    end

    $display("[TB][PASS] FP32 transpose correct (custom R-type).");
    wait_cycles(10);
    $finish;
  end

endmodule

`default_nettype wire
