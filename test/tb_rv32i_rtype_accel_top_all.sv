
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_accel.vvp \
  ./test/tb_rv32i_rtype_accel_top_all.sv

vvp ./vvp/tb_accel.vvp
*/

`include "./src/EPU/rv32i_rtype_accel_top.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rv32i_rtype_accel_top_all;

  // ------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------
  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = (DATA_W/8);

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk, rst;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst = 1'b1;
    repeat (10) @(posedge clk);
    rst = 1'b0;
  end

  // ------------------------------------------------------------
  // CPU interface
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

  logic accel_busy, accel_done, accel_C_valid;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  rv32i_rtype_accel_top #(
    .M(M), .N(N), .KMAX(KMAX)
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
  // opcode / funct7
  // ------------------------------------------------------------
  localparam logic [6:0] OPC_RTYPE = 7'h33;

  localparam logic [6:0] F7_MAC   = 7'h01;
  localparam logic [6:0] F7_BA    = 7'h04;
  localparam logic [6:0] F7_AM    = 7'h05;
  localparam logic [6:0] F7_IMG   = 7'h06;
  localparam logic [6:0] F7_LAYER = 7'h07;

  // ------------------------------------------------------------
  // helper
  // ------------------------------------------------------------
  function automatic logic [31:0] mk_instr(
    input logic [6:0] f7,
    input logic [2:0] f3
  );
    logic [31:0] t;
    begin
      t = 32'b0;
      t[6:0]   = OPC_RTYPE;
      t[14:12] = f3;
      t[31:25] = f7;
      mk_instr = t;
    end
  endfunction

  function automatic logic [31:0] pack_low_row_col(
    input int unsigned r,
    input int unsigned c
  );
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]      = r[ROW_W-1:0];
      t[ROW_W +: COL_W] = c[COL_W-1:0];
      pack_low_row_col = t;
    end
  endfunction

  // ------------------------------------------------------------
  // FP32 const
  // ------------------------------------------------------------
  localparam logic [31:0] FP_0   = 32'h00000000;
  localparam logic [31:0] FP_1   = 32'h3f800000;
  localparam logic [31:0] FP_N1  = 32'hbf800000;
  localparam logic [31:0] FP_N2  = 32'hc0000000;
  localparam logic [31:0] FP_3   = 32'h40400000;
  localparam logic [31:0] FP_0_5 = 32'h3f000000;

  // ------------------------------------------------------------
  // CPU issue task
  // ------------------------------------------------------------
  task automatic cpu_issue(
    input logic [31:0] iword,
    input logic [31:0] rs1,
    input logic [31:0] rs2,
    input logic [4:0]  rd
  );
    begin
      instr       <= iword;
      rs1_val     <= rs1;
      rs2_val     <= rs2;
      rd_addr     <= rd;
      instr_valid <= 1'b1;

      @(posedge clk);
      while (!instr_ready) @(posedge clk);

      instr_valid <= 1'b0;
      @(posedge clk);
    end
  endtask

  task automatic cpu_read(
    input  logic [31:0] iword,
    input  logic [31:0] rs1,
    input  logic [4:0]  rd,
    output logic [31:0] rdata
  );
    begin
      cpu_issue(iword, rs1, 32'd0, rd);
      wait (rd_we && rd_waddr == rd);
      rdata = rd_wdata;
      @(posedge clk);
    end
  endtask

  // ------------------------------------------------------------
  // main test
  // ------------------------------------------------------------
  logic [31:0] v;
  logic [31:0] stat;
  logic [31:0] cls;

  int i;

  initial begin
    instr_valid = 0;
    instr = 0;
    rs1_val = 0;
    rs2_val = 0;
    rd_addr = 0;

    wait(!rst);
    @(posedge clk);

    $display("=== IMG preload ===");

    // write image row0
    cpu_issue(mk_instr(F7_IMG,3'b000), pack_low_row_col(0,0), FP_1, 5'd0);
    cpu_issue(mk_instr(F7_IMG,3'b000), pack_low_row_col(0,1), FP_0, 5'd0);

    cpu_read(mk_instr(F7_IMG,3'b001), pack_low_row_col(0,0), 5'd1, v);
    $display("IMG[0,0]=%08x", v);

    // --------------------------------------------------------
    // preload MAC A
    // --------------------------------------------------------
    $display("=== MAC preload ===");
    for (i=0;i<M;i++) begin
      if (i==1)
        cpu_issue(mk_instr(F7_MAC,3'b000), pack_low_row_col(i,0), FP_1, 5'd0);
      else
        cpu_issue(mk_instr(F7_MAC,3'b000), pack_low_row_col(i,0), FP_0, 5'd0);
    end

    // preload MAC B
    cpu_issue(mk_instr(F7_MAC,3'b001), pack_low_row_col(0,0), FP_N1, 5'd0);
    cpu_issue(mk_instr(F7_MAC,3'b001), pack_low_row_col(0,1), FP_N2, 5'd0);
    cpu_issue(mk_instr(F7_MAC,3'b001), pack_low_row_col(0,2), FP_3,  5'd0);
    cpu_issue(mk_instr(F7_MAC,3'b001), pack_low_row_col(0,3), FP_0_5,5'd0);

    // bias = 0
    for (i=0;i<N;i++)
      cpu_issue(mk_instr(F7_BA,3'b001), i, FP_0, 5'd0);

    // --------------------------------------------------------
    // LAYER cfg/start
    // --------------------------------------------------------
    $display("=== LAYER start ===");

    cpu_issue(
      mk_instr(F7_LAYER,3'b000),
      32'd1, // K_len=1, argmax_row=1
      32'd0,
      5'd0
    );

    cpu_issue(
      mk_instr(F7_LAYER,3'b001),
      32'd0,32'd0,5'd0
    );

    // poll stat
    do begin
      cpu_read(mk_instr(F7_LAYER,3'b010),32'd0,5'd2,stat);
      @(posedge clk);
    end while (!stat[1]);

    $display("LAYER done");
    $display("=== ARGMAX direct test ===");

// row = 1, write 8 logits
cpu_issue(mk_instr(F7_AM,3'b000), pack_low_row_col(1,0), FP_N1, 5'd0);
cpu_issue(mk_instr(F7_AM,3'b000), pack_low_row_col(1,1), FP_N2, 5'd0);
cpu_issue(mk_instr(F7_AM,3'b000), pack_low_row_col(1,2), FP_3,  5'd0); // max here
cpu_issue(mk_instr(F7_AM,3'b000), pack_low_row_col(1,3), FP_0_5,5'd0);
for (i=4; i<N; i++)
  cpu_issue(mk_instr(F7_AM,3'b000), pack_low_row_col(1,i), FP_0, 5'd0);

// start argmax on row=1 (rs1 low bits = row)
cpu_issue(mk_instr(F7_AM,3'b001), 32'd1, 32'd0, 5'd0);

// poll ARG_STAT (funct3=010) until done
do begin
  cpu_read(mk_instr(F7_AM,3'b010), 32'd0, 5'd4, stat);
end while (!stat[1]); // 你若 stat bit 定義不同再調

// read idx
cpu_read(mk_instr(F7_AM,3'b011), 32'd0, 5'd5, cls);
$display("ARGMAX idx = %0d", cls[COL_W-1:0]);

if (cls[COL_W-1:0] !== 2) $fatal(1, "ARGMAX direct FAIL");
else $display("ARGMAX direct PASS");

    cpu_read(mk_instr(F7_LAYER,3'b011),32'd0,5'd3,cls);
    $display("CLASS = %0d", cls[COL_W-1:0]);

    if (cls[COL_W-1:0] !== 2)
      $fatal(1, "TEST FAIL");

    else
      $display("TEST PASS");

    #50;
    $finish;
  end

endmodule

`default_nettype wire
