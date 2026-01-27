/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_argmax.vvp \
  ./test/tb_argmax_custom_rtype.sv

vvp ./vvp/tb_argmax.vvp
*/
`include "./src/EPU/rv32i_rtype_argmax.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_argmax_custom_rtype;

  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned DATA_W = 32;

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);

  // ----------------------------
  // clock / reset
  // ----------------------------
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

  // ----------------------------
  // DUT interface
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

  rv32i_rtype_argmax #(
    .M(M), .N(N), .DATA_W(DATA_W)
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
  // Instruction encoding helpers
  // ----------------------------
  localparam logic [6:0] OPCODE_RTYPE = 7'h33;
  localparam logic [6:0] FUNCT7_AM    = 7'h05;

  localparam logic [2:0] F3_XWR   = 3'b000;
  localparam logic [2:0] F3_START = 3'b001;
  localparam logic [2:0] F3_STAT  = 3'b010;
  localparam logic [2:0] F3_RIDX  = 3'b011;
  localparam logic [2:0] F3_RMAX  = 3'b100;

  function automatic logic [31:0] make_rtype(
    input logic [6:0] funct7,
    input logic [4:0] rs2,
    input logic [4:0] rs1,
    input logic [2:0] funct3,
    input logic [4:0] rd,
    input logic [6:0] opcode
  );
    make_rtype = {funct7, rs2, rs1, funct3, rd, opcode};
  endfunction

  // 這個 tb 不真的用 rs1/rs2/rd 欄位 decode（DUT 用 rs1_val/rs2_val/rd_addr）
  // 但 instr 的 opcode/funct7/funct3 需要對
  task automatic send_instr(
  input logic [2:0]  funct3,
  input logic [31:0] rs1v,
  input logic [31:0] rs2v,
  input logic [4:0]  rd_a
);
  logic [31:0] inst;
  begin
    inst = make_rtype(FUNCT7_AM, 5'd0, 5'd0, funct3, 5'd0, OPCODE_RTYPE);

    // 等 ready（如果你 instr_ready 永遠 1，這段也不會卡）
    while (!instr_ready) @(posedge clk);

    // 在 negedge 提前擺好，posedge 才會被 always_ff 抓到
    @(negedge clk);
    instr_valid = 1'b1;
    instr       = inst;
    rs1_val     = rs1v;
    rs2_val     = rs2v;
    rd_addr     = rd_a;

    // DUT 在下一個 posedge 取樣
    @(posedge clk);

    // 再下一個 negedge 放掉 valid
    @(negedge clk);
    instr_valid = 1'b0;
    instr       = 32'h0;
    rs1_val     = 32'h0;
    rs2_val     = 32'h0;
    rd_addr     = 5'd0;
  end
endtask
  task automatic exec_read_instr(
  input  logic [2:0]  funct3,
  input  logic [31:0] rs1v,
  input  logic [31:0] rs2v,
  input  logic [4:0]  rd_a,
  output logic [31:0] rdata
);
  logic [31:0] inst;
  begin
    inst = make_rtype(FUNCT7_AM, 5'd0, 5'd0, funct3, 5'd0, OPCODE_RTYPE);

    while (!instr_ready) @(posedge clk);

    @(negedge clk);
    instr_valid = 1'b1;
    instr       = inst;
    rs1_val     = rs1v;
    rs2_val     = rs2v;
    rd_addr     = rd_a;

    // 這個 posedge 是 DUT 取樣/同時組合出 rd_wdata 的時間點
    @(posedge clk);
    #1;                 // 給 combinational 一個 delta
    rdata = rd_wdata;   // <<<<<< 在 instr_valid 還沒被你拉低前取樣

    @(negedge clk);
    instr_valid = 1'b0;
    instr       = 32'h0;
    rs1_val     = 32'h0;
    rs2_val     = 32'h0;
    rd_addr     = 5'd0;
  end
endtask


  // pack row/col into rs1_val like DUT expects: [ROW_W+COL_W-1:COL_W]=row, [COL_W-1:0]=col
  function automatic logic [31:0] pack_row_col(input int unsigned row, input int unsigned col);
    logic [31:0] t;
    begin
      t = 32'h0;
      t[COL_W-1:0] = col[COL_W-1:0];
      t[ROW_W+COL_W-1:COL_W] = row[ROW_W-1:0];
      pack_row_col = t;
    end
  endfunction

  // read STAT / RIDX / RMAX
  task automatic read_stat(output logic busy, output logic done, output logic valid);
  logic [31:0] tmp;
  begin
    exec_read_instr(F3_STAT, 32'h0, 32'h0, 5'd1, tmp);
    busy  = tmp[0];
    done  = tmp[1];
    valid = tmp[2];
  end
endtask

task automatic read_ridx(output int unsigned idx);
  logic [31:0] tmp;
  begin
    exec_read_instr(F3_RIDX, 32'h0, 32'h0, 5'd2, tmp);
    idx = tmp[COL_W-1:0];
  end
endtask

task automatic read_rmax(output logic [31:0] vmax_bits);
  logic [31:0] tmp;
  begin
    exec_read_instr(F3_RMAX, 32'h0, 32'h0, 5'd3, tmp);
    vmax_bits = tmp;
  end
endtask




  // write one logit
  task automatic write_logit(input int unsigned row, input int unsigned col, input logic [31:0] fp32_bits);
    begin
      send_instr(F3_XWR, pack_row_col(row, col), fp32_bits, 5'd0);
    end
  endtask

  // start argmax on row (DUT 取 rs1_val[ROW_W-1:0] 當 row)
  task automatic start_row(input int unsigned row);
    logic [31:0] rs1v;
    begin
      rs1v = 32'h0;
      rs1v[ROW_W-1:0] = row[ROW_W-1:0];
      send_instr(F3_START, rs1v, 32'h0, 5'd0);
    end
  endtask

  task automatic poll_done;
    logic b, d, v;
    int cyc;
    begin
      cyc = 0;
      do begin
        read_stat(b, d, v);
        cyc++;
        if (cyc > 200) begin
          $fatal(1, "[TB] poll_done timeout (stuck busy/done?)");
        end
      end while (!d);
    end
  endtask

  // ----------------------------
  // Tests
  // ----------------------------
  initial begin
    instr_valid = 0;
    instr       = 0;
    rs1_val     = 0;
    rs2_val     = 0;
    rd_addr     = 0;

    // wait reset deassert
    @(negedge rst);
    repeat (2) @(posedge clk);

    $display("[TB] Test1: max in middle");
    // row0 logits (fp32 bits)
    // 1.0, 2.5, -3.0, 9.0, 4.0, 8.0, 0.0, -1.0  => max=9.0 @ idx=3
    write_logit(0,0,32'h3F80_0000);
    write_logit(0,1,32'h4020_0000);
    write_logit(0,2,32'hC040_0000);
    write_logit(0,3,32'h4110_0000);
    write_logit(0,4,32'h4080_0000);
    write_logit(0,5,32'h4100_0000);
    write_logit(0,6,32'h0000_0000);
    write_logit(0,7,32'hBF80_0000);

    start_row(0);
    poll_done();

    begin
      int unsigned idx;
      logic [31:0] vmax;
      read_ridx(idx);
      read_rmax(vmax);
      if (idx !== 3) $fatal(1, "[TB] Test1 FAIL: idx=%0d (expected 3)", idx);
      if (vmax !== 32'h4110_0000) $fatal(1, "[TB] Test1 FAIL: vmax=%h (expected 4110_0000)", vmax);
      $display("[TB] Test1 PASS");
    end

    $display("[TB] Test2: max at last index");
    // 0.5, 0.25, 0.125, 0.0625, 0, -1, -2, 10.0 => idx=7
    write_logit(0,0,32'h3F00_0000);
    write_logit(0,1,32'h3E80_0000);
    write_logit(0,2,32'h3E00_0000);
    write_logit(0,3,32'h3D80_0000);
    write_logit(0,4,32'h0000_0000);
    write_logit(0,5,32'hBF80_0000);
    write_logit(0,6,32'hC000_0000);
    write_logit(0,7,32'h4120_0000);

    start_row(0);
    poll_done();

    begin
      int unsigned idx;
      logic [31:0] vmax;
      read_ridx(idx);
      read_rmax(vmax);
      if (idx !== 7) $fatal(1, "[TB] Test2 FAIL: idx=%0d (expected 7)", idx);
      if (vmax !== 32'h4120_0000) $fatal(1, "[TB] Test2 FAIL: vmax=%h (expected 4120_0000)", vmax);
      $display("[TB] Test2 PASS");
    end

    $display("[TB] Test3: all negative numbers");
    // -1, -2, -3, -0.5, -4, -8, -0.25, -16 => max is -0.25 @ idx=6
    write_logit(0,0,32'hBF80_0000);
    write_logit(0,1,32'hC000_0000);
    write_logit(0,2,32'hC040_0000);
    write_logit(0,3,32'hBF00_0000);
    write_logit(0,4,32'hC080_0000);
    write_logit(0,5,32'hC100_0000);
    write_logit(0,6,32'hBE80_0000);
    write_logit(0,7,32'hC180_0000);

    start_row(0);
    poll_done();

    begin
      int unsigned idx;
      logic [31:0] vmax;
      read_ridx(idx);
      read_rmax(vmax);
      if (idx !== 6) $fatal(1, "[TB] Test3 FAIL: idx=%0d (expected 6)", idx);
      if (vmax !== 32'hBE80_0000) $fatal(1, "[TB] Test3 FAIL: vmax=%h (expected BE80_0000)", vmax);
      $display("[TB] Test3 PASS");
    end

    $display("[TB] Test4: tie case (equal max) -> expect smallest index");
    // 5.0 at idx=2 and idx=5, expect idx=2 (because use '>' not '>=')
    write_logit(0,0,32'h3F80_0000); // 1.0
    write_logit(0,1,32'h4000_0000); // 2.0
    write_logit(0,2,32'h40A0_0000); // 5.0
    write_logit(0,3,32'h4080_0000); // 4.0
    write_logit(0,4,32'h3F00_0000); // 0.5
    write_logit(0,5,32'h40A0_0000); // 5.0 (tie)
    write_logit(0,6,32'h0000_0000); // 0.0
    write_logit(0,7,32'hBF80_0000); // -1.0

    start_row(0);
    poll_done();

    begin
      int unsigned idx;
      logic [31:0] vmax;
      read_ridx(idx);
      read_rmax(vmax);
      if (idx !== 2) $fatal(1, "[TB] Test4 FAIL: idx=%0d (expected 2)", idx);
      if (vmax !== 32'h40A0_0000) $fatal(1, "[TB] Test4 FAIL: vmax=%h (expected 40A0_0000)", vmax);
      $display("[TB] Test4 PASS");
    end

    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire
