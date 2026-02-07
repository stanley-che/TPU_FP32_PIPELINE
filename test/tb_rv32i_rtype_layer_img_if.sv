// tb_rv32i_rtype_layer_img_if.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_img_if.vvp \
  ./test/tb_rv32i_rtype_layer_img_if.sv

vvp ./vvp/tb_img_if.vvp
*/

`include "./src/EPU/rv32i_rtype_layer_img_if.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_rv32i_rtype_layer_img_if;

  localparam int unsigned IMG_H  = 8;
  localparam int unsigned IMG_W  = 8;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned DATA_W = 32;

  // clock/reset
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

  // DUT interface
  logic        instr_valid;
  logic        instr_ready;
  logic [31:0] instr;
  logic [31:0] rs1_val;
  logic [31:0] rs2_val;
  logic [4:0]  rd_addr;

  logic        rd_we;
  logic [4:0]  rd_waddr;
  logic [31:0] rd_wdata;

  logic        accel_busy;
  logic        accel_done;
  logic        accel_C_valid;

  // ----------------------------
  // DUT
  // ----------------------------
  rv32i_rtype_layer_img_if #(
    .IMG_H (IMG_H),
    .IMG_W (IMG_W),
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W)
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
  // encoding helpers
  // ----------------------------
  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_IMG    = 7'h06;

  localparam logic [2:0] F3_IM_WR   = 3'b000;
  localparam logic [2:0] F3_IM_RD   = 3'b001;
  localparam logic [2:0] F3_IM_STAT = 3'b010;

  function automatic logic [31:0] mk_instr(input logic [2:0] f3);
    logic [31:0] x;
    begin
      // only fields used by DUT: opcode[6:0], funct3[14:12], funct7[31:25]
      x = 32'h0;
      x[6:0]   = OPC_RTYPE;
      x[14:12] = f3;
      x[31:25] = F7_IMG;
      mk_instr = x;
    end
  endfunction

  function automatic logic [31:0] pack_rc(input logic [15:0] r, input logic [15:0] c);
    pack_rc = {r, c};
  endfunction

  // ----------------------------
  // tasks
  // ----------------------------
  task automatic send_ci(
  input logic [2:0]  f3,
  input logic [31:0] rs1,
  input logic [31:0] rs2,
  input logic [4:0]  rd
  );
  begin
    instr       <= mk_instr(f3);
    rs1_val     <= rs1;
    rs2_val     <= rs2;
    rd_addr     <= rd;
    instr_valid <= 1'b1;

    // wait until accepted on a posedge
    do @(posedge clk); while (!instr_ready);

    // IMPORTANT: drop valid immediately so next posedge won't re-accept
    instr_valid <= 1'b0;
    instr       <= 32'h0;
    rs1_val     <= 32'h0;
    rs2_val     <= 32'h0;
    rd_addr     <= 5'h0;

    // optional: give one cycle gap
    @(posedge clk);
  end
  endtask


  task automatic do_wr(
    input logic [15:0] r,
    input logic [15:0] c,
    input logic [31:0] data
  );
    begin
      send_ci(F3_IM_WR, pack_rc(r,c), data, 5'd0);
      // WR 的 accel_done 會在 seq_we_q 那拍 pulsed（大概送出後下一拍）
      // 不強求，但我們等兩拍讓寫入穩定
      repeat (2) @(posedge clk);
    end
  endtask

  task automatic do_rd_expect(
    input logic [15:0] r,
    input logic [15:0] c,
    input logic [31:0] exp
  );
    logic [31:0] got;
    begin
      got = 32'hDEAD_BEEF;

      // 發讀指令（下一拍回 rd）
      send_ci(F3_IM_RD, pack_rc(r,c), 32'h0, 5'd7);

      // 等 rd_we pulse（你的 wrapper 是 pending_rd 那拍）
      do @(posedge clk); while (!rd_we);

      got = rd_wdata;

      if (got !== exp) begin
        $display("[FAIL] RD (%0d,%0d): got=0x%08x exp=0x%08x", r, c, got, exp);
        $fatal(1);
      end else begin
        $display("[PASS] RD (%0d,%0d): 0x%08x", r, c, got);
      end

      // 讓 pulse 結束
      @(posedge clk);
    end
  endtask

  task automatic do_stat_check;
    logic [31:0] s;
    begin
      send_ci(F3_IM_STAT, 32'h0, 32'h0, 5'd3);

      do @(posedge clk); while (!rd_we);
      s = rd_wdata;

      // s[0]=busy, s[1]=done, s[2]=rvalid (依你 wrapper 的 packing)
      if (s[0] !== 1'b0 || s[1] !== 1'b1) begin
        $display("[FAIL] STAT: wdata=0x%08x (busy=%b done=%b rvalid=%b)", s, s[0], s[1], s[2]);
        $fatal(1);
      end else begin
        $display("[PASS] STAT: wdata=0x%08x (busy=%b done=%b rvalid=%b)", s, s[0], s[1], s[2]);
      end

      @(posedge clk);
    end
  endtask

  // ----------------------------
  // test sequence
  // ----------------------------
  initial begin
    // init
    instr_valid = 1'b0;
    instr       = 32'h0;
    rs1_val     = 32'h0;
    rs2_val     = 32'h0;
    rd_addr     = 5'h0;

    // wait reset deassert
    wait(rst == 1'b0);
    repeat (2) @(posedge clk);

    $display("=== IMG_IF CI TB start ===");

    // write some points
    do_wr(16'd0, 16'd0, 32'h3f800000); // 1.0
    do_wr(16'd0, 16'd1, 32'h40000000); // 2.0
    do_wr(16'd2, 16'd3, 32'h40400000); // 3.0
    do_wr(16'd7, 16'd7, 32'h40800000); // 4.0

    // read back
    do_rd_expect(16'd0, 16'd0, 32'h3f800000);
    do_rd_expect(16'd0, 16'd1, 32'h40000000);
    do_rd_expect(16'd2, 16'd3, 32'h40400000);
    do_rd_expect(16'd7, 16'd7, 32'h40800000);

    // status
    do_stat_check();

    $display("=== ALL PASS ===");
    $finish;
  end
always @(posedge clk) begin
  if (!rst) begin
    if (dut.seq_we_q) begin
      $display("[WR] t=%0t row=%0d col=%0d wdata=0x%08x",
               $time, dut.row, dut.col, dut.rs2_val);
    end
    if (dut.cpu_rd_req_q) begin
      $display("[RD_REQ] t=%0t row=%0d col=%0d", $time, dut.rd_row_q, dut.rd_col_q);
    end
    if (dut.rd_we) begin
      $display("[RD_RSP] t=%0t rdata=0x%08x", $time, dut.rd_wdata);
    end
  end
end

endmodule

`default_nettype wire
