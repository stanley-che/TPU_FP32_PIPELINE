// tb_rv32i_rtype_accel_top_segments.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_accel_seg.vvp \
  ./test/tb_rv32i_rtype_accel_top_segments.sv

vvp ./vvp/tb_accel_seg.vvp
*/

`include "./src/EPU/rv32i_rtype_accel_top.sv"   // 你把 top 存成這個檔名

`timescale 1ns/1ps
`default_nettype none


module tb_rv32i_rtype_accel_top_all;

  // ------------------------------------------------------------
  // Parameters
  // ------------------------------------------------------------
  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;

  localparam int unsigned IMG_H  = 8;
  localparam int unsigned IMG_W  = 8;

  localparam int unsigned DATA_W = 32;
  localparam int unsigned ADDR_W = 16;
  localparam int unsigned NB     = 2;
  localparam int unsigned BYTE_W = (DATA_W/8);

  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

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
    repeat (10) @(posedge clk);
    rst = 1'b0;
  end

  // ------------------------------------------------------------
  // CPU custom-instruction interface
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

  logic        accel_busy;
  logic        accel_done;
  logic        accel_C_valid;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  rv32i_rtype_accel_top #(
    .M(M), .N(N), .KMAX(KMAX),
    .IMG_H(IMG_H), .IMG_W(IMG_W),
    .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(1)
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
  // ISA fields / funct7
  // ------------------------------------------------------------
  localparam logic [6:0] OPC_RTYPE = 7'h33;

  localparam logic [6:0] F7_MAC    = 7'h01;
  localparam logic [6:0] F7_TP     = 7'h02;
  localparam logic [6:0] F7_RELU   = 7'h03;
  localparam logic [6:0] F7_BA     = 7'h04;
  localparam logic [6:0] F7_AM     = 7'h05;
  localparam logic [6:0] F7_IMG    = 7'h06;

  // ------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------
  function automatic logic [31:0] mk_instr(input logic [6:0] f7, input logic [2:0] f3);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[6:0]   = OPC_RTYPE;
      t[14:12] = f3;
      t[31:25] = f7;
      mk_instr = t;
    end
  endfunction

  // pack row/col in low bits (ROW then COL) : [ROW_W-1:0]=row, [ROW_W +: COL_W]=col
  function automatic logic [31:0] pack_low_row_col(input int unsigned r, input int unsigned c);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]      = r[ROW_W-1:0];
      t[ROW_W +: COL_W] = c[COL_W-1:0];
      pack_low_row_col = t;
    end
  endfunction

  // MAC pack: row,k  (row in low, k above)
  function automatic logic [31:0] pack_mac_row_k(input int unsigned r, input int unsigned k);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]    = r[ROW_W-1:0];
      t[ROW_W +: K_W] = k[K_W-1:0];
      pack_mac_row_k = t;
    end
  endfunction

  // MAC pack: k,n (k in low, n above)
  function automatic logic [31:0] pack_mac_k_n(input int unsigned k, input int unsigned nidx);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[K_W-1:0]    = k[K_W-1:0];
      t[K_W +: N_W] = nidx[N_W-1:0];
      pack_mac_k_n = t;
    end
  endfunction

  // BA bias col pack: col in low bits
  function automatic logic [31:0] pack_bias_col(input int unsigned c);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[COL_W-1:0] = c[COL_W-1:0];
      pack_bias_col = t;
    end
  endfunction

  // ARG_WR idx pack: idx in low bits (用 COL_W 夠用就好)
  function automatic logic [31:0] pack_arg_idx(input int unsigned idx);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[COL_W-1:0] = idx[COL_W-1:0];
      pack_arg_idx = t;
    end
  endfunction

  // ------------------------------------------------------------
  // FP32 constants
  // ------------------------------------------------------------
  localparam logic [31:0] FP_0    = 32'h00000000;
  localparam logic [31:0] FP_1    = 32'h3f800000; // 1.0
  localparam logic [31:0] FP_2    = 32'h40000000; // 2.0
  localparam logic [31:0] FP_3    = 32'h40400000; // 3.0
  localparam logic [31:0] FP_4    = 32'h40800000; // 4.0
  localparam logic [31:0] FP_N1   = 32'hbf800000; // -1.0
  localparam logic [31:0] FP_0_5  = 32'h3f000000; // 0.5

  // ------------------------------------------------------------
  // Handshake tasks (推薦用 strict：valid 至少撐滿一拍)
  // ------------------------------------------------------------
  task automatic cpu_issue_strict(
    input logic [31:0] iword,
    input logic [31:0] rs1,
    input logic [31:0] rs2,
    input logic [4:0]  rd
  );
  begin
    instr   <= iword;
    rs1_val <= rs1;
    rs2_val <= rs2;
    rd_addr <= rd;

    @(posedge clk);
    instr_valid <= 1'b1;

    do @(posedge clk); while (!instr_ready);

    @(posedge clk);
    instr_valid <= 1'b0;
  end
  endtask

  task automatic cpu_readback_wait(
    input  logic [31:0] iword,
    input  logic [31:0] rs1,
    input  logic [31:0] rs2,
    input  logic [4:0]  rd,
    output logic [31:0] rdata
  );
  begin
    cpu_issue_strict(iword, rs1, rs2, rd);
    wait (rd_we && (rd_waddr == rd));
    rdata = rd_wdata;
    @(posedge clk);
  end
  endtask

  // polling STAT：假設 wrapper 的 STAT 回 {busy,done} 放在 bit0/bit1
  task automatic poll_done_stat(
    input  logic [31:0] stat_instr,
    input  int unsigned max_poll,
    output logic [31:0] last_stat
  );
    int unsigned pcnt;
    begin
      last_stat = 32'd0;
      pcnt = 0;
      do begin
        cpu_readback_wait(stat_instr, 32'd0, 32'd0, 5'd31, last_stat);
        pcnt++;
        if (pcnt > max_poll) begin
          $error("poll_done_stat TIMEOUT: stat=0x%08x", last_stat);
          disable poll_done_stat;
        end
        // 給硬體跑幾拍
        repeat (3) @(posedge clk);
      end while (last_stat[1] == 1'b0); // done==0
    end
  endtask

  // ------------------------------------------------------------
  // Convenience: per-accelerator ops
  // ------------------------------------------------------------
  task automatic mac_AWR(input int unsigned row, input int unsigned k, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_MAC, 3'b000), pack_mac_row_k(row,k), data, 5'd0);
  endtask

  task automatic mac_BWR(input int unsigned k, input int unsigned col, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_MAC, 3'b001), pack_mac_k_n(k,col), data, 5'd0);
  endtask

  task automatic mac_START();
    cpu_issue_strict(mk_instr(F7_MAC, 3'b010), 32'd0, 32'd0, 5'd0);
  endtask

  task automatic mac_CRD(input int unsigned row, input int unsigned col, output logic [31:0] data);
    cpu_readback_wait(mk_instr(F7_MAC, 3'b011), pack_low_row_col(row,col), 32'd0, 5'd10, data);
  endtask

  task automatic mac_STAT(output logic [31:0] st);
    cpu_readback_wait(mk_instr(F7_MAC, 3'b100), 32'd0, 32'd0, 5'd11, st);
  endtask

  task automatic tp_AWR(input int unsigned row, input int unsigned col, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_TP, 3'b000), pack_low_row_col(row,col), data, 5'd0);
  endtask

  task automatic tp_START();
    cpu_issue_strict(mk_instr(F7_TP, 3'b001), 32'd0, 32'd0, 5'd0);
  endtask

  task automatic tp_STAT(output logic [31:0] st);
    cpu_readback_wait(mk_instr(F7_TP, 3'b010), 32'd0, 32'd0, 5'd12, st);
  endtask

  task automatic tp_BRD(input int unsigned row, input int unsigned col, output logic [31:0] data);
    cpu_readback_wait(mk_instr(F7_TP, 3'b011), pack_low_row_col(row,col), 32'd0, 5'd13, data);
  endtask

  task automatic relu_XWR(input int unsigned row, input int unsigned col, input logic [31:0] data);
    // 你表格：RELU_XWR funct3=000
    cpu_issue_strict(mk_instr(F7_RELU, 3'b000), {row[15:0], col[15:0]}, data, 5'd0);
  endtask

  task automatic relu_START();
    cpu_issue_strict(mk_instr(F7_RELU, 3'b001), 32'd0, 32'd0, 5'd0);
  endtask

  task automatic relu_STAT(output logic [31:0] st);
    cpu_readback_wait(mk_instr(F7_RELU, 3'b010), 32'd0, 32'd0, 5'd14, st);
  endtask

  task automatic relu_YRD(input int unsigned row, input int unsigned col, output logic [31:0] data);
    cpu_readback_wait(mk_instr(F7_RELU, 3'b011), {row[15:0], col[15:0]}, 32'd0, 5'd15, data);
  endtask

  task automatic ba_XWR(input int unsigned row, input int unsigned col, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_BA, 3'b000), pack_low_row_col(row,col), data, 5'd0);
  endtask

  task automatic ba_BWR(input int unsigned col, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_BA, 3'b001), pack_bias_col(col), data, 5'd0);
  endtask

  task automatic ba_START();
    cpu_issue_strict(mk_instr(F7_BA, 3'b010), 32'd0, 32'd0, 5'd0);
  endtask

  task automatic ba_CRD(input int unsigned row, input int unsigned col, output logic [31:0] data);
    cpu_readback_wait(mk_instr(F7_BA, 3'b011), pack_low_row_col(row,col), 32'd0, 5'd16, data);
  endtask

  task automatic ba_STAT(output logic [31:0] st);
    cpu_readback_wait(mk_instr(F7_BA, 3'b100), 32'd0, 32'd0, 5'd17, st);
  endtask

  task automatic arg_WR(input int unsigned idx, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_AM, 3'b000), pack_arg_idx(idx), data, 5'd0);
  endtask

  task automatic arg_START();
    cpu_issue_strict(mk_instr(F7_AM, 3'b001), 32'd0, 32'd0, 5'd0);
  endtask

  task automatic arg_STAT(output logic [31:0] st);
    cpu_readback_wait(mk_instr(F7_AM, 3'b010), 32'd0, 32'd0, 5'd18, st);
  endtask

  task automatic arg_RD(output logic [31:0] idx);
    cpu_readback_wait(mk_instr(F7_AM, 3'b011), 32'd0, 32'd0, 5'd19, idx);
  endtask

  task automatic im_WR(input int unsigned row, input int unsigned col, input logic [31:0] data);
    cpu_issue_strict(mk_instr(F7_IMG, 3'b000), pack_low_row_col(row,col), data, 5'd0);
  endtask

  task automatic im_RD(input int unsigned row, input int unsigned col, output logic [31:0] data);
    cpu_readback_wait(mk_instr(F7_IMG, 3'b001), pack_low_row_col(row,col), 32'd0, 5'd20, data);
  endtask

  task automatic im_STAT(output logic [31:0] st);
    cpu_readback_wait(mk_instr(F7_IMG, 3'b010), 32'd0, 32'd0, 5'd21, st);
  endtask

  // ------------------------------------------------------------
  // Main
  // ------------------------------------------------------------
  logic [31:0] v0, v1, v2, v3;
  logic [31:0] st;

  initial begin
    instr_valid = 1'b0;
    instr       = 32'd0;
    rs1_val     = 32'd0;
    rs2_val     = 32'd0;
    rd_addr     = 5'd0;

    wait (!rst);
    @(posedge clk);

    $display("====================================================");
    $display(" TB: rv32i_rtype_accel_top ALL wrappers sanity test");
    $display("====================================================");

    // ----------------------------------------------------------
    // 0) IMG_IF sanity: WR then RD
    // ----------------------------------------------------------
    $display("[IMG] WR/RD sanity");
    im_WR(0, 0, FP_1);
    im_WR(0, 1, FP_2);
    im_WR(1, 0, FP_3);
    im_WR(1, 1, FP_4);

    im_RD(0, 0, v0);  $display("IMG[0,0]=%08x", v0);
    im_RD(0, 1, v1);  $display("IMG[0,1]=%08x", v1);
    im_RD(1, 0, v2);  $display("IMG[1,0]=%08x", v2);
    im_RD(1, 1, v3);  $display("IMG[1,1]=%08x", v3);

    if (v0!==FP_1 || v1!==FP_2 || v2!==FP_3 || v3!==FP_4) begin
      $error("[IMG] FAIL: readback mismatch");
    end else begin
      $display("[IMG] PASS");
    end

    // ----------------------------------------------------------
    // 1) MAC sanity: K_len=1 的概念（你的 MAC wrapper 內部怎麼跑我不假設）
    //    這裡只做最小：A(row,k0), B(k0,col) + START + CRD
    // ----------------------------------------------------------
    $display("[MAC] preload A/B then START, read C");
    // A(:,0) all 0, A(1,0)=1
    for (int unsigned r=0; r<M; r++) begin
      mac_AWR(r, 0, (r==1)?FP_1:FP_0);
    end
    // B(0,:) : [-1, -2, 3, 0.5, 0...]
    mac_BWR(0, 0, FP_N1);
    mac_BWR(0, 1, 32'hc0000000); // -2.0
    mac_BWR(0, 2, FP_3);
    mac_BWR(0, 3, FP_0_5);
    for (int unsigned c=4; c<N; c++) mac_BWR(0, c, FP_0);

    mac_START();
    poll_done_stat(mk_instr(F7_MAC, 3'b100), 2000, st);

    mac_CRD(1, 2, v0);
    $display("MAC_C[1,2]=%08x (expect ~3.0 if design is 1*3)", v0);

    // ----------------------------------------------------------
    // 2) BIAS_ADD sanity: X + B
    // ----------------------------------------------------------
    $display("[BA] preload X/B then START, read C");
    // X(1,2)=3.0 其餘 0；Bias 全 0
    for (int unsigned r=0; r<M; r++) begin
      for (int unsigned c=0; c<N; c++) begin
        ba_XWR(r, c, (r==1 && c==2)?FP_3:FP_0);
      end
    end
    for (int unsigned c=0; c<N; c++) ba_BWR(c, FP_0);

    ba_START();
    poll_done_stat(mk_instr(F7_BA, 3'b100), 2000, st);

    ba_CRD(1, 2, v1);
    $display("BA_C[1,2]=%08x (expect 3.0)", v1);
    if (v1 !== FP_3) $error("[BA] FAIL: C[1,2] != 3.0");

    // ----------------------------------------------------------
    // 3) ReLU sanity: X -> Y
    // ----------------------------------------------------------
    $display("[RELU] preload X then START, read Y");
    // X(1,0)=-1, X(1,1)=0, X(1,2)=3, X(1,3)=0.5
    // 其餘 0
    for (int unsigned r=0; r<M; r++) begin
      for (int unsigned c=0; c<N; c++) begin
        relu_XWR(r, c, FP_0);
      end
    end
    relu_XWR(1,0,FP_N1);
    relu_XWR(1,1,FP_0);
    relu_XWR(1,2,FP_3);
    relu_XWR(1,3,FP_0_5);

    relu_START();
    poll_done_stat(mk_instr(F7_RELU, 3'b010), 2000, st);

    relu_YRD(1,0,v2); $display("ReLU_Y[1,0]=%08x (expect 0)", v2);
    relu_YRD(1,2,v3); $display("ReLU_Y[1,2]=%08x (expect 3.0)", v3);

    if (v2 !== FP_0) $error("[RELU] FAIL: Y[1,0] should be 0");
    if (v3 !== FP_3) $error("[RELU] FAIL: Y[1,2] should be 3.0");

    // ----------------------------------------------------------
    // 4) ARGMAX sanity: 寫入向量 -> START -> RD index
    // ----------------------------------------------------------
    $display("[ARGMAX] WR logits then START, RD index");
    // logits: [-1, -2, 3, 0.5, 0, 0, 0, 0] => idx=2
    arg_WR(0, FP_N1);
    arg_WR(1, 32'hc0000000); // -2.0
    arg_WR(2, FP_3);
    arg_WR(3, FP_0_5);
    for (int unsigned i=4; i<N; i++) arg_WR(i, FP_0);

    arg_START();
    poll_done_stat(mk_instr(F7_AM, 3'b010), 2000, st);

    arg_RD(v0);
    $display("ARG_RD = %08x (idx in low bits, expect 2)", v0);
    if (v0[COL_W-1:0] !== 'd2) $error("[ARGMAX] FAIL: expect idx=2, got %0d", v0[COL_W-1:0]);
    else $display("[ARGMAX] PASS");

    // ----------------------------------------------------------
    // 5) TRANSPOSE sanity: A -> START -> BRD
    // ----------------------------------------------------------
    $display("[TP] preload A then START, read B=A^T");
    // A(0,1)=2.0, A(1,0)=3.0
    for (int unsigned r=0; r<M; r++) begin
      for (int unsigned c=0; c<N; c++) begin
        tp_AWR(r,c,FP_0);
      end
    end
    tp_AWR(0,1,FP_2);
    tp_AWR(1,0,FP_3);

    tp_START();
    poll_done_stat(mk_instr(F7_TP, 3'b010), 2000, st);

    tp_BRD(1,0,v1); $display("TP_B[1,0]=%08x (expect A[0,1]=2.0)", v1);
    tp_BRD(0,1,v2); $display("TP_B[0,1]=%08x (expect A[1,0]=3.0)", v2);

    if (v1 !== FP_2) $error("[TP] FAIL: B[1,0] should be 2.0");
    if (v2 !== FP_3) $error("[TP] FAIL: B[0,1] should be 3.0");

    // ----------------------------------------------------------
    // done
    // ----------------------------------------------------------
    $display("====================================================");
    $display(" TB DONE");
    $display("====================================================");

    #100;
    $finish;
  end

endmodule

`default_nettype wire
