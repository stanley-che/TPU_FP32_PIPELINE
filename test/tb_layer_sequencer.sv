/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_layer_sequencer.vvp \
  ./test/tb_layer_sequencer.sv

vvp ./vvp/tb_layer_sequencer.vvp
*/
`include "./src/EPU/layer_sequencer.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_layer_sequencer;

  // ------------------------------------------------------------
  // Parameters (和子模組一致)
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
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);

  // ------------------------------------------------------------
  // clock / reset
  // ------------------------------------------------------------
  logic clk;
  logic rst;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // 100MHz
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
  layer_sequencer #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .COL_W(COL_W),
    .N_W(N_W), .K_W(K_W)
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
  // 常用 localparam / function：和 layer_sequencer 內的定義一致
  // ------------------------------------------------------------
  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_MAC    = 7'h01;
  localparam logic [6:0] F7_TP     = 7'h02;
  localparam logic [6:0] F7_RELU   = 7'h03;
  localparam logic [6:0] F7_BA     = 7'h04;
  localparam logic [6:0] F7_AM     = 7'h05;
  localparam logic [6:0] F7_LAYER  = 7'h06;

  // 建 instr（只有 funct7/funct3/opcode 有意義）
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

  // pack row/col 低位 (MAC 的 C / BA / Argmax / 你的格式)
  function automatic logic [31:0] pack_low_row_col(input int unsigned r, input int unsigned c);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]       = r[ROW_W-1:0];
      t[ROW_W +: COL_W]  = c[COL_W-1:0];
      pack_low_row_col = t;
    end
  endfunction
  task automatic cpu_issue_strict(
  input logic [31:0] iword,
  input logic [31:0] rs1,
  input logic [31:0] rs2,
  input logic [4:0]  rd
  );
  begin
    // 先把 payload 放好
    instr   <= iword;
    rs1_val <= rs1;
    rs2_val <= rs2;
    rd_addr <= rd;

    // 讓 valid 從下一個 posedge 開始維持至少一整拍
    @(posedge clk);
    instr_valid <= 1'b1;

    // 等到 ready=1 的那個 posedge 確認 accept
    do @(posedge clk); while (!instr_ready);

    // accept 發生的這個 posedge 已經被 DUT 看到 valid=1
    // 下一拍再撤 valid
    @(posedge clk);
    instr_valid <= 1'b0;
  end
  endtask

  // MAC 專用：row,k
  function automatic logic [31:0] pack_mac_row_k(input int unsigned r, input int unsigned k);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]    = r[ROW_W-1:0];
      t[ROW_W +: K_W] = k[K_W-1:0];
      pack_mac_row_k = t;
    end
  endfunction

  // MAC 專用：k,n
  function automatic logic [31:0] pack_mac_k_n(input int unsigned k, input int unsigned nidx);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[K_W-1:0]    = k[K_W-1:0];
      t[K_W +: N_W] = nidx[N_W-1:0];
      pack_mac_k_n = t;
    end
  endfunction

  // BA Bias column pack（rs1[COL_W-1:0] = col）
  function automatic logic [31:0] pack_bias_col(input int unsigned c);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[COL_W-1:0] = c[COL_W-1:0];
      pack_bias_col = t;
    end
  endfunction
  // Argmax packer: col in low bits, row above it
  

  // ------------------------------------------------------------
  // FP32 常數 (logits 設計)
  // 0, -1, -2, 3, 0.5
  // ------------------------------------------------------------
  localparam logic [31:0] FP_0    = 32'h00000000;
  localparam logic [31:0] FP_N1   = 32'hbf800000; // -1.0
  localparam logic [31:0] FP_N2   = 32'hc0000000; // -2.0
  localparam logic [31:0] FP_3    = 32'h40400000; //  3.0
  localparam logic [31:0] FP_0_5  = 32'h3f000000; //  0.5
  localparam logic [31:0] FP_1    = 32'h3f800000; //  1.0

  // ------------------------------------------------------------
  // CPU 發指令小 task
  // ------------------------------------------------------------

  // 一般 write 類指令 (不管有沒有 writeback)
  task automatic cpu_issue(
    input logic [31:0] iword,
    input logic [31:0] rs1,
    input logic [31:0] rs2,
    input logic [4:0]  rd
  );
    begin
      instr      <= iword;
      rs1_val    <= rs1;
      rs2_val    <= rs2;
      rd_addr    <= rd;
      instr_valid <= 1'b1;

      // 等待 ready=1（握手）
      @(posedge clk);
      while (!instr_ready) begin
        @(posedge clk);
      end

      // 完成握手後拉低 valid
      instr_valid <= 1'b0;
      @(posedge clk); // 留一拍空隙
    end
  endtask

  // layer 專用 read（STAT / RCLASS）
  task automatic cpu_layer_read(
    input  logic [31:0] iword,
    input  logic [4:0]  rd,
    output logic [31:0] rdata
  );
    begin
      instr      <= iword;
      rs1_val    <= 32'd0;
      rs2_val    <= 32'd0;
      rd_addr    <= rd;
      instr_valid <= 1'b1;

      // 等待 ready
      @(posedge clk);
      while (!instr_ready) begin
        @(posedge clk);
      end

      // 完成握手，拉低 instr_valid
      instr_valid <= 1'b0;

      // 等待 rd_we
      wait (rd_we && (rd_waddr == rd));
      rdata = rd_wdata;

      @(posedge clk);
    end
  endtask

  // 封裝成對 MAC/BA 的寫入
  task automatic write_mac_A(
    input int unsigned row,
    input int unsigned k,
    input logic [31:0] data
  );
    begin
      // ACC_AWR: funct7=F7_MAC, funct3=3'b000
      cpu_issue(
        mk_instr(F7_MAC, 3'b000),
        pack_mac_row_k(row, k),
        data,
        5'd0
      );
    end
  endtask

  task automatic write_mac_B(
    input int unsigned k,
    input int unsigned col,
    input logic [31:0] data
  );
    begin
      // ACC_BWR: funct7=F7_MAC, funct3=3'b001
      cpu_issue(
        mk_instr(F7_MAC, 3'b001),
        pack_mac_k_n(k, col),
        data,
        5'd0
      );
    end
  endtask

  task automatic write_bias(
    input int unsigned col,
    input logic [31:0] data
  );
    begin
      // BA_BWR: funct7=F7_BA, funct3=3'b001
      cpu_issue(
        mk_instr(F7_BA, 3'b001),
        pack_bias_col(col),
        data,
        5'd0
      );
    end
  endtask
  task automatic cpu_readback_wait(
  input  logic [31:0] iword,
  input  logic [31:0] rs1,
  input  logic [4:0]  rd,
  output logic [31:0] rdata
);
  begin
    cpu_issue_strict(iword, rs1, 32'd0, rd);
    // 等 rd_we 真的出現（注意：這裡要等 waddr）
    wait (rd_we && (rd_waddr == rd));
    rdata = rd_wdata;
    @(posedge clk);
  end
endtask
task automatic dump_relu_y_hex_csv(string fn);
  int f;
  int unsigned r, c;
  logic [31:0] v;
  begin
    f = $fopen(fn, "w");
    $fwrite(f, "r,c,hex\n");
    for (r = 0; r < M; r++) begin
      for (c = 0; c < N; c++) begin
        cpu_readback_wait(mk_instr(F7_RELU, 3'b011), {r[15:0], c[15:0]}, 5'd20, v);
        $fwrite(f, "%0d,%0d,%08x\n", r, c, v);
      end
    end
    $fclose(f);
  end
endtask
task automatic dump_relu_y_hex_csv_255(string fn);
  int f;
  int unsigned r, c;
  logic [31:0] v;
  begin
    f = $fopen(fn, "w");
    $fwrite(f, "r,c,hex\n");
    for (r = 0; r < 255; r++) begin
      for (c = 0; c < 255; c++) begin
        cpu_readback_wait(mk_instr(F7_RELU, 3'b011), {r[15:0], c[15:0]}, 5'd20, v);
        $fwrite(f, "%0d,%0d,%08x\n", r, c, v);
      end
    end
    $fclose(f);
  end
endtask


logic [31:0] v;
  // ------------------------------------------------------------
  // main test
  // ------------------------------------------------------------
  int unsigned i;
  logic [31:0] cfg_word;
  logic [31:0] stat_word;
  int unsigned poll_cnt;
  logic [31:0] rclass_word;
  initial begin
    // 初始化
    instr_valid = 1'b0;
    instr       = 32'd0;
    rs1_val     = 32'd0;
    rs2_val     = 32'd0;
    rd_addr     = 5'd0;

    wait(!rst);
    @(posedge clk);

    $display("=== Preload A / B / Bias for layer_sequencer test ===");

    // --------------------------------------------------------
    // 1) 預載 MAC A / B
    //    設定 K_len = 1，Argmax 看 row=1
    //    A(row,k=0): row1=1.0，其它列=0.0
    // --------------------------------------------------------
    for (i = 0; i < M; i++) begin
      if (i == 1)
        write_mac_A(i, 0, FP_1);
      else
        write_mac_A(i, 0, FP_0);
    end

    // B(k=0, col):
    // col0 -> -1, col1 -> -2, col2 -> 3, col3 -> 0.5, 其他 0
    write_mac_B(0, 0, FP_N1);
    write_mac_B(0, 1, FP_N2);
    write_mac_B(0, 2, FP_3);
    write_mac_B(0, 3, FP_0_5);

    for (i = 4; i < N; i++) begin
      write_mac_B(0, i, FP_0);
    end

    // --------------------------------------------------------
    // 2) 預載 Bias (全部 0，讓 BA 只是 C + 0)
    // --------------------------------------------------------
    for (i = 0; i < N; i++) begin
      write_bias(i, FP_0);
    end

    $display("[%0t] Preload done, start LAYER...", $time);

    // --------------------------------------------------------
    // 3) LAYER_CFG
    //    rs1[15:0] = K_len = 1
    //    rs1[16]   = use_tp = 0 (先不用 TP)
    //    rs1[ROW_W-1:0] = argmax_row = 1
    //    注意：原本你的設計 K_len 與 argmax_row 有 bit 重疊，
    //    這裡選 argmax_row=1，K_len=1（low bits 一樣是 1）
    // --------------------------------------------------------
   
    cfg_word = 32'd0;
    cfg_word[15:0]              = 16'd1;  // K_len = 1
    cfg_word[16]                = 1'b0;   // use_tp = 0
    cfg_word[ROW_W-1:0]         = 'd1;    // argmax_row = 1

    cpu_issue(
      mk_instr(F7_LAYER, 3'b000), // LAY_CFG
      cfg_word,
      32'd0,
      5'd1
    );

    // --------------------------------------------------------
    // 4) LAYER_START
    // --------------------------------------------------------
    cpu_issue(
      mk_instr(F7_LAYER, 3'b001), // LAY_START
      32'd0,
      32'd0,
      5'd1
    );

    // --------------------------------------------------------
    // 5) polling LAY_STAT
    //     LAY_STAT: { 2'b0, 22'b0, st[5:0], done, busy }
    //     => busy = bit0, done = bit1, phase = [7:2]
    // --------------------------------------------------------
    poll_cnt = 0;

    do begin
      cpu_layer_read(
        mk_instr(F7_LAYER, 3'b010), // LAY_STAT
        5'd2,
        stat_word
      );
      poll_cnt++;
      //$display("[%0t] STAT: busy=%0d done=%0d state=%0d (poll=%0d)",
      //       $time, stat_word[0], stat_word[1], stat_word[7:2], poll_cnt);
      // 給一點時間讓硬體跑
      repeat (5) @(posedge clk);
    end while (stat_word[1] == 1'b0); // done==0

    $display("[%0t] LAYER done!", $time);
    cpu_readback_wait(mk_instr(F7_MAC, 3'b011), pack_low_row_col(1,2), 5'd10, v);
    $display("MAC_C[1,2] = %08x", v);
    dump_relu_y_hex_csv("img255_hex.csv");



    cpu_readback_wait(mk_instr(F7_BA, 3'b011), pack_low_row_col(1,2), 5'd11, v);
    $display("BA_C[1,2]  = %08x", v);

    // ReLU 的 pack 是 16/16：{row,col}
    cpu_readback_wait(mk_instr(F7_RELU, 3'b011), {16'd1,16'd2}, 5'd12, v);
    $display("ReLU_Y[1,2]= %08x", v);
    // --------------------------------------------------------
    // 6) 讀 LAY_RCLASS，檢查 Argmax 結果
    // --------------------------------------------------------
    cpu_layer_read(
      mk_instr(F7_LAYER, 3'b011), // LAY_RCLASS
      5'd3,
      rclass_word
    );

    $display("[%0t] LAY_RCLASS = 0x%08x (class_idx=%0d)",
             $time, rclass_word, rclass_word[COL_W-1:0]);

    // 設計預期：ReLU 後 row1 logits = [0,0,3,0.5,0,0,0,0]
    // Argmax(row=1) -> class_idx = 2
    if (rclass_word[COL_W-1:0] !== 'd2) begin
      $error("TEST FAIL: expected class_idx=2, got %0d", rclass_word[COL_W-1:0]);
    end else begin
      $display("TEST PASS: layer_sequencer image-accel pipeline OK, class_idx=2");
    end

    #100;
    $finish;
  end


endmodule

`default_nettype wire
