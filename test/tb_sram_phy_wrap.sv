// ============================================================
// tb_sram_phy_wrap.sv
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_sram_phy_wrap.vvp ./test/tb_sram_phy_wrap.sv
//
// Run:
// vvp ./vvp/tb_sram_phy_wrap.vvp
// gtkwave ./vvp/tb_sram_phy_wrap.vcd
// ============================================================

`include "./src/AMOLED/feature_sram/sram_phy_wrap.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_sram_phy_wrap;

  // ------------------------------------------------------------
  // params (keep small)
  // ------------------------------------------------------------
  localparam int unsigned SRAM_BUS_W = 32;
  localparam int unsigned MEM_AW     = 6;   // 64 words
  localparam int unsigned RD_LAT     = 2;   // test latency=2
  localparam int unsigned MEM_WORDS  = (1 << MEM_AW);

  // ------------------------------------------------------------
  // clock
  // ------------------------------------------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  // ------------------------------------------------------------
  // DUT I/O
  // ------------------------------------------------------------
  logic                   mem_cmd_valid;
  logic                   mem_cmd_ready;
  logic                   mem_cmd_we;
  logic [MEM_AW-1:0]       mem_cmd_addr;
  logic [SRAM_BUS_W-1:0]   mem_cmd_wdata;
  logic [SRAM_BUS_W/8-1:0] mem_cmd_wmask;

  logic                   mem_rvalid;
  logic [SRAM_BUS_W-1:0]   mem_rdata;

  // ------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------
  sram_phy_wrap #(
    .SRAM_BUS_W(SRAM_BUS_W),
    .MEM_AW(MEM_AW),
    .RD_LAT(RD_LAT),
    .MEM_WORDS(MEM_WORDS)
  ) dut (
    .clk(clk),
    .mem_cmd_valid(mem_cmd_valid),
    .mem_cmd_ready(mem_cmd_ready),
    .mem_cmd_we(mem_cmd_we),
    .mem_cmd_addr(mem_cmd_addr),
    .mem_cmd_wdata(mem_cmd_wdata),
    .mem_cmd_wmask(mem_cmd_wmask),
    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata)
  );

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_sram_phy_wrap.vcd");
    $dumpvars(0, tb_sram_phy_wrap);
  end

  // ------------------------------------------------------------
  // tasks
  // ------------------------------------------------------------
  task automatic drive_idle();
    begin
      mem_cmd_valid <= 1'b0;
      mem_cmd_we    <= 1'b0;
      mem_cmd_addr  <= '0;
      mem_cmd_wdata <= '0;
      mem_cmd_wmask <= '0;
    end
  endtask

    task automatic do_write(
    input [MEM_AW-1:0] addr,
    input [SRAM_BUS_W-1:0] data,
    input [SRAM_BUS_W/8-1:0] wmask
  );
    int unsigned t;
    begin
      mem_cmd_addr  <= addr;
      mem_cmd_wdata <= data;
      mem_cmd_wmask <= wmask;
      mem_cmd_we    <= 1'b1;
      mem_cmd_valid <= 1'b1;

      // wait handshake (sampled on clock edge)
      t = 0;
      do begin
        @(posedge clk);
        t++;
        if (t > 2000) begin
          $display("ERROR: do_write timeout waiting ready");
          $finish(1);
        end
      end while (!(mem_cmd_valid && mem_cmd_ready));

      // deassert immediately AFTER the handshake edge (no extra cycle)
      mem_cmd_valid <= 1'b0;
      mem_cmd_we    <= 1'b0;
      mem_cmd_wmask <= '0;
    end
  endtask

  task automatic do_read(
    input [MEM_AW-1:0] addr
  );
    int unsigned t;
    begin
      mem_cmd_addr  <= addr;
      mem_cmd_we    <= 1'b0;
      mem_cmd_valid <= 1'b1;
      mem_cmd_wmask <= '0;
      mem_cmd_wdata <= '0;

      t = 0;
      do begin
        @(posedge clk);
        t++;
        if (t > 2000) begin
          $display("ERROR: do_read timeout waiting ready");
          $finish(1);
        end
      end while (!(mem_cmd_valid && mem_cmd_ready));

      mem_cmd_valid <= 1'b0;
    end
  endtask


task automatic wait_rvalid_edge_and_check(
  input [SRAM_BUS_W-1:0] exp_data,
  input int unsigned timeout_cycles
);
  int unsigned t;
  logic rv_d;

  begin
    t    = 0;
    rv_d = mem_rvalid; // 記住前一拍

    // 等到偵測到 0->1 的上升緣
    while (!(~rv_d && mem_rvalid)) begin
      @(posedge clk);
      t++;
      if (t > timeout_cycles) begin
        $display("ERROR: wait_rvalid_edge timeout");
        $finish(1);
      end
      rv_d = mem_rvalid;
    end

    // 這一拍 mem_rvalid=1，抓資料比對
    if (mem_rdata !== exp_data) begin
      $display("ERROR: rdata mismatch exp=0x%08x got=0x%08x", exp_data, mem_rdata);
      $finish(1);
    end
  end
endtask

task automatic wait_rvalid_consume_and_check(
  input [SRAM_BUS_W-1:0] exp_data,
  input int unsigned timeout_cycles
);
  int unsigned t;
  begin
    t = 0;

    // 等 rvalid 變成 1
    while (!mem_rvalid) begin
      @(posedge clk);
      t++;
      if (t > timeout_cycles) begin
        $display("ERROR: wait_rvalid timeout");
        $finish(1);
      end
    end

    // 在 rvalid=1 的這一拍比對
    if (mem_rdata !== exp_data) begin
      $display("ERROR: rdata mismatch exp=0x%08x got=0x%08x",
               exp_data, mem_rdata);
      $finish(1);
    end

    // 🔑 關鍵：強制離開這一拍，避免下一次 task 吃到同一筆
    @(posedge clk);
  end
endtask


  // ------------------------------------------------------------
  // simple monitor
  // ------------------------------------------------------------
  always @(posedge clk) begin
    if (mem_cmd_valid && mem_cmd_ready) begin
      if (mem_cmd_we)
        $display("[CMD] WR addr=%0d wdata=0x%08x wmask=0x%0x t=%0t",
                 mem_cmd_addr, mem_cmd_wdata, mem_cmd_wmask, $time);
      else
        $display("[CMD] RD addr=%0d t=%0t", mem_cmd_addr, $time);
    end
    if (mem_rvalid) begin
      $display("[RET] rdata=0x%08x t=%0t", mem_rdata, $time);
    end
  end

  // ------------------------------------------------------------
  // main test
  // ------------------------------------------------------------
  initial begin
    drive_idle();

    // wait a few cycles
    repeat (5) @(posedge clk);

    // =========================================================
    // TC1: full write then read (RD_LAT check)
    // =========================================================
    $display("== TC1: full write/read, RD_LAT=%0d ==", RD_LAT);

    do_write(6'd10, 32'hA1B2C3D4, 4'b1111);
    do_read (6'd10);

    // Should return after RD_LAT cycles from RD accept,
    // but we just wait with timeout.
    wait_rvalid_consume_and_check(32'hA1B2C3D4, 50);

    // =========================================================
    // TC2: masked write (byte lanes)
    // start: 0x11223344
    // then write only byte[1]=0xAA with wmask=0010
    // expect: 0x1122AA44  (little-endian byte lanes)
    // =========================================================
    $display("== TC2: wmask byte write ==");

    do_write(6'd11, 32'h11223344, 4'b1111);
     do_write(6'd11, 32'h0000AA00, 4'b0010); // byte1 = 0xAA
    do_read (6'd11);
    wait_rvalid_consume_and_check(32'h1122AA44, 50);

    // =========================================================
    // TC3: two reads back-to-back (pipeline)
    // =========================================================
    $display("== TC3: back-to-back reads ==");

    do_write(6'd20, 32'hDEADBEEF, 4'b1111);
    do_write(6'd21, 32'hCAFEBABE, 4'b1111);

    do_read(6'd20);
    do_read(6'd21);

    // expect first return then second return
    wait_rvalid_consume_and_check(32'hDEADBEEF, 100);
    wait_rvalid_consume_and_check(32'hCAFEBABE, 100);
    $display("PASS ✅ tb_sram_phy_wrap");
    #20;
    $finish;
  end

endmodule

`default_nettype wire
