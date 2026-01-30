/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_cpu.vvp ./top_tb.sv

vvp ./vvp/tb_cpu.vvp
*/

`include "./src/CPU/Top.v"
`timescale 1ns/10ps

`define CYCLE         10.0
`define MAX_CYCLES    10000000

`define prog_path     "./test/prog1/mergesort.hex"
`define gold_path     "./test/prog1/golden.hex"

`define ANSWER_START  32'h0000_9000
`define DONE_ADDR     32'h0000_FFFC
`define DONE_VALUE    8'hFF

module top_tb;

  // ----------------------------
  // localparams
  // ----------------------------
  localparam integer GOLDEN_MAX = 128;
  localparam integer DM_ADDR_W  = 16;   // Top 裡 address 用 [15:0]

  // ----------------------------
  // DUT
  // ----------------------------
  reg clk;
  reg rst;

  Top top (
    .clk(clk),
    .rst(rst)
  );

  // ----------------------------
  // clock
  // ----------------------------
  initial clk = 1'b0;
  always #(`CYCLE/2.0) clk = ~clk;

  // ----------------------------
  // golden
  // ----------------------------
  reg [31:0] GOLDEN [0:GOLDEN_MAX-1];
  integer gf, num, err, i;
  integer ret;

  // ============================================================
  // addr helper
  // ============================================================
  function [DM_ADDR_W-1:0] dm_idx;
    input [31:0] addr;
    begin
      dm_idx = addr[DM_ADDR_W-1:0];
    end
  endfunction

  // ============================================================
  // DM read/write word (byte array)
  // ============================================================
  function [31:0] dm_read_word;
    input [31:0] addr;
    reg [DM_ADDR_W-1:0] a;
    begin
      a = dm_idx(addr);
      dm_read_word = {
        top.u_SRAM_dm.mem[a + 16'd3],
        top.u_SRAM_dm.mem[a + 16'd2],
        top.u_SRAM_dm.mem[a + 16'd1],
        top.u_SRAM_dm.mem[a + 16'd0]
      };
    end
  endfunction

  task dm_write_word;
    input [31:0] addr;
    input [31:0] data;
    reg [DM_ADDR_W-1:0] a;
    begin
      a = dm_idx(addr);
      top.u_SRAM_dm.mem[a + 16'd0] = data[ 7: 0];
      top.u_SRAM_dm.mem[a + 16'd1] = data[15: 8];
      top.u_SRAM_dm.mem[a + 16'd2] = data[23:16];
      top.u_SRAM_dm.mem[a + 16'd3] = data[31:24];
    end
  endtask

  // ============================================================
  // reset (注意：你的 Top 內部 rst 會 delay 一拍，所以 rst 必須至少撐過幾個 posedge)
  // ============================================================
  task apply_reset;
    begin
      rst = 1'b1;
      repeat (10) @(posedge clk);  // ✅ 撐久一點，避免 rst_reg 初期 X
      rst = 1'b0;
      repeat (2) @(posedge clk);
    end
  endtask
  function [31:0] im_read_word;
  input [31:0] addr;
  reg [15:0] a;
  begin
    a = addr[15:0];
    im_read_word = {
      top.u_SRAM_im.mem[a + 16'd3],
      top.u_SRAM_im.mem[a + 16'd2],
      top.u_SRAM_im.mem[a + 16'd1],
      top.u_SRAM_im.mem[a + 16'd0]
    };
  end
endfunction

task periodic_debug;
  input integer cyc;
  reg [31:0] pc;
  reg [31:0] inst_from_mem;
  reg [31:0] inst_from_sramport;
  reg [7:0]  done_b;
  begin
    if ((cyc % 2000) == 0) begin
      pc = top.u_Reg_PC.current_pc;

      // 1) 直接從 IM byte array 組指令（最可靠）
      inst_from_mem = im_read_word(pc);

      // 2) 從 SRAM read_data port 看（你現在看的那條路徑）
      inst_from_sramport = top.u_Reg_D.inst;

      done_b = top.u_SRAM_dm.mem[dm_idx(`DONE_ADDR)];

      $display("[TB] cyc=%0d PC=%08h IM_mem=%08h IM_port=%08h DONE=%02h",
               cyc, pc, inst_from_mem, inst_from_sramport, done_b);
    end
  end
endtask

  // ============================================================
  // load program to IM/DM and init DM vars
  // ============================================================
  task load_program_and_init;
    begin
      $display("[TB] Loading program: %s", `prog_path);

      // 同一份 hex 同時載入 IM/DM（跟你原本一致）
      $readmemh(`prog_path, top.u_SRAM_im.mem);
      $readmemh(`prog_path, top.u_SRAM_dm.mem);

      // init DM locations required by program
      dm_write_word(32'h0000_9078, 32'd0);
      dm_write_word(32'h0000_907C, 32'd0);
      dm_write_word(32'h0000_9080, 32'd0);
      dm_write_word(32'h0000_9084, 32'd0);
      dm_write_word(`DONE_ADDR,     32'd0);

      $display("[TB] DONE init: DM[%h] word=%08h byte0=%02h",
               `DONE_ADDR,
               dm_read_word(`DONE_ADDR),
               top.u_SRAM_dm.mem[dm_idx(`DONE_ADDR)]);
    end
  endtask

  // ============================================================
  // load golden
  // ============================================================
  task load_golden;
    begin
      num = 0;
      gf  = $fopen(`gold_path, "r");
      if (gf == 0) begin
        $display("[TB][ERROR] Cannot open golden file: %s", `gold_path);
        $finish;
      end

      while (!$feof(gf) && num < GOLDEN_MAX) begin
        ret = $fscanf(gf, "%h\n", GOLDEN[num]);
        if (ret == 1) num = num + 1;
      end
      $fclose(gf);

      $display("[TB] Loaded %0d golden words from %s", num, `gold_path);
    end
  endtask

  // ============================================================
  // periodic debug: PC / IM inst / done
  // ============================================================


  // ============================================================
  // wait done with timeout (iverilog-friendly: named block + disable)
  // ============================================================
  task wait_done_or_timeout;
    integer cyc;
    reg done_hit;
    begin
      done_hit = 1'b0;

      $display("[TB] Waiting DONE: DM[%h].byte0 == %02h", `DONE_ADDR, `DONE_VALUE);

      begin : WAIT_LOOP
        for (cyc = 0; cyc < `MAX_CYCLES; cyc = cyc + 1) begin
          @(posedge clk);

          periodic_debug(cyc);

          if (top.u_SRAM_dm.mem[dm_idx(`DONE_ADDR)] === `DONE_VALUE) begin
            done_hit = 1'b1;
            $display("\n[TB] DONE at cycle %0d (byte0=%02h word=%08h)\n",
                     cyc,
                     top.u_SRAM_dm.mem[dm_idx(`DONE_ADDR)],
                     dm_read_word(`DONE_ADDR));
            disable WAIT_LOOP;
          end
        end
      end

      if (!done_hit) begin
        $display("[TB][TIMEOUT] exceeded MAX cycles (%0d). DONE_byte0=%02h DONE_word=%08h",
                 `MAX_CYCLES,
                 top.u_SRAM_dm.mem[dm_idx(`DONE_ADDR)],
                 dm_read_word(`DONE_ADDR));
        $finish;
      end
    end
  endtask

  // ============================================================
  // compare with golden
  // ============================================================
  task compare_with_golden;
    reg [31:0] got;
    begin
      err = 0;
      for (i = 0; i < num; i = i + 1) begin
        got = dm_read_word(`ANSWER_START + i*4);

        if (got !== GOLDEN[i]) begin
          $display("[FAIL] DM[%h]=%08h expect=%08h",
                   (`ANSWER_START + i*4), got, GOLDEN[i]);
          err = err + 1;
        end else begin
          $display("[PASS] DM[%h]=%08h",
                   (`ANSWER_START + i*4), got);
        end
      end

      if (err == 0)
        $display("\n[TB] PASS: checked %0d words.\n", num);
      else
        $display("\n[TB] FAIL: %0d errors (checked %0d words).\n", err, num);
    end
  endtask

  // ============================================================
  // main
  // ============================================================
  initial begin
    rst = 1'b1;

    apply_reset();
    load_program_and_init();
    load_golden();

    wait_done_or_timeout();
    compare_with_golden();

    $finish;
  end

  // ============================================================
  // waveform
  // ============================================================
  initial begin
    $dumpfile("wave.vcd");
    $dumpvars(2, top);
    $dumpvars(0, top.u_Reg_File);
    $dumpvars(0, top.u_Reg_PC);
    $dumpvars(0, top.u_Reg_D);
  end

endmodule
