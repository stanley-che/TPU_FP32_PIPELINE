// tb_tile_addr_gen.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_tile_addr_gen.vvp \
  ./test/tb_tile_addr_gen.sv

vvp ./vvp/tb_tile_addr_gen.vvp
gtkwave ./vvp/tb_tile_addr_gen.vcd
*/
`include "./src/AMOLED/feature_sram/tile_addr_gen.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_tile_addr_gen;

  parameter int unsigned tile_x   = 320;
  parameter int unsigned tile_y   = 180;
  parameter int unsigned sram_bus = 32;
  parameter int unsigned feat_dim = 8;
  parameter int unsigned elen_W   = 32;

  localparam int unsigned FEAT_BITS       = feat_dim * elen_W;
  localparam int unsigned FEAT_SRAM_WORDS = (FEAT_BITS + sram_bus - 1) / sram_bus;

  localparam int TI_W      = $clog2(tile_y);
  localparam int TJ_W      = $clog2(tile_x);
  localparam int TILE_ID_W = $clog2(tile_x * tile_y);
  localparam int ADDR_W    = $clog2(tile_x * tile_y * FEAT_SRAM_WORDS);

  logic [TI_W-1:0] tile_i;
  logic [TJ_W-1:0] tile_j;
  logic [TILE_ID_W-1:0] tile_id;
  logic [ADDR_W-1:0]    sram_addr;

  tile_addr_gen #(
    .tile_x(tile_x),
    .tile_y(tile_y),
    .sram_bus(sram_bus),
    .feat_dim(feat_dim),
    .elen_W(elen_W)
  ) dut (
    .tile_i(tile_i),
    .tile_j(tile_j),
    .tile_id(tile_id),
    .sram_addr(sram_addr)
  );

  integer err_cnt = 0;

  function automatic int unsigned clamp_int(
    input int unsigned v,
    input int unsigned maxv
  );
    if (v >= maxv) clamp_int = (maxv == 0) ? 0 : (maxv - 1);
    else          clamp_int = v;
  endfunction

  task automatic check_one(input int unsigned i_in, input int unsigned j_in);
    int unsigned i_c, j_c;
    int unsigned exp_tile_id;
    int unsigned exp_sram_addr;
    begin
      tile_i = i_in[TI_W-1:0];
      tile_j = j_in[TJ_W-1:0];
      #1;

      i_c = clamp_int(tile_i, tile_y);
      j_c = clamp_int(tile_j, tile_x);

      exp_tile_id   = i_c * tile_x + j_c;
      exp_sram_addr = exp_tile_id * FEAT_SRAM_WORDS;

      if (tile_id !== exp_tile_id) begin
        $display("[FAIL] tile_id  i=%0d j=%0d  got=%0d exp=%0d",
                 i_in, j_in, tile_id, exp_tile_id);
        err_cnt = err_cnt + 1;
      end

      if (sram_addr !== exp_sram_addr) begin
        $display("[FAIL] sram_addr i=%0d j=%0d  got=%0d exp=%0d",
                 i_in, j_in, sram_addr, exp_sram_addr);
        err_cnt = err_cnt + 1;
      end
    end
  endtask

  task automatic testcase1;
    begin
      err_cnt = 0;

      tile_i = 0;
      tile_j = 0;
      #5;

      // sweep all legal tiles
      for (int i = 0; i < tile_y; i = i + 1) begin
        for (int j = 0; j < tile_x; j = j + 1) begin
          check_one(i, j);
        end
      end

      // clamp test: all-ones input
      check_one((1<<TI_W)-1, (1<<TJ_W)-1);

      if (err_cnt == 0)
        $display("\n[PASS] testcase1 tb_tile_addr_gen\n");
      else
        $display("\n[FAIL] testcase1 tb_tile_addr_gen  err_cnt=%0d\n", err_cnt);
    end
  endtask

  initial begin
    $dumpfile("./vvp/tb_tile_addr_gen.vcd");
    $dumpvars(0, tb_tile_addr_gen);

    testcase1();

    #10;
    $finish;
  end

endmodule

`default_nettype wire
