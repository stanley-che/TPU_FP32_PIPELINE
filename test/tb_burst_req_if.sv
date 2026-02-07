// tb_burst_req_if.sv
/*
iverilog -g2012 -Wall -I./src \
  -o ./vvp/tb_burst_req_if.vvp \
  ./test/tb_burst_req_if.sv

vvp ./vvp/tb_burst_req_if.vvp
gtkwave ./vvp/tb_burst_req_if.vcd
*/

`include "./src/AMOLED/feature_sram/burst_req_if.sv"

`timescale 1ns/1ps
`default_nettype none

module tb_burst_req_if;

  // -----------------------------
  // params (match DUT)
  // -----------------------------
  localparam int unsigned tile_x   = 320;
  localparam int unsigned tile_y   = 180;
  localparam int unsigned sram_bus = 32;
  localparam int unsigned feat_dim = 8;
  localparam int unsigned elen_W   = 32;
  localparam int unsigned tag_w    = 16;
  localparam bit          isclamp  = 1'b0;

  localparam int unsigned FEAT_BITS_P      = feat_dim * elen_W;
  localparam int unsigned WORDS_PER_FEAT_P = (FEAT_BITS_P + sram_bus - 1) / sram_bus;
  localparam int unsigned TOTAL_WORDS_P    = tile_x * tile_y * WORDS_PER_FEAT_P;
  localparam int unsigned MEM_AW           = (TOTAL_WORDS_P <= 1) ? 1 : $clog2(TOTAL_WORDS_P);

  localparam int TI_W = $clog2(tile_y);
  localparam int TJ_W = $clog2(tile_x);

  // -----------------------------
  // clock/reset
  // -----------------------------
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  // -----------------------------
  // DUT ports
  // -----------------------------
  logic                        valid_wr;
  logic                        ready_wr;
  logic [TI_W-1:0]             tile_i_wr;
  logic [TJ_W-1:0]             tile_j_wr;
  logic [feat_dim*elen_W-1:0]  feat_in;

  logic                        valid_rd;
  logic                        ready_rd;
  logic [TI_W-1:0]             tile_i_rd;
  logic [TJ_W-1:0]             tile_j_rd;
  logic [tag_w-1:0]            tag_rd;

  logic                        req_valid;
  logic                        req_ready;
  logic                        req_is_wr;
  logic [MEM_AW-1:0]           sram_addr;
  logic [feat_dim*elen_W-1:0]  sram_wdata;
  logic [tag_w-1:0]            sram_tag;

  burst_req_if #(
    .tile_x(tile_x),
    .tile_y(tile_y),
    .sram_bus(sram_bus),
    .feat_dim(feat_dim),
    .elen_W(elen_W),
    .tag_w(tag_w),
    .isclamp(isclamp),
    .FEAT_BITS_P(FEAT_BITS_P),
    .WORDS_PER_FEAT_P(WORDS_PER_FEAT_P),
    .TOTAL_WORDS_P(TOTAL_WORDS_P),
    .MEM_AW(MEM_AW)
  ) dut (
    .clk(clk),
    .rst(rst),

    .valid_wr(valid_wr),
    .ready_wr(ready_wr),
    .tile_i_wr(tile_i_wr),
    .tile_j_wr(tile_j_wr),
    .feat_in(feat_in),

    .valid_rd(valid_rd),
    .ready_rd(ready_rd),
    .tile_i_rd(tile_i_rd),
    .tile_j_rd(tile_j_rd),
    .tag_rd(tag_rd),

    .req_valid(req_valid),
    .req_ready(req_ready),
    .req_is_wr(req_is_wr),
    .sram_addr(sram_addr),
    .sram_wdata(sram_wdata),
    .sram_tag(sram_tag)
  );

  // -----------------------------
  // scoreboard utils (iverilog-friendly)
  // -----------------------------
  integer err_cnt = 0;

  task automatic expect_eq_u(input [1023:0] name, input integer got, input integer exp);
    if (got !== exp) begin
      $display("[FAIL] %0s got=%0d exp=%0d @t=%0t", name, got, exp, $time);
      err_cnt = err_cnt + 1;
    end
  endtask

  task automatic expect_eq_vec(input [1023:0] name, input logic [feat_dim*elen_W-1:0] got,
                               input logic [feat_dim*elen_W-1:0] exp);
    if (got !== exp) begin
      $display("[FAIL] %0s got=0x%0h exp=0x%0h @t=%0t", name, got, exp, $time);
      err_cnt = err_cnt + 1;
    end
  endtask

  // compute expected base addr (same as DUT)
  function automatic integer exp_addr(input integer i, input integer j);
    integer tile_id;
    begin
      tile_id = i * tile_x + j;
      exp_addr = tile_id * WORDS_PER_FEAT_P;
    end
  endfunction

  // wait for a cycle
  task automatic tick;
    begin
      @(posedge clk);
      #1;
    end
  endtask

  // drive defaults
  task automatic drive_idle;
    begin
      valid_wr  = 0;
      tile_i_wr = '0;
      tile_j_wr = '0;
      feat_in   = '0;

      valid_rd  = 0;
      tile_i_rd = '0;
      tile_j_rd = '0;
      tag_rd    = '0;
    end
  endtask

  // -----------------------------
  // Testcases
  // -----------------------------
  task automatic testcase1_basic_write;
    integer i, j;
    logic [feat_dim*elen_W-1:0] data;
    begin
      $display("\n--- testcase1_basic_write ---");
      drive_idle();
      req_ready = 1;

      i = 3; j = 5;
      data = 256'h0123_4567_89ab_cdef_1111_2222_3333_4444_aaaa_bbbb_cccc_dddd_5555_6666_7777_8888;

      // issue write for 1 cycle
      valid_wr  = 1;
      tile_i_wr = i[TI_W-1:0];
      tile_j_wr = j[TJ_W-1:0];
      feat_in   = data;

      tick(); // latch into pending on this cycle

      // After tick, pending should be visible on outputs
      expect_eq_u("req_valid(write)", req_valid, 1);
      expect_eq_u("req_is_wr(write)", req_is_wr, 1);
      expect_eq_u("sram_addr(write)", sram_addr, exp_addr(i, j));
      expect_eq_vec("sram_wdata(write)", sram_wdata, data);
      // tag for write currently is '0 in your RTL
      expect_eq_u("sram_tag(write)", sram_tag, 0);

      // handshake clears pending next tick
      valid_wr = 0;
      tick();
      expect_eq_u("req_valid(cleared)", req_valid, 0);
    end
  endtask

  task automatic testcase2_basic_read;
    integer i, j;
    integer tag;
    begin
      $display("\n--- testcase2_basic_read ---");
      drive_idle();
      req_ready = 1;

      i = 10; j = 20;
      tag = 16'h00A5;

      valid_rd  = 1;
      tile_i_rd = i[TI_W-1:0];
      tile_j_rd = j[TJ_W-1:0];
      tag_rd    = tag[tag_w-1:0];

      tick();

      expect_eq_u("req_valid(read)", req_valid, 1);
      expect_eq_u("req_is_wr(read)", req_is_wr, 0);
      expect_eq_u("sram_addr(read)", sram_addr, exp_addr(i, j));
      expect_eq_u("sram_tag(read)",  sram_tag, tag);

      valid_rd = 0;
      tick();
      expect_eq_u("req_valid(cleared)", req_valid, 0);
    end
  endtask

  task automatic testcase3_write_priority;
    integer iw, jw, ir, jr;
    logic [feat_dim*elen_W-1:0] data;
    integer tag;
    begin
      $display("\n--- testcase3_write_priority ---");
      drive_idle();
      req_ready = 1;

      iw = 1; jw = 2;
      ir = 7; jr = 8;
      data = 'hDEAD_BEEF;
      tag  = 16'h1234;

      // both valid same cycle => must pick write
      valid_wr  = 1;
      tile_i_wr = iw[TI_W-1:0];
      tile_j_wr = jw[TJ_W-1:0];
      feat_in   = data;

      valid_rd  = 1;
      tile_i_rd = ir[TI_W-1:0];
      tile_j_rd = jr[TJ_W-1:0];
      tag_rd    = tag[tag_w-1:0];

      tick();

      expect_eq_u("req_is_wr(priority)", req_is_wr, 1);
      expect_eq_u("sram_addr(priority)", sram_addr, exp_addr(iw, jw));

      // clear
      drive_idle();
      tick();
      expect_eq_u("req_valid(cleared)", req_valid, 0);
    end
  endtask

  task automatic testcase4_backpressure_pending;
    integer i, j, tag;
    begin
      $display("\n--- testcase4_backpressure_pending ---");
      drive_idle();

      // block downstream
      req_ready = 0;

      // issue read
      i = 2; j = 3; tag = 16'h55AA;
      valid_rd  = 1;
      tile_i_rd = i[TI_W-1:0];
      tile_j_rd = j[TJ_W-1:0];
      tag_rd    = tag[tag_w-1:0];

      tick();

      // now pending exists, req_valid must stay 1 while req_ready=0
      expect_eq_u("req_valid(stall)", req_valid, 1);
      expect_eq_u("ready_rd(stall)",  ready_rd, 0);
      expect_eq_u("ready_wr(stall)",  ready_wr, 0);

      // try to send a write while pending -> should NOT be accepted (ready_wr=0)
      valid_wr  = 1;
      tile_i_wr = TI_W'(9);
      tile_j_wr = TJ_W'(9);
      feat_in   = 'hCAFE;
      tick();
      expect_eq_u("still req_is_wr(stall)", req_is_wr, 0);
      expect_eq_u("still sram_addr(stall)", sram_addr, exp_addr(i, j));

      // release downstream
      req_ready = 1;
      drive_idle();
      tick(); // handshake happens here -> clear pending
      expect_eq_u("req_valid(cleared)", req_valid, 0);
    end
  endtask

  // -----------------------------
  // main
  // -----------------------------
  initial begin
    $dumpfile("./vvp/tb_burst_req_if.vcd");
    $dumpvars(0, tb_burst_req_if);

    // reset
    drive_idle();
    req_ready = 0;
    rst = 1;
    repeat (3) tick();
    rst = 0;
    tick();

    // run tests
    testcase1_basic_write();
    testcase2_basic_read();
    testcase3_write_priority();
    testcase4_backpressure_pending();

    if (err_cnt == 0) $display("\n[PASS] tb_burst_req_if all tests passed.\n");
    else              $display("\n[FAIL] tb_burst_req_if err_cnt=%0d\n", err_cnt);

    #20;
    $finish;
  end

endmodule

`default_nettype wire
