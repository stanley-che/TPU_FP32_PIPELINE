/*
iverilog -g2012 -Wall -o ./vvp/tb_sram.vvp ./test/tb_sram_mem.sv 
 
vvp ./vvp/tb_sram.vvp

*/
`include "./src/axil_2p_sram.sv"
`timescale 1ns/1ps
`default_nettype none 

module tb_sram_mem_ab_iverilog;

  // ----------------------------
  // parameters
  // ----------------------------
  localparam integer ADDR_W = 6;
  localparam integer DATA_W = 32;
  localparam integer BYTE_W = (DATA_W/8);
  localparam integer DEPTH  = (1 << ADDR_W);
  localparam integer VERBOSE = 1; 
  // 0=READ_FIRST, 1=WRITE_FIRST
  localparam integer CONFLICT_POLICY = 1;

  // ----------------------------
  // DUT I/O
  // ----------------------------
  reg                   clk;
  reg                   rst;

  reg                   a_en;
  reg                   a_re;
  reg  [ADDR_W-1:0]     a_addr;
  wire [DATA_W-1:0]     a_rdata;
  wire                  a_rvalid;

  reg                   b_en;
  reg                   b_we;
  reg  [ADDR_W-1:0]     b_addr;
  reg  [DATA_W-1:0]     b_wdata;
  reg  [BYTE_W-1:0]     b_wmask;
  reg [ADDR_W-1:0] exp_addr_q; 
  // ----------------------------
  // instantiate DUT
  // ----------------------------
  sram_mem_ab #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) dut (
    .clk(clk),
    .rst(rst),

    .a_en(a_en),
    .a_re(a_re),
    .a_addr(a_addr),
    .a_rdata(a_rdata),
    .a_rvalid(a_rvalid),

    .b_en(b_en),
    .b_we(b_we),
    .b_addr(b_addr),
    .b_wdata(b_wdata),
    .b_wmask(b_wmask)
  );

  // ----------------------------
  // clock
  // ----------------------------
  initial clk = 1'b0;
  always #5 clk = ~clk;

  // ----------------------------
  // reference model memory (byte array)
  // ----------------------------
  reg [7:0] ref_mem [0:DEPTH-1][0:BYTE_W-1];

  function [DATA_W-1:0] ref_pack;
    input [ADDR_W-1:0] addr;
    reg [DATA_W-1:0] tmp;
    integer i;
    begin
      tmp = {DATA_W{1'b0}};
      for (i=0; i<BYTE_W; i=i+1)
        tmp[i*8 +: 8] = ref_mem[addr][i];
      ref_pack = tmp;
    end
  endfunction

  function [DATA_W-1:0] merge_write_first;
    input [DATA_W-1:0] old_data;
    input [DATA_W-1:0] wdata;
    input [BYTE_W-1:0] wmask;
    reg [DATA_W-1:0] tmp;
    integer i;
    begin
      tmp = old_data;
      for (i=0; i<BYTE_W; i=i+1)
        if (wmask[i]) tmp[i*8 +: 8] = wdata[i*8 +: 8];
      merge_write_first = tmp;
    end
  endfunction

  task ref_write;
    input [ADDR_W-1:0] addr;
    input [DATA_W-1:0] wdata;
    input [BYTE_W-1:0] wmask;
    integer i;
    begin
      for (i=0; i<BYTE_W; i=i+1)
        if (wmask[i]) ref_mem[addr][i] = wdata[i*8 +: 8];
    end
  endtask

  // ----------------------------
  // scoreboard pipeline (1-cycle latency)
  // ----------------------------
  reg                exp_valid_q;
  reg [DATA_W-1:0]   exp_rdata_q;
  integer            err_cnt;

  // helper temps for always block (declare outside for Icarus)
  reg [DATA_W-1:0] base;
  reg [DATA_W-1:0] next_exp;
  reg              do_w;
  reg              same;

  always @(posedge clk) begin
    if (rst) begin
      exp_valid_q <= 1'b0;
      exp_rdata_q <= {DATA_W{1'b0}};
      err_cnt     <= 0;
    end else begin
      // check last cycle output
            // check last cycle output
      if (a_rvalid) begin
        if (VERBOSE) begin
          $display("[%0t] READ  addr=%0d  got=%08x  exp=%08x",
                   $time, exp_addr_q, a_rdata, exp_rdata_q);
        end

        if (a_rdata !== exp_rdata_q) begin
          $display("[%0t] ERROR: read mismatch addr=%0d got=%08x exp=%08x",
                   $time, exp_addr_q, a_rdata, exp_rdata_q);
          err_cnt <= err_cnt + 1;
        end
      end


      // compute expected for NEXT cycle based on current inputs
      base = ref_pack(a_addr);
      do_w = (b_en && b_we);
      same = do_w && (b_addr == a_addr);

      if (same && (CONFLICT_POLICY == 1)) begin
        next_exp = merge_write_first(base, b_wdata, b_wmask);
      end else begin
        next_exp = base;
      end
            if (VERBOSE && same) begin
        $display("[%0t] CONFLICT same-cycle addr=%0d policy=%0d base(old)=%08x wdata=%08x wmask=%0b -> exp(next)=%08x",
                 $time, a_addr, CONFLICT_POLICY, base, b_wdata, b_wmask, next_exp);
      end

      exp_valid_q <= (a_en && a_re);
      exp_addr_q  <= a_addr;   // 下一拍回來時，用這個addr印

      exp_rdata_q <= next_exp;

      // update reference memory at end (same edge)
      if (do_w) begin
        ref_write(b_addr, b_wdata, b_wmask);
      end
    end
  end

  // ----------------------------
  // drive tasks
  // ----------------------------
  task idle_cycle;
    begin
      a_en=0; a_re=0; a_addr={ADDR_W{1'b0}};
      b_en=0; b_we=0; b_addr={ADDR_W{1'b0}}; b_wdata={DATA_W{1'b0}}; b_wmask={BYTE_W{1'b0}};
      @(posedge clk);
    end
  endtask

  task do_write;
    input [ADDR_W-1:0] addr;
    input [DATA_W-1:0] data;
    input [BYTE_W-1:0] mask;
    begin
      b_en=1; b_we=1; b_addr=addr; b_wdata=data; b_wmask=mask;
      a_en=0; a_re=0; a_addr={ADDR_W{1'b0}};
      @(posedge clk);
      b_en=0; b_we=0; b_addr={ADDR_W{1'b0}}; b_wdata={DATA_W{1'b0}}; b_wmask={BYTE_W{1'b0}};
    end
  endtask

  task do_read;
    input [ADDR_W-1:0] addr;
    begin
      a_en=1; a_re=1; a_addr=addr;
      b_en=0; b_we=0; b_addr={ADDR_W{1'b0}}; b_wdata={DATA_W{1'b0}}; b_wmask={BYTE_W{1'b0}};
      @(posedge clk);
      a_en=0; a_re=0; a_addr={ADDR_W{1'b0}};
    end
  endtask

  task do_read_write_same_cycle;
    input [ADDR_W-1:0] addr;
    input [DATA_W-1:0] data;
    input [BYTE_W-1:0] mask;
    begin
      a_en=1; a_re=1; a_addr=addr;
      b_en=1; b_we=1; b_addr=addr; b_wdata=data; b_wmask=mask;
      @(posedge clk);
      a_en=0; a_re=0; a_addr={ADDR_W{1'b0}};
      b_en=0; b_we=0; b_addr={ADDR_W{1'b0}}; b_wdata={DATA_W{1'b0}}; b_wmask={BYTE_W{1'b0}};
    end
  endtask

  // ----------------------------
  // tests
  // ----------------------------
  integer t;
  reg [ADDR_W-1:0] addr;
  reg [DATA_W-1:0] data;
  reg [BYTE_W-1:0] mask;

  initial begin
    // init
    a_en=0; a_re=0; a_addr=0;
    b_en=0; b_we=0; b_addr=0; b_wdata=0; b_wmask=0;
    err_cnt = 0;

    // init ref_mem
    for (t=0; t<DEPTH; t=t+1) begin : INIT_MEM
      integer i;
      for (i=0; i<BYTE_W; i=i+1) begin
        ref_mem[t][i] = 8'h00;
      end
    end

    // reset
    rst = 1;
    repeat (3) @(posedge clk);
    rst = 0;
    @(posedge clk);

    $display("=== Test1: full writes + reads ===");
    for (t=0; t<10; t=t+1) begin
      addr = $urandom % DEPTH;
      data = $urandom;
      do_write(addr, data, {BYTE_W{1'b1}});
      do_read(addr);
      idle_cycle(); // let a_rvalid fire + scoreboard check
    end

    $display("=== Test2: byte-mask writes ===");
    for (t=0; t<20; t=t+1) begin
      addr = $urandom % DEPTH;
      data = $urandom;
      mask = ($urandom % (1<<BYTE_W));
      if (mask == 0) mask = 1; // avoid all-zero
      do_write(addr, data, mask);
      do_read(addr);
      idle_cycle();
    end

    $display("=== Test3: conflict same addr same cycle, policy=%0d ===", CONFLICT_POLICY);
    for (t=0; t<20; t=t+1) begin
      addr = $urandom % DEPTH;
      data = $urandom;
      mask = ($urandom % (1<<BYTE_W));
      if (mask == 0) mask = 1;
      do_read_write_same_cycle(addr, data, mask);
      idle_cycle();
    end

    idle_cycle(); // flush last

    if (err_cnt == 0) $display("ALL TESTS PASSED ✅");
    else              $display("TESTS FAILED ❌ err_cnt=%0d", err_cnt);

    $finish;
  end

endmodule
