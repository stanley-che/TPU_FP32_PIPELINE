// ============================================================
// tb_rd_assemble_fifo.sv
// ============================================================
// Build:
// iverilog -g2012 -Wall -I./src -o ./vvp/tb_rd_assemble_fifo.vvp  ./test/tb_rd_assemble_fifo.sv
//
// Run:
// vvp ./vvp/tb_rd_assemble_fifo.vvp
// gtkwave ./vvp/tb_rd_assemble_fifo.vcd
// ============================================================
`include "./src/AMOLED/feature_sram/rd_assemble_fifo.sv"
`timescale 1ns/1ps
`default_nettype none



module tb_rd_assemble_fifo;

  // ------------------------------------------------------------
  // params
  // ------------------------------------------------------------
  localparam int unsigned SRAM_BUS_W      = 32;
  localparam int unsigned WORDS_PER_FEAT  = 8;
  localparam int unsigned FEAT_W          = SRAM_BUS_W * WORDS_PER_FEAT; // 256
  localparam int unsigned TAG_W           = 16;
  localparam int unsigned FIFO_DEPTH      = 2;

  localparam int unsigned IDX_W =
    (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);

  // ------------------------------------------------------------
  // clock/reset
  // ------------------------------------------------------------
  logic clk = 0;
  always #5 clk = ~clk;
  logic rst;

  // ------------------------------------------------------------
  // DUT I/O
  // ------------------------------------------------------------
  logic beat_valid;
  logic beat_ready;
  logic [TAG_W-1:0] beat_tag;
  logic [IDX_W-1:0] beat_idx;
  logic [SRAM_BUS_W-1:0] beat_data;

  logic feat_out_valid;
  logic feat_out_ready;
  logic [TAG_W-1:0] feat_out_tag;
  logic [FEAT_W-1:0] feat_out_data;

  rd_assemble_fifo #(
    .SRAM_BUS_W(SRAM_BUS_W),
    .FEAT_W(FEAT_W),
    .WORDS_PER_FEAT(WORDS_PER_FEAT),
    .TAG_W(TAG_W),
    .FIFO_DEPTH(FIFO_DEPTH)
  ) dut (
    .clk(clk),
    .rst(rst),

    .beat_valid(beat_valid),
    .beat_ready(beat_ready),
    .beat_tag(beat_tag),
    .beat_idx(beat_idx),
    .beat_data(beat_data),

    .feat_out_valid(feat_out_valid),
    .feat_out_ready(feat_out_ready),
    .feat_out_tag(feat_out_tag),
    .feat_out_data(feat_out_data)
  );

  // ------------------------------------------------------------
  // waveform
  // ------------------------------------------------------------
  initial begin
    $dumpfile("./vvp/tb_rd_assemble_fifo.vcd");
    $dumpvars(0, tb_rd_assemble_fifo);
  end

  // ------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------
  function automatic [FEAT_W-1:0] put_word_ref(
    input [FEAT_W-1:0] feat,
    input int unsigned idx,
    input [SRAM_BUS_W-1:0] w
  );
    reg [FEAT_W-1:0] tmp;
    int unsigned sh;
    begin
      tmp = feat;
      sh  = idx * SRAM_BUS_W;
      tmp = tmp & ~({{(FEAT_W-SRAM_BUS_W){1'b0}}, {SRAM_BUS_W{1'b1}}} << sh);
      tmp = tmp | ({{(FEAT_W-SRAM_BUS_W){1'b0}}, w} << sh);
      put_word_ref = tmp;
    end
  endfunction

  // xorshift pseudo-rand (iverilog friendly)
  int unsigned rng = 32'hc0ffee12;
  function automatic int unsigned urand();
    begin
      rng ^= (rng << 13);
      rng ^= (rng >> 17);
      rng ^= (rng << 5);
      urand = rng;
    end
  endfunction

  // ------------------------------------------------------------
  // drive one beat (wait until accepted)
  // ------------------------------------------------------------
  task automatic send_beat(
    input logic [TAG_W-1:0] tag,
    input int unsigned idx,
    input logic [SRAM_BUS_W-1:0] data
  );
    int unsigned t;
    begin
      beat_tag  <= tag;
      beat_idx  <= idx[IDX_W-1:0];
      beat_data <= data;
      beat_valid<= 1'b1;

      t = 0;
      while (!(beat_valid && beat_ready)) begin
        @(posedge clk);
        t++;
        if (t > 5000) begin
          $display("ERROR: send_beat timeout (beat_ready stuck low)");
          $finish(1);
        end
      end

      // drop next cycle
      @(posedge clk);
      beat_valid <= 1'b0;
    end
  endtask

  // wait for one feature output handshake
  task automatic wait_feat_hs(input int unsigned timeout_cycles);
    int unsigned t;
    begin
      t = 0;
      while (!(feat_out_valid && feat_out_ready)) begin
        @(posedge clk);
        t++;
        if (t > timeout_cycles) begin
          $display("ERROR: wait_feat_hs timeout");
          $finish(1);
        end
      end
    end
  endtask

  // ------------------------------------------------------------
  // scoreboard
  // ------------------------------------------------------------
  logic [FEAT_W-1:0] exp_feat_q [0:15];
  logic [TAG_W-1:0]  exp_tag_q  [0:15];
  int unsigned exp_wptr, exp_rptr, exp_cnt;

  task automatic exp_push(input logic [TAG_W-1:0] tag, input logic [FEAT_W-1:0] feat);
    begin
      exp_tag_q[exp_wptr]  = tag;
      exp_feat_q[exp_wptr] = feat;
      exp_wptr++;
      exp_cnt++;
    end
  endtask

  task automatic exp_pop_and_check();
    begin
      if (exp_cnt == 0) begin
        $display("ERROR: unexpected feat_out");
        $finish(1);
      end
      if (feat_out_tag !== exp_tag_q[exp_rptr]) begin
        $display("ERROR: tag mismatch exp=0x%h got=0x%h",
                 exp_tag_q[exp_rptr], feat_out_tag);
        $finish(1);
      end
      if (feat_out_data !== exp_feat_q[exp_rptr]) begin
        $display("ERROR: data mismatch");
        $display("  exp=0x%h", exp_feat_q[exp_rptr]);
        $display("  got=0x%h", feat_out_data);
        $finish(1);
      end
      exp_rptr++;
      exp_cnt--;
    end
  endtask

  // ------------------------------------------------------------
  // ready generator for downstream
  // ------------------------------------------------------------
  bit backpressure_mode;

  always @(posedge clk) begin
    if (rst) begin
      feat_out_ready <= 1'b0;
    end else if (backpressure_mode) begin
      // 70% ready
      feat_out_ready <= (urand() % 10) < 7;
    end else begin
      feat_out_ready <= 1'b1;
    end
  end

  // whenever feature handshake happens, check it
  always @(posedge clk) begin
    if (!rst && feat_out_valid && feat_out_ready) begin
      $display("[FEAT] tag=0x%h data[31:0]=0x%08x time=%0t",
               feat_out_tag, feat_out_data[31:0], $time);
      exp_pop_and_check();
    end
  end

  // ------------------------------------------------------------
  // testcases
  // ------------------------------------------------------------
  task automatic tc_basic_shuffle();
    logic [FEAT_W-1:0] exp;
    logic [TAG_W-1:0] tag;
    int unsigned order [0:WORDS_PER_FEAT-1];
    int i, j;
    int unsigned tmp;
    begin
      $display("== tc_basic_shuffle ==");

      tag = 16'hBEEF;
      exp = '0;

      // prepare known words: word[idx] = 0x1000 + idx
      for (i=0; i<WORDS_PER_FEAT; i++) begin
        exp = put_word_ref(exp, i, 32'h1000 + i);
        order[i] = i;
      end

      // Fisher-Yates shuffle
      for (i=WORDS_PER_FEAT-1; i>0; i--) begin
        j = urand() % (i+1);
        tmp = order[i];
        order[i] = order[j];
        order[j] = tmp;
      end

      // enqueue expected
      exp_push(tag, exp);

      // send beats in shuffled order
      for (i=0; i<WORDS_PER_FEAT; i++) begin
        send_beat(tag, order[i], 32'h1000 + order[i]);
      end

      // wait until the feature pops (checked by monitor)
      // give enough cycles
      repeat (200) @(posedge clk);

      if (exp_cnt != 0) begin
        $display("ERROR: expected feature not produced");
        $finish(1);
      end
    end
  endtask

  task automatic tc_duplicate_idx();
    logic [FEAT_W-1:0] exp;
    logic [TAG_W-1:0] tag;
    int i;
    begin
      $display("== tc_duplicate_idx ==");

      tag = 16'hCAFE;
      exp = '0;

      for (i=0; i<WORDS_PER_FEAT; i++) begin
        exp = put_word_ref(exp, i, 32'h2000 + i);
      end
      exp_push(tag, exp);

      // send idx 0 twice (duplicate)
      send_beat(tag, 0, 32'h2000);
      send_beat(tag, 0, 32'hDEAD_BEEF); // should be ignored by DUT
      // send the rest
      for (i=1; i<WORDS_PER_FEAT; i++) begin
        send_beat(tag, i, 32'h2000 + i);
      end

      repeat (200) @(posedge clk);

      if (exp_cnt != 0) begin
        $display("ERROR: expected feature not produced (dup test)");
        $finish(1);
      end
    end
  endtask

  task automatic tc_backpressure_and_fifo_full();
    logic [FEAT_W-1:0] exp0, exp1, exp2;
    logic [TAG_W-1:0] tag0, tag1, tag2;
    int i;
    begin
      $display("== tc_backpressure_and_fifo_full ==");

      // enable downstream backpressure
      backpressure_mode = 1'b1;

      // build 3 features; FIFO depth is 2 => will exercise full condition
      tag0 = 16'h1111; exp0 = '0;
      tag1 = 16'h2222; exp1 = '0;
      tag2 = 16'h3333; exp2 = '0;

      for (i=0; i<WORDS_PER_FEAT; i++) begin
        exp0 = put_word_ref(exp0, i, 32'h3000 + i);
        exp1 = put_word_ref(exp1, i, 32'h4000 + i);
        exp2 = put_word_ref(exp2, i, 32'h5000 + i);
      end

      exp_push(tag0, exp0);
      exp_push(tag1, exp1);
      exp_push(tag2, exp2);

      // send feature0 beats
      for (i=0; i<WORDS_PER_FEAT; i++) send_beat(tag0, i, 32'h3000 + i);
      // send feature1 beats
      for (i=0; i<WORDS_PER_FEAT; i++) send_beat(tag1, i, 32'h4000 + i);

      // Now force ready low for a while to fill FIFO & trigger beat backpressure
      feat_out_ready <= 1'b0;
      repeat (30) @(posedge clk);

      // send feature2 beats while FIFO likely full -> beat_ready should stall sometimes but task waits so it won't hang
      for (i=0; i<WORDS_PER_FEAT; i++) send_beat(tag2, i, 32'h5000 + i);

      // release ready
      feat_out_ready <= 1'b1;
      repeat (500) @(posedge clk);

      if (exp_cnt != 0) begin
        $display("ERROR: not all expected features were produced (backpressure)");
        $finish(1);
      end

      backpressure_mode = 1'b0;
    end
  endtask

  // ------------------------------------------------------------
  // main
  // ------------------------------------------------------------
  initial begin
    rst = 1'b1;

    beat_valid = 1'b0;
    beat_tag   = '0;
    beat_idx   = '0;
    beat_data  = '0;

    feat_out_ready = 1'b0;
    backpressure_mode = 1'b0;

    exp_wptr = 0;
    exp_rptr = 0;
    exp_cnt  = 0;

    repeat (10) @(posedge clk);
    rst = 1'b0;

    // default ready high
    feat_out_ready = 1'b1;

    tc_basic_shuffle();
    tc_duplicate_idx();
    tc_backpressure_and_fifo_full();

    $display("PASS ✅ rd_assemble_fifo");
    #20;
    $finish;
  end

endmodule

`default_nettype wire
