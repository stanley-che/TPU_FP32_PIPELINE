`include "./src/sram_seq_wr_rd.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_sram_seq_wr_rd;

  localparam int ADDR_W = 16;
  localparam int DATA_W = 32;
  localparam int BYTE_W = (DATA_W/8);
  localparam int BANKS  = 8;
  localparam int MEM_ADDR_W = 10;
  localparam int CONFLICT_POLICY = 1;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic rst;

  logic start;
  wire  busy, done;

  logic [ADDR_W-1:0] wr_addr;
  logic [DATA_W-1:0] wr_data;
  logic [BYTE_W-1:0] wr_mask;

  logic [ADDR_W-1:0] rd_addr;
  wire  rd_valid;
  wire [DATA_W-1:0] rd_data;

  sram_seq_wr_rd #(
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .BANKS(BANKS),
    .MEM_ADDR_W(MEM_ADDR_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) dut (
    .clk(clk), .rst(rst),
    .start(start), .busy(busy), .done(done),
    .wr_addr(wr_addr), .wr_data(wr_data), .wr_mask(wr_mask),
    .rd_addr(rd_addr), .rd_valid(rd_valid), .rd_data(rd_data)
  );

  function automatic int clog2i(input int v);
    int r; begin r=0; while ((1<<r)<v) r++; return r; end
  endfunction
  localparam int BANK_W = (BANKS<=1)?1:clog2i(BANKS);

  function automatic int unsigned mk_key(input logic [ADDR_W-1:0] a);
    logic [MEM_ADDR_W-1:0] w;
    logic [BANK_W-1:0]     b;
    begin
      w = a[2 +: MEM_ADDR_W];
      b = a[2+MEM_ADDR_W +: BANK_W];
      mk_key = (int'(b) << MEM_ADDR_W) | int'(w);
    end
  endfunction

  localparam int SHADOW_DEPTH = (BANKS * (1<<MEM_ADDR_W));
  logic [DATA_W-1:0] shadow [0:SHADOW_DEPTH-1];

  // ★改點(1)：mask=0 才寫；mask=1 不寫
  function automatic logic [DATA_W-1:0] apply_wmask_mask1_disable(
    input logic [DATA_W-1:0] oldv,
    input logic [DATA_W-1:0] newv,
    input logic [BYTE_W-1:0] m
  );
    logic [DATA_W-1:0] outv;
    int bi;
    begin
      outv = oldv;
      for (bi = 0; bi < BYTE_W; bi++) begin
        if (!m[bi]) outv[8*bi +: 8] = newv[8*bi +: 8];
      end
      return outv;
    end
  endfunction

  task automatic do_wr_rd(
    input logic [ADDR_W-1:0] a_wr,
    input logic [DATA_W-1:0] d_wr,
    input logic [BYTE_W-1:0] m_wr,
    input logic [ADDR_W-1:0] a_rd
  );
    int unsigned key_wr, key_rd;
    logic [DATA_W-1:0] exp;
    int timeout;
    bit saw_rd_valid;

    begin
      wr_addr = a_wr;
      wr_data = d_wr;
      wr_mask = m_wr;
      rd_addr = a_rd;

      key_wr = mk_key(a_wr);
      shadow[key_wr] = apply_wmask_mask1_disable(shadow[key_wr], d_wr, m_wr);

      key_rd = mk_key(a_rd);
      exp = shadow[key_rd];

      @(posedge clk);
      start = 1'b1;
      @(posedge clk);
      start = 1'b0;

      timeout = 0;
      saw_rd_valid = 0;
      while (!done) begin
        @(posedge clk);
        timeout++;
        if (rd_valid) saw_rd_valid = 1;
        if (timeout > 200) begin
          $display("[TB][FAIL] timeout waiting done. busy=%0d", busy);
          $fatal(1);
        end
      end

      if (!saw_rd_valid) begin
        $display("[TB][FAIL] never saw rd_valid pulse before done");
        $fatal(1);
      end

      if (rd_data !== exp) begin
        $display("[TB][FAIL] MISMATCH!");
        $display("  wr_addr=0x%0h wr_data=0x%08h wr_mask=0x%0h", a_wr, d_wr, m_wr);
        $display("  rd_addr=0x%0h got=0x%08h exp=0x%08h", a_rd, rd_data, exp);
        $fatal(1);
      end else begin
        $display("[TB][PASS] wr->rd OK  wr_addr=0x%0h rd_addr=0x%0h rd=0x%08h mask=0x%0h",
                 a_wr, a_rd, rd_data, m_wr);
      end

      @(posedge clk);
    end
  endtask

  int t;
  initial begin
    start=0; wr_addr='0; wr_data='0; wr_mask='0; rd_addr='0;
    for (t=0; t<SHADOW_DEPTH; t++) shadow[t] = '0;

    rst = 1;
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (2) @(posedge clk);

    // ★改點(2)：full write 用 0（全都不 mask）
    do_wr_rd(16'h0000, 32'hDEADBEEF, 4'b0000, 16'h0000);
    do_wr_rd(16'h0000, 32'h000000AA, 4'b1110, 16'h0000); // 只寫 byte0 => 其他 mask=1
    do_wr_rd(16'h0000, 32'hBEEF0000, 4'b0011, 16'h0000); // 只寫 byte2/3

    $display("\n[TB] BASIC TESTS PASSED ✅\n");
    $finish;
  end

endmodule

`default_nettype wire
