`timescale 1ns/1ps
//iverilog -g2012 -Wall -o tb_mac_booth_fixed_unsigned.vvp ./test/tb_mac_booth_fixed_unsigned.sv mac_booth_fixed_unsigned.sv
//vvp tb_mac_booth_fixed_unsigned.vvp
module tb_mac_booth_fixed_unsigned;

  // ---- Parameters (match DUT defaults) ----
  localparam int mul_len = 24;
  localparam int out_len = 48;
  localparam int add_len = 102;

  // ---- DUT I/O ----
  logic clk;
  logic rst;
  logic start;
  logic [mul_len-1:0] a;
  logic [mul_len-1:0] b;
  logic [out_len-1:0] product;
  logic mac_done;

  // ---- Instantiate DUT ----
  mac_booth_fixed_unsigned #(
    .mul_len(mul_len),
    .out_len(out_len),
    .add_len(add_len)
  ) dut (
    .clk(clk),
    .rst(rst),
    .start(start),
    .a(a),
    .b(b),
    .product(product),
    .mac_done(mac_done)
  );

  // ---- Clock ----
  initial clk = 0;
  always #5 clk = ~clk; // 100MHz

  // ---- Reference function (unsigned 24x24 -> 48) ----
  function automatic logic [out_len-1:0] ref_mul(
    input logic [mul_len-1:0] aa,
    input logic [mul_len-1:0] bb
  );
    logic [47:0] tmp;
    begin
      tmp = aa * bb;          // unsigned multiply in SV
      ref_mul = tmp[out_len-1:0];
    end
  endfunction

  // ---- Drive one transaction and check ----
  task automatic run_one(
    input logic [mul_len-1:0] aa,
    input logic [mul_len-1:0] bb
  );
    logic [out_len-1:0] exp;
    int unsigned timeout;

    begin
      exp = ref_mul(aa, bb);

      // apply inputs
      @(negedge clk);
      a     <= aa;
      b     <= bb;
      start <= 1'b1;

      // 1-cycle start pulse
      @(negedge clk);
      start <= 1'b0;

      // wait for done (with timeout)
      timeout = 0;
      while (mac_done !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 2000) begin
          $fatal(1, "[TB] TIMEOUT waiting mac_done. a=%0h b=%0h", aa, bb);
        end
      end

      // sample output on done
      @(posedge clk);
      if (product !== exp) begin
        $display("[TB] FAIL a=%0h b=%0h got=%0h exp=%0h", aa, bb, product, exp);
        $fatal(1);
      end else begin
        $display("[TB] PASS a=%0d b=%0d product=%0d", aa, bb, product);
      end
    end
  endtask

  // ---- Reset sequence ----
  initial begin
    // init
    rst   = 1'b1;
    start = 1'b0;
    a     = '0;
    b     = '0;

    // hold reset for a few cycles
    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // ---- Directed tests (edge cases) ----
    run_one(24'h000000, 24'h000000);
    run_one(24'h000001, 24'h000001);
    run_one(24'h000001, 24'hFFFFFF);
    run_one(24'hFFFFFF, 24'h000001);
    run_one(24'hFFFFFF, 24'hFFFFFF);
    run_one(24'h800000, 24'h000002);
    run_one(24'h123456, 24'h654321);

    // ---- Random tests ----
    for (int i = 0; i < 200; i++) begin
      logic [mul_len-1:0] ra, rb;
      ra = $urandom();
      rb = $urandom();
      run_one(ra, rb);
    end

    $display("[TB] ALL TESTS PASSED ✅");
    $finish;
  end

endmodule
