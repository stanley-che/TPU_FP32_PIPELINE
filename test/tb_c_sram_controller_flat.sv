/*
iverilog -g2012 -Wall -I./src -o ./vvp/sim  ./test/tb_c_sram_controller_flat.sv 

vvp ./vvp/sim
*/
`include "./src/c_sram_ctrl_flat.sv"
`timescale 1ns/1ps
`default_nettype none

module tb_c_sram_ctrl_flat;

  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = (DATA_W/8);
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned COL_W  = (N<=1)?1:$clog2(N);

  logic clk, rst;
  initial clk = 0;
  always #5 clk = ~clk;

  // DUT IO
  logic start, flush;

  logic [M*N*DATA_W-1:0] c_out_flat;
  logic [M*N-1:0]        c_valid_flat;

  logic              cpu_c_en, cpu_c_re;
  logic [ROW_W-1:0]  cpu_c_row;
  logic [COL_W-1:0]  cpu_c_col;
  logic [DATA_W-1:0] cpu_c_rdata;
  logic              cpu_c_rvalid;

  logic [M*N*DATA_W-1:0] c_out_flat_o;
  logic [M*N-1:0]        c_valid_flat_o;
  logic                  C_valid;

  // local matrix
  logic signed [DATA_W-1:0] C_in [M][N];

  // DUT
  c_sram_ctrl_flat #(
    .M(M), .N(N), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(1)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start(start),
    .flush(flush),

    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_flat),

    .cpu_c_en(cpu_c_en),
    .cpu_c_re(cpu_c_re),
    .cpu_c_row(cpu_c_row),
    .cpu_c_col(cpu_c_col),
    .cpu_c_rdata(cpu_c_rdata),
    .cpu_c_rvalid(cpu_c_rvalid),

    .c_out_flat_o(c_out_flat_o),
    .c_valid_flat_o(c_valid_flat_o),
    .C_valid(C_valid)
  );

  task automatic do_reset();
    begin
      rst = 1'b1;
      start = 1'b0;
      flush = 1'b0;
      c_out_flat = '0;
      c_valid_flat = '0;

      cpu_c_en = 1'b0; cpu_c_re = 1'b0;
      cpu_c_row = '0; cpu_c_col = '0;

      repeat (10) @(posedge clk);
      rst = 1'b0;
      repeat (5) @(posedge clk);
    end
  endtask

  // pack C_in into c_out_flat
  task automatic pack_out();
    int i,j;
    int unsigned idx;
    begin
      c_out_flat = '0;
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          idx = i*N + j;
          c_out_flat[idx*DATA_W +: DATA_W] = C_in[i][j];
        end
      end
    end
  endtask

  // pulse valid for all elements (1 cycle)
  task automatic pulse_valid_all();
    begin
      @(posedge clk);
      c_valid_flat <= {M*N{1'b1}};
      @(posedge clk);
      c_valid_flat <= '0;
    end
  endtask

  task automatic print_in();
    int i,j;
    begin
      $display("=== INPUT C_in ===");
      for (i=0;i<M;i++) begin
        $write("row %0d: ", i);
        for (j=0;j<N;j++) $write("%0d ", $signed(C_in[i][j]));
        $write("\n");
      end
    end
  endtask

  // CPU read one element (hold request until rvalid)
  task automatic cpu_read_one(
    input int unsigned row,
    input int unsigned col,
    output logic [DATA_W-1:0] data
  );
    int timeout;
    begin
      // wait until C_valid
      timeout = 0;
      while (C_valid !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 50000) $fatal(1, "[TB] timeout waiting C_valid");
      end

      cpu_c_row <= row[ROW_W-1:0];
      cpu_c_col <= col[COL_W-1:0];

      @(posedge clk);
      cpu_c_en <= 1'b1;
      cpu_c_re <= 1'b1;

      timeout = 0;
      while (cpu_c_rvalid !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 5000) $fatal(1, "[TB] cpu_read timeout row=%0d col=%0d", row, col);
      end

      data = cpu_c_rdata;

      @(posedge clk);
      cpu_c_en <= 1'b0;
      cpu_c_re <= 1'b0;

      @(posedge clk);
    end
  endtask

  task automatic read_and_check();
    int i,j;
    logic [DATA_W-1:0] got;
    int err;
    begin
      err = 0;
      $display("=== OUTPUT (CPU read from SRAM) ===");
      for (i=0;i<M;i++) begin
        $write("row %0d: ", i);
        for (j=0;j<N;j++) begin
          cpu_read_one(i,j,got);
          $write("%0d ", $signed(got));
          if (got !== C_in[i][j]) begin
            err++;
            $display("\n[MIS] C[%0d][%0d] got=%0d exp=%0d (0x%08x vs 0x%08x)",
              i,j,$signed(got),$signed(C_in[i][j]),got,C_in[i][j]);
          end
        end
        $write("\n");
      end

      if (err==0) $display("[TB] PASS");
      else $fatal(1, "[TB] FAIL mismatches=%0d", err);
    end
  endtask
  int tmo;
  initial begin
    do_reset();

    // deterministic pattern
    for (int i=0;i<M;i++)
      for (int j=0;j<N;j++)
        C_in[i][j] = $signed(100*i + j);

    print_in();

    // new tile
    @(posedge clk);
    start <= 1'b1;
    flush <= 1'b1;
    @(posedge clk);
    start <= 1'b0;
    flush <= 1'b0;

    // drive inputs + valid
    pack_out();
    pulse_valid_all();

    // wait valid flag
    tmo = 0;
    while (C_valid !== 1'b1) begin
      @(posedge clk);
      tmo++;
      if (tmo > 50000) $fatal(1, "[TB] timeout waiting C_valid");
    end
    $display("[TB] C_valid=1 (SRAM write complete).");

    read_and_check();

    $display("[TB] finished.");
    $finish;
  end

endmodule

`default_nettype wire
