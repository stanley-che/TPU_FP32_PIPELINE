// tb_tile_module_fixedexp.sv
// Compile example:
// iverilog -g2012 -Wall -I. -o ./vvp/tb_tile_module.vvp ./test/tb_tile.sv
// vvp ./vvp/tb_tile_module.vvp

`include "./src/tile_module.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_tile_module_fixedexp;

  localparam int M    = 8;
  localparam int N    = 8;
  localparam int KMAX = 1024;   // match your shell/top
  localparam int KLEN = 4;

  localparam int ADDR_W = 16;
  localparam int DATA_W = 32;
  localparam int BYTE_W = (DATA_W/8);

  localparam logic [ADDR_W-1:0] W_BASE = 'h0000;
  localparam logic [ADDR_W-1:0] X_BASE = 'h4000;
  localparam logic [ADDR_W-1:0] C_BASE = 'h8000;

  // ---------------- clock/reset ----------------
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // ---------------- tile_module(top) I/O ----------------
  logic        start;
  logic [15:0] K_len;
  wire         busy;
  wire         done;

  // ---------------- DUT ----------------
  epu_sa_top #(
    .M(M), .N(N), .KMAX(KMAX),
    .ADDR_W(ADDR_W), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .W_BASE(W_BASE), .X_BASE(X_BASE), .C_BASE(C_BASE)
  ) dut (
    .clk(clk),
    .rst(rst),
    .start(start),
    .K_len(K_len),
    .busy(busy),
    .done(done)
  );

  // ============================================================
  // Helpers
  // ============================================================
  function automatic int C_idx(input int i, input int j);
    return i*N + j;
  endfunction

  function automatic logic [ADDR_W-1:0] addr_from_idx(
    input logic [ADDR_W-1:0] base,
    input int unsigned idx_word
  );
    addr_from_idx = base + logic'(idx_word << 2);
  endfunction

  // ============================================================
  // Fixed expected C (same as your systolic_wrap TB)
  // ============================================================
  logic [31:0] C_exp_flat [0:M*N-1];

  task automatic init_fixed_expected_c;
    begin
      // Row 0
      C_exp_flat[0]  = 32'h41400000; C_exp_flat[1]  = 32'h41100000; C_exp_flat[2]  = 32'h41200000; C_exp_flat[3]  = 32'h41340000;
      C_exp_flat[4]  = 32'h41400000; C_exp_flat[5]  = 32'h41100000; C_exp_flat[6]  = 32'h41200000; C_exp_flat[7]  = 32'h41340000;
      // Row 1
      C_exp_flat[8]  = 32'h41340000; C_exp_flat[9]  = 32'h41400000; C_exp_flat[10] = 32'h41100000; C_exp_flat[11] = 32'h41200000;
      C_exp_flat[12] = 32'h41340000; C_exp_flat[13] = 32'h41400000; C_exp_flat[14] = 32'h41100000; C_exp_flat[15] = 32'h41200000;
      // Row 2
      C_exp_flat[16] = 32'h41200000; C_exp_flat[17] = 32'h41340000; C_exp_flat[18] = 32'h41400000; C_exp_flat[19] = 32'h41100000;
      C_exp_flat[20] = 32'h41200000; C_exp_flat[21] = 32'h41340000; C_exp_flat[22] = 32'h41400000; C_exp_flat[23] = 32'h41100000;
      // Row 3
      C_exp_flat[24] = 32'h41100000; C_exp_flat[25] = 32'h41200000; C_exp_flat[26] = 32'h41340000; C_exp_flat[27] = 32'h41400000;
      C_exp_flat[28] = 32'h41100000; C_exp_flat[29] = 32'h41200000; C_exp_flat[30] = 32'h41340000; C_exp_flat[31] = 32'h41400000;
      // Row 4
      C_exp_flat[32] = 32'h41400000; C_exp_flat[33] = 32'h41100000; C_exp_flat[34] = 32'h41200000; C_exp_flat[35] = 32'h41340000;
      C_exp_flat[36] = 32'h41400000; C_exp_flat[37] = 32'h41100000; C_exp_flat[38] = 32'h41200000; C_exp_flat[39] = 32'h41340000;
      // Row 5
      C_exp_flat[40] = 32'h41340000; C_exp_flat[41] = 32'h41400000; C_exp_flat[42] = 32'h41100000; C_exp_flat[43] = 32'h41200000;
      C_exp_flat[44] = 32'h41340000; C_exp_flat[45] = 32'h41400000; C_exp_flat[46] = 32'h41100000; C_exp_flat[47] = 32'h41200000;
      // Row 6
      C_exp_flat[48] = 32'h41200000; C_exp_flat[49] = 32'h41340000; C_exp_flat[50] = 32'h41400000; C_exp_flat[51] = 32'h41100000;
      C_exp_flat[52] = 32'h41200000; C_exp_flat[53] = 32'h41340000; C_exp_flat[54] = 32'h41400000; C_exp_flat[55] = 32'h41100000;
      // Row 7
      C_exp_flat[56] = 32'h41100000; C_exp_flat[57] = 32'h41200000; C_exp_flat[58] = 32'h41340000; C_exp_flat[59] = 32'h41400000;
      C_exp_flat[60] = 32'h41100000; C_exp_flat[61] = 32'h41200000; C_exp_flat[62] = 32'h41340000; C_exp_flat[63] = 32'h41400000;

      $display("[TB] init_fixed_expected_c done");
    end
  endtask

  // ============================================================
  // iverilog-safe MMIO drive into dut.u_shell (no automatic force)
  // ============================================================
  logic              drv_valid;
  logic              drv_is_write;
  logic [ADDR_W-1:0] drv_addr;
  logic [DATA_W-1:0] drv_wdata;
  logic [BYTE_W-1:0] drv_wmask;

  initial begin
    // Override top tie-off by binding these regs into the shell
    force dut.u_shell.mem_cmd_valid    = drv_valid;
    force dut.u_shell.mem_cmd_is_write = drv_is_write;
    force dut.u_shell.mem_cmd_addr     = drv_addr;
    force dut.u_shell.mem_cmd_wdata    = drv_wdata;
    force dut.u_shell.mem_cmd_wmask    = drv_wmask;

    drv_valid    = 1'b0;
    drv_is_write = 1'b0;
    drv_addr     = '0;
    drv_wdata    = '0;
    drv_wmask    = '0;
  end

  task mmio_write(input logic [ADDR_W-1:0] addr, input logic [31:0] data);
    int unsigned guard;
    begin
      drv_addr     = addr;
      drv_wdata    = data;
      drv_wmask    = '1;
      drv_is_write = 1'b1;
      drv_valid    = 1'b1;

      guard = 0;
      while (dut.u_shell.mem_cmd_ready !== 1'b1) begin
        @(posedge clk);
        guard++;
        if (guard > 1_000_000) $fatal(1, "[TB] TIMEOUT waiting mem_cmd_ready on write");
      end

      @(posedge clk); // accept 1 cycle
      drv_valid    = 1'b0;
      drv_is_write = 1'b0;
      drv_addr     = '0;
      drv_wdata    = '0;
      drv_wmask    = '0;
    end
  endtask

  task mmio_read(input logic [ADDR_W-1:0] addr, output logic [31:0] data);
    int unsigned guard;
    begin
      drv_addr     = addr;
      drv_wdata    = '0;
      drv_wmask    = '1;
      drv_is_write = 1'b0;
      drv_valid    = 1'b1;

      guard = 0;
      while (dut.u_shell.mem_cmd_ready !== 1'b1) begin
        @(posedge clk);
        guard++;
        if (guard > 1_000_000) $fatal(1, "[TB] TIMEOUT waiting mem_cmd_ready on read");
      end

      @(posedge clk); // accept
      drv_valid = 1'b0;

      guard = 0;
      while (dut.u_shell.mem_rsp_valid !== 1'b1) begin
        @(posedge clk);
        guard++;
        if (guard > 1_000_000) $fatal(1, "[TB] TIMEOUT waiting mem_rsp_valid on read");
      end
      data = dut.u_shell.mem_rsp_rdata;

      drv_addr     = '0;
      drv_wdata    = '0;
      drv_wmask    = '0;
      drv_is_write = 1'b0;
    end
  endtask

  // ============================================================
  // Fixed stimulus init (same pattern as your systolic_wrap TB)
  // BUT: write into SRAM via MMIO
  // ============================================================
  task automatic init_fixed_tiles;
    int i, j, kk;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    logic [ADDR_W-1:0] addr;
    begin
      aval[0] = 32'h3f800000; // 1.0
      aval[1] = 32'h40000000; // 2.0
      aval[2] = 32'h3f000000; // 0.5
      aval[3] = 32'h40400000; // 3.0

      bval[0] = 32'h3f800000; // 1.0
      bval[1] = 32'h3f000000; // 0.5
      bval[2] = 32'h40000000; // 2.0
      bval[3] = 32'h40400000; // 3.0

      // Fill only KLEN into W SRAM: W[i][k] @ W_BASE + (i*KMAX + k)
      for (kk=0; kk<KLEN; kk++) begin
        for (i=0; i<M; i++) begin
          addr = addr_from_idx(W_BASE, (i*KMAX + kk));
          mmio_write(addr, aval[(kk + i) % 4]);
        end
      end

      // Fill only KLEN into X SRAM: X[k][j] @ X_BASE + (k*N + j)
      for (kk=0; kk<KLEN; kk++) begin
        for (j=0; j<N; j++) begin
          addr = addr_from_idx(X_BASE, (kk*N + j));
          mmio_write(addr, bval[(kk + j) % 4]);
        end
      end

      $display("[TB] init_fixed_tiles done (KLEN=%0d)", KLEN);
    end
  endtask

  // ============================================================
  // Debug dump (read C SRAM)
  // ============================================================
  task automatic dump_c_matrix;
    int i, j;
    logic [31:0] data;
    logic [ADDR_W-1:0] addr;
    begin
      $display("========== C (read from C SRAM) ==========");
      for (i=0;i<M;i++) begin
        $write("Row %0d :", i);
        for (j=0;j<N;j++) begin
          addr = addr_from_idx(C_BASE, (i*N + j));
          mmio_read(addr, data);
          $write(" %08h", data);
        end
        $write("\n");
      end
      $display("==========================================");
    end
  endtask

  // ============================================================
  // Final compare (read C SRAM vs fixed expected)
  // ============================================================
  task automatic check_final;
    int i, j;
    logic [31:0] got, exp;
    logic [ADDR_W-1:0] addr;
    begin
      $display("[TB] Final compare C(SRAM) vs fixed expected ...");

      // wait a couple cycles for safety
      @(posedge clk);
      @(posedge clk);

      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          addr = addr_from_idx(C_BASE, (i*N + j));
          mmio_read(addr, got);
          exp = C_exp_flat[C_idx(i,j)];
          if (got !== exp) begin
            $display("[TB] FAIL C[%0d][%0d] got=%08h exp=%08h", i, j, got, exp);
            $fatal(1);
          end
        end
      end

      $display("[TB] PASS ✅ final C matched fixed expected");
    end
  endtask

  // ============================================================
  // Main (same flow as your original TB)
  // ============================================================
  initial begin
    rst   = 1'b1;
    start = 1'b0;
    K_len = KLEN;

    init_fixed_expected_c();

    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (2) @(posedge clk);

    // IMPORTANT: load SRAM while idle (busy=0), since shell blocks MMIO when busy=1
    init_fixed_tiles();

    $display("=== TB start: tile_module(epu_sa_top)  M=%0d N=%0d KLEN=%0d ===", M, N, KLEN);

    // start tile (1-cycle pulse)
    @(posedge clk);
    start = 1'b1;
    @(posedge clk);
    start = 1'b0;

    // optional: check busy goes high sometime
    begin : WAIT_BUSY
      int unsigned t;
      t = 0;
      while (busy !== 1'b1) begin
        @(posedge clk);
        t++;
        if (t > 1_000_000) begin
          $display("[TB] WARN busy never asserted (may still be ok depending on driver)");
          disable WAIT_BUSY;
        end
      end
      if (busy) $display("[TB] busy asserted");
    end

    // wait done
    begin : WAIT_DONE
      int unsigned timeout;
      timeout = 0;
      while (done !== 1'b1) begin
        @(posedge clk);
        timeout++;
        if (timeout > 50_000_000) $fatal(1, "[TB] TIMEOUT waiting done");
      end
    end

    // wait until busy low before reading C SRAM
    while (busy === 1'b1) @(posedge clk);

    dump_c_matrix();
    check_final();

    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule

`default_nettype wire
