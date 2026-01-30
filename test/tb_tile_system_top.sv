/*
iverilog -g2012 -Wall -I./src -o ./vvp/tb_sys.vvp ./test/tb_tile_system_top.sv
vvp ./vvp/tb_sys.vvp
gtkwave ./vvp/tb_sys.vcd

目的：
1) 用 cpu_* ports preload W SRAM 與 X SRAM（模擬 CPU/DRAM 寫 SRAM）
2) pulse start + 設 K_len，controller 會對 k=0..K_len-1 依序觸發 W/X loader
3) 在 loader 讀回 SRAM 的每個 beat 上做 scoreboard 驗證（最穩）
4) 顯示 preload 值 + 顯示 loader 最後寫進 tile 的值（用 shadow array，Icarus 不會 X）
*/

`include "./src/tile_system_top.sv"
`timescale 1ns/1ps
`default_nettype none


module tb_tile_system_top_flat;

  localparam int unsigned M      = 8;
  localparam int unsigned N      = 8;
  localparam int unsigned KMAX   = 1024;
  localparam int unsigned DATA_W = 32;
  localparam int unsigned BYTE_W = DATA_W/8;
  localparam int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX);
  localparam int unsigned ROW_W  = (M<=1)?1:$clog2(M);
  localparam int unsigned N_W    = (N<=1)?1:$clog2(N);

  // test range
  localparam int unsigned K_LO = 0;
  localparam int unsigned K_HI = 9;
  localparam int unsigned K_LEN = (K_HI-K_LO+1);

  // clock/reset
  logic clk = 1'b0;
  always #5 clk = ~clk;
  logic rst;

  // command
  logic start;
  logic [15:0] K_len;
  wire busy, done;

  // cpu writes
  logic                 cpu_w_we;
  logic [ROW_W-1:0]     cpu_w_row;
  logic [K_W-1:0]       cpu_w_k;
  logic [DATA_W-1:0]    cpu_w_wdata;
  logic [BYTE_W-1:0]    cpu_w_wmask;

  logic                 cpu_x_we;
  logic [K_W-1:0]       cpu_x_k;
  logic [N_W-1:0]       cpu_x_n;
  logic [DATA_W-1:0]    cpu_x_wdata;
  logic [BYTE_W-1:0]    cpu_x_wmask;

  wire [M*KMAX*DATA_W-1:0] W_tile_flat;
  wire [KMAX*N*DATA_W-1:0] X_tile_flat;

  tile_system_top #(
    .M(M), .N(N), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W)
  ) dut (
    .clk(clk),
    .rst(rst),

    .start(start),
    .K_len(K_len),
    .busy(busy),
    .done(done),

    .cpu_w_we(cpu_w_we),
    .cpu_w_row(cpu_w_row),
    .cpu_w_k(cpu_w_k),
    .cpu_w_wdata(cpu_w_wdata),
    .cpu_w_wmask(cpu_w_wmask),

    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .W_tile_flat(W_tile_flat),
    .X_tile_flat(X_tile_flat)
  );

  // patterns
  function automatic [DATA_W-1:0] w_val(input int unsigned row, input int unsigned kk);
    w_val = 32'hA000_0000 + (row<<16) + kk;
  endfunction

  function automatic [DATA_W-1:0] x_val(input int unsigned kk, input int unsigned nn);
    x_val = 32'hB000_0000 + (kk<<16) + nn;
  endfunction

  function automatic [DATA_W-1:0] W_flat_at(input int unsigned row, input int unsigned kk);
    int unsigned idx;
    begin
      idx = (row*KMAX + kk)*DATA_W;
      W_flat_at = W_tile_flat[idx +: DATA_W];
    end
  endfunction

  function automatic [DATA_W-1:0] X_flat_at(input int unsigned kk, input int unsigned nn);
    int unsigned idx;
    begin
      idx = (kk*N + nn)*DATA_W;
      X_flat_at = X_tile_flat[idx +: DATA_W];
    end
  endfunction
  function automatic [DATA_W-1:0] X_flat_at_kn(input int unsigned kk, input int unsigned nn);
  int unsigned idx;
  begin
    idx = (kk*N + nn)*DATA_W;         // layout A: k-major, N stride
    X_flat_at_kn = X_tile_flat[idx +: DATA_W];
  end
endfunction

function automatic [DATA_W-1:0] X_flat_at_nk(input int unsigned kk, input int unsigned nn);
  int unsigned idx;
  begin
    idx = (nn*KMAX + kk)*DATA_W;      // layout B: n-major, KMAX stride (常見錯)
    X_flat_at_nk = X_tile_flat[idx +: DATA_W];
  end
endfunction

  // cpu write tasks
  task automatic cpu_write_w(input int unsigned row, input int unsigned kk, input logic [DATA_W-1:0] data);
    begin
      @(negedge clk);
      cpu_w_we    <= 1'b1;
      cpu_w_row   <= row[ROW_W-1:0];
      cpu_w_k     <= kk[K_W-1:0];
      cpu_w_wdata <= data;
      cpu_w_wmask <= {BYTE_W{1'b1}};
      @(negedge clk);
      cpu_w_we    <= 1'b0;
    end
  endtask
    task automatic sanity_check_preload();
    begin
      // 等一拍讓最後一筆 CPU write 進 mem
      repeat(2) @(posedge clk);

      // check a few W locations
      if (dut.w_mem[0][0] !== w_val(0,0)) begin
        $display("[PRELOAD_FAIL][W] w_mem[0][0]=%08x exp=%08x", dut.w_mem[0][0], w_val(0,0));
        $fatal(1);
      end
      if (dut.w_mem[7][9] !== w_val(7,9)) begin
        $display("[PRELOAD_FAIL][W] w_mem[7][9]=%08x exp=%08x", dut.w_mem[7][9], w_val(7,9));
        $fatal(1);
      end

      // check a few X locations
      if (dut.x_mem[0][0] !== x_val(0,0)) begin
        $display("[PRELOAD_FAIL][X] x_mem[0][0]=%08x exp=%08x", dut.x_mem[0][0], x_val(0,0));
        $fatal(1);
      end
      if (dut.x_mem[9][7] !== x_val(9,7)) begin
        $display("[PRELOAD_FAIL][X] x_mem[9][7]=%08x exp=%08x", dut.x_mem[9][7], x_val(9,7));
        $fatal(1);
      end

      $display("[PRELOAD_OK] sample points match.");
    end
  endtask

  task automatic cpu_write_x(input int unsigned kk, input int unsigned nn, input logic [DATA_W-1:0] data);
    begin
      @(negedge clk);
      cpu_x_we    <= 1'b1;
      cpu_x_k     <= kk[K_W-1:0];
      cpu_x_n     <= nn[N_W-1:0];
      cpu_x_wdata <= data;
      cpu_x_wmask <= {BYTE_W{1'b1}};
      @(negedge clk);
      cpu_x_we    <= 1'b0;
    end
  endtask

  task automatic preload_all();
    begin
      for (int r=0; r<M; r++)
        for (int k=K_LO; k<=K_HI; k++)
          cpu_write_w(r,k,w_val(r,k));

      for (int k=K_LO; k<=K_HI; k++)
        for (int n=0; n<N; n++)
          cpu_write_x(k,n,x_val(k,n));
    end
  endtask

  task automatic pulse_start(input int unsigned len);
    begin
      @(negedge clk);
      K_len <= len[15:0];
      start <= 1'b1;
      @(negedge clk);
      start <= 1'b0;
      $display("[CTRL_START] K_len=%0d t=%0t", len, $time);
    end
  endtask

  // scoreboard: count beats by observing internal rvalids
    int unsigned w_beats, x_beats;
  int unsigned errs;

  int unsigned row_tmp, k_tmp, n_tmp;
  logic [DATA_W-1:0] exp_tmp;
  bit w_chk_pending, x_chk_pending;
int unsigned w_row_p, w_k_p, x_k_p, x_n_p;
logic [DATA_W-1:0] w_exp_p, x_exp_p;

always @(posedge clk) begin
  if (rst) begin
    w_chk_pending = 0;
    x_chk_pending = 0;
    w_beats = 0;
    x_beats = 0;
    errs = 0;
  end else begin
    // 先做上拍 pending 的比較（這拍 tile 已經寫好了）
    if (w_chk_pending) begin
      if (W_flat_at(w_row_p, w_k_p) !== w_exp_p) begin
        $display("[FAIL][W] row=%0d k=%0d got=%08x exp=%08x t=%0t",
                 w_row_p, w_k_p, W_flat_at(w_row_p,w_k_p), w_exp_p, $time);
        errs = errs + 1;
      end
      w_chk_pending = 0;
    end
    if (x_chk_pending) begin
      if (X_flat_at(x_k_p, x_n_p) !== x_exp_p) begin
        $display("[FAIL][X] k=%0d n=%0d got=%08x exp=%08x t=%0t",
                 x_k_p, x_n_p, X_flat_at(x_k_p,x_n_p), x_exp_p, $time);
        errs = errs + 1;
      end
      x_chk_pending = 0;
    end

    // 看到 rvalid 就先記下來，下一拍再 check
    if (dut.w_rvalid) begin
      w_row_p = dut.w_row_q;
      w_k_p   = dut.w_k_q;
      w_exp_p = w_val(w_row_p, w_k_p);
      w_chk_pending = 1;
      w_beats = w_beats + 1;
    end
    $display("[XCHK] k=%0d n=%0d exp=%08x  flat_kn=%08x flat_nk=%08x",
         x_k_p, x_n_p, x_exp_p,
         X_flat_at_kn(x_k_p,x_n_p),
         X_flat_at_nk(x_k_p,x_n_p));

    if (dut.x_rvalid) begin
      x_k_p = dut.x_k_q;
      x_n_p = dut.x_n_q;
      x_exp_p = x_val(x_k_p, x_n_p);
      x_chk_pending = 1;
      x_beats = x_beats + 1;
    end
  end
end



  // waves
  initial begin
    $dumpfile("./vvp/tb_sys_flat.vcd");
    $dumpvars(0, tb_tile_system_top_flat);
  end

  initial begin
    // init
    start = 1'b0; K_len = '0;

    cpu_w_we='0; cpu_w_row='0; cpu_w_k='0; cpu_w_wdata='0; cpu_w_wmask='0;
    cpu_x_we='0; cpu_x_k='0; cpu_x_n='0; cpu_x_wdata='0; cpu_x_wmask='0;

    rst = 1'b1;
    repeat(5) @(posedge clk);
    rst = 1'b0;
    repeat(2) @(posedge clk);

    preload_all();
    sanity_check_preload();
    repeat(5) @(posedge clk);

    pulse_start(K_LEN);

    wait(done===1'b1);
    repeat(5) @(posedge clk);

    if (w_beats != (K_LEN*M)) begin
      $display("[FAIL] W beats got=%0d exp=%0d", w_beats, (K_LEN*M));
      $fatal(1);
    end
    if (x_beats != (K_LEN*N)) begin
      $display("[FAIL] X beats got=%0d exp=%0d", x_beats, (K_LEN*N));
      $fatal(1);
    end
    if (errs != 0) begin
      $display("[FAIL] errs=%0d", errs);
      $fatal(1);
    end

    $display("\n[PASS] tile_system_top_flat OK. W_beats=%0d X_beats=%0d", w_beats, x_beats);
    $finish;
  end

endmodule

`default_nettype wire
