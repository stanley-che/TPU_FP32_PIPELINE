// tb_soc_epu_min.sv
// iverilog -g2012 -Wall -I. -o ./vvp/tb_soc_epu_min.vvp ./test/tb_soc_epu_min.sv
// vvp ./vvp/tb_soc_epu_min.vvp

`include "./src/soc_epu_min_top.sv"


`timescale 1ns/1ps
module tb_soc_epu_min;

  localparam int M=8, N=8, KMAX=1024, KLEN=4;

  // clock/reset
  logic clk=0; always #5 clk=~clk;
  logic rst;

  // AXI-Lite master wires
  logic [31:0] awaddr; logic awvalid; wire awready;
  logic [31:0] wdata;  logic [3:0] wstrb; logic wvalid; wire wready;
  wire  [1:0]  bresp;  wire bvalid; logic bready;

  logic [31:0] araddr; logic arvalid; wire arready;
  wire  [31:0] rdata;  wire [1:0] rresp; wire rvalid; logic rready;

  soc_epu_min_top #(.M(M),.N(N),.KMAX(KMAX)) dut (
    .clk(clk), .rst(rst),
    .m_awaddr(awaddr), .m_awvalid(awvalid), .m_awready(awready),
    .m_wdata(wdata), .m_wstrb(wstrb), .m_wvalid(wvalid), .m_wready(wready),
    .m_bresp(bresp), .m_bvalid(bvalid), .m_bready(bready),
    .m_araddr(araddr), .m_arvalid(arvalid), .m_arready(arready),
    .m_rdata(rdata), .m_rresp(rresp), .m_rvalid(rvalid), .m_rready(rready)
  );

  // ---------------- Fixed tiles & expected (same as你之前) ----------------
  logic [31:0] W_tile_local [M][KMAX];
  logic [31:0] X_tile_local [KMAX][N];
  logic [31:0] C_exp_flat [0:M*N-1];

  function automatic int C_idx(input int i, input int j);
    return i*N + j;
  endfunction

  task automatic init_fixed_tiles;
    int i,j,kk;
    logic [31:0] aval [0:3];
    logic [31:0] bval [0:3];
    begin
      aval[0]=32'h3f800000; aval[1]=32'h40000000; aval[2]=32'h3f000000; aval[3]=32'h40400000;
      bval[0]=32'h3f800000; bval[1]=32'h3f000000; bval[2]=32'h40000000; bval[3]=32'h40400000;

      for (i=0;i<M;i++) for (kk=0;kk<KLEN;kk++) W_tile_local[i][kk]=32'h0;
      for (kk=0;kk<KLEN;kk++) for (j=0;j<N;j++) X_tile_local[kk][j]=32'h0;

      for (kk=0;kk<KLEN;kk++) begin
        for (i=0;i<M;i++) W_tile_local[i][kk]=aval[(kk+i)%4];
        for (j=0;j<N;j++) X_tile_local[kk][j]=bval[(kk+j)%4];
      end
    end
  endtask

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
    end
  endtask

  // ---------------- AXI-Lite BFM (single-beat) ----------------
  task automatic axil_write(input logic [31:0] addr, input logic [31:0] data);
    begin
      @(posedge clk);
      awaddr<=addr; awvalid<=1;
      wdata<=data;  wstrb<=4'hF; wvalid<=1;
      bready<=1;

      // wait ready
      while (!(awready && wready)) @(posedge clk);

      @(posedge clk);
      awvalid<=0; wvalid<=0;

      while (!bvalid) @(posedge clk);
      @(posedge clk);
      bready<=0;
    end
  endtask

  task automatic axil_read(input logic [31:0] addr, output logic [31:0] data);
    begin
      @(posedge clk);
      araddr<=addr; arvalid<=1;
      rready<=1;

      while (!arready) @(posedge clk);
      @(posedge clk);
      arvalid<=0;

      while (!rvalid) @(posedge clk);
      data = rdata;
      @(posedge clk);
      rready<=0;
    end
  endtask

  // base addrs
  localparam logic [31:0] CSR_BASE = 32'h0000_0000;
  localparam logic [31:0] W_BASE   = 32'h0001_0000;
  localparam logic [31:0] X_BASE   = 32'h0002_0000;
  localparam logic [31:0] C_BASE   = 32'h0003_0000;

  function automatic logic [31:0] w_addr(input int i, input int k);
    return W_BASE + 32'( (i*KMAX + k) * 4 );
  endfunction
  function automatic logic [31:0] x_addr(input int k, input int j);
    return X_BASE + 32'( (k*N + j) * 4 );
  endfunction
  function automatic logic [31:0] c_addr(input int i, input int j);
    return C_BASE + 32'( (i*N + j) * 4 );
  endfunction

  task automatic program_tiles_to_sram;
    int i,j,k;
    begin
      for (i=0;i<M;i++)
        for (k=0;k<KLEN;k++)
          axil_write(w_addr(i,k), W_tile_local[i][k]);

      for (k=0;k<KLEN;k++)
        for (j=0;j<N;j++)
          axil_write(x_addr(k,j), X_tile_local[k][j]);
    end
  endtask

  task automatic check_c_from_sram;
    int i,j;
    logic [31:0] got, exp;
    begin
      for (i=0;i<M;i++) begin
        for (j=0;j<N;j++) begin
          axil_read(c_addr(i,j), got);
          exp = C_exp_flat[C_idx(i,j)];
          if (got !== exp) begin
            $display("[TB] FAIL C[%0d][%0d] got=%08h exp=%08h", i, j, got, exp);
            $fatal(1);
          end
        end
      end
      $display("[TB] PASS ✅ C from SRAM_C matched expected");
    end
  endtask

  // ---------------- main ----------------
  initial begin
    // init bus signals
    awaddr=0; awvalid=0; wdata=0; wstrb=0; wvalid=0; bready=0;
    araddr=0; arvalid=0; rready=0;

    rst=1;
    init_fixed_tiles();
    init_fixed_expected_c();

    repeat(5) @(posedge clk);
    rst=0;
    repeat(2) @(posedge clk);

    $display("[TB] program tiles to SRAM_W/X ...");
    program_tiles_to_sram();

    // program klen
    axil_write(CSR_BASE + 32'h04, KLEN);

    // start
    $display("[TB] start EPU ...");
    axil_write(CSR_BASE + 32'h00, 32'h1);

    // poll done
    begin : POLL_DONE
        integer t;
        reg [31:0] st;
        t = 0;
        while (1) begin
            axil_read(CSR_BASE + 32'h08, st);
            if (st[1]) disable POLL_DONE;  // done_latched

            t = t + 1;
            if (t > 2_000_000) $fatal(1, "[TB] TIMEOUT waiting done");
        end
    end



    $display("[TB] done latched, check SRAM_C ...");
    check_c_from_sram();

    $display("[TB] ALL PASS ✅");
    $finish;
  end

endmodule
