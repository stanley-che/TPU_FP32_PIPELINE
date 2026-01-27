`include "./src/epu_bringup_top.sv"
`timescale 1ns/1ps
/*
iverilog -g2012 -Wall -I./src -I./test -o ./vvp/tb_epu.vvp ./test/tb_epu_bringup_top.sv 


vvp ./vvp/tb_epu.vvp
*/
module tb_epu_bringup_top;

  localparam int M=8;
  localparam int N=8;
  localparam int KMAX=1024;
  localparam int MEM_AW=10;
  localparam int AXI_AW=32;

  // Address map (must match your top)
  localparam logic [31:0] CSR_BASE  = 32'h4000_0000;
  localparam logic [31:0] CWIN_BASE = 32'h4000_1000;
  localparam logic [31:0] W_BASE    = 32'h5000_0000;
  localparam logic [31:0] X_BASE    = 32'h5000_8000;

  // clock/reset
  logic clk=0; always #5 clk=~clk;
  logic rst;

  // AXI-Lite signals
  logic [AXI_AW-1:0] s_axi_awaddr;
  logic              s_axi_awvalid;
  wire               s_axi_awready;

  logic [31:0]        s_axi_wdata;
  logic [3:0]         s_axi_wstrb;
  logic               s_axi_wvalid;
  wire                s_axi_wready;

  wire [1:0]          s_axi_bresp;
  wire                s_axi_bvalid;
  logic               s_axi_bready;

  logic [AXI_AW-1:0]  s_axi_araddr;
  logic               s_axi_arvalid;
  wire                s_axi_arready;

  wire [31:0]         s_axi_rdata;
  wire [1:0]          s_axi_rresp;
  wire                s_axi_rvalid;
  logic               s_axi_rready;

  // DUT
  epu_bringup_top #(
    .M(M), .N(N), .KMAX(KMAX), .MEM_AW(MEM_AW), .AXI_AW(AXI_AW)
  ) dut (
    .clk(clk),
    .rst(rst),

    .s_axi_awaddr(s_axi_awaddr),
    .s_axi_awvalid(s_axi_awvalid),
    .s_axi_awready(s_axi_awready),

    .s_axi_wdata(s_axi_wdata),
    .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wvalid(s_axi_wvalid),
    .s_axi_wready(s_axi_wready),

    .s_axi_bresp(s_axi_bresp),
    .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),

    .s_axi_araddr(s_axi_araddr),
    .s_axi_arvalid(s_axi_arvalid),
    .s_axi_arready(s_axi_arready),

    .s_axi_rdata(s_axi_rdata),
    .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid),
    .s_axi_rready(s_axi_rready)
  );

  // ---------------- AXI-Lite master tasks ----------------
  task automatic axil_write32(input logic [31:0] addr, input logic [31:0] data);
    begin
      // AW
      s_axi_awaddr  <= addr;
      s_axi_awvalid <= 1'b1;

      // W
      s_axi_wdata   <= data;
      s_axi_wstrb   <= 4'hF;
      s_axi_wvalid  <= 1'b1;

      // B
      s_axi_bready  <= 1'b1;

      // wait ready (both)
      do @(posedge clk); while (!(s_axi_awready && s_axi_wready));

      // drop valids next cycle
      @(posedge clk);
      s_axi_awvalid <= 1'b0;
      s_axi_wvalid  <= 1'b0;

      // wait bvalid
      do @(posedge clk); while (!s_axi_bvalid);

      // handshake B
      @(posedge clk);
      s_axi_bready <= 1'b0;
    end
  endtask

  task automatic axil_read32(input logic [31:0] addr, output logic [31:0] data);
    begin
      s_axi_araddr  <= addr;
      s_axi_arvalid <= 1'b1;
      s_axi_rready  <= 1'b1;

      do @(posedge clk); while (!s_axi_arready);

      @(posedge clk);
      s_axi_arvalid <= 1'b0;

      do @(posedge clk); while (!s_axi_rvalid);

      data = s_axi_rdata;

      @(posedge clk);
      s_axi_rready <= 1'b0;
    end
  endtask

  // ---------------- helpers for bank addressing ----------------
  function automatic logic [31:0] w_addr(input int i, input int k);
    w_addr = W_BASE + (i * 32'h1000) + (k * 4);
  endfunction

  function automatic logic [31:0] x_addr(input int j, input int k);
    x_addr = X_BASE + (j * 32'h1000) + (k * 4);
  endfunction

  function automatic logic [31:0] c_addr(input int i, input int j);
    c_addr = CWIN_BASE + 4 * (i*N + j);
  endfunction

  // ---------------- main test ----------------
  int K;
  logic [31:0] rd;
  int errors;

  initial begin
    // init
    s_axi_awaddr  = '0; s_axi_awvalid = 0;
    s_axi_wdata   = '0; s_axi_wstrb = 4'h0; s_axi_wvalid = 0;
    s_axi_bready  = 0;
    s_axi_araddr  = '0; s_axi_arvalid = 0;
    s_axi_rready  = 0;

    rst = 1;
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);

    // choose K_len (<=1024)
    K = 16;

    $display("---- Fill W/X banks with 1s, K=%0d ----", K);

    // Fill W banks: W[i][k]=1
    for (int i=0;i<M;i++) begin
      for (int k=0;k<K;k++) begin
        axil_write32(w_addr(i,k), 32'd1);
      end
    end

    // Fill X banks: X[k][j]=1 (stored in bank j at addr k)
    for (int j=0;j<N;j++) begin
      for (int k=0;k<K;k++) begin
        axil_write32(x_addr(j,k), 32'd1);
      end
    end

    // Program K_len
    axil_write32(CSR_BASE + 32'h008, K);

    // Start go (CTRL.go=1)
    axil_write32(CSR_BASE + 32'h000, 32'h1);

    // Poll STATUS until done (bit2)
    $display("---- Poll done ----");
    do begin
      axil_read32(CSR_BASE + 32'h004, rd);
    end while (rd[2] == 1'b0);

    $display("DONE! STATUS=%h", rd);

    // Read C and check: expect C[i][j] == K (since sum_{k=0..K-1} 1*1 = K)
    errors = 0;
    for (int i=0;i<M;i++) begin
      for (int j=0;j<N;j++) begin
        axil_read32(c_addr(i,j), rd);
        if (rd !== logic'(K)) begin
          $display("Mismatch C[%0d][%0d] = %0d (0x%08x), expect %0d",
                    i,j, rd, rd, K);
          errors++;
        end
      end
    end

    if (errors==0) $display("PASS: all C matched K=%0d", K);
    else           $display("FAIL: %0d mismatches", errors);

    $finish;
  end

endmodule
