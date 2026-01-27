  // tile_system_top_flat.sv
  `include "./src/EPU/MAC/tile_load_controller.sv"
  `include "./src/EPU/MAC/wtile_loader_top.sv"
  `include "./src/EPU/MAC/xtile_loader_top.sv"
  `timescale 1ns/1ps
  `default_nettype none


module tile_system_top#(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = DATA_W/8,

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // command
  input  logic        start,
  input  logic [15:0] K_len,
  output logic        busy,
  output logic        done,

  // CPU write W SRAM
  input  logic                 cpu_w_we,
  input  logic [ROW_W-1:0]     cpu_w_row,
  input  logic [K_W-1:0]       cpu_w_k,
  input  logic [DATA_W-1:0]    cpu_w_wdata,
  input  logic [BYTE_W-1:0]    cpu_w_wmask,

  // CPU write X SRAM
  input  logic                 cpu_x_we,
  input  logic [K_W-1:0]       cpu_x_k,
  input  logic [N_W-1:0]       cpu_x_n,
  input  logic [DATA_W-1:0]    cpu_x_wdata,
  input  logic [BYTE_W-1:0]    cpu_x_wmask,

  // tiles flat
  output wire [M*KMAX*DATA_W-1:0] W_tile_flat,
  output wire [KMAX*N*DATA_W-1:0] X_tile_flat
);

  // ---------------- Controller <-> loaders ----------------
  logic           w_start_k, w_col_valid, w_col_accept;
  logic [K_W-1:0] w_k_idx;

  logic           x_start_k, x_row_valid, x_row_accept;
  logic [K_W-1:0] x_k_idx;

  tile_load_controller #(
    .KMAX(KMAX), .K_W(K_W)
  ) u_ctrl (
    .clk(clk),
    .rst(rst),

    .start(start),
    .K_len(K_len),
    .busy(busy),
    .done(done),

    .w_start_k(w_start_k),
    .w_k_idx(w_k_idx),
    .w_col_valid(w_col_valid),
    .w_col_accept(w_col_accept),

    .x_start_k(x_start_k),
    .x_k_idx(x_k_idx),
    .x_row_valid(x_row_valid),
    .x_row_accept(x_row_accept)
  );

  // ---------------- W loader wires ----------------
  logic                 w_en, w_re, w_we;
  logic [ROW_W-1:0]     w_row;
  logic [K_W-1:0]       w_k;
  logic [DATA_W-1:0]    w_wdata;
  logic [BYTE_W-1:0]    w_wmask;
  logic [DATA_W-1:0]    w_rdata;
  logic                 w_rvalid;

  // ---------------- X loader wires ----------------
  logic                 x_en, x_re, x_we;
  logic [K_W-1:0]       x_k;
  logic [N_W-1:0]       x_n;
  logic [DATA_W-1:0]    x_wdata;
  logic [BYTE_W-1:0]    x_wmask;
  logic [DATA_W-1:0]    x_rdata;
  logic                 x_rvalid;
    // W memory: [M][KMAX]
  logic [DATA_W-1:0] w_mem [0:M-1][0:KMAX-1];

  // X memory: [KMAX][N]
  logic [DATA_W-1:0] x_mem [0:KMAX-1][0:N-1];
 
  // ---------------- W loader instance (flat out) ----------------
  w_sram_to_Wtile_col #(
    .M(M), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .K_W(K_W)
  ) u_w_loader (
    .clk(clk),
    .rst(rst),

    .start_k(w_start_k),
    .k_idx(w_k_idx),
    .col_valid(w_col_valid),
    .col_accept(w_col_accept),

    .w_en(w_en),
    .w_re(w_re),
    .w_we(w_we),
    .w_row(w_row),
    .w_k(w_k),
    .w_wdata(w_wdata),
    .w_wmask(w_wmask),
    .w_rdata(w_rdata),
    .w_rvalid(w_rvalid),

    .W_tile_flat(W_tile_flat)
  );

  // ---------------- X loader instance (flat out) ----------------
  x_sram_to_Xtile_row #(
    .N(N), .KMAX(KMAX), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .N_W(N_W), .K_W(K_W)
  ) u_x_loader (
    .clk(clk),
    .rst(rst),

    .start_k(x_start_k),
    .k_idx(x_k_idx),
    .row_valid(x_row_valid),
    .row_accept(x_row_accept),

    .x_en(x_en),
    .x_re(x_re),
    .x_we(x_we),
    .x_k(x_k),
    .x_n(x_n),
    .x_wdata(x_wdata),
    .x_wmask(x_wmask),
    .x_rdata(x_rdata),
    .x_rvalid(x_rvalid),

    .X_tile_flat(X_tile_flat)
  );

  // ============================================================
  // Simple SRAM models (1-cycle read latency)
  //  - W SRAM indexed by (row,k)
  //  - X SRAM indexed by (k,n)
  //  CPU write has priority; loader reads use x_en/x_re, w_en/w_re.
  // ============================================================
    // --- W SRAM 1-cycle read latency (correct)
  logic                 w_req_q;
  logic [ROW_W-1:0]     w_row_q;
  logic [K_W-1:0]       w_k_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      w_req_q  <= 1'b0;
      w_row_q  <= '0;
      w_k_q    <= '0;
      w_rvalid <= 1'b0;
      w_rdata  <= '0;
    end else begin
      // CPU write
      if (cpu_w_we) begin
        logic [DATA_W-1:0] tmp;
        tmp = w_mem[cpu_w_row][cpu_w_k];
        for (int b=0; b<BYTE_W; b++) begin
          if (cpu_w_wmask[b]) tmp[8*b +: 8] = cpu_w_wdata[8*b +: 8];
        end
        w_mem[cpu_w_row][cpu_w_k] <= tmp;
      end


      // stage-0 capture request address
      w_req_q <= (w_en && w_re && !cpu_w_we);
      w_row_q <= w_row;
      w_k_q   <= w_k;

      // stage-1 response (uses previous captured addr)
      w_rvalid <= w_req_q;
      w_rdata  <= (w_req_q) ? w_mem[w_row_q][w_k_q] : '0;
    end
  end
  
    logic              x_req_q;
  logic [K_W-1:0]    x_k_q;
  logic [N_W-1:0]    x_n_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      x_req_q  <= 1'b0;
      x_k_q    <= '0;
      x_n_q    <= '0;
      x_rvalid <= 1'b0;
      x_rdata  <= '0;
    end else begin
      if (cpu_x_we) begin
        logic [DATA_W-1:0] tmp;
        tmp = x_mem[cpu_x_k][cpu_x_n];
        for (int b=0; b<BYTE_W; b++) begin
          if (cpu_x_wmask[b]) tmp[8*b +: 8] = cpu_x_wdata[8*b +: 8];
        end
        x_mem[cpu_x_k][cpu_x_n] <= tmp;
      end


      x_req_q <= (x_en && x_re && !cpu_x_we);
      x_k_q   <= x_k;
      x_n_q   <= x_n;

      x_rvalid <= x_req_q;
      x_rdata  <= (x_req_q) ? x_mem[x_k_q][x_n_q] : '0;
    end
  end

endmodule

`default_nettype wire
