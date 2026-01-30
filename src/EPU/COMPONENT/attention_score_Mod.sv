// attention_score_top_no_fsm.sv
`include "./src/EPU/attention_score/transpose_crtl.sv"
`include "./src/EPU/attention_score/sramsa.sv"

`timescale 1ns/1ps
`default_nettype none


module attention_score_Mod #(
  parameter int unsigned T      = 8,
  parameter int unsigned DMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 8,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY_C = 1,
  parameter int unsigned TR_M   = 6,

  localparam int unsigned T_W   = (T<=1)?1:$clog2(T),
  localparam int unsigned D_W   = (DMAX<=1)?1:$clog2(DMAX)
)(
  input  logic clk,
  input  logic rst_n,

  // ---------------- manual control ----------------
  input  logic        tr_start,
  output logic        tr_busy,
  output logic        tr_done,

  input  logic        gemm_start,
  output logic        gemm_busy,
  output logic        gemm_done,

  input  logic [15:0] D_len,

  output logic        busy,
  output logic        done,
  output logic        C_valid,

  // ---------------- Q -> W SRAM ----------------
  input  logic                 cpu_q_we,
  input  logic [T_W-1:0]       cpu_q_t,
  input  logic [D_W-1:0]       cpu_q_d,
  input  logic [DATA_W-1:0]    cpu_q_wdata,
  input  logic [BYTE_W-1:0]    cpu_q_wmask,

  // ---------------- K -> transpose A ----------------
  input  logic                 cpu_k_we,
  input  logic [31:0]          cpu_k_t,
  input  logic [31:0]          cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // ---------------- read K^T from transpose B ----------------
  input  logic                 tr_b_re,
  input  logic [31:0]          tr_b_row,
  input  logic [31:0]          tr_b_col,
  output logic [DATA_W-1:0]    tr_b_rdata,
  output logic                 tr_b_rvalid,

  // ---------------- X write (load K^T) ----------------
  input  logic                 cpu_x_we,
  input  logic [D_W-1:0]       cpu_x_k,
  input  logic [T_W-1:0]       cpu_x_n,
  input  logic [DATA_W-1:0]    cpu_x_wdata,
  input  logic [BYTE_W-1:0]    cpu_x_wmask,

  // ---------------- Score read (C SRAM) ----------------
  input  logic                 score_re,
  input  logic [T_W-1:0]       score_tq,
  input  logic [T_W-1:0]       score_tk,
  output logic [DATA_W-1:0]    score_rdata,
  output logic                 score_rvalid
);

  // ============================================================
  // D_len clamp
  // ============================================================
  logic [15:0] D_eff;
  always_comb begin
    if (D_len > DMAX[15:0]) D_eff = DMAX[15:0];
    else                    D_eff = D_len;
  end

  // ============================================================
  // (A) Transpose block: K[t][d] -> A[row=t][col=d]
  // ============================================================
  logic              cpu_a_we;
  logic [31:0]       cpu_a_row, cpu_a_col;
  logic [DATA_W-1:0] cpu_a_wdata;

  assign cpu_a_we    = cpu_k_we;
  assign cpu_a_row   = cpu_k_t;
  assign cpu_a_col   = cpu_k_d;
  assign cpu_a_wdata = cpu_k_wdata;

  top_transpose_cpu #(
    .NRows (T),
    .NCols (DMAX),
    .NB    (NB),
    .ADDR_W(ADDR_W),
    .Data_W(DATA_W),
    .M     (TR_M)
  ) u_tr (
    .clk          (clk),
    .rst_n        (rst_n),

    .start        (tr_start),
    .done         (tr_done),
    .busy         (tr_busy),

    .cpu_a_we     (cpu_a_we),
    .cpu_a_row    (cpu_a_row),
    .cpu_a_col    (cpu_a_col),
    .cpu_a_wdata  (cpu_a_wdata),

    .cpu_b_re     (tr_b_re),
    .cpu_b_row    (tr_b_row),
    .cpu_b_col    (tr_b_col),
    .cpu_b_rdata  (tr_b_rdata),
    .cpu_b_rvalid (tr_b_rvalid)
  );

  // ============================================================
  // (B) GEMM tile system: C[M=T][N=T] = W[M=T][K=D_eff] * X[K=D_eff][N=T]
  // TB 會寫：
  //   W[t][d]  via cpu_q_*
  //   X[d][tk] via cpu_x_*  (n=tk)
  // ============================================================

  // W write mapping
  logic           cpu_w_we;
  logic [T_W-1:0] cpu_w_row;
  logic [D_W-1:0] cpu_w_k;
  logic [DATA_W-1:0] cpu_w_wdata;
  logic [BYTE_W-1:0] cpu_w_wmask;

  assign cpu_w_we    = cpu_q_we;
  assign cpu_w_row   = cpu_q_t;
  assign cpu_w_k     = cpu_q_d;
  assign cpu_w_wdata = cpu_q_wdata;
  assign cpu_w_wmask = cpu_q_wmask;   // <<< 不要反相！跟 TB 規格一致

  // C read mapping (IMPORTANT: keep enable on)
  logic           c_rd_en, c_rd_re;
  logic [T_W-1:0] c_rd_row, c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic           c_rd_rvalid;

  assign c_rd_en  = 1'b1;      // <<< 永遠開，避免 latency 造成 X
  assign c_rd_re  = score_re;  // request pulse
  assign c_rd_row = score_tq;
  assign c_rd_col = score_tk;

  assign score_rdata  = c_rd_rdata;
  assign score_rvalid = c_rd_rvalid;

  // dbg ports (optional)
  logic [T*T*DATA_W-1:0]     c_out_flat_o;
  logic [T*T-1:0]            c_valid_flat_o;
  logic [T*DMAX*DATA_W-1:0]  W_tile_flat_dbg;
  logic [DMAX*T*DATA_W-1:0]  X_tile_flat_dbg;

  logic gemm_C_valid;

  tile_compute_system_top #(
    .M(T),
    .N(T),
    .KMAX(DMAX),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C)
  ) u_gemm (
    .clk(clk),
    .rst(~rst_n),

    .start(gemm_start),
    .K_len(D_eff),

    .busy(gemm_busy),
    .done(gemm_done),

    .cpu_w_we    (cpu_w_we),
    .cpu_w_row   (cpu_w_row),
    .cpu_w_k     (cpu_w_k),
    .cpu_w_wdata (cpu_w_wdata),
    .cpu_w_wmask (cpu_w_wmask),   // <<< 原樣

    .cpu_x_we    (cpu_x_we),
    .cpu_x_k     (cpu_x_k),
    .cpu_x_n     (cpu_x_n),
    .cpu_x_wdata (cpu_x_wdata),
    .cpu_x_wmask (cpu_x_wmask),   // <<< 原樣

    .c_rd_en     (c_rd_en),
    .c_rd_re     (c_rd_re),
    .c_rd_row    (c_rd_row),
    .c_rd_col    (c_rd_col),
    .c_rd_rdata  (c_rd_rdata),
    .c_rd_rvalid (c_rd_rvalid),

    .c_out_flat_o    (c_out_flat_o),
    .c_valid_flat_o  (c_valid_flat_o),
    .C_valid         (gemm_C_valid),

    .W_tile_flat_dbg (W_tile_flat_dbg),
    .X_tile_flat_dbg (X_tile_flat_dbg)
  );

  assign C_valid = gemm_C_valid;

  // ============================================================
  // Top-level status
  // ============================================================
  always_comb begin
    busy = tr_busy | gemm_busy;
    done = gemm_done;
  end

endmodule

`default_nettype wire
