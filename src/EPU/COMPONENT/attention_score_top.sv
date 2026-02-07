// attention_score_top.sv
`include "./src/EPU/attention_score/QKT_FSM.sv"
`include "./src/EPU/attention_score/attention_score_Mod.sv"

`timescale 1ns/1ps
`default_nettype none

module attention_score_top_with_fsm #(
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

  // ---------- FSM control ----------
  input  logic        fsm_start,
  input  logic [15:0] D_len,
  output logic        fsm_busy,
  output logic        fsm_done,

  // ---------- CPU write Q ----------
  input  logic                 cpu_q_we,
  input  logic [T_W-1:0]       cpu_q_t,
  input  logic [D_W-1:0]       cpu_q_d,
  input  logic [DATA_W-1:0]    cpu_q_wdata,
  input  logic [BYTE_W-1:0]    cpu_q_wmask,

  // ---------- CPU write K (transpose A) ----------
  input  logic                 cpu_k_we,
  input  logic [31:0]          cpu_k_t,
  input  logic [31:0]          cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // ---------- CPU read Score ----------
  input  logic                 score_re,
  input  logic [T_W-1:0]       score_tq,
  input  logic [T_W-1:0]       score_tk,
  output logic [DATA_W-1:0]    score_rdata,
  output logic                 score_rvalid
);

  // ----------------------------
  // Wires between FSM <-> attention_score_Mod
  // ----------------------------
  logic        tr_start;
  logic        tr_busy, tr_done;

  logic        gemm_start;
  logic        gemm_busy, gemm_done;

  logic        top_busy, top_done;
  logic        C_valid;

  logic                 tr_b_re;
  logic [31:0]          tr_b_row;
  logic [31:0]          tr_b_col;
  logic [DATA_W-1:0]    tr_b_rdata;
  logic                 tr_b_rvalid;

  logic                 cpu_x_we;
  logic [D_W-1:0]       cpu_x_k;
  logic [T_W-1:0]       cpu_x_n;
  logic [DATA_W-1:0]    cpu_x_wdata;
  logic [BYTE_W-1:0]    cpu_x_wmask;

  // ----------------------------
  // Instantiate original attention_score_Mod (manual ports)
  // (FSM drives tr_start/gemm_start/tr_b_*/cpu_x_*)
  // ----------------------------
  attention_score_Mod #(
    .T(T),
    .DMAX(DMAX),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .NB(NB),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) dut (
    .clk(clk),
    .rst_n(rst_n),

    // manual control (now driven by FSM)
    .tr_start(tr_start),
    .tr_busy(tr_busy),
    .tr_done(tr_done),

    .gemm_start(gemm_start),
    .gemm_busy(gemm_busy),
    .gemm_done(gemm_done),

    .D_len(D_len),

    .busy(top_busy),
    .done(top_done),
    .C_valid(C_valid),

    // Q -> W SRAM (CPU writes)
    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    // K -> transpose A (CPU writes)
    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    // read K^T from transpose B (FSM reads)
    .tr_b_re(tr_b_re),
    .tr_b_row(tr_b_row),
    .tr_b_col(tr_b_col),
    .tr_b_rdata(tr_b_rdata),
    .tr_b_rvalid(tr_b_rvalid),

    // X write (FSM loads X)
    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    // Score read (CPU reads)
    .score_re(score_re),
    .score_tq(score_tq),
    .score_tk(score_tk),
    .score_rdata(score_rdata),
    .score_rvalid(score_rvalid)
  );

  // ----------------------------
  // QKT FSM
  // ----------------------------
  QKT_FSM #(
    .T(T),
    .DMAX(DMAX),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W)
  ) u_fsm (
    .clk(clk),
    .rst_n(rst_n),

    .start(fsm_start),
    .D_len(D_len),

    .busy(fsm_busy),
    .done(fsm_done),

    .tr_start(tr_start),
    .tr_busy(tr_busy),
    .tr_done(tr_done),

    .tr_b_re(tr_b_re),
    .tr_b_row(tr_b_row),
    .tr_b_col(tr_b_col),
    .tr_b_rdata(tr_b_rdata),
    .tr_b_rvalid(tr_b_rvalid),

    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .gemm_start(gemm_start),
    .gemm_busy(gemm_busy),
    .gemm_done(gemm_done),
    .C_valid(C_valid)
  );

endmodule

`default_nettype wire
