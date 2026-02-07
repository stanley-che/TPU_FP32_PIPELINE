`include "./src/EPU/attention_score/attention_score_top.sv"
`include "./src/EPU/attention_score/attention_scaling_tile.sv"
`timescale 1ns/1ps
`default_nettype none
module attention_score_top_qkt_scale #(
  parameter int unsigned T      = 8,
  parameter int unsigned DMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 8,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY_C = 1,
  parameter int unsigned TR_M   = 6,

  localparam int unsigned T_W   = (T<=1)?1:$clog2(T),
  localparam int unsigned D_W   = (DMAX<=1)?1:$clog2(DMAX),
  localparam int unsigned ROW_W = T_W,
  localparam int unsigned COL_W = T_W
)(
  input  logic clk,
  input  logic rst_n,

  input  logic        start,     // 一個 start 跑完 QKT + scaling
  input  logic [15:0] D_len,
  output logic        busy,
  output logic        done,

  // CPU write Q
  input  logic                 cpu_q_we,
  input  logic [T_W-1:0]       cpu_q_t,
  input  logic [D_W-1:0]       cpu_q_d,
  input  logic [DATA_W-1:0]    cpu_q_wdata,
  input  logic [BYTE_W-1:0]    cpu_q_wmask,

  // CPU write K
  input  logic                 cpu_k_we,
  input  logic [31:0]          cpu_k_t,
  input  logic [31:0]          cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // CPU read SCALED score (from scaling tile C SRAM)
  input  logic                 sc_re,
  input  logic [ROW_W-1:0]     sc_tq,
  input  logic [COL_W-1:0]     sc_tk,
  output logic [DATA_W-1:0]    sc_rdata,
  output logic                 sc_rvalid

  // (可選) 如果你還要保留 raw score debug，再另外拉一組 raw score read port
);

  // -------------------------
  // stage1: QKT top (raw score producer)
  // -------------------------
  logic fsm_start, fsm_busy, fsm_done;

  // score port (shared)
  logic                 score_re_mux;
  logic [T_W-1:0]       score_tq_mux, score_tk_mux;
  logic [DATA_W-1:0]    score_rdata;
  logic                 score_rvalid;

  attention_score_top_with_fsm #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) u_qkt (
    .clk(clk),
    .rst_n(rst_n),

    .fsm_start(fsm_start),
    .D_len(D_len),
    .fsm_busy(fsm_busy),
    .fsm_done(fsm_done),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .score_re(score_re_mux),
    .score_tq(score_tq_mux),
    .score_tk(score_tk_mux),
    .score_rdata(score_rdata),
    .score_rvalid(score_rvalid)
  );

  // -------------------------
  // stage2: scaling tile
  // -------------------------
  logic scale_start, scale_busy, scale_done;
  logic scale_in_score_re;
  logic [ROW_W-1:0] scale_in_score_tq;
  logic [COL_W-1:0] scale_in_score_tk;
  logic scale_C_valid;

  attention_scaling_tile #(
    .T(T),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W)
  ) u_scale (
    .clk(clk),
    .rst(~rst_n), // 你的 scaling 用 rst 高有效，所以反相
    .start(scale_start),
    .D_len(D_len),
    .busy(scale_busy),
    .done(scale_done),

    .in_score_re(scale_in_score_re),
    .in_score_tq(scale_in_score_tq),
    .in_score_tk(scale_in_score_tk),
    .in_score_rdata(score_rdata),
    .in_score_rvalid(score_rvalid),

    .c_rd_en(1'b1),          // 讓 CPU 讀時打開（你也可以用 sc_re 控制）
    .c_rd_re(sc_re),
    .c_rd_row(sc_tq),
    .c_rd_col(sc_tk),
    .c_rd_rdata(sc_rdata),
    .c_rd_rvalid(sc_rvalid),

    .C_valid(scale_C_valid)
  );

  // -------------------------
  // score read port arbitration (scaling優先)
  // -------------------------
  // 這裡先假設 CPU 不讀 raw score，所以 cpu_score_re=0
  // 若你要 CPU 同時支援 raw score debug，再加 cpu_score_* port 進來 mux
  assign score_re_mux = scale_in_score_re;
  assign score_tq_mux = scale_in_score_tq;
  assign score_tk_mux = scale_in_score_tk;

  // -------------------------
  // top FSM: run QKT then scaling
  // -------------------------
  typedef enum logic [1:0] {S_IDLE, S_QKT, S_SCALE, S_DONE} st_t;
  st_t st;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE;
      fsm_start <= 1'b0;
      scale_start <= 1'b0;
      done <= 1'b0;
      busy <= 1'b0;
    end else begin
      fsm_start <= 1'b0;
      scale_start <= 1'b0;
      done <= 1'b0;

      case (st)
        S_IDLE: begin
          busy <= 1'b0;
          if (start) begin
            fsm_start <= 1'b1;   // pulse
            busy <= 1'b1;
            st <= S_QKT;
          end
        end

        S_QKT: begin
          busy <= 1'b1;
          if (fsm_done) begin
            scale_start <= 1'b1; // pulse
            st <= S_SCALE;
          end
        end

        S_SCALE: begin
          busy <= 1'b1;
          if (scale_C_valid) begin
            st <= S_DONE;
          end
        end

        S_DONE: begin
          busy <= 1'b0;
          done <= 1'b1;
          if (!start) st <= S_IDLE;
        end
      endcase
    end
  end

endmodule
`default_nettype wire
