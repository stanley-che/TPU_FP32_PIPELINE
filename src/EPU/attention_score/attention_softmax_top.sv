`ifndef ATTENTIOM_SOFTMAX_TOP_SV
`define ATTENTIOM_SOFTMAX_TOP_SV
// ============================================================================
// attention_softmax_top.sv  (FULL SOFTMAX)
// 功能：
//  A) 跑 pre-softmax: E = exp(x - rowmax), Sum[row], InvSum[row]
//     (using attention_presoft_submax_exp_sum_top_inv)
//  B) Normalize: P[row][k] = E[row][k] * InvSum[row]
//  C) 提供 P 讀取 (1-cycle latency)
// ============================================================================

`include "./src/EPU/attention_score/attention_presoft_submax_exp_sum_top_inv.sv"
`include "./src/EPU/attention_score/fp_mul_driver.sv" 

`timescale 1ns/1ps
`default_nettype none

module attention_softmax_top #(
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

  input  logic        start,
  input  logic [15:0] D_len,
  output logic        busy,
  output logic        done,

  input  logic [T-1:0] pad_valid,
  input  logic         causal_en,

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

  // ---------------- Optional debug: read E ----------------
  input  logic              e_re,
  input  logic [ROW_W-1:0]  e_tq,
  input  logic [COL_W-1:0]  e_tk,
  output logic [DATA_W-1:0] e_rdata,
  output logic              e_rvalid,

  // ---------------- Softmax output P read -----------------
  input  logic              p_re,
  input  logic [ROW_W-1:0]  p_tq,
  input  logic [COL_W-1:0]  p_tk,
  output logic [DATA_W-1:0] p_rdata,
  output logic              p_rvalid
);

  // ==========================================================================
  // 1) Pre-softmax: E, Sum, InvSum
  // ==========================================================================
  logic pre_busy, pre_done;

  // sum / invsum read ports (internal for normalize FSM)
  logic              sum_re_i;
  logic [ROW_W-1:0]  sum_row_i;
  logic [DATA_W-1:0] sum_rdata_i;
  logic              sum_rvalid_i;

  logic              invsum_re_i;
  logic [ROW_W-1:0]  invsum_row_i;
  logic [DATA_W-1:0] invsum_rdata_i;
  logic              invsum_rvalid_i;

  attention_presoft_submax_exp_sum_top_inv #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) u_pre (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .D_len(D_len),
    .busy(pre_busy),
    .done(pre_done),

    .pad_valid(pad_valid),
    .causal_en(causal_en),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    // debug E passthrough (external)
    .e_re(e_re_mux),
    .e_tq(e_tq_mux),
    .e_tk(e_tk_mux),
    .e_rdata(e_rdata),
    .e_rvalid(e_rvalid),


    // sum port (internal for normalize)
    .sum_re(sum_re_i),
    .sum_row(sum_row_i),
    .sum_rdata(sum_rdata_i),
    .sum_rvalid(sum_rvalid_i),

    // invsum port (internal for normalize)
    .invsum_re(invsum_re_i),
    .invsum_row(invsum_row_i),
    .invsum_rdata(invsum_rdata_i),
    .invsum_rvalid(invsum_rvalid_i)
  );

  // ==========================================================================
  // 2) P memory (T x T)
  // ==========================================================================
  logic [31:0] P_mem [0:T-1][0:T-1];

  // P read 1-cycle
  logic p_rd_pending;
  logic [ROW_W-1:0] p_tq_q;
  logic [COL_W-1:0] p_tk_q;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      p_rd_pending <= 1'b0;
      p_tq_q       <= '0;
      p_tk_q       <= '0;
      p_rdata      <= 32'h0;
      p_rvalid     <= 1'b0;
    end else begin
      p_rd_pending <= p_re;
      if (p_re) begin
        p_tq_q <= p_tq;
        p_tk_q <= p_tk;
      end
      p_rvalid <= p_rd_pending;
      if (p_rd_pending) p_rdata <= P_mem[p_tq_q][p_tk_q];
    end
  end

  // ==========================================================================
  // 3) FP MUL: P = E * InvSum[row]
  // ==========================================================================
  logic        mul_start;
  logic        mul_done;
  logic        mul_busy;
  logic [31:0] mul_a, mul_b, mul_z;

  fp_mul_driver u_mul (
    .clk    (clk),
    .rst    (~rst_n),
    .start  (mul_start),
    .a_bits (mul_a),
    .b_bits (mul_b),
    .busy   (mul_busy),
    .done   (mul_done),
    .z_bits (mul_z)
  );

  // ==========================================================================
  // 4) Normalize FSM
  //    After pre_done:
  //      for row:
  //        inv = InvSum[row]
  //        for col:
  //          read E[row][col] (use u_pre's E external port!)
  //          mul -> write P_mem[row][col]
  // ==========================================================================
  typedef enum logic [3:0] {
    N_IDLE,
    N_WAIT_PRE,
    N_RD_INVSUM,
    N_WAIT_INVSUM,
    N_ISSUE_E,
    N_WAIT_E,
    N_MUL,
    N_WAIT_MUL,
    N_WRITE_P,
    N_DONE
  } nst_t;

  nst_t nst;

  logic [ROW_W-1:0] nr;
  logic [COL_W-1:0] nc;

  logic [31:0] inv_hold;
  logic [31:0] e_hold;

  // use u_pre read ports internally (don’t touch external debug e_re)
  // We'll drive u_pre.invsum_re + u_pre.e_re through INTERNAL signals:
  // BUT: u_pre already uses external e_re port for debug. E easiest solution:
  //   - reuse external e_re inputs? not allowed.
  // So: in u_pre you should keep debug e_re as-is, and for normalize read E,
  // we add a SECOND E read port in u_pre. If you don't want to change u_pre,
  // then normalize must be done inside u_pre.
  //
  // ---- Practical approach (no extra port change):
  // We'll *not* use external e_re at all during normalize, and instead drive it
  // from this top-level by muxing between user's debug and normalize.
  //
  logic e_re_norm;
  logic [ROW_W-1:0] e_tq_norm;
  logic [COL_W-1:0] e_tk_norm;

  // Mux debug E read vs normalize E read
  logic e_re_mux;
  logic [ROW_W-1:0] e_tq_mux;
  logic [COL_W-1:0] e_tk_mux;

 logic norm_active;

always_ff @(posedge clk) begin
  if (!rst_n) norm_active <= 1'b0;
  else if (nst == N_WAIT_PRE && pre_done) norm_active <= 1'b1; // 進入normalize
  else if (nst == N_DONE && !start)       norm_active <= 1'b0; // 回IDLE後關掉
end

always_comb begin
  e_re_mux = e_re;
  e_tq_mux = e_tq;
  e_tk_mux = e_tk;

  if (norm_active) begin
    e_re_mux = e_re_norm;
    e_tq_mux = e_tq_norm;
    e_tk_mux = e_tk_norm;
  end
end


  // re-connect muxed E read into u_pre by wiring override:
  // Because u_pre ports are already connected to e_re/e_tq/e_tk,
  // you must connect u_pre to these muxed signals instead of raw ports.
  // -> Replace u_pre connections above:
  //    .e_re(e_re_mux), .e_tq(e_tq_mux), .e_tk(e_tk_mux)
  //
  // !! IMPORTANT: You must do that replacement in your file.

  // For invsum read, we only use internal port (no user port here)
  assign sum_re_i     = 1'b0;       // not used in normalize now
  assign sum_row_i    = '0;

  // invsum read driven by FSM
  assign invsum_re_i  = (nst == N_RD_INVSUM) || (nst == N_WAIT_INVSUM);
  assign invsum_row_i = nr;

  // E read driven by FSM (1-cycle pulse)
  assign e_re_norm = (nst == N_ISSUE_E) || (nst == N_WAIT_E);

  assign e_tq_norm = nr;
  assign e_tk_norm = nc;

  // MUL controls (1-cycle pulse)
  assign mul_start = (nst == N_MUL);
  assign mul_a     = e_hold;
  assign mul_b     = inv_hold;

  // busy/done
  assign busy = pre_busy || ((nst != N_IDLE) && (nst != N_DONE));
  assign done = (nst == N_DONE);

  // Sequential normalize FSM
  integer rr, cc;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      nst      <= N_IDLE;
      nr       <= '0;
      nc       <= '0;
      inv_hold <= 32'h0;
      e_hold   <= 32'h0;
      for (rr = 0; rr < T; rr = rr + 1)
        for (cc = 0; cc < T; cc = cc + 1)
          P_mem[rr][cc] <= 32'h0;
    end else begin
      case (nst)
        N_IDLE: begin
          if (start) nst <= N_WAIT_PRE;
        end

        N_WAIT_PRE: begin
          if (pre_done) begin
            nr  <= '0;
            nc  <= '0;
            nst <= N_RD_INVSUM;
          end
        end

        N_RD_INVSUM: begin
          nst <= N_WAIT_INVSUM;
        end

        N_WAIT_INVSUM: begin
          if (invsum_rvalid_i) begin
            inv_hold <= invsum_rdata_i;
            nst      <= N_ISSUE_E;
          end
        end

        N_ISSUE_E: begin
          nst <= N_WAIT_E;
        end

        N_WAIT_E: begin
          if (e_rvalid) begin
            e_hold <= e_rdata;
            nst    <= N_MUL;
          end
        end

        N_MUL: begin
          nst <= N_WAIT_MUL;
        end

        N_WAIT_MUL: begin
          if (mul_done) begin
            nst <= N_WRITE_P;
          end
        end

        N_WRITE_P: begin
          P_mem[nr][nc] <= mul_z;

          if (nc == COL_W'(T-1)) begin
            if (nr == ROW_W'(T-1)) begin
              nst <= N_DONE;
            end else begin
              nr  <= nr + 1'b1;
              nc  <= '0;
              nst <= N_RD_INVSUM;
            end
          end else begin
            nc  <= nc + 1'b1;
            nst <= N_ISSUE_E;
          end
        end

        N_DONE: begin
          if (!start) nst <= N_IDLE;
        end

        default: nst <= N_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
`endif 