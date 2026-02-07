// ============================================================================
// attention_presoft_submax_exp_top.sv
// Combines:
//   - attention_presoft_submax_top : produces Y[tq][tk] = score - rowmax
//   - fp32_exp_no_lut              : computes exp(Y) and stores into E_mem
//
// Flow:
//   start -> run submax core -> core_done(level)
//   then sweep Y[tq][tk] -> (mask) -> exp -> store E[tq][tk]
//   provides E read port (1-cycle latency)
//
// Masking:
//   if pad_valid[tq]==0 or pad_valid[tk]==0 => E=0
//   if causal_en && (tk > tq)               => E=0
// ============================================================================

`include "./src/EPU/attention_score/submax_core_top.sv"
`include "./src/EPU/attention_score/fp32_exp_matrix_2d.sv"

`timescale 1ns/1ps
`default_nettype none


module attention_presoft_submax_exp_top #(
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
  localparam int unsigned COL_W = T_W,
  localparam int unsigned N     = T*T,
  localparam int unsigned IDX_W = (N<=1)?1:$clog2(N)
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
  input  logic [T_W-1:0]       cpu_k_t,
  input  logic [D_W-1:0]       cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // read exp result E[tq][tk]
  input  logic              e_re,
  input  logic [ROW_W-1:0]  e_tq,
  input  logic [COL_W-1:0]  e_tk,
  output logic [DATA_W-1:0] e_rdata,
  output logic              e_rvalid
);

  localparam logic [31:0] FP32_PZERO = 32'h0000_0000;
  // ==========================================================================
  // helper: idx = r*T + c (row-major)
  // ==========================================================================
  function automatic [IDX_W-1:0] rc_to_idx(input logic [ROW_W-1:0] rr,
                                           input logic [COL_W-1:0] cc);
    rc_to_idx = rr*T + cc;
  endfunction

  function automatic logic mask_hit(input logic [ROW_W-1:0] rr,
                                    input logic [COL_W-1:0] cc);
    logic pv;
    begin
      pv = pad_valid[rr] & pad_valid[cc];
      if (!pv) mask_hit = 1'b1;
      else if (causal_en && (cc > rr)) mask_hit = 1'b1;
      else mask_hit = 1'b0;
    end
  endfunction

  // ==========================================================================
  // 1) submax core
  // ==========================================================================
  logic core_busy, core_done;

  // expose score read ports only if你要debug；這裡不拉出來（內部掃描由 core 自己做）
  logic                 score_re_dummy;
  logic [T_W-1:0]       score_tq_dummy;
  logic [T_W-1:0]       score_tk_dummy;
  logic [DATA_W-1:0]    score_rdata_dummy;
  logic                 score_rvalid_dummy;

  // core internal adder passthrough not used here
  logic add_busy_dummy, add_done_dummy;
  logic [31:0] add_z_dummy;

  // Y read port from core
  logic              y_re;
  logic [ROW_W-1:0]  y_tq;
  logic [COL_W-1:0]  y_tk;
  logic [DATA_W-1:0] y_rdata;
  logic              y_rvalid;

  attention_presoft_submax_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) u_core (
    .clk(clk),
    .rst_n(rst_n),

    .fsm_start(start),   // 用 start 觸發 core（core 內部有 ST_DONE 防抖）
    .D_len(D_len),
    .fsm_busy(),
    .fsm_done(),

    .cpu_q_we(cpu_q_we),
    .cpu_q_t(cpu_q_t),
    .cpu_q_d(cpu_q_d),
    .cpu_q_wdata(cpu_q_wdata),
    .cpu_q_wmask(cpu_q_wmask),

    .cpu_k_we(cpu_k_we),
    .cpu_k_t(cpu_k_t),
    .cpu_k_d(cpu_k_d),
    .cpu_k_wdata(cpu_k_wdata),

    .score_re(score_re_dummy),
    .score_tq(score_tq_dummy),
    .score_tk(score_tk_dummy),
    .score_rdata(score_rdata_dummy),
    .score_rvalid(score_rvalid_dummy),

    .add_start(1'b0),
    .add_a_bits(32'd0),
    .add_b_bits(32'd0),
    .add_busy(add_busy_dummy),
    .add_done(add_done_dummy),
    .add_z_bits(add_z_dummy),

    .y_re(y_re),
    .y_tq(y_tq),
    .y_tk(y_tk),
    .y_rdata(y_rdata),
    .y_rvalid(y_rvalid),

    .busy(core_busy),
    .done(core_done)
  );

  // ==========================================================================
  // 2) Sweep Y to build Y_flat (masked)
  // ==========================================================================
  logic [DATA_W*N-1:0] y_flat_masked;
  logic [DATA_W*N-1:0] e_flat;

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_WAIT_CORE_DONE,
    ST_Y_REQ,
    ST_Y_CAP,
    ST_EXP_RUN,
    ST_DONE
  } st_t;

  st_t st;

  logic [ROW_W-1:0] rr;
  logic [COL_W-1:0] cc;

  // y port is 1-cycle latency: y_rvalid is delayed y_re
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st            <= ST_IDLE;
      rr            <= '0;
      cc            <= '0;
      y_re          <= 1'b0;
      y_tq          <= '0;
      y_tk          <= '0;
      y_flat_masked <= '0;
      done          <= 1'b0;
    end else begin
      done <= 1'b0;
      y_re <= 1'b0;

      case (st)
        ST_IDLE: begin
          rr <= '0;
          cc <= '0;
          if (start) begin
            // 等 core 自己跑完 submax（core_done 是 level）
            st <= ST_WAIT_CORE_DONE;
          end
        end

        ST_WAIT_CORE_DONE: begin
          if (core_done) begin
            rr <= '0;
            cc <= '0;
            st <= ST_Y_REQ;
          end
        end

        ST_Y_REQ: begin
          // issue read request
          y_tq <= rr;
          y_tk <= cc;
          y_re <= 1'b1;
          st   <= ST_Y_CAP;
        end

        ST_Y_CAP: begin
          // capture on rvalid
          if (y_rvalid) begin
            logic [IDX_W-1:0] idx;
            idx = rc_to_idx(rr, cc);

            if (mask_hit(rr, cc)) begin
              y_flat_masked[DATA_W*idx +: DATA_W] <= FP32_PZERO;
            end else begin
              y_flat_masked[DATA_W*idx +: DATA_W] <= fp32_div2(y_rdata);
            end

            // advance
            if (cc == COL_W'(T-1)) begin
              cc <= '0;
              if (rr == ROW_W'(T-1)) begin
                st <= ST_EXP_RUN;
              end else begin
                rr <= rr + 1'b1;
                st <= ST_Y_REQ;
              end
            end else begin
              cc <= cc + 1'b1;
              st <= ST_Y_REQ;
            end
          end
        end

        ST_EXP_RUN: begin
          // wait exp done below (exp_start will be generated as 1-cycle pulse)
          if (exp_done_pulse) begin
            st   <= ST_DONE;
          end
        end

        ST_DONE: begin
          done <= 1'b1; // 1-cycle pulse
          if (!start) st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

  // ==========================================================================
  // 3) exp matrix core
  //    assumes fp32_exp_matrix_2d interface: start/busy/done + in_flat/out_flat
  // ==========================================================================
  logic exp_start;
  logic exp_busy;
  logic exp_done;

  logic exp_done_q;
  logic exp_done_pulse;

  // fire exp_start exactly once when entering ST_EXP_RUN
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      exp_start <= 1'b0;
    end else begin
      exp_start <= 1'b0;
      if (st == ST_EXP_RUN && !exp_busy && !exp_done) begin
        // 如果 exp_matrix_2d 是 done pulse，這個條件也 OK（只會打一發）
        exp_start <= 1'b1;
      end
    end
  end
function automatic logic [31:0] fp32_div2(input logic [31:0] a);
  logic s; logic [7:0] e; logic [22:0] f;
  begin
    s = a[31]; e = a[30:23]; f = a[22:0];
    if (e == 8'h00 || e == 8'hFF) fp32_div2 = a; // 0/denorm/Inf/NaN 先不動
    else fp32_div2 = {s, (e - 8'd1), f};
  end
endfunction

  // make done pulse from exp_done (if exp_done is level)
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      exp_done_q <= 1'b0;
    end else begin
      exp_done_q <= exp_done;
    end
  end
  assign exp_done_pulse = exp_done & ~exp_done_q;

  fp32_exp_matrix_flat #(
    .ROWS  (T),
    .COLS  (T),
    .DATA_W(DATA_W)
  ) u_exp_mat (
    .clk      (clk),
    .rst_n    (rst_n),

    .start    (exp_start),
    .busy     (exp_busy),
    .done     (exp_done),

    .in_flat  (y_flat_masked),
    .out_flat (e_flat),

    .cur_r    (),
    .cur_c    (),
    .cur_idx  ()
  );

  // ==========================================================================
  // 4) busy/done
  // ==========================================================================
  always_comb begin
    busy = (st != ST_IDLE) && (st != ST_DONE);
  end

  // ==========================================================================
  // 5) E read port (1-cycle latency)
  // ==========================================================================
  logic rd_pending;
  logic [ROW_W-1:0] rd_row_q;
  logic [COL_W-1:0] rd_col_q;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rd_pending <= 1'b0;
      rd_row_q   <= '0;
      rd_col_q   <= '0;
    end else begin
      rd_pending <= e_re;
      if (e_re) begin
        rd_row_q <= e_tq;
        rd_col_q <= e_tk;
      end
    end
  end

  always_comb begin
    e_rvalid = rd_pending;
    e_rdata  = e_flat[DATA_W*rc_to_idx(rd_row_q, rd_col_q) +: DATA_W];
  end

endmodule

`default_nettype wire
