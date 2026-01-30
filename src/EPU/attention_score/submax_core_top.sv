// ============================================================================
// submax_core_top.sv  (FIXED SWEEP: 1-req / wait-rvalid)
// 功能：
//  1) 跑 QKT (attention_score_top_with_fsm) 產生 score SRAM
//  2) wrapper 自動掃描 T×T，把 score_rdata 寫入 X_mem
//  3) 同步計算每 row 最大值 RowMax_vec[row]
//  4) 掃描結束後做 submax：Y[row][col] = X[row][col] - RowMax_vec[row]
//     (使用 fp_adder_driver_ba：a + (-b))
//  5) 對外提供 Y 的讀取埠 (1-cycle latency)
// ============================================================================

`include "./src/EPU/attention_score/attention_score_top.sv"
`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"

`timescale 1ns/1ps
`default_nettype none

module attention_presoft_submax_top #(
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

  // ---------------- QKT FSM control ----------------
  input  logic        fsm_start,
  input  logic [15:0] D_len,
  output logic        fsm_busy,
  output logic        fsm_done,

  // ---------------- CPU write Q ----------------
  input  logic                 cpu_q_we,
  input  logic [T_W-1:0]       cpu_q_t,
  input  logic [D_W-1:0]       cpu_q_d,
  input  logic [DATA_W-1:0]    cpu_q_wdata,
  input  logic [BYTE_W-1:0]    cpu_q_wmask,

  // ---------------- CPU write K ----------------
  input  logic                 cpu_k_we,
  input  logic [31:0]          cpu_k_t,
  input  logic [31:0]          cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // ---------------- score SRAM read (from QKT core) ----------------
  // (wrapper 會自己產生 sweep，但也保留輸入讓你 debug/覆蓋)
  input  logic                 score_re,
  input  logic [T_W-1:0]       score_tq,
  input  logic [T_W-1:0]       score_tk,
  output logic [DATA_W-1:0]    score_rdata,
  output logic                 score_rvalid,

  // ---------------- FP adder driver (extra passthrough, optional) -----------
  input  logic        add_start,
  input  logic [31:0] add_a_bits,
  input  logic [31:0] add_b_bits,
  output logic        add_busy,
  output logic        add_done,
  output logic [31:0] add_z_bits,

  // ---------------- Submax result read: Y = X - max(row) -------------------
  input  logic              y_re,
  input  logic [ROW_W-1:0]  y_tq,
  input  logic [COL_W-1:0]  y_tk,
  output logic [DATA_W-1:0] y_rdata,
  output logic              y_rvalid,

  output logic              busy,
  output logic              done
);

  // ==========================================================================
  // 0) helper functions
  // ==========================================================================
  function automatic logic [31:0] fp32_neg(input logic [31:0] a);
    fp32_neg = {~a[31], a[30:0]};
  endfunction

  function automatic logic [31:0] fp32_key(input logic [31:0] a);
    fp32_key = a[31] ? ~a : (a ^ 32'h8000_0000);
  endfunction

  // ==========================================================================
  // 1) QKT core instance (score SRAM producer)
  // ==========================================================================
  logic              score_re_i;
  logic [T_W-1:0]    score_tq_i;
  logic [T_W-1:0]    score_tk_i;
  
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

    .score_re(score_re_i),
    .score_tq(score_tq_i),
    .score_tk(score_tk_i),
    .score_rdata(score_rdata),
    .score_rvalid(score_rvalid)
  );

  // ==========================================================================
  // 2) External adder passthrough (optional debug)
  // ==========================================================================
  fp_adder_driver_ba u_add (
    .clk    (clk),
    .rst    (~rst_n),
    .start  (add_start),
    .a_bits (add_a_bits),
    .b_bits (add_b_bits),
    .busy   (add_busy),
    .done   (add_done),
    .z_bits (add_z_bits)
  );

  // ==========================================================================
  // 3) Internal storage: X, RowMax, Y
  // ==========================================================================
  logic [31:0] X_mem      [0:T-1][0:T-1];
  logic [31:0] Y_mem      [0:T-1][0:T-1];
  logic [31:0] RowMax_vec [0:T-1];

  // ==========================================================================
  // 4) Auto-sweep controller (after fsm_done)  [FIXED]
  //    1 request -> wait score_rvalid -> next request
  // ==========================================================================
  typedef enum logic [2:0] {ST_IDLE, ST_WAIT_QKT, ST_REQ, ST_WAIT_R, ST_SUB, ST_DONE} st_t;
  st_t st;

  logic [ROW_W-1:0] tq_cnt;
  logic [COL_W-1:0] tk_cnt;

  logic [ROW_W-1:0] tq_req;
  logic [COL_W-1:0] tk_req;

  // streaming row max (for current row tq_req)
  logic [31:0] cur_row_max;

  // drive score read mux:
  // - in ST_REQ: internal sweep issues exactly 1 request pulse
  // - otherwise: pass-through external debug ports
  always_comb begin
    score_re_i = score_re;
    score_tq_i = score_tq;
    score_tk_i = score_tk;

    if (st == ST_REQ) begin
      score_re_i = 1'b1;      // one-cycle pulse
      score_tq_i = tq_cnt;
      score_tk_i = tk_cnt;
    end
  end

  // capture X_mem and compute RowMax_vec ONLY when response comes back
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int r=0; r<T; r++) begin
        RowMax_vec[r] <= 32'hFF800000; // -INF
        for (int c=0; c<T; c++) begin
          X_mem[r][c] <= 32'h00000000;
          Y_mem[r][c] <= 32'h00000000;
        end
      end
      cur_row_max <= 32'hFF800000;
    end else begin
      if (st == ST_WAIT_R && score_rvalid) begin
        // store X at the correct indexed cell
        X_mem[tq_req][tk_req] <= score_rdata;

        // row max update (tk_req is guaranteed in-order 0..T-1)
        if (tk_req == '0) begin
          cur_row_max <= score_rdata;
        end else if (fp32_key(score_rdata) > fp32_key(cur_row_max)) begin
          cur_row_max <= score_rdata;
        end

        if (tk_req == COL_W'(T-1)) begin
          RowMax_vec[tq_req] <= (fp32_key(score_rdata) > fp32_key(cur_row_max))
                                ? score_rdata : cur_row_max;
        end
      end
    end
  end

  // ==========================================================================
  // 5) Submax compute FSM: Y = X + (-RowMax[row])  using internal adder
  // ==========================================================================
  logic        sub_add_start, sub_add_busy, sub_add_done;
  logic [31:0] sub_add_z;
  logic [31:0] a_cur, b_cur;

  fp_adder_driver_ba u_sub_add (
    .clk    (clk),
    .rst    (~rst_n),
    .start  (sub_add_start),
    .a_bits (a_cur),
    .b_bits (b_cur),
    .busy   (sub_add_busy),
    .done   (sub_add_done),
    .z_bits (sub_add_z)
  );

  typedef enum logic [2:0] {SM_IDLE, SM_PREP, SM_LAUNCH, SM_WAIT, SM_WRITE, SM_DONE} sm_t;
  sm_t sm;

  logic [ROW_W-1:0] r_idx;
  logic [COL_W-1:0] c_idx;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sm            <= SM_IDLE;
      r_idx         <= '0;
      c_idx         <= '0;
      sub_add_start <= 1'b0;
      a_cur         <= '0;
      b_cur         <= '0;
    end else begin
      sub_add_start <= 1'b0;

      case (sm)
        SM_IDLE: begin
          if (st == ST_SUB) begin
            r_idx <= '0;
            c_idx <= '0;
            sm    <= SM_PREP;
          end
        end

        SM_PREP: begin
          a_cur <= X_mem[r_idx][c_idx];
          b_cur <= fp32_neg(RowMax_vec[r_idx]);
          sm    <= SM_LAUNCH;
        end

        SM_LAUNCH: begin
          sub_add_start <= 1'b1;
          if (sub_add_busy) begin
            sub_add_start <= 1'b0;
            sm <= SM_WAIT;
          end
        end

        SM_WAIT: begin
          if (sub_add_done) sm <= SM_WRITE;
        end

        SM_WRITE: begin
          Y_mem[r_idx][c_idx] <= sub_add_z;

          if (c_idx == COL_W'(T-1)) begin
            c_idx <= '0;
            if (r_idx == ROW_W'(T-1)) begin
              sm <= SM_DONE;
            end else begin
              r_idx <= r_idx + 1'b1;
              sm <= SM_PREP;
            end
          end else begin
            c_idx <= c_idx + 1'b1;
            sm <= SM_PREP;
          end
        end

        SM_DONE: begin
          sm <= SM_DONE;
        end

        default: sm <= SM_IDLE;
      endcase
    end
  end

  // ==========================================================================
  // 6) Top-level FSM: QKT done -> sweep(req/wait) -> sub -> done  [FIXED]
  // ==========================================================================
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st     <= ST_IDLE;
      tq_cnt <= '0;
      tk_cnt <= '0;
      tq_req <= '0;
      tk_req <= '0;
    end else begin
      case (st)
        ST_IDLE: begin
          tq_cnt <= '0;
          tk_cnt <= '0;
          if (fsm_start) st <= ST_WAIT_QKT;
        end

        ST_WAIT_QKT: begin
          if (fsm_done) begin
            tq_cnt <= '0;
            tk_cnt <= '0;
            st <= ST_REQ;
          end
        end

        ST_REQ: begin
          // latch request indices so response writes correct cell
          tq_req <= tq_cnt;
          tk_req <= tk_cnt;
          st <= ST_WAIT_R;
        end

        ST_WAIT_R: begin
          if (score_rvalid) begin
            // advance counters AFTER response
            if (tk_cnt == COL_W'(T-1)) begin
              tk_cnt <= '0;
              if (tq_cnt == ROW_W'(T-1)) begin
                st <= ST_SUB;
              end else begin
                tq_cnt <= tq_cnt + 1'b1;
                st <= ST_REQ;
              end
            end else begin
              tk_cnt <= tk_cnt + 1'b1;
              st <= ST_REQ;
            end
          end
        end

        ST_SUB: begin
          if (sm == SM_DONE) st <= ST_DONE;
        end

        ST_DONE: begin
          if (!fsm_start) st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

  // overall busy/done
  always_comb begin
    busy = (st != ST_IDLE) && (st != ST_DONE);
    done = (st == ST_DONE);
  end

  // ==========================================================================
  // 7) Y read port (1-cycle latency)
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
      rd_pending <= y_re;
      if (y_re) begin
        rd_row_q <= y_tq;
        rd_col_q <= y_tk;
      end
    end
  end

  always_comb begin
    y_rvalid = rd_pending;
    y_rdata  = Y_mem[rd_row_q][rd_col_q];
  end

endmodule

`default_nettype wire
