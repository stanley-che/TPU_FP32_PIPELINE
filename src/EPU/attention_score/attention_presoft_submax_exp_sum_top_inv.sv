// ============================================================================
// attention_presoft_submax_exp_sum_top.sv  (SUM + INVSUM)
// 功能：
//  1) 跑 attention_presoft_submax_exp_top：得到 E = exp(Y) (FP32) in E SRAM
//  2) 掃描每個 row：Sum[row] = Σ_k E[row][k]   (FP32 adder)
//  3) 再做：InvSum[row] = 1 / Sum[row]         (fp32_softmax_inv_lut)
//  4) 對外提供：
//      - E[tq][tk] 讀取 (1-cycle, passthrough core)
//      - Sum[row]  讀取 (1-cycle)
//      - InvSum[row] 讀取 (1-cycle)
// ============================================================================

`include "./src/EPU/attention_score/attention_presoft_submax_exp_top.sv"
`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`include "./src/EPU/attention_score/fp32_softmax_inv_nr_2iter.sv"

`timescale 1ns/1ps
`default_nettype none

module attention_presoft_submax_exp_sum_top_inv #(
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

  // run whole pipeline
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

  // ---------------- E read (debug / optional passthrough) ----------------
  input  logic              e_re,
  input  logic [ROW_W-1:0]  e_tq,
  input  logic [COL_W-1:0]  e_tk,
  output logic [DATA_W-1:0] e_rdata,
  output logic              e_rvalid,

  // ---------------- Sum read: Sum[row] (1-cycle latency) ------------------
  input  logic              sum_re,
  input  logic [ROW_W-1:0]  sum_row,
  output logic [DATA_W-1:0] sum_rdata,
  output logic              sum_rvalid,

  // ---------------- InvSum read: InvSum[row]=1/Sum[row] (1-cycle) --------
  input  logic              invsum_re,
  input  logic [ROW_W-1:0]  invsum_row,
  output logic [DATA_W-1:0] invsum_rdata,
  output logic              invsum_rvalid
);

  // ==========================================================================
  // 1) Core: E = exp(Y)
  // ==========================================================================
  logic core_busy, core_done;

  // internal E read port for SUM scanner
  logic              e_re_i;
  logic [ROW_W-1:0]  e_tq_i;
  logic [COL_W-1:0]  e_tk_i;
  logic [DATA_W-1:0] e_rdata_i;
  logic              e_rvalid_i;

  attention_presoft_submax_exp_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W), .CONFLICT_POLICY_C(CONFLICT_POLICY_C), .TR_M(TR_M)
  ) u_exp_core (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .D_len(D_len),
    .busy(core_busy),
    .done(core_done),

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

    .e_re(e_re_i),
    .e_tq(e_tq_i),
    .e_tk(e_tk_i),
    .e_rdata(e_rdata_i),
    .e_rvalid(e_rvalid_i)
  );

  // ==========================================================================
  // 2) Memories: Sum_mem + InvSum_mem
  // ==========================================================================
  localparam logic [31:0] FP32_PZERO = 32'h00000000;

  logic [31:0] Sum_mem    [0:T-1];
  logic [31:0] InvSum_mem [0:T-1];

  // --- Sum read (1-cycle) ---
  logic sum_rd_pending;
  logic [ROW_W-1:0] sum_row_q;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sum_rd_pending <= 1'b0;
      sum_row_q      <= '0;
      sum_rdata      <= 32'h0;
      sum_rvalid     <= 1'b0;
    end else begin
      sum_rd_pending <= sum_re;
      if (sum_re) sum_row_q <= sum_row;

      sum_rvalid <= sum_rd_pending;
      if (sum_rd_pending) sum_rdata <= Sum_mem[sum_row_q];
    end
  end

  // --- InvSum read (1-cycle) ---
  logic invsum_rd_pending;
  logic [ROW_W-1:0] invsum_row_q;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      invsum_rd_pending <= 1'b0;
      invsum_row_q      <= '0;
      invsum_rdata      <= 32'h0;
      invsum_rvalid     <= 1'b0;
    end else begin
      invsum_rd_pending <= invsum_re;
      if (invsum_re) invsum_row_q <= invsum_row;

      invsum_rvalid <= invsum_rd_pending;
      if (invsum_rd_pending) invsum_rdata <= InvSum_mem[invsum_row_q];
    end
  end

  // ==========================================================================
  // 3) FP adder (accumulate): acc = acc + e_hold
  // ==========================================================================
  logic        add_start_i;
  logic        add_busy_i;
  logic        add_done_i;
  logic [31:0] add_z_i;
  logic [31:0] add_a_i, add_b_i;

  fp_adder_driver_ba u_sum_add (
    .clk    (clk),
    .rst    (~rst_n),
    .start  (add_start_i),
    .a_bits (add_a_i),
    .b_bits (add_b_i),
    .busy   (add_busy_i),
    .done   (add_done_i),
    .z_bits (add_z_i)
  );

  // ==========================================================================
  // 4) FP inverter for InvSum[row] = 1/Sum[row]
  // ==========================================================================
  logic        inv_in_valid;
  logic [31:0] inv_in_fp32;
  logic        inv_out_valid;
  logic [31:0] inv_out_fp32;

  fp32_softmax_inv_nr_2iter #(
    .M_BITS(6)
  ) u_inv (
    .clk      (clk),
    .rst_n    (rst_n),
    .in_valid (inv_in_valid),
    .in_fp32  (inv_in_fp32),
    .out_valid(inv_out_valid),
    .out_fp32 (inv_out_fp32)
  );

  // ==========================================================================
  // 5) SUM scan FSM (+ InvSum scan FSM)
  //    - internal E read scanning uses core read port when SUM/INVSUM active
  // ==========================================================================
  typedef enum logic [3:0] {
    S_IDLE,
    S_WAIT_CORE,
    S_ISSUE_E,
    S_WAIT_E,
    S_ADD,
    S_WRITE,

    S_INV_INIT,
    S_INV_ISSUE,
    S_INV_WAIT,
    S_INV_WRITE,

    S_DONE
  } sumst_t;

  sumst_t sst;

  // scan counters
  logic [ROW_W-1:0] rq;      // row for SUM scan
  logic [COL_W-1:0] ck;      // col for SUM scan

  logic [31:0] acc;          // row accumulator
  logic [31:0] e_hold;       // latch E data
  logic [ROW_W-1:0] rq_lat;
  logic [COL_W-1:0] ck_lat;

  // inv scan row
  logic [ROW_W-1:0] inv_rq;

  // --------------------------------------------------------------------------
  // Internal scan signals to core read port
  // pulse generation by state decode (CRITICAL)
  // --------------------------------------------------------------------------
  logic use_internal_e;

  assign use_internal_e = (sst != S_IDLE) && (sst != S_DONE);

  // For SUM scan:
  // - issue request in S_ISSUE_E (1-cycle pulse)
  logic e_re_scan;
  assign e_re_scan = (sst == S_ISSUE_E);

  logic [ROW_W-1:0] e_tq_scan;
  logic [COL_W-1:0] e_tk_scan;
  assign e_tq_scan = rq;
  assign e_tk_scan = ck;

  // MUX external/internal driving core read port
  always_comb begin
    if (use_internal_e) begin
      e_re_i = e_re_scan;
      e_tq_i = e_tq_scan;
      e_tk_i = e_tk_scan;
    end else begin
      e_re_i = e_re;
      e_tq_i = e_tq;
      e_tk_i = e_tk;
    end
  end

  // external read outputs always whatever core returns
  assign e_rdata  = e_rdata_i;
  assign e_rvalid = e_rvalid_i;

  // --------------------------------------------------------------------------
  // Adder controls: start pulse in S_ADD
  // --------------------------------------------------------------------------
  assign add_start_i = (sst == S_ADD);
  assign add_a_i     = acc;
  assign add_b_i     = e_hold;

  // --------------------------------------------------------------------------
  // Inverter controls: start pulse in S_INV_ISSUE (1-cycle)
  // --------------------------------------------------------------------------
  assign inv_in_valid = (sst == S_INV_ISSUE);
  assign inv_in_fp32  = Sum_mem[inv_rq];

  // --------------------------------------------------------------------------
  // busy/done
  // --------------------------------------------------------------------------
  assign busy = (sst != S_IDLE) && (sst != S_DONE);
  assign done = (sst == S_DONE);

  // ==========================================================================
  // FSM sequential
  // ==========================================================================
  integer r_init;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sst    <= S_IDLE;
      rq     <= '0;
      ck     <= '0;
      acc    <= FP32_PZERO;
      e_hold <= 32'h0;
      rq_lat <= '0;
      ck_lat <= '0;
      inv_rq <= '0;

      for (r_init = 0; r_init < T; r_init = r_init + 1) begin
        Sum_mem[r_init]    <= FP32_PZERO;
        InvSum_mem[r_init] <= FP32_PZERO;
      end

    end else begin
      case (sst)

        S_IDLE: begin
          if (start) sst <= S_WAIT_CORE;
        end

        S_WAIT_CORE: begin
          if (core_done) begin
            rq  <= '0;
            ck  <= '0;
            acc <= FP32_PZERO;
            sst <= S_ISSUE_E;
          end
        end

        S_ISSUE_E: begin
          // one-cycle pulse via e_re_scan = (sst==S_ISSUE_E)
          rq_lat <= rq;
          ck_lat <= ck;
          sst    <= S_WAIT_E;
        end

        S_WAIT_E: begin
          if (e_rvalid_i) begin
            e_hold <= e_rdata_i;
            sst    <= S_ADD;
          end
        end

        S_ADD: begin
          // adder start pulse via add_start_i = (sst==S_ADD)
          sst <= S_WRITE;
        end

        S_WRITE: begin
          if (add_done_i) begin
            // update accumulator
            acc <= add_z_i;

            if (ck_lat == COL_W'(T-1)) begin
              // row finished: write Sum_mem[row] = acc_new
              Sum_mem[rq_lat] <= add_z_i;

              if (rq_lat == ROW_W'(T-1)) begin
                // all rows summed -> start InvSum scan
                inv_rq <= '0;
                sst    <= S_INV_INIT;
              end else begin
                // next row
                rq  <= rq_lat + 1'b1;
                ck  <= '0;
                acc <= FP32_PZERO;
                sst <= S_ISSUE_E;
              end

            end else begin
              // next col
              ck  <= ck_lat + 1'b1;
              sst <= S_ISSUE_E;
            end
          end
        end

        // ---------------------------
        // InvSum scan: InvSum[row] = 1/Sum[row]
        // ---------------------------
        S_INV_INIT: begin
          // optional: could clear invsum mem here, but already reset
          sst <= S_INV_ISSUE;
        end

        S_INV_ISSUE: begin
          // inverter start pulse via inv_in_valid = (sst==S_INV_ISSUE)
          sst <= S_INV_WAIT;
        end

        S_INV_WAIT: begin
          if (inv_out_valid) begin
            sst <= S_INV_WRITE;
          end
        end

        S_INV_WRITE: begin
          InvSum_mem[inv_rq] <= inv_out_fp32;

          if (inv_rq == ROW_W'(T-1)) begin
            sst <= S_DONE;
          end else begin
            inv_rq <= inv_rq + 1'b1;
            sst    <= S_INV_ISSUE;
          end
        end

        S_DONE: begin
          // wait start deassert to re-arm (same style as你原本)
          if (!start) sst <= S_IDLE;
        end

        default: sst <= S_IDLE;

      endcase
    end
  end

endmodule

`default_nettype wire
