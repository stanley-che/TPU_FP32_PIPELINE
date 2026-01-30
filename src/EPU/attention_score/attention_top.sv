// ============================================================================
// attention_top.sv  (Single-head attention: O = softmax(QK^T)*V)
// - Uses your attention_softmax_top to produce P (T x T)
// - Uses your tile_compute_system_top (sramsa.sv) as GEMM engine for O = P * V
//
// Mapping into tile_compute_system_top:
//   W SRAM (M x K)  <= P[tq][tk]      where M=T, K_len=T
//   X SRAM (K x N)  <= V[tk][d]       where N=DMAX (we only care d < D_len)
//
// Notes:
// - This implementation stores V into an internal V_mem (written by cpu_v_we).
// - After softmax done, this top FSM "acts like CPU" to write W/X SRAMs,
//   then kicks tile_compute start.
// - tile_compute_system_top's `done` is tile-loader done; real result-valid is C_valid.
// ============================================================================

`ifndef ATTENTION_TOP_SV
`define ATTENTION_TOP_SV

`include "./src/EPU/attention_score/attention_softmax_top.sv"
`include "./src/EPU/attention_score/sramsa.sv"

`timescale 1ns/1ps
`default_nettype none

module attention_top #(
  parameter int unsigned T      = 8,
  parameter int unsigned DMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 8,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY_C = 1,
  parameter int unsigned TR_M   = 10,

  localparam int unsigned T_W   = (T<=1)?1:$clog2(T),
  localparam int unsigned D_W   = (DMAX<=1)?1:$clog2(DMAX)
)(
  input  logic clk,
  input  logic rst_n,

  input  logic        start,
  input  logic [15:0] D_len,
  output logic        busy,
  output logic        done,

  input  logic [T-1:0] pad_valid,
  input  logic         causal_en,

  // Q
  input  logic                 cpu_q_we,
  input  logic [T_W-1:0]       cpu_q_t,
  input  logic [D_W-1:0]       cpu_q_d,
  input  logic [DATA_W-1:0]    cpu_q_wdata,
  input  logic [BYTE_W-1:0]    cpu_q_wmask,

  // K
  input  logic                 cpu_k_we,
  input  logic [T_W-1:0]       cpu_k_t,
  input  logic [D_W-1:0]       cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // V
  input  logic                 cpu_v_we,
  input  logic [T_W-1:0]       cpu_v_tk,
  input  logic [D_W-1:0]       cpu_v_d,
  input  logic [DATA_W-1:0]    cpu_v_wdata,
  input  logic [BYTE_W-1:0]    cpu_v_wmask,

  // O read (from C SRAM of GEMM)
  input  logic              o_re,
  input  logic              o_en,
  input  logic [T_W-1:0]    o_tq,
  input  logic [D_W-1:0]    o_d,
  output logic [DATA_W-1:0] o_rdata,
  output logic              o_rvalid
);

  // ==========================================================================
  // 0) Store V locally (T x DMAX)
  // ==========================================================================
  logic [31:0] V_mem [0:T-1][0:DMAX-1];
  integer vr, vc;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (vr = 0; vr < T; vr = vr + 1)
        for (vc = 0; vc < DMAX; vc = vc + 1)
          V_mem[vr][vc] <= 32'h0;
    end else begin
      if (cpu_v_we) begin
        // byte-mask support (optional); simplest: if any byte enabled, write whole word
        if (|cpu_v_wmask) V_mem[cpu_v_tk][cpu_v_d] <= cpu_v_wdata;
      end
    end
  end

  // ==========================================================================
  // 1) Softmax block: produces P via p_re/p_rdata
  // ==========================================================================
  logic sm_busy, sm_done;

  logic        p_re;
  logic [T_W-1:0] p_tq;
  logic [T_W-1:0] p_tk;
  logic [31:0] p_rdata;
  logic        p_rvalid;

  // we don't use E debug port here
  logic        e_re_unused;
  logic [T_W-1:0] e_tq_unused, e_tk_unused;
  logic [31:0] e_rdata_unused;
  logic        e_rvalid_unused;

  attention_softmax_top #(
    .T(T), .DMAX(DMAX), .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) u_sm (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .D_len(D_len),
    .busy(sm_busy),
    .done(sm_done),

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

    .e_re(e_re_unused),
    .e_tq(e_tq_unused),
    .e_tk(e_tk_unused),
    .e_rdata(e_rdata_unused),
    .e_rvalid(e_rvalid_unused),

    .p_re(p_re),
    .p_tq(p_tq),
    .p_tk(p_tk),
    .p_rdata(p_rdata),
    .p_rvalid(p_rvalid)
  );

  // ==========================================================================
  // 2) GEMM engine for O = P * V
  //    Configure as: M=T, N=DMAX, KMAX=T
  // ==========================================================================
  localparam int unsigned KMAX_MATMUL = T;
  localparam int unsigned K_W_MATMUL  = (KMAX_MATMUL<=1)?1:$clog2(KMAX_MATMUL);
  localparam int unsigned N_W_MATMUL  = (DMAX<=1)?1:$clog2(DMAX);
  localparam int unsigned ROW_W_MM    = (T<=1)?1:$clog2(T);
  localparam int unsigned COL_W_MM    = (DMAX<=1)?1:$clog2(DMAX);

  // tile_compute I/O
  logic mm_start;
  logic [15:0] mm_K_len;
  logic mm_busy_tile;
  logic mm_done_tile;

  logic                 mm_cpu_w_we;
  logic [ROW_W_MM-1:0]  mm_cpu_w_row;
  logic [K_W_MATMUL-1:0]mm_cpu_w_k;
  logic [DATA_W-1:0]    mm_cpu_w_wdata;
  logic [BYTE_W-1:0]    mm_cpu_w_wmask;

  logic                 mm_cpu_x_we;
  logic [K_W_MATMUL-1:0]mm_cpu_x_k;
  logic [N_W_MATMUL-1:0]mm_cpu_x_n;
  logic [DATA_W-1:0]    mm_cpu_x_wdata;
  logic [BYTE_W-1:0]    mm_cpu_x_wmask;

  logic                 c_rd_en;
  logic                 c_rd_re;
  logic [ROW_W_MM-1:0]   c_rd_row;
  logic [COL_W_MM-1:0]   c_rd_col;
  logic [DATA_W-1:0]     c_rd_rdata;
  logic                 c_rd_rvalid;

  logic [T*DMAX*DATA_W-1:0] c_out_flat_o_unused;
  logic [T*DMAX-1:0]        c_valid_flat_o_unused;
  logic                     C_valid;

  logic [T*KMAX_MATMUL*DATA_W-1:0] W_tile_flat_dbg_unused;
  logic [KMAX_MATMUL*DMAX*DATA_W-1:0] X_tile_flat_dbg_unused;

  tile_compute_system_top #(
    .M(T),
    .N(DMAX),
    .KMAX(KMAX_MATMUL),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C)
  ) u_mm (
    .clk(clk),
    .rst(~rst_n),

    .start(mm_start),
    .K_len(mm_K_len),
    .busy(mm_busy_tile),
    .done(mm_done_tile),

    .cpu_w_we(mm_cpu_w_we),
    .cpu_w_row(mm_cpu_w_row),
    .cpu_w_k(mm_cpu_w_k),
    .cpu_w_wdata(mm_cpu_w_wdata),
    .cpu_w_wmask(mm_cpu_w_wmask),

    .cpu_x_we(mm_cpu_x_we),
    .cpu_x_k(mm_cpu_x_k),
    .cpu_x_n(mm_cpu_x_n),
    .cpu_x_wdata(mm_cpu_x_wdata),
    .cpu_x_wmask(mm_cpu_x_wmask),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid),

    .c_out_flat_o(c_out_flat_o_unused),
    .c_valid_flat_o(c_valid_flat_o_unused),
    .C_valid(C_valid),

    .W_tile_flat_dbg(W_tile_flat_dbg_unused),
    .X_tile_flat_dbg(X_tile_flat_dbg_unused)
  );

  // O read passthrough
  assign c_rd_en  = o_en;
  assign c_rd_re  = o_re;
  assign c_rd_row = o_tq;
  assign c_rd_col = o_d;      // col index = d
  assign o_rdata  = c_rd_rdata;
  assign o_rvalid = c_rd_rvalid;

  // ==========================================================================
  // 3) Control FSM: softmax -> load W(P) & X(V) -> start mm -> wait C_valid
  // ==========================================================================
  typedef enum logic [3:0] {
    S_IDLE,
    S_WAIT_SM,
    S_LOAD_W_REQ,    // issue P read (p_re)
    S_LOAD_W_WAIT,   // wait p_rvalid then write W
    S_LOAD_X,        // write X from V_mem
    S_START_MM,
    S_WAIT_CVALID,
    S_DONE
  } st_t;

  st_t st;

  logic [T_W-1:0]  w_r;  // tq
  logic [T_W-1:0]  w_c;  // tk
  logic [T_W-1:0]  x_k;  // tk
  logic [D_W-1:0]  x_n;  // d

  // default assigns
  always_comb begin
  // ---------------- defaults ----------------
  p_re = 1'b0;
  p_tq = w_r;
  p_tk = w_c;

  mm_cpu_w_we    = 1'b0;
  mm_cpu_w_row   = w_r;
  mm_cpu_w_k     = w_c;
  mm_cpu_w_wdata = p_rdata;
  mm_cpu_w_wmask = {BYTE_W{1'b1}};

  mm_cpu_x_we    = 1'b0;
  mm_cpu_x_k     = x_k;
  mm_cpu_x_n     = x_n[N_W_MATMUL-1:0];
  mm_cpu_x_wdata = V_mem[x_k][x_n];
  mm_cpu_x_wmask = {BYTE_W{1'b1}};

  mm_start = 1'b0;
  mm_K_len = T[15:0];

  // ---------------- per-state overrides ----------------
  case (st)
    S_LOAD_W_REQ: begin
      // 1-cycle request -> next cycle p_rvalid
      p_re = 1'b1;
    end

    S_LOAD_W_WAIT: begin
      // write W exactly when p_rvalid
      mm_cpu_w_we = p_rvalid;
    end

    S_LOAD_X: begin
      // stream V into X SRAM
      mm_cpu_x_we = (x_n < D_len);
    end

    S_START_MM: begin
      mm_start = 1'b1;
    end

    default: begin end
  endcase
end

  // busy/done
  always_comb begin
    busy = sm_busy || (st != S_IDLE && st != S_DONE);
    done = (st == S_DONE);
  end

  // sequential FSM
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st  <= S_IDLE;
      w_r <= '0;
      w_c <= '0;
      x_k <= '0;
      x_n <= '0;
    end else begin
      case (st)
        S_IDLE: begin
          if (start) st <= S_WAIT_SM;
        end

        S_WAIT_SM: begin
          if (sm_done) begin
            w_r <= '0;
            w_c <= '0;
            st  <= S_LOAD_W_REQ;
          end
        end

        // --- Load W (P matrix) into mm W SRAM ---
        S_LOAD_W_REQ: begin
          // issue p_re pulse
          // (p_re is driven in always_comb when st==S_LOAD_W_REQ)
          st <= S_LOAD_W_WAIT;
        end

        S_LOAD_W_WAIT: begin
          if (p_rvalid) begin
            // write happens same cycle via mm_cpu_w_we in always_comb when st==S_LOAD_W_WAIT
            // advance indices
            if (w_c == T_W'(T-1)) begin
              if (w_r == T_W'(T-1)) begin
                x_k <= '0;
                x_n <= '0;
                st  <= S_LOAD_X;
              end else begin
                w_r <= w_r + 1'b1;
                w_c <= '0;
                st  <= S_LOAD_W_REQ;
              end
            end else begin
              w_c <= w_c + 1'b1;
              st  <= S_LOAD_W_REQ;
            end
          end
        end

        // --- Load X (V matrix) into mm X SRAM ---
        S_LOAD_X: begin
          // write X every cycle (mm_cpu_x_we asserted in always_comb when st==S_LOAD_X)
          if (x_n == D_W'(D_len-1)) begin
            if (x_k == T_W'(T-1)) begin
              st <= S_START_MM;
            end else begin
              x_k <= x_k + 1'b1;
              x_n <= '0;
            end
          end else begin
            x_n <= x_n + 1'b1;
          end
        end

        S_START_MM: begin
          // mm_start pulse (in always_comb)
          st <= S_WAIT_CVALID;
        end

        S_WAIT_CVALID: begin
          if (C_valid) st <= S_DONE;
        end

        S_DONE: begin
          if (!start) st <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end



endmodule

`default_nettype wire
`endif
