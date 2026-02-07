// QKT_FSM.sv (iverilog-safe)
// Flow:
// 1) start -> transpose start (pulse) -> wait tr_busy -> wait tr_done
// 2) for k=0..D_eff-1, t=0..T-1: read K^T(d=k, t=t) from transpose B, write into GEMM X[k][t]
// 3) start GEMM (pulse) -> wait gemm_busy -> wait gemm_done -> wait C_valid
// 4) done (sticky until start deassert)

`timescale 1ns/1ps
`default_nettype none

module QKT_FSM #(
  parameter int unsigned T      = 8,
  parameter int unsigned DMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  localparam int unsigned T_W   = (T<=1)?1:$clog2(T),
  localparam int unsigned D_W   = (DMAX<=1)?1:$clog2(DMAX)
)(
  input  logic clk,
  input  logic rst_n,

  // control
  input  logic        start,
  input  logic [15:0] D_len,
  output logic        busy,
  output logic        done,

  // ---- transpose control ----
  output logic        tr_start,
  input  logic        tr_busy,
  input  logic        tr_done,

  // ---- transpose B read port (K^T) ----
  output logic        tr_b_re,
  output logic [31:0] tr_b_row,
  output logic [31:0] tr_b_col,
  input  logic [DATA_W-1:0] tr_b_rdata,
  input  logic              tr_b_rvalid,

  // ---- GEMM X write port ----
  output logic                 cpu_x_we,
  output logic [D_W-1:0]       cpu_x_k,
  output logic [T_W-1:0]       cpu_x_n,
  output logic [DATA_W-1:0]    cpu_x_wdata,
  output logic [BYTE_W-1:0]    cpu_x_wmask,

  // ---- GEMM control ----
  output logic        gemm_start,
  input  logic        gemm_busy,
  input  logic        gemm_done,
  input  logic        C_valid
);

  // ----------------------------
  // D clamp
  // ----------------------------
  logic [15:0] D_eff;
  always @* begin
    if (D_len > DMAX[15:0]) D_eff = DMAX[15:0];
    else                    D_eff = D_len;
  end

  // protect D_eff==0
  wire d_eff_is_zero = (D_eff == 16'd0);

  // ----------------------------
  // State
  // ----------------------------
  typedef enum logic [3:0] {
    S_IDLE        = 4'd0,

    S_TR_PULSE    = 4'd1,
    S_TR_WAITBUSY = 4'd2,
    S_TR_WAITDONE = 4'd3,

    S_RD_REQ      = 4'd4,
    S_RD_WAIT     = 4'd5,
    S_X_WR        = 4'd6,

    S_GEMM_PULSE  = 4'd7,
    S_GEMM_WAITB  = 4'd8,
    S_GEMM_WAITD  = 4'd9,
    S_GEMM_WAITCV = 4'd10,

    S_DONE        = 4'd11
  } state_t;

  state_t st, st_n;

  // counters (k: 0..D_eff-1, t: 0..T-1)
  logic [15:0] k_cnt, k_cnt_n;
  logic [15:0] t_cnt, t_cnt_n;

  // latch request address so rvalid return is aligned
  logic [15:0] k_req, k_req_n;
  logic [15:0] t_req, t_req_n;

  // latch read data
  logic [DATA_W-1:0] kt_bits, kt_bits_n;

  // sliced
  wire [D_W-1:0] k_req_dw = k_req[D_W-1:0];
  wire [T_W-1:0] t_req_tw = t_req[T_W-1:0];

  // ----------------------------
  // Combinational next
  // ----------------------------
  always @* begin
    // defaults
    st_n      = st;
    k_cnt_n   = k_cnt;
    t_cnt_n   = t_cnt;
    k_req_n   = k_req;
    t_req_n   = t_req;
    kt_bits_n = kt_bits;

    // outputs default
    tr_start   = 1'b0;

    tr_b_re    = 1'b0;
    tr_b_row   = 32'd0;
    tr_b_col   = 32'd0;

    cpu_x_we    = 1'b0;
    cpu_x_k     = '0;
    cpu_x_n     = '0;
    cpu_x_wdata = '0;
    cpu_x_wmask = '0;

    gemm_start  = 1'b0;

    busy = 1'b1;
    done = 1'b0;

    case (st)
      // ----------------------------
      S_IDLE: begin
        busy = 1'b0;
        if (start) begin
          k_cnt_n = 16'd0;
          t_cnt_n = 16'd0;
          st_n    = S_TR_PULSE;
        end
      end

      // ----------------------------
      // Transpose
      // ----------------------------
      S_TR_PULSE: begin
        tr_start = 1'b1;          // 1-cycle pulse
        st_n     = S_TR_WAITBUSY; // next wait busy
      end

      S_TR_WAITBUSY: begin
        // wait transpose accepts start
        if (tr_busy) st_n = S_TR_WAITDONE;
      end

      S_TR_WAITDONE: begin
        if (tr_done) begin
          // if D_eff==0, skip load and go GEMM (or DONE) — here go GEMM
          if (d_eff_is_zero) st_n = S_GEMM_PULSE;
          else               st_n = S_RD_REQ;
        end
      end

      // ----------------------------
      // Read K^T (transpose B) -> Write GEMM X
      // ----------------------------
      S_RD_REQ: begin
        // lock request address
        k_req_n = k_cnt;
        t_req_n = t_cnt;

        tr_b_re  = 1'b1;
        tr_b_row = k_cnt; // d
        tr_b_col = t_cnt; // t
        st_n     = S_RD_WAIT;
      end

      S_RD_WAIT: begin
        // wait data
        if (tr_b_rvalid) begin
          kt_bits_n = tr_b_rdata;
          st_n      = S_X_WR;
        end
      end

      S_X_WR: begin
        // 1-cycle write into GEMM X using latched (k_req,t_req)
        cpu_x_we    = 1'b1;
        cpu_x_k     = k_req_dw;
        cpu_x_n     = t_req_tw;
        cpu_x_wdata = kt_bits;
        cpu_x_wmask = {BYTE_W{1'b1}};

        // advance counters
        if (t_cnt == (T-1)) begin
          t_cnt_n = 16'd0;
          if (k_cnt == (D_eff-1)) begin
            st_n = S_GEMM_PULSE;
          end else begin
            k_cnt_n = k_cnt + 16'd1;
            st_n    = S_RD_REQ;
          end
        end else begin
          t_cnt_n = t_cnt + 16'd1;
          st_n    = S_RD_REQ;
        end
      end

      // ----------------------------
      // GEMM
      // ----------------------------
      S_GEMM_PULSE: begin
        gemm_start = 1'b1;      // 1-cycle pulse
        st_n       = S_GEMM_WAITB;
      end

      S_GEMM_WAITB: begin
        // wait GEMM accepts start
        if (gemm_busy) st_n = S_GEMM_WAITD;
      end

      S_GEMM_WAITD: begin
        if (gemm_done) st_n = S_GEMM_WAITCV;
      end

      S_GEMM_WAITCV: begin
        if (C_valid) st_n = S_DONE;
      end

      // ----------------------------
      S_DONE: begin
        busy = 1'b0;
        done = 1'b1;
        // sticky until start deassert
        if (!start) st_n = S_IDLE;
      end

      default: st_n = S_IDLE;
    endcase
  end

  // ----------------------------
  // Sequential
  // ----------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st      <= S_IDLE;
      k_cnt   <= 16'd0;
      t_cnt   <= 16'd0;
      k_req   <= 16'd0;
      t_req   <= 16'd0;
      kt_bits <= '0;
    end else begin
      st      <= st_n;
      k_cnt   <= k_cnt_n;
      t_cnt   <= t_cnt_n;
      k_req   <= k_req_n;
      t_req   <= t_req_n;
      kt_bits <= kt_bits_n;
    end
  end

endmodule

`default_nettype wire
