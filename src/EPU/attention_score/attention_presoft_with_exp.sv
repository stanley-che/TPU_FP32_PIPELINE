`include "./src/EPU/attention_score/submax_core_top.sv"
`include "./src/EPU/attention_score/fp32_exp_lut.sv"
`timescale 1ns/1ps
`default_nettype none

module attention_presoft_with_exp #(
  parameter int unsigned T      = 4,
  parameter int unsigned DMAX   = 16,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 2,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY_C = 1,
  parameter int unsigned TR_M   = 2,

  localparam int unsigned T_W = (T<=1)?1:$clog2(T),
  localparam int unsigned D_W = (DMAX<=1)?1:$clog2(DMAX)
)(
  input  logic clk,
  input  logic rst_n,

  // ---- original control ----
  input  logic        start,
  input  logic [15:0] D_len,
  output logic        busy,
  output logic        done,

  input  logic [T-1:0] pad_valid,
  input  logic         causal_en,

  // CPU write Q
  input  logic              cpu_q_we,
  input  logic [T_W-1:0]    cpu_q_t,
  input  logic [D_W-1:0]    cpu_q_d,
  input  logic [DATA_W-1:0] cpu_q_wdata,
  input  logic [BYTE_W-1:0] cpu_q_wmask,

  // CPU write K
  input  logic              cpu_k_we,
  input  logic [31:0]       cpu_k_t,
  input  logic [31:0]       cpu_k_d,
  input  logic [DATA_W-1:0] cpu_k_wdata,

  // ---- keep original SC masked read port (optional) ----
  input  logic           sc_re,
  input  logic [T_W-1:0] sc_tq,
  input  logic [T_W-1:0] sc_tk,
  output logic [DATA_W-1:0] scm_rdata,
  output logic              scm_rvalid,

  // ---- new: exp(SC) matrix read port ----
  input  logic           exp_re,
  input  logic [T_W-1:0] exp_tq,
  input  logic [T_W-1:0] exp_tk,
  output logic [31:0]    exp_rdata,
  output logic           exp_rvalid,

  output logic           exp_done   // asserted when exp matrix is ready
);

  // ----------------------------
  // Instantiate attention_presoft
  // ----------------------------
  logic           sc_re_mux;
  logic [T_W-1:0] sc_tq_mux, sc_tk_mux;

  // scanner-generated sc read (internal)
  logic           sc_re_scan;
  logic [T_W-1:0] sc_tq_scan, sc_tk_scan;

  // simple arbitration:
  // if external sc_re is asserted, it wins; otherwise scanner drives.
  always_comb begin
    if (sc_re) begin
      sc_re_mux = 1'b1;
      sc_tq_mux = sc_tq;
      sc_tk_mux = sc_tk;
    end else begin
      sc_re_mux = sc_re_scan;
      sc_tq_mux = sc_tq_scan;
      sc_tk_mux = sc_tk_scan;
    end
  end

  attention_presoft #(
    .T(T),
    .DMAX(DMAX),
    .DATA_W(DATA_W),
    .ADDR_W(ADDR_W),
    .NB(NB),
    .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .TR_M(TR_M)
  ) u_presoft (
    .clk(clk),
    .rst_n(rst_n),

    .start(start),
    .D_len(D_len),
    .busy(busy),
    .done(done),

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

    .sc_re(sc_re_mux),
    .sc_tq(sc_tq_mux),
    .sc_tk(sc_tk_mux),
    .scm_rdata(scm_rdata),
    .scm_rvalid(scm_rvalid)
  );

  // ----------------------------
  // Instantiate exp LUT (1-cycle)
  // ----------------------------
  logic        exp_in_valid;
  logic [31:0] exp_in_q16_16;   
  logic        exp_out_valid;
  logic [31:0] exp_out_q16_16;

  fp32_exp_lut u_exp_lut (
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(exp_in_valid),
    .x_q16_16(exp_in_q16_16),
    .out_valid(exp_out_valid),
    .y_q16_16  (exp_out_q16_16)
  );

  // ----------------------------
  // Storage for exp(SC)
  // ----------------------------
  logic [31:0] exp_mat [0:T-1][0:T-1];

  // exp read port: 1-cycle valid
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      exp_rvalid <= 1'b0;
      exp_rdata  <= 32'h0;
    end else begin
      exp_rvalid <= exp_re;
      if (exp_re) exp_rdata <= exp_mat[exp_tq][exp_tk];
      else        exp_rdata <= 32'h0;
    end
  end
  function automatic logic [31:0] fp32_to_q16_16_simple(input logic [31:0] b);
  int e;
  int frac;
  int signed mant_q23;     // 1.frac in Q1.23
  int signed val_q16_16;
  int shift;
  begin
    e    = b[30:23];
    frac = b[22:0];

    if (e == 0) begin
      fp32_to_q16_16_simple = 32'h0; // treat subnormal as 0
    end else if (e == 255) begin
      fp32_to_q16_16_simple = 32'h7FFF_FFFF; // inf/nan saturate
    end else begin
      mant_q23 = (1<<23) | frac; // Q1.23

      // value = mant * 2^(e-127)
      // want Q16.16 => shift from Q1.23 to Q16.16 : total shift = (e-127) + (16-23) = (e-127) -7
      shift = (e - 127) - 7;

      if (shift >= 0)
        val_q16_16 = mant_q23 <<< shift;
      else
        val_q16_16 = mant_q23 >>> (-shift);

      // sign
      if (b[31]) val_q16_16 = -val_q16_16;

      fp32_to_q16_16_simple = val_q16_16[31:0];
    end
  end
  endfunction

  // ----------------------------
  // Scanner FSM:
  // after done, sweep tq/tk, read scm -> feed exp -> store
  // ----------------------------
  typedef enum logic [2:0] {
    S_IDLE,
    S_WAIT_DONE,
    S_REQ_SC,
    S_WAIT_SC,
    S_WAIT_EXP,
    S_NEXT,
    S_DONE
  } state_t;

  state_t st;

  logic [T_W-1:0] tq_it, tk_it;
  logic [T_W-1:0] tq_hold, tk_hold; // index being processed

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st          <= S_IDLE;
      sc_re_scan  <= 1'b0;
      sc_tq_scan  <= '0;
      sc_tk_scan  <= '0;

      exp_in_valid<= 1'b0;
      exp_in_q16_16 <= 32'h0;

      tq_it       <= '0;
      tk_it       <= '0;
      tq_hold     <= '0;
      tk_hold     <= '0;

      exp_done    <= 1'b0;
    end else begin
      // defaults each cycle
      sc_re_scan   <= 1'b0;
      exp_in_valid <= 1'b0;

      case (st)
        S_IDLE: begin
          exp_done <= 1'b0;
          // clear iter
          tq_it <= '0;
          tk_it <= '0;
          if (start) st <= S_WAIT_DONE;
        end

        S_WAIT_DONE: begin
          // wait attention_presoft done
          if (done) begin
            tq_it <= '0;
            tk_it <= '0;
            st    <= S_REQ_SC;
          end
        end

        S_REQ_SC: begin
          // only scan when external sc_re is not using port
          if (!sc_re) begin
            sc_re_scan <= 1'b1;
            sc_tq_scan <= tq_it;
            sc_tk_scan <= tk_it;
            tq_hold    <= tq_it;
            tk_hold    <= tk_it;
            st         <= S_WAIT_SC;
          end
        end

        S_WAIT_SC: begin
          // wait scm_rvalid then feed exp LUT
          if (scm_rvalid) begin
            exp_in_valid <= 1'b1;
            exp_in_q16_16 <= fp32_to_q16_16_simple(scm_rdata);
            st           <= S_WAIT_EXP;
          end
        end

        S_WAIT_EXP: begin
          if (exp_out_valid) begin
            exp_mat[tq_hold][tk_hold] <= exp_out_q16_16;
            st <= S_NEXT;
          end
        end

        S_NEXT: begin
          // advance tk then tq
          if (tk_it == T-1) begin
            tk_it <= '0;
            if (tq_it == T-1) begin
              st <= S_DONE;
            end else begin
              tq_it <= tq_it + 1'b1;
              st    <= S_REQ_SC;
            end
          end else begin
            tk_it <= tk_it + 1'b1;
            st    <= S_REQ_SC;
          end
        end

        S_DONE: begin
          exp_done <= 1'b1;
          // stay done until next start
          if (start) begin
            exp_done <= 1'b0;
            st <= S_WAIT_DONE;
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
