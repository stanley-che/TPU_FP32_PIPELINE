// attention_presoft_with_rowmax.sv
`include "./src/EPU/attention_score/attention_score_top.sv"
`include "./src/EPU/attention_score/attention_scaling_tile.sv"
`timescale 1ns/1ps
`default_nettype none

module attention_presoft #(
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

  // one start runs QKT + scaling
  input  logic        start,
  input  logic [15:0] D_len,
  output logic        busy,
  output logic        done,

  // ---------------- Mask control ----------------
  input  logic [T-1:0] pad_valid,   // 1=valid token, 0=padding
  input  logic         causal_en,    // 1=enable causal mask

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

  // ---------------- CPU read SCALED score (masked output) ----------------
  input  logic                 sc_re,
  input  logic [ROW_W-1:0]     sc_tq,
  input  logic [COL_W-1:0]     sc_tk,
  output logic [DATA_W-1:0]    scm_rdata,    // masked rdata
  output logic                 scm_rvalid,   // masked rvalid

  // ---------------- Row max output (for Softmax pre-pass) ----------------
  output logic                 row_max_valid,
  output logic [31:0]          row_max_fp32
);

  // ============================================================
  // stage1: QKT top (raw score producer)
  // ============================================================
  logic fsm_start, fsm_busy, fsm_done;

  // score port (shared, scaling reads raw score)
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

  // ============================================================
  // stage2: scaling tile
  // ============================================================
  logic scale_start, scale_busy, scale_done;
  logic scale_in_score_re;
  logic [ROW_W-1:0] scale_in_score_tq;
  logic [COL_W-1:0] scale_in_score_tk;
  logic scale_C_valid;

  // raw scaled readout (before masking)
  logic [DATA_W-1:0] sc_rdata_raw;
  logic              sc_rvalid_raw;

  attention_scaling_tile #(
    .T(T),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W)
  ) u_scale (
    .clk(clk),
    .rst(~rst_n), // scaling tile uses active-high rst

    .start(scale_start),
    .D_len(D_len),
    .busy(scale_busy),
    .done(scale_done),

    .in_score_re(scale_in_score_re),
    .in_score_tq(scale_in_score_tq),
    .in_score_tk(scale_in_score_tk),
    .in_score_rdata(score_rdata),
    .in_score_rvalid(score_rvalid),

    .c_rd_en(1'b1),
    .c_rd_re(sc_re),
    .c_rd_row(sc_tq),
    .c_rd_col(sc_tk),
    .c_rd_rdata(sc_rdata_raw),
    .c_rd_rvalid(sc_rvalid_raw),

    .C_valid(scale_C_valid)
  );

  // score read port arbitration (scaling priority)
  assign score_re_mux = scale_in_score_re;
  assign score_tq_mux = scale_in_score_tq;
  assign score_tk_mux = scale_in_score_tk;

  // ============================================================
  // Readout-time masking (padding + causal)
  // IMPORTANT: align (q,k) index with returned rvalid
  // Assumption: scaling tile SRAM read latency = 1 cycle
  // ============================================================
  localparam logic [31:0] FP32_NINF = 32'hFF800000;

  logic [T_W-1:0] sc_q_d, sc_k_d;
  logic masked;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sc_q_d <= '0;
      sc_k_d <= '0;
    end else begin
      if (sc_re) begin
        sc_q_d <= sc_tq;
        sc_k_d <= sc_tk;
      end
    end
  end

  always_comb begin
    masked = 1'b0;

    // padding mask uses key index
    if (pad_valid[sc_k_d] == 1'b0)
      masked = 1'b1;

    // causal mask masks upper triangle (k > q)
    if (causal_en && (sc_k_d > sc_q_d))
      masked = 1'b1;

    scm_rvalid = sc_rvalid_raw;
    scm_rdata  = (sc_rvalid_raw && masked) ? FP32_NINF : sc_rdata_raw;
  end

  logic [31:0] cur_row_max;
  logic        row_start_d, row_last_d;

  // ============================================================
  // Row max (integrated)
  // ============================================================

  function automatic logic [31:0] fp32_key(input logic [31:0] a);
    begin
      fp32_key = a[31] ? ~a : (a ^ 32'h8000_0000);
    end
  endfunction


  // ---- FIX for iverilog: avoid (T-1)[T_W-1:0] ----
  localparam logic [T_W-1:0] TK0      = '0;
  localparam logic [T_W-1:0] TK_LAST  = (T-1);   // iverilog OK (constant sized by LHS)
  // ------------------------------------------------

  // align row_start/row_last with scm_rvalid (1-cycle delayed indices sc_k_d)
  always_comb begin
    row_start_d = (sc_k_d == TK0);       // tk==0
    row_last_d  = (sc_k_d == TK_LAST);   // tk==T-1
  end


  always_ff @(posedge clk) begin
    if (!rst_n) begin
      cur_row_max   <= 32'h0000_0000;
      row_max_valid <= 1'b0;
      row_max_fp32  <= 32'h0000_0000;
    end else begin
      row_max_valid <= 1'b0;

      if (scm_rvalid) begin
        if (row_start_d) begin
          cur_row_max <= scm_rdata;
        end else begin
          if (fp32_key(scm_rdata) > fp32_key(cur_row_max))
            cur_row_max <= scm_rdata;
        end

        if (row_last_d) begin
          // commit the max for this row
          row_max_fp32  <= row_start_d ? scm_rdata
                                       : ((fp32_key(scm_rdata) > fp32_key(cur_row_max)) ? scm_rdata : cur_row_max);
          row_max_valid <= 1'b1;
        end
      end
    end
  end

  // ============================================================
  // top FSM: run QKT then scaling
  // ============================================================
  typedef enum logic [1:0] {S_IDLE, S_QKT, S_SCALE, S_DONE} st_t;
  st_t st;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE;
      fsm_start   <= 1'b0;
      scale_start <= 1'b0;
      done <= 1'b0;
      busy <= 1'b0;
    end else begin
      fsm_start   <= 1'b0;
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
