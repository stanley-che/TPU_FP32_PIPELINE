// QKT_FSM.sv (iverilog-safe)
// Control flow:
// 1) start -> transpose start
// 2) read K^T from transpose B and write into GEMM X SRAM
// 3) start GEMM
// 4) done

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

  // D clamp
  logic [15:0] D_eff;
  always @* begin
    if (D_len > DMAX[15:0]) D_eff = DMAX[15:0];
    else                    D_eff = D_len;
  end

  // state
  typedef enum logic [3:0] {
  S_IDLE          = 4'd0,
  S_TR_START      = 4'd1,
  S_TR_WAIT       = 4'd2,
  S_LOAD_REQ      = 4'd3,
  S_LOAD_WAIT     = 4'd4,
  S_GEMM_START    = 4'd5,
  S_GEMM_WAITDONE = 4'd6,
  S_GEMM_WAITCV   = 4'd7,
  S_DONE          = 4'd8
} state_t;

  state_t st, st_n;

  logic [15:0] k_cnt, k_cnt_n;
  logic [15:0] t_cnt, t_cnt_n;

  logic [DATA_W-1:0] kt_data, kt_data_n;

  // --------- pre-sliced wires (avoid constant selects in always blocks)
  wire [D_W-1:0] k_cnt_dw = k_cnt[D_W-1:0];
  wire [T_W-1:0] t_cnt_tw = t_cnt[T_W-1:0];

  // next-state logic (iverilog: use always @*)
  always @* begin
    // defaults
    st_n      = st;
    k_cnt_n   = k_cnt;
    t_cnt_n   = t_cnt;
    kt_data_n = kt_data;

    tr_start   = 1'b0;

    tr_b_re    = 1'b0;
    tr_b_row   = 32'd0;
    tr_b_col   = 32'd0;

    cpu_x_we    = 1'b0;
    cpu_x_k     = {D_W{1'b0}};
    cpu_x_n     = {T_W{1'b0}};
    cpu_x_wdata = {DATA_W{1'b0}};
    cpu_x_wmask = {BYTE_W{1'b0}};

    gemm_start  = 1'b0;

    busy = 1'b1;
    done = 1'b0;

    case (st)
      S_IDLE: begin
        busy = 1'b0;
        if (start) begin
          k_cnt_n = 16'd0;
          t_cnt_n = 16'd0;
          st_n    = S_TR_START;
        end
      end

      S_TR_START: begin
        tr_start = 1'b1;
        st_n     = S_TR_WAIT;
      end

      S_TR_WAIT: begin
        if (tr_done) st_n = S_LOAD_REQ;
      end

      S_LOAD_REQ: begin
        tr_b_re  = 1'b1;
        tr_b_row = k_cnt;   // d
        tr_b_col = t_cnt;   // t
        st_n     = S_LOAD_WAIT;
      end

      S_LOAD_WAIT: begin
        if (tr_b_rvalid) begin
          kt_data_n = tr_b_rdata;

          cpu_x_we    = 1'b1;
          cpu_x_k     = k_cnt_dw;
          cpu_x_n     = t_cnt_tw;
          cpu_x_wdata = tr_b_rdata;
          cpu_x_wmask = {BYTE_W{1'b1}};

          if (t_cnt == (T-1)) begin
            t_cnt_n = 16'd0;
            if (k_cnt == (D_eff-1)) begin
              st_n = S_GEMM_START;
            end else begin
              k_cnt_n = k_cnt + 16'd1;
              st_n    = S_LOAD_REQ;
            end
          end else begin
            t_cnt_n = t_cnt + 16'd1;
            st_n    = S_LOAD_REQ;
          end
        end
      end

      S_GEMM_START: begin
  gemm_start = 1'b1;
  if (gemm_busy) st_n = S_GEMM_WAITDONE;
end

S_GEMM_WAITDONE: begin
  if (gemm_done) st_n = S_GEMM_WAITCV;
end

S_GEMM_WAITCV: begin
  if (C_valid) st_n = S_DONE;
end


      S_DONE: begin
        busy = 1'b0;
        done = 1'b1;
        if (!start) st_n = S_IDLE;
      end

      default: st_n = S_IDLE;
    endcase
  end

  // regs
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st      <= S_IDLE;
      k_cnt   <= 16'd0;
      t_cnt   <= 16'd0;
      kt_data <= {DATA_W{1'b0}};
    end else begin
      st      <= st_n;
      k_cnt   <= k_cnt_n;
      t_cnt   <= t_cnt_n;
      kt_data <= kt_data_n;
    end
  end

endmodule

`default_nettype wire
