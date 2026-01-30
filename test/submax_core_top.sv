// ============================================================================
// attention_presoft_submax_top.sv
// 整合：attention_presoft(含row_max) + (x[j][k] - max[j]) subtract core
//
// 功能：
//  1) 先跑 QKT + scaling + mask + row_max
//  2) wrapper 自動掃描整個 T×T (tq,tk)，把 scm_rdata 寫進 subtract core 的 X_mem
//  3) row_max_valid 出現時，把 row_max_fp32 寫進 subtract core 的 RowMax_vec[row]
//  4) 掃描結束後啟動 subtract core，計算 Y[j][k] = X[j][k] - RowMax_vec[j]
//  5) 對外提供 Y 的讀取埠 (1-cycle latency)
//
// ============================================================================

`include "./src/EPU/attention_score/attention_score_top.sv"
`include "./src/EPU/attention_score/attention_scaling_tile.sv"
`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"

`timescale 1ns/1ps
`default_nettype none

// ============================================================================
// Subtract core：Y = X - RowMax[row]  (用加法器做：X + (-RowMax))
// ============================================================================
module submax_core_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N)
)(
  input  logic clk,
  input  logic rst,

  input  logic start,
  output logic busy,
  output logic done,
  output logic C_valid,

  // write X
  input  logic              cpu_x_we,
  input  logic [ROW_W-1:0]  cpu_x_row,
  input  logic [COL_W-1:0]  cpu_x_col,
  input  logic [DATA_W-1:0] cpu_x_wdata,
  input  logic [BYTE_W-1:0] cpu_x_wmask,

  // write RowMax (indexed by row)
  input  logic              cpu_rm_we,
  input  logic [ROW_W-1:0]  cpu_rm_row,
  input  logic [DATA_W-1:0] cpu_rm_wdata,
  input  logic [BYTE_W-1:0] cpu_rm_wmask,

  // Y read (1-cycle)
  input  logic              c_rd_en,
  input  logic              c_rd_re,
  input  logic [ROW_W-1:0]  c_rd_row,
  input  logic [COL_W-1:0]  c_rd_col,
  output logic [DATA_W-1:0] c_rd_rdata,
  output logic              c_rd_rvalid
);

  logic [DATA_W-1:0] X_mem      [0:M-1][0:N-1];
  logic [DATA_W-1:0] C_mem      [0:M-1][0:N-1];
  logic [DATA_W-1:0] RowMax_vec [0:M-1];
  logic              C_vld      [0:M-1][0:N-1];

  function automatic [DATA_W-1:0] apply_wmask(
    input [DATA_W-1:0] oldv,
    input [DATA_W-1:0] newv,
    input [BYTE_W-1:0] wmask
  );
    logic [DATA_W-1:0] outv;
    outv = oldv;
    for (int i=0; i<BYTE_W; i++) begin
      if (wmask[i]) outv[i*8 +: 8] = newv[i*8 +: 8];
    end
    return outv;
  endfunction

  function automatic logic [31:0] fp32_neg(input logic [31:0] a);
    fp32_neg = {~a[31], a[30:0]};
  endfunction

  // writes
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int r=0; r<M; r++) RowMax_vec[r] <= '0;
      for (int r=0; r<M; r++) begin
        for (int c=0; c<N; c++) begin
          X_mem[r][c] <= '0;
          C_mem[r][c] <= '0;
          C_vld[r][c] <= 1'b0;
        end
      end
    end else begin
      if (cpu_x_we) begin
        X_mem[cpu_x_row][cpu_x_col] <= apply_wmask(
          X_mem[cpu_x_row][cpu_x_col], cpu_x_wdata, cpu_x_wmask
        );
      end
      if (cpu_rm_we) begin
        RowMax_vec[cpu_rm_row] <= apply_wmask(
          RowMax_vec[cpu_rm_row], cpu_rm_wdata, cpu_rm_wmask
        );
      end
    end
  end

  // read (1-cycle)
  logic rd_pending;
  logic [ROW_W-1:0] rd_row_q;
  logic [COL_W-1:0] rd_col_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      rd_pending <= 1'b0;
      rd_row_q   <= '0;
      rd_col_q   <= '0;
    end else begin
      rd_pending <= (c_rd_en && c_rd_re);
      if (c_rd_en && c_rd_re) begin
        rd_row_q <= c_rd_row;
        rd_col_q <= c_rd_col;
      end
    end
  end

  always_comb begin
    c_rd_rvalid = rd_pending;
    c_rd_rdata  = C_mem[rd_row_q][rd_col_q];
  end

  // adder driver
  logic        add_start, add_busy, add_done;
  logic [31:0] add_z;
  logic [31:0] a_cur, b_cur;

  fp_adder_driver_ba u_add (
    .clk    (clk),
    .rst    (rst),
    .start  (add_start),
    .a_bits (a_cur),
    .b_bits (b_cur),
    .busy   (add_busy),
    .done   (add_done),
    .z_bits (add_z)
  );

  typedef enum logic [2:0] {C_IDLE, C_PREP, C_LAUNCH, C_WAIT, C_WRITE} cstate_t;
  cstate_t cst;

  logic [ROW_W-1:0] r_idx;
  logic [COL_W-1:0] c_idx;

  always_comb begin
    busy    = (cst != C_IDLE);
    C_valid = done;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      cst       <= C_IDLE;
      done      <= 1'b0;
      add_start <= 1'b0;
      r_idx     <= '0;
      c_idx     <= '0;
      a_cur     <= '0;
      b_cur     <= '0;
      for (int r=0; r<M; r++) begin
        for (int c=0; c<N; c++) begin
          C_vld[r][c] <= 1'b0;
        end
      end
    end else begin
      add_start <= 1'b0;

      case (cst)
        C_IDLE: begin
          if (start) begin
            done  <= 1'b0;
            r_idx <= '0;
            c_idx <= '0;
            for (int r=0; r<M; r++) begin
              for (int c=0; c<N; c++) begin
                C_vld[r][c] <= 1'b0;
              end
            end
            cst <= C_PREP;
          end
        end

        C_PREP: begin
          a_cur <= X_mem[r_idx][c_idx];
          b_cur <= fp32_neg(RowMax_vec[r_idx]); // X - max(row)
          cst   <= C_LAUNCH;
        end

        C_LAUNCH: begin
          add_start <= 1'b1;
          if (add_busy) begin
            add_start <= 1'b0;
            cst       <= C_WAIT;
          end
        end

        C_WAIT: begin
          if (add_done) cst <= C_WRITE;
        end

        C_WRITE: begin
          C_mem[r_idx][c_idx] <= add_z;
          C_vld[r_idx][c_idx] <= 1'b1;

          if (c_idx == COL_W'(N-1)) begin
            c_idx <= '0;
            if (r_idx == ROW_W'(M-1)) begin
              done <= 1'b1;
              cst  <= C_IDLE;
            end else begin
              r_idx <= r_idx + 1'b1;
              cst   <= C_PREP;
            end
          end else begin
            c_idx <= c_idx + 1'b1;
            cst   <= C_PREP;
          end
        end

        default: cst <= C_IDLE;
      endcase
    end
  end

endmodule

// ============================================================================
// attention_presoft 版本：只多加 row_max_row 輸出
// 你原本的 module 內容我保留，只有「row max commit」那邊補 row_max_row。
// ============================================================================
module attention_presoft_rowmax_idx #(
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

  input  logic                 cpu_q_we,
  input  logic [T_W-1:0]       cpu_q_t,
  input  logic [D_W-1:0]       cpu_q_d,
  input  logic [DATA_W-1:0]    cpu_q_wdata,
  input  logic [BYTE_W-1:0]    cpu_q_wmask,

  input  logic                 cpu_k_we,
  input  logic [31:0]          cpu_k_t,
  input  logic [31:0]          cpu_k_d,
  input  logic [DATA_W-1:0]    cpu_k_wdata,

  // wrapper drive scan
  input  logic                 sc_re,
  input  logic [ROW_W-1:0]     sc_tq,
  input  logic [COL_W-1:0]     sc_tk,
  output logic [DATA_W-1:0]    scm_rdata,
  output logic                 scm_rvalid,

  output logic                 row_max_valid,
  output logic [31:0]          row_max_fp32,
  output logic [T_W-1:0]       row_max_row
);

  // stage1
  logic fsm_start, fsm_busy, fsm_done;

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

  // stage2 scaling
  logic scale_start, scale_busy, scale_done;
  logic scale_in_score_re;
  logic [ROW_W-1:0] scale_in_score_tq;
  logic [COL_W-1:0] scale_in_score_tk;
  logic scale_C_valid;

  logic [DATA_W-1:0] sc_rdata_raw;
  logic              sc_rvalid_raw;

  attention_scaling_tile #(
    .T(T),
    .DATA_W(DATA_W),
    .BYTE_W(BYTE_W)
  ) u_scale (
    .clk(clk),
    .rst(~rst_n),

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

  assign score_re_mux = scale_in_score_re;
  assign score_tq_mux = scale_in_score_tq;
  assign score_tk_mux = scale_in_score_tk;

  // mask align (1-cycle)
  localparam logic [31:0] FP32_NINF = 32'hFF800000;
  logic [T_W-1:0] sc_q_d, sc_k_d;
  logic masked;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sc_q_d <= '0;
      sc_k_d <= '0;
    end else if (sc_re) begin
      sc_q_d <= sc_tq;
      sc_k_d <= sc_tk;
    end
  end

  always_comb begin
    masked = 1'b0;
    if (pad_valid[sc_k_d] == 1'b0) masked = 1'b1;
    if (causal_en && (sc_k_d > sc_q_d)) masked = 1'b1;

    scm_rvalid = sc_rvalid_raw;
    scm_rdata  = (sc_rvalid_raw && masked) ? FP32_NINF : sc_rdata_raw;
  end

  // row max
  function automatic logic [31:0] fp32_key(input logic [31:0] a);
    fp32_key = a[31] ? ~a : (a ^ 32'h8000_0000);
  endfunction

  localparam logic [T_W-1:0] TK0     = '0;
  localparam logic [T_W-1:0] TK_LAST = (T-1);

  logic [31:0] cur_row_max;
  logic        row_start_d, row_last_d;

  always_comb begin
    row_start_d = (sc_k_d == TK0);
    row_last_d  = (sc_k_d == TK_LAST);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      cur_row_max   <= 32'h0000_0000;
      row_max_valid <= 1'b0;
      row_max_fp32  <= 32'h0000_0000;
      row_max_row   <= '0;
    end else begin
      row_max_valid <= 1'b0;

      if (scm_rvalid) begin
        if (row_start_d) begin
          cur_row_max <= scm_rdata;
        end else if (fp32_key(scm_rdata) > fp32_key(cur_row_max)) begin
          cur_row_max <= scm_rdata;
        end

        if (row_last_d) begin
          row_max_row  <= sc_q_d; // <<< 這列(row)的 index
          row_max_fp32 <= row_start_d ? scm_rdata
                                      : ((fp32_key(scm_rdata) > fp32_key(cur_row_max)) ? scm_rdata : cur_row_max);
          row_max_valid <= 1'b1;
        end
      end
    end
  end

  // top FSM (QKT then scaling)
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
            fsm_start <= 1'b1;
            busy <= 1'b1;
            st <= S_QKT;
          end
        end

        S_QKT: begin
          busy <= 1'b1;
          if (fsm_done) begin
            scale_start <= 1'b1;
            st <= S_SCALE;
          end
        end

        S_SCALE: begin
          busy <= 1'b1;
          if (scale_C_valid) st <= S_DONE;
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

// ============================================================================
// 最終整合 TOP：自動掃描 + submax 計算
// ============================================================================
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

  // one start => 跑完整個 (QKT+scaling+mask+rowmax) + (submax)
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

  // read Y = X - max(row)  (1-cycle)
  input  logic              y_re,
  input  logic [ROW_W-1:0]  y_tq,
  input  logic [COL_W-1:0]  y_tk,
  output logic [DATA_W-1:0] y_rdata,
  output logic              y_rvalid
);

  // ---------------- presoft outputs ----------------
  logic pre_busy, pre_done;

  logic                 sc_re_i;
  logic [ROW_W-1:0]     sc_tq_i;
  logic [COL_W-1:0]     sc_tk_i;
  logic [DATA_W-1:0]    scm_rdata;
  logic                 scm_rvalid;

  logic                 row_max_valid;
  logic [31:0]          row_max_fp32;
  logic [T_W-1:0]       row_max_row;

  attention_presoft_rowmax_idx #(
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

    .sc_re(sc_re_i),
    .sc_tq(sc_tq_i),
    .sc_tk(sc_tk_i),
    .scm_rdata(scm_rdata),
    .scm_rvalid(scm_rvalid),

    .row_max_valid(row_max_valid),
    .row_max_fp32(row_max_fp32),
    .row_max_row(row_max_row)
  );

  // ---------------- submax core ----------------
  logic sub_start, sub_busy, sub_done, sub_c_valid;

  logic              sub_x_we;
  logic [ROW_W-1:0]  sub_x_row;
  logic [COL_W-1:0]  sub_x_col;
  logic [31:0]       sub_x_wdata;

  logic              sub_rm_we;
  logic [ROW_W-1:0]  sub_rm_row;
  logic [31:0]       sub_rm_wdata;

  submax_core_top #(
    .M(T),
    .N(T),
    .DATA_W(32)
  ) u_sub (
    .clk(clk),
    .rst(~rst_n),

    .start(sub_start),
    .busy(sub_busy),
    .done(sub_done),
    .C_valid(sub_c_valid),

    .cpu_x_we   (sub_x_we),
    .cpu_x_row  (sub_x_row),
    .cpu_x_col  (sub_x_col),
    .cpu_x_wdata(sub_x_wdata),
    .cpu_x_wmask({BYTE_W{1'b1}}),

    .cpu_rm_we   (sub_rm_we),
    .cpu_rm_row  (sub_rm_row),
    .cpu_rm_wdata(sub_rm_wdata),
    .cpu_rm_wmask({BYTE_W{1'b1}}),

    .c_rd_en     (1'b1),
    .c_rd_re     (y_re),
    .c_rd_row    (y_tq),
    .c_rd_col    (y_tk),
    .c_rd_rdata  (y_rdata),
    .c_rd_rvalid (y_rvalid)
  );

  // row max write-through
  always_comb begin
    sub_rm_we    = row_max_valid;
    sub_rm_row   = row_max_row;
    sub_rm_wdata = row_max_fp32;
  end

  // ---------------- sweep controller ----------------
  typedef enum logic [2:0] {ST_IDLE, ST_WAIT_PRE, ST_SWEEP, ST_FLUSH, ST_SUB_START, ST_SUB_WAIT, ST_DONE} st_t;
  st_t st;

  logic [ROW_W-1:0] tq_cnt;
  logic [COL_W-1:0] tk_cnt;

  // 1-cycle align capture (because scm_rvalid is 1-cycle after sc_re_i)
  logic req_d;
  logic [ROW_W-1:0] tq_d;
  logic [COL_W-1:0] tk_d;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      req_d <= 1'b0;
      tq_d  <= '0;
      tk_d  <= '0;
    end else begin
      req_d <= sc_re_i;
      if (sc_re_i) begin
        tq_d <= sc_tq_i;
        tk_d <= sc_tk_i;
      end
    end
  end

  // write X when valid comes back
  always_comb begin
    sub_x_we    = (scm_rvalid && req_d);
    sub_x_row   = tq_d;
    sub_x_col   = tk_d;
    sub_x_wdata = scm_rdata;
  end

  // drive presoft scan ports
  always_comb begin
    sc_re_i = 1'b0;
    sc_tq_i = tq_cnt;
    sc_tk_i = tk_cnt;

    sub_start = 1'b0;

    busy = 1'b1;
    done = 1'b0;

    case (st)
      ST_IDLE: begin
        busy = 1'b0;
      end

      ST_WAIT_PRE: begin
        busy = 1'b1;
      end

      ST_SWEEP: begin
        busy = 1'b1;
        sc_re_i = 1'b1; // one request per cycle
      end

      ST_FLUSH: begin
        busy = 1'b1; // wait last return (1 cycle)
      end

      ST_SUB_START: begin
        busy = 1'b1;
        sub_start = 1'b1; // pulse
      end

      ST_SUB_WAIT: begin
        busy = 1'b1;
      end

      ST_DONE: begin
        busy = 1'b0;
        done = 1'b1;
      end

      default: ;
    endcase
  end

  // main FSM
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st     <= ST_IDLE;
      tq_cnt <= '0;
      tk_cnt <= '0;
    end else begin
      case (st)
        ST_IDLE: begin
          tq_cnt <= '0;
          tk_cnt <= '0;
          if (start) st <= ST_WAIT_PRE;
        end

        ST_WAIT_PRE: begin
          // 等 presoft 自己跑完（QKT+scale）
          if (pre_done) begin
            tq_cnt <= '0;
            tk_cnt <= '0;
            st <= ST_SWEEP;
          end
        end

        ST_SWEEP: begin
          // 每 cycle 發 sc_re_i=1，掃 tq,tk
          if (tk_cnt == COL_W'(T-1)) begin
            tk_cnt <= '0;
            if (tq_cnt == ROW_W'(T-1)) begin
              // 最後一筆 request 已送出 -> 下一拍 flush
              st <= ST_FLUSH;
            end else begin
              tq_cnt <= tq_cnt + 1'b1;
            end
          end else begin
            tk_cnt <= tk_cnt + 1'b1;
          end
        end

        ST_FLUSH: begin
          // 等最後一次 scm_rvalid 回來（1-cycle latency）
          st <= ST_SUB_START;
        end

        ST_SUB_START: begin
          // pulse sub_start
          st <= ST_SUB_WAIT;
        end

        ST_SUB_WAIT: begin
          if (sub_done) st <= ST_DONE;
        end

        ST_DONE: begin
          if (!start) st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
