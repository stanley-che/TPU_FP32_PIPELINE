// ============================================================================
// attention_presoft_submax_exp_sum_top.sv
// 功能：
//  1) 跑 attention_presoft_submax_exp_top：得到 E = exp(Y) (FP32) in E SRAM
//  2) 掃描每個 row：Sum[row] = Σ_k E[row][k]   (FP32 adder)
//  3) 對外提供 Sum[row] 讀取 (1-cycle latency)
// ============================================================================
`include "./src/EPU/attention_score/attention_presoft_submax_exp_top.sv"
`include "./src/EPU/attention_score/fp_adder_driver_ba.sv"
`timescale 1ns/1ps
`default_nettype none



module attention_presoft_submax_exp_sum_top #(
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

  // ---------------- Sum read: Sum[row] (1-cycle) --------------------------
  input  logic              sum_re,
  input  logic [ROW_W-1:0]  sum_row,
  output logic [DATA_W-1:0] sum_rdata,
  output logic              sum_rvalid
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

    // merge external debug read and internal scan read by mux below
    .e_re(e_re_i),
    .e_tq(e_tq_i),
    .e_tk(e_tk_i),
    .e_rdata(e_rdata_i),
    .e_rvalid(e_rvalid_i)
  );

  // 外部 debug 讀 E：直接 passthrough core 的 read port（但要 mux）
  // 規則：SUM 計算期間我們用 internal scan；其他時間可用外部輸入
  // 若你不需要外部 debug，這段可以更簡單（永遠用 internal scan）
  // 這裡採用：SUM state != IDLE 時 internal 優先
  // --------------------------------------------------------------------------
  // 先宣告 state，後面再 assign mux
  typedef enum logic [2:0] {S_IDLE, S_WAIT_CORE, S_ISSUE_E, S_WAIT_E, S_ADD, S_WRITE, S_DONE} sumst_t;
  sumst_t sst;

  logic use_internal_e;
  always_comb use_internal_e = (sst != S_IDLE) && (sst != S_DONE);

  always_comb begin
    if (use_internal_e) begin
      // internal scan drives core
      e_re_i = e_re_scan;
      e_tq_i = e_tq_scan;
      e_tk_i = e_tk_scan;
    end else begin
      // external drives core
      e_re_i = e_re;
      e_tq_i = e_tq;
      e_tk_i = e_tk;
    end
  end

  // external read outputs are always whatever core returns
  assign e_rdata  = e_rdata_i;
  assign e_rvalid = e_rvalid_i;

  // ==========================================================================
  // 2) SUM memory
  // ==========================================================================
  logic [31:0] Sum_mem [0:T-1];

  // sum read (1-cycle)
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

  // ==========================================================================
  // 3) FP adder (accumulate)
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
  // 4) SUM scan FSM: one request, wait rvalid, then add, write back at row end
  // ==========================================================================
  logic [ROW_W-1:0] rq;
  logic [COL_W-1:0] ck;

  logic [31:0] acc;        // current row accumulator (FP32 bits)
  logic [31:0] e_hold;     // latch E read data for add
  logic [ROW_W-1:0] rq_lat;
  logic [COL_W-1:0] ck_lat;

  // internal scan signals to core mux
  logic             e_re_scan;
  logic [ROW_W-1:0] e_tq_scan;
  logic [COL_W-1:0] e_tk_scan;

  // defaults
  // combinational decode (NO assignment in always_ff for these)
  assign e_re_scan   = (sst == S_ISSUE_E);   // 這樣就是一拍 pulse
  assign e_tq_scan   = rq;
  assign e_tk_scan   = ck;

  assign add_start_i = (sst == S_ADD);       // 這樣就是一拍 pulse
  assign add_a_i     = acc;
  assign add_b_i     = e_hold;

  assign busy = (sst != S_IDLE) && (sst != S_DONE);
  assign done = (sst == S_DONE);


  // NOTE: 初始化 acc=0
  localparam logic [31:0] FP32_PZERO = 32'h00000000;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      sst    <= S_IDLE;
      rq     <= '0;
      ck     <= '0;
      acc    <= FP32_PZERO;
      e_hold <= 32'h0;
      rq_lat <= '0;
      ck_lat <= '0;
      for (int r=0; r<T; r++) Sum_mem[r] <= FP32_PZERO;
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
          // pulse one E read request
          rq_lat <= rq;
          ck_lat <= ck;
          sst <= S_WAIT_E;
        end

        S_WAIT_E: begin
          if (e_rvalid_i) begin
            e_hold <= e_rdata_i;
            sst <= S_ADD;
          end
        end

        S_ADD: begin
          // acc = acc + e_hold
          sst <= S_WRITE;
        end

        S_WRITE: begin
          // wait adder done
          if (add_done_i) begin
            acc <= add_z_i;

            if (ck_lat == COL_W'(T-1)) begin
              // row finished: write Sum_mem[row] = acc_new
              Sum_mem[rq_lat] <= add_z_i;

              // next row or done
              if (rq_lat == ROW_W'(T-1)) begin
                sst <= S_DONE;
              end else begin
                rq  <= rq_lat + 1'b1;
                ck  <= '0;
                acc <= FP32_PZERO;
                sst <= S_ISSUE_E;
              end
            end else begin
              // next col
              ck <= ck_lat + 1'b1;
              sst <= S_ISSUE_E;
            end
          end
        end

        S_DONE: begin
          if (!start) sst <= S_IDLE;
        end

        default: sst <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
