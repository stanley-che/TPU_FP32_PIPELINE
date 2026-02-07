`include "./src/EPU/attention_score/bank_sram.sv"
`timescale 1ns/1ps

module top_transpose_cpu #(
  parameter integer NRows  = 64,
  parameter integer NCols  = 64,
  parameter integer NB     = 8,
  parameter integer ADDR_W = 16,
  parameter integer Data_W = 32,   // FP32 bits
  parameter integer M      = 6
)(
  input  wire clk,
  input  wire rst_n,

  // command
  input  wire start,
  output reg  done,
  output wire busy,

  // ---------------- CPU write A ----------------
  input  wire                 cpu_a_we,
  input  wire [31:0]          cpu_a_row,
  input  wire [31:0]          cpu_a_col,
  input  wire [Data_W-1:0]    cpu_a_wdata,

  // ---------------- CPU read B -----------------
  input  wire                 cpu_b_re,
  input  wire [31:0]          cpu_b_row,
  input  wire [31:0]          cpu_b_col,
  output reg  [Data_W-1:0]    cpu_b_rdata,
  output reg                  cpu_b_rvalid
);

  // ===== clog2 =====
  function integer clog2; input integer v; integer i; begin
    clog2=0; for (i=v-1;i>0;i=i>>1) clog2=clog2+1;
  end endfunction

  localparam integer ROW_W   = (NRows<=1)?1:clog2(NRows);
  localparam integer COL_W   = (NCols<=1)?1:clog2(NCols);

  // ===== bank_sram ports (shared) =====
  reg                  A_req_v;
  reg                  A_req_we;
  reg  [ADDR_W-1:0]    A_req_addr;
  reg  [Data_W-1:0]    A_req_wData;
  wire                 A_req_ready;
  wire [Data_W-1:0]    A_rsp_rData;
  wire                 A_rsp_v;

  reg                  B_req_v;
  reg                  B_req_we;
  reg  [ADDR_W-1:0]    B_req_addr;
  reg  [Data_W-1:0]    B_req_wData;
  wire                 B_req_ready;
  wire [Data_W-1:0]    B_rsp_rData;
  wire                 B_rsp_v;
  reg        cpu_b_pend;
  reg [ADDR_W-1:0] cpu_b_addr_hold;

  bank_sram #(.NB(NB), .ADDR_W(ADDR_W), .Data_W(Data_W), .M(M)) u_mem_A (
    .clk(clk), .rst_n(rst_n),
    .req_v(A_req_v), .req_we(A_req_we),
    .Req_addr(A_req_addr), .Req_wData(A_req_wData),
    .req_ready(A_req_ready),
    .Rsp_rData(A_rsp_rData),
    .rsp_v(A_rsp_v)
  );

  bank_sram #(.NB(NB), .ADDR_W(ADDR_W), .Data_W(Data_W), .M(M)) u_mem_B (
    .clk(clk), .rst_n(rst_n),
    .req_v(B_req_v), .req_we(B_req_we),
    .Req_addr(B_req_addr), .Req_wData(B_req_wData),
    .req_ready(B_req_ready),
    .Rsp_rData(B_rsp_rData),
    .rsp_v(B_rsp_v)
  );

  // ===== address helpers =====
  function [ADDR_W-1:0] lin_addr_A;
    input [31:0] row, col; reg [ADDR_W-1:0] t;
    begin
      t = $unsigned(row) * NCols[ADDR_W-1:0];
      lin_addr_A = t + $unsigned(col);
    end
  endfunction

  function [ADDR_W-1:0] lin_addr_B;
    input [31:0] rowp, colp; reg [ADDR_W-1:0] t;
    begin
      t = $unsigned(rowp) * NRows[ADDR_W-1:0];
      lin_addr_B = t + $unsigned(colp);
    end
  endfunction

  // =========================================================
  // Transpose engine FSM (same as your top_transpose)
  // BUT: fix write-data alignment: use "a_data_hold" captured on rsp_v
  // =========================================================
  localparam [2:0] S_IDLE=3'd0, S_ISSUE_RD=3'd1, S_WAIT_RD=3'd2,
                   S_ISSUE_WR=3'd3, S_NEXT=3'd4, S_DONE=3'd5;

  reg [2:0] state_now, state_next;
  reg [ROW_W-1:0] row_now, row_next;
  reg [COL_W-1:0] col_now, col_next;

  localparam integer TOTAL_PIX = NRows * NCols;
  localparam integer CNT_W     = (TOTAL_PIX<=1)?1:clog2(TOTAL_PIX)+1;
  reg [CNT_W-1:0] pix_cnt, pix_cnt_next;

  reg [Data_W-1:0] a_data_hold;

  assign busy = (state_now != S_IDLE);

  // ---------------- CPU B read 2-phase ----------------
  localparam [1:0] CR_IDLE=2'd0, CR_WAIT=2'd1;
  reg [1:0] cr_state;
  reg [ADDR_W-1:0] cpu_b_addr_q;

  // ---------------- combinational ----------------
  always @* begin
    // defaults
    A_req_v     = 1'b0; A_req_we    = 1'b0; A_req_addr  = {ADDR_W{1'b0}}; A_req_wData = {Data_W{1'b0}};
    B_req_v     = 1'b0; B_req_we    = 1'b0; B_req_addr  = {ADDR_W{1'b0}}; B_req_wData = {Data_W{1'b0}};
    done        = 1'b0;

    state_next   = state_now;
    row_next     = row_now;
    col_next     = col_now;
    pix_cnt_next = pix_cnt;

    // -------- ENGINE OWNS MEM WHEN busy --------
    if (busy) begin
      case (state_now)
        S_IDLE: begin
          // should not happen (busy=0)
          state_next = S_IDLE;
        end

        S_ISSUE_RD: begin
          A_req_v    = 1'b1;
          A_req_we   = 1'b0;
          A_req_addr = lin_addr_A(row_now, col_now);
          if (A_req_ready) state_next = S_WAIT_RD;
        end

        S_WAIT_RD: begin
          if (A_rsp_v) state_next = S_ISSUE_WR;
        end

        S_ISSUE_WR: begin
          B_req_v     = 1'b1;
          B_req_we    = 1'b1;
          B_req_addr  = lin_addr_B(col_now, row_now);
          B_req_wData = a_data_hold;   // ✅ use captured A data
          if (B_req_ready) begin
            pix_cnt_next = pix_cnt + 1'b1;
            state_next   = S_NEXT;
          end
        end

        S_NEXT: begin
          if (pix_cnt_next < TOTAL_PIX) begin
            if ($unsigned(col_now) + 1 < NCols) begin
              col_next   = col_now + 1'b1;
              state_next = S_ISSUE_RD;
            end else begin
              col_next   = {COL_W{1'b0}};
              if ($unsigned(row_now) + 1 < NRows) begin
                row_next   = row_now + 1'b1;
                state_next = S_ISSUE_RD;
              end else begin
                state_next = S_DONE;
              end
            end
          end else begin
            state_next = S_DONE;
          end
        end

        S_DONE: begin
          done       = 1'b1;
          state_next = S_IDLE;
        end

        default: state_next = S_IDLE;
      endcase

    end else begin
      // -------- NOT busy: CPU may access memories --------

      // CPU write A
      if (cpu_a_we) begin
        A_req_v     = 1'b1;
        A_req_we    = 1'b1;
        A_req_addr  = lin_addr_A(cpu_a_row, cpu_a_col);
        A_req_wData = cpu_a_wdata;
      end

      // CPU read B (issue request; response handled in sequential below)
      if ((cr_state == CR_IDLE) && cpu_b_pend) begin
        B_req_v    = 1'b1;
        B_req_we   = 1'b0;
        B_req_addr = cpu_b_addr_hold;
      end
    end
  end

  // ---------------- sequential ----------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_now   <= S_IDLE;
      row_now     <= {ROW_W{1'b0}};
      col_now     <= {COL_W{1'b0}};
      pix_cnt     <= {CNT_W{1'b0}};
      a_data_hold <= {Data_W{1'b0}};
      cr_state    <= CR_IDLE;
      cpu_b_rdata <= {Data_W{1'b0}};
      cpu_b_addr_q<= {ADDR_W{1'b0}};
      cpu_b_rvalid<= 1'b0;
      cpu_b_pend      <= 1'b0;
      cpu_b_addr_hold <= {ADDR_W{1'b0}};

    end else begin
      cpu_b_rvalid <= 1'b0;

      // engine start
      if (state_now == S_IDLE) begin
        if (start) begin
          state_now <= S_ISSUE_RD;
          row_now   <= {ROW_W{1'b0}};
          col_now   <= {COL_W{1'b0}};
          pix_cnt   <= {CNT_W{1'b0}};
        end
      end else begin
        state_now <= state_next;
        row_now   <= row_next;
        col_now   <= col_next;
        pix_cnt   <= pix_cnt_next;
      end

      // capture A read data on rsp_v
      if (A_rsp_v) a_data_hold <= A_rsp_rData;

      // CPU B read state machine (only valid when !busy)
      if (!busy) begin
  case (cr_state)
    CR_IDLE: begin
      // latch a request when cpu_b_re pulses
      if (cpu_b_re) begin
        cpu_b_pend      <= 1'b1;
        cpu_b_addr_hold <= lin_addr_B(cpu_b_row, cpu_b_col);
      end

      // if pending, wait until memory accepts
      if (cpu_b_pend && B_req_ready) begin
        cr_state   <= CR_WAIT;
        cpu_b_pend <= 1'b0;  // consumed
      end
    end

    CR_WAIT: begin
      if (B_rsp_v) begin
        cpu_b_rdata  <= B_rsp_rData;
        cpu_b_rvalid <= 1'b1;
        cr_state     <= CR_IDLE;
      end
    end

    default: cr_state <= CR_IDLE;
  endcase
end else begin
  cr_state   <= CR_IDLE;
  cpu_b_pend <= 1'b0;
end

    end
  end

endmodule

`default_nettype wire
