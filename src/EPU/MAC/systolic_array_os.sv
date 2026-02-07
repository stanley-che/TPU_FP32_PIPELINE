`include "./src/EPU/MAC/pe.sv"
`timescale 1ns/1ps
`default_nettype none

module systolic_array_os_flat #(
  parameter int M = 8,
  parameter int N = 8
)(
  input  logic clk,
  input  logic rst,

  input  logic              step_valid,
  input  logic [M*32-1:0]   a_row_flat,   // A[:,k] flatten
  input  logic [N*32-1:0]   b_col_flat,   // B[k,:] flatten
  input  logic              k_first,
  input  logic              k_last,

  output logic              step_ready,

  output logic [M*N*32-1:0] c_out_flat,
  output logic [M*N-1:0]    c_valid_flat,

  // debug
  output logic [M*N*32-1:0] psum_out_flat
);

  // ============================================================
  // Internal storage (OS)
  // ============================================================
  logic [31:0] psum_reg   [M][N];
  logic        psum_valid [M][N];

  // Latched inputs per step
  logic [31:0] a_lat [M];
  logic [31:0] b_lat [N];
  logic        k_last_lat;

  // PE connections
  logic        pe_in_valid;
  logic [31:0] pe_psum_in   [M][N];
  logic        pe_out_valid [M][N];
  logic [31:0] pe_psum_out  [M][N];

  // done tracking
  logic pe_done_seen [M][N];

  // ============================================================
  // FSM
  // ============================================================
  typedef enum logic [1:0] {S_IDLE, S_LAUNCH, S_WAIT, S_CVALID} state_t;
  state_t st;

  assign step_ready = (st == S_IDLE);

  // flat index helper: idx = i*N + j
  function automatic int idx(input int i, input int j);
    return i*N + j;
  endfunction

  // ============================================================
  // PSUM mux  (✅ FIXED: per-PE psum_valid)
  // ============================================================
  genvar gi, gj;
  generate
    for (gi = 0; gi < M; gi++) begin : G_PSUM_MUX_ROW
      for (gj = 0; gj < N; gj++) begin : G_PSUM_MUX_COL
        always_comb begin
          pe_psum_in[gi][gj] =
            (psum_valid[gi][gj]) ? psum_reg[gi][gj] : 32'h00000000;
        end
      end
    end
  endgenerate

  // ============================================================
  // PEs
  // ============================================================
  generate
    for (gi = 0; gi < M; gi++) begin : G_ROW
      for (gj = 0; gj < N; gj++) begin : G_COL
        pe u_pe (
          .clk      (clk),
          .rst      (rst),
          .in_valid (pe_in_valid),
          .a_bits   (a_lat[gi]),
          .b_bits   (b_lat[gj]),
          .psum_in  (pe_psum_in[gi][gj]),
          .out_valid(pe_out_valid[gi][gj]),
          .psum_out (pe_psum_out[gi][gj])
        );
      end
    end
  endgenerate

  function automatic bit all_done_seen();
    int i, j;
    begin
      all_done_seen = 1'b1;
      for (i = 0; i < M; i++)
        for (j = 0; j < N; j++)
          if (!pe_done_seen[i][j]) all_done_seen = 1'b0;
    end
  endfunction

  // ============================================================
  // Sequential
  // ============================================================
  int i, j;

  always_ff @(posedge clk) begin
    if (rst) begin
      st          <= S_IDLE;
      pe_in_valid <= 1'b0;
      k_last_lat  <= 1'b0;

      for (i = 0; i < M; i++) begin
        a_lat[i] <= 32'h0;
        for (j = 0; j < N; j++) begin
          psum_reg[i][j]   <= 32'h0;
          psum_valid[i][j] <= 1'b0;
          pe_done_seen[i][j] <= 1'b0;
        end
      end
      for (j = 0; j < N; j++) b_lat[j] <= 32'h0;

      psum_out_flat <= '0;
      c_out_flat    <= '0;
      c_valid_flat  <= '0;

    end else begin
      pe_in_valid  <= 1'b0;
      c_valid_flat <= '0;   // pulse

      // capture PE outputs
      for (i = 0; i < M; i++) begin
        for (j = 0; j < N; j++) begin
          if (pe_out_valid[i][j]) begin
            psum_reg[i][j]   <= pe_psum_out[i][j];
            psum_out_flat[idx(i,j)*32 +: 32] <= pe_psum_out[i][j];
            psum_valid[i][j] <= 1'b1;
            pe_done_seen[i][j] <= 1'b1;
          end
        end
      end

      case (st)
        // ------------------------------------------------------
        S_IDLE: begin
  if (step_valid) begin
    for (i = 0; i < M; i++)
      a_lat[i] <= a_row_flat[i*32 +: 32];
    for (j = 0; j < N; j++)
      b_lat[j] <= b_col_flat[j*32 +: 32];

    k_last_lat <= k_last;

    // ⭐ 關鍵修正：新一輪 K 累加，清掉 psum_valid
    if (k_first) begin
      for (i = 0; i < M; i++)
        for (j = 0; j < N; j++)
          psum_valid[i][j] <= 1'b0;
    end

    for (i = 0; i < M; i++)
      for (j = 0; j < N; j++)
        pe_done_seen[i][j] <= 1'b0;

    st <= S_LAUNCH;
  end
end


        // ------------------------------------------------------
        S_LAUNCH: begin
          pe_in_valid <= 1'b1;
          st <= S_WAIT;
        end

        // ------------------------------------------------------
        S_WAIT: begin
          if (all_done_seen()) begin
            if (k_last_lat) begin
              for (i = 0; i < M; i++)
                for (j = 0; j < N; j++)
                  c_out_flat[idx(i,j)*32 +: 32] <= psum_reg[i][j];
              st <= S_CVALID;
            end else begin
              st <= S_IDLE;
            end
          end
        end

        // ------------------------------------------------------
        S_CVALID: begin
          for (i = 0; i < M; i++)
            for (j = 0; j < N; j++)
              c_valid_flat[idx(i,j)] <= 1'b1;
          st <= S_IDLE;
        end
      endcase
    end
  end

endmodule

`default_nettype wire
