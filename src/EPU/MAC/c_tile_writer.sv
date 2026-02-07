`timescale 1ns/1ps
`default_nettype none

module c_tile_writer_flat #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N)
)(
  input  logic clk,
  input  logic rst,

  // 每次 tile start 清 pending（建議接 start）
  input  logic flush,

  // ---------------- FLAT inputs ----------------
  // idx = i*N + j
  input  logic [M*N*DATA_W-1:0] c_out_flat,
  input  logic [M*N-1:0]        c_valid_flat,

  // to C SRAM write port
  output logic              c_we_en,
  output logic              c_we,
  output logic [ROW_W-1:0]  c_wrow,
  output logic [COL_W-1:0]  c_wcol,
  output logic [DATA_W-1:0] c_wdata,
  output logic [BYTE_W-1:0] c_wmask
);

  // ------------------------------------------------------------
  // Helpers: (i,j) <-> flat index
  // ------------------------------------------------------------
  function automatic int unsigned IJ_IDX(input int unsigned i, input int unsigned j);
    IJ_IDX = i*N + j;
  endfunction

  function automatic logic [DATA_W-1:0] COUT_AT(input int unsigned i, input int unsigned j);
    int unsigned idx;
    begin
      idx = IJ_IDX(i,j);
      COUT_AT = c_out_flat[idx*DATA_W +: DATA_W];
    end
  endfunction

  function automatic logic CVALID_AT(input int unsigned i, input int unsigned j);
    int unsigned idx;
    begin
      idx = IJ_IDX(i,j);
      CVALID_AT = c_valid_flat[idx];
    end
  endfunction

  // ------------------------------------------------------------
  // Pending / latched (same as your original)
  // ------------------------------------------------------------
  logic pending [M][N];
  logic [DATA_W-1:0] latched [M][N];

  logic sel_found;
  int unsigned sel_i, sel_j;

  // latch new valid results + clear after write
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int unsigned i=0;i<M;i++)
        for (int unsigned j=0;j<N;j++) begin
          pending[i][j] <= 1'b0;
          latched[i][j] <= '0;
        end
    end else begin
      if (flush) begin
        for (int unsigned i=0;i<M;i++)
          for (int unsigned j=0;j<N;j++)
            pending[i][j] <= 1'b0;
      end

      // capture any newly-valid outputs (even if many in same cycle)
      for (int unsigned i=0;i<M;i++) begin
        for (int unsigned j=0;j<N;j++) begin
          if (CVALID_AT(i,j)) begin
            pending[i][j] <= 1'b1;
            latched[i][j] <= COUT_AT(i,j);
          end
        end
      end

      // consume the one we write this cycle
      if (c_we_en && c_we && sel_found) begin
        pending[sel_i][sel_j] <= 1'b0;
      end
    end
  end

  // choose first pending in i,j order
  always_comb begin
    sel_found = 1'b0;
    sel_i     = '0;
    sel_j     = '0;

    for (int unsigned i=0;i<M;i++) begin
      for (int unsigned j=0;j<N;j++) begin
        if (!sel_found && pending[i][j]) begin
          sel_found = 1'b1;
          sel_i = i;
          sel_j = j;
        end
      end
    end
  end

  // drive SRAM write
  always_comb begin
    c_we_en = sel_found;
    c_we    = sel_found;
    c_wrow  = ROW_W'(sel_i);
    c_wcol  = COL_W'(sel_j);
    c_wdata = sel_found ? latched[sel_i][sel_j] : '0;
    c_wmask = {BYTE_W{1'b1}};
  end

endmodule

`default_nettype wire
