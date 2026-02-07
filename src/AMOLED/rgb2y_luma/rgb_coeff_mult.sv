// rgb_coeff_mult.sv
// Fixed-coefficient multiply for RGB->Y partial terms
// r_term = 77*R, g_term = 150*G, b_term = 29*B
// + en clock-enable: en=0 -> hold pipeline (valid and data hold)
// + BYPASS: bypass multiplication for debug (pass-through 8-bit to 16-bit)

`ifndef RGB_COEFF_MULT_SV
`define RGB_COEFF_MULT_SV

`timescale 1ns/1ps
`default_nettype none

module rgb_coeff_mult #(
  parameter int unsigned LAT = 1,          // pipeline latency for outputs/valid
  parameter bit USE_DSP = 1'b0,            // 0: shift-add, 1: use '*'
  parameter bit BYPASS  = 1'b0             // 1: bypass mult (debug)
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        en,                  // clock enable: 1=advance, 0=hold

  input  logic        v1_valid,
  input  logic [7:0]  r8,
  input  logic [7:0]  g8,
  input  logic [7:0]  b8,

  output logic        v2_valid,
  output logic [15:0] r_term,
  output logic [15:0] g_term,
  output logic [15:0] b_term
);

  // -----------------------------
  // stage0: compute combinational partials
  // -----------------------------
  logic [15:0] r0, g0, b0;

  // shift-add helpers (8-bit -> 16-bit)
  function automatic [15:0] mul_77(input [7:0] x);
    logic [15:0] xx;
    begin
      xx = {8'h00, x};
      // 77 = 64 + 8 + 4 + 1
      mul_77 = (xx << 6) + (xx << 3) + (xx << 2) + xx;
    end
  endfunction

  function automatic [15:0] mul_150(input [7:0] x);
    logic [15:0] xx;
    begin
      xx = {8'h00, x};
      // 150 = 128 + 16 + 4 + 2
      mul_150 = (xx << 7) + (xx << 4) + (xx << 2) + (xx << 1);
    end
  endfunction

  function automatic [15:0] mul_29(input [7:0] x);
    logic [15:0] xx;
    begin
      xx = {8'h00, x};
      // 29 = 16 + 8 + 4 + 1
      mul_29 = (xx << 4) + (xx << 3) + (xx << 2) + xx;
    end
  endfunction

  always @* begin
    // default
    r0 = 16'h0000;
    g0 = 16'h0000;
    b0 = 16'h0000;

    if (BYPASS) begin
      // debug bypass: pass-through 8-bit into 16-bit
      r0 = {8'h00, r8};
      g0 = {8'h00, g8};
      b0 = {8'h00, b8};
    end else begin
      if (USE_DSP) begin
        r0 = r8 * 16'd77;
        g0 = g8 * 16'd150;
        b0 = b8 * 16'd29;
      end else begin
        r0 = mul_77(r8);
        g0 = mul_150(g8);
        b0 = mul_29(b8);
      end
    end
  end

  // -----------------------------
  // valid+data pipeline (LAT) with enable
  // -----------------------------
  generate
    if (LAT == 0) begin : G_LAT0
      // LAT0: purely combinational; en has no effect on combinational output
      always @* begin
        v2_valid = v1_valid;
        r_term   = r0;
        g_term   = g0;
        b_term   = b0;
      end
    end else begin : G_LATN
      integer k;

      logic [LAT:0] v_sh;
      logic [15:0]  r_pipe [0:LAT];
      logic [15:0]  g_pipe [0:LAT];
      logic [15:0]  b_pipe [0:LAT];

      always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
          v_sh <= '0;
          for (k = 0; k <= LAT; k = k + 1) begin
            r_pipe[k] <= 16'h0000;
            g_pipe[k] <= 16'h0000;
            b_pipe[k] <= 16'h0000;
          end
        end else if (en) begin
          // advance pipeline only when en=1
          v_sh[0]   <= v1_valid;
          r_pipe[0] <= r0;
          g_pipe[0] <= g0;
          b_pipe[0] <= b0;

          for (k = 1; k <= LAT; k = k + 1) begin
            v_sh[k]   <= v_sh[k-1];
            r_pipe[k] <= r_pipe[k-1];
            g_pipe[k] <= g_pipe[k-1];
            b_pipe[k] <= b_pipe[k-1];
          end
        end
        // else en==0: hold state (do nothing)
      end

      assign v2_valid = v_sh[LAT];
      assign r_term   = r_pipe[LAT];
      assign g_term   = g_pipe[LAT];
      assign b_term   = b_pipe[LAT];

    end
  endgenerate

endmodule

`default_nettype wire
`endif
