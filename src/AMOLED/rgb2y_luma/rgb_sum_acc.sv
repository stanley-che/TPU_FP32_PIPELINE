// rgb_sum_acc.sv
// Sum accumulator for RGB->Y partial terms
// sum = r_term + g_term + b_term
// max = 255*(77+150+29)=65280 -> needs 17 bits

`ifndef RGB_SUM_ACC_SV
`define RGB_SUM_ACC_SV

`timescale 1ns/1ps
`default_nettype none

module rgb_sum_acc #(
  parameter int unsigned LAT = 1,                 // pipeline latency for v3_valid/sum
  parameter bit          ZERO_WHEN_INVALID = 1'b1, // invalid => force sum/y/ovf to 0
  parameter bit          HOLD_VALID_WHEN_STALL = 1'b1, // en=0: 1=hold valid, 0=force v3_valid=0

  // Optional normalize/clip to y_out
  parameter int unsigned SHIFT = 8,    // y = (sum + bias) >> SHIFT
  parameter bit          ROUND = 1'b1, // add rounding bias before shift
  parameter int unsigned YW = 8,       // y_out width (8~10 typical)
  parameter bit          SATURATE = 1'b1 // after shift, clamp to max of YW bits
)(
  input  logic        clk,
  input  logic        rst,

  input  logic        en,        // clock enable: 1=advance, 0=hold (LAT>0). LAT==0: acts as output gate.
  input  logic        v2_valid,

  input  logic [15:0] r_term,
  input  logic [15:0] g_term,
  input  logic [15:0] b_term,

  output logic        v3_valid,
  output logic [16:0] sum,        // sum17 (debug / downstream)
  output logic [YW-1:0] y_out,    // optional normalized/clipped output
  output logic        ovf         // sum0 > SUM_MAX (flag)
);

  // ------------------------------------------------------------
  // constants
  // ------------------------------------------------------------
  localparam logic [16:0] SUM_MAX = 17'd65280;

  // rounding bias for right shift
  localparam int unsigned BIAS_VAL = (ROUND && (SHIFT > 0)) ? (1 << (SHIFT-1)) : 0;

  // ------------------------------------------------------------
  // stage0 combinational add (17-bit) + ovf detect
  // ------------------------------------------------------------
  logic [16:0] sum0;
  logic        ovf0;

  always @* begin
    // cast to 17b to avoid overflow truncation
    sum0 = {1'b0, r_term} + {1'b0, g_term} + {1'b0, b_term};
    ovf0 = (sum0 > SUM_MAX);
  end

  // ------------------------------------------------------------
  // helper: compute y from sum17
  // ------------------------------------------------------------
  function automatic [YW-1:0] f_y_from_sum(input logic [16:0] s17);
    logic [17:0] s18_bias;
    logic [17:0] shifted;
    logic [YW-1:0] ysat;
    begin
      s18_bias = {1'b0, s17} + BIAS_VAL[17:0];

      if (SHIFT >= 18) begin
        shifted = 18'd0;
      end else begin
        shifted = (s18_bias >> SHIFT);
      end

      if (SATURATE) begin
        if (shifted > {{(18-YW){1'b0}}, {YW{1'b1}}})
          ysat = {YW{1'b1}};
        else
          ysat = shifted[YW-1:0];
      end else begin
        ysat = shifted[YW-1:0];
      end

      f_y_from_sum = ysat;
    end
  endfunction

  // ------------------------------------------------------------
  // pipeline (LAT) with enable + invalid zeroing + stall valid policy
  // ------------------------------------------------------------
  generate
    if (LAT == 0) begin : G_LAT0
      // NOTE: LAT==0 is purely combinational; "hold state" on en=0 is not possible.
      // We treat en=0 as an output gate: force v3_valid=0 and (optionally) outputs to 0.
      always @* begin
        if (!en) begin
          v3_valid = 1'b0;
          sum      = 17'h0;
          y_out    = {YW{1'b0}};
          ovf      = 1'b0;
        end else begin
          v3_valid = v2_valid;

          if (ZERO_WHEN_INVALID && !v2_valid) begin
            sum   = 17'h0;
            y_out = {YW{1'b0}};
            ovf   = 1'b0;
          end else begin
            sum   = sum0;
            y_out = f_y_from_sum(sum0);
            ovf   = ovf0;
          end
        end
      end

    end else begin : G_LATN
      integer k;

      logic [LAT:0] v_sh;
      logic [16:0]  s_pipe   [0:LAT];
      logic         ovf_pipe [0:LAT];

      always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
          v_sh <= '0;
          for (k = 0; k <= LAT; k = k + 1) begin
            s_pipe[k]   <= 17'h0;
            ovf_pipe[k] <= 1'b0;
          end
        end else if (en) begin
          // stage 0 load
          v_sh[0] <= v2_valid;

          if (ZERO_WHEN_INVALID && !v2_valid) begin
            s_pipe[0]   <= 17'h0;
            ovf_pipe[0] <= 1'b0;
          end else begin
            s_pipe[0]   <= sum0;
            ovf_pipe[0] <= ovf0;
          end

          // shift registers
          for (k = 1; k <= LAT; k = k + 1) begin
            v_sh[k]     <= v_sh[k-1];
            s_pipe[k]   <= s_pipe[k-1];
            ovf_pipe[k] <= ovf_pipe[k-1];
          end
        end
        // en==0: hold state
      end

      // output policy when stalled:
      // - HOLD_VALID_WHEN_STALL=1: v3_valid reflects held pipeline state
      // - HOLD_VALID_WHEN_STALL=0: force v3_valid=0 while en=0 (data still held)
      wire v3_valid_w;
      assign v3_valid_w = (HOLD_VALID_WHEN_STALL) ? v_sh[LAT]
                                            : (en ? v_sh[LAT] : 1'b0);

      assign v3_valid = v3_valid_w;
      assign sum      = s_pipe[LAT];
      assign y_out    = f_y_from_sum(s_pipe[LAT]);
      assign ovf      = ovf_pipe[LAT];

    end
  endgenerate

endmodule

`default_nettype wire
`endif
