// rgb_round_shift.sv
// Rounding + right shift stage for RGB->Y (Q8 -> integer)
//
// Features added:
// 1) LAT==0 respects en (implemented via 1-stage output register hold)
// 2) Saturation/clamp: y10 in [0..1023], y8 depending on mode
// 3) Selectable rounding modes
// 4) Selectable y8 generation modes (from y10 >> 2, or clamp to 255)
// 5) Debug: ovf flag indicates saturation occurred
// 6) Stream handshake: in_ready/out_ready (AXI-stream-like simplified)
//
// Notes:
// - For hold correctness, output is registered even when LAT==0.
//   i.e., effective latency is max(LAT,1).

`ifndef RGB_ROUND_SHIFT_SV
`define RGB_ROUND_SHIFT_SV

`timescale 1ns/1ps
`default_nettype none

module rgb_round_shift #(
  parameter int unsigned LAT = 1,                  // pipeline latency stages (effective >=1)
  parameter bit          ZERO_WHEN_INVALID = 1'b1,  // invalid => force output 0 (and ovf=0)

  // rounding modes on division by 256 (>>8)
  // 0: TRUNCATE        : floor(sum/256)
  // 1: ROUND_NEAREST   : r>128 => +1; r==128 => +1 (ties away from zero)
  // 2: ROUND_AWAY      : r>=128 => +1
  // 3: ROUND_EVEN      : r>128 => +1; r==128 => +1 if q is odd (banker's rounding)
  parameter int unsigned ROUND_MODE = 1,

  // y8 generation from clamped y10:
  // 0: y8 = y10[9:2] (10-bit -> 8-bit by >>2)   [common for 10->8]
  // 1: y8 = min(y10, 255) (clamp to 8-bit range) [if you want strict 0..255]
  parameter int unsigned Y8_MODE = 0
)(
  input  logic        clk,
  input  logic        rst,

  // global clock-enable (pipeline advances only when en && can_accept)
  input  logic        en,

  // input
  input  logic        v3_valid,
  input  logic [16:0] sum,

  // downstream backpressure
  input  logic        out_ready,
  output logic        in_ready,

  // output
  output logic        v4_valid,
  output logic [9:0]  y10,
  output logic [7:0]  y8,
  output logic        ovf
);

  // effective pipeline depth (>=1 so LAT==0 can hold)
  localparam int unsigned DEPTH = (LAT == 0) ? 1 : LAT;

  // ------------------------------------------------------------
  // rounding core (combinational)
  // ------------------------------------------------------------
  logic [8:0]  r;          // remainder bits [7:0] plus guard
  logic [9:0]  q;          // quotient candidate (needs extra headroom)
  logic        inc;        // rounding increment
  logic [10:0] q_inc;      // q + inc (11-bit to detect overflow)
  logic [10:0] y10_ext;    // extended before clamp

  always @* begin
    // q = sum >> 8, r = sum[7:0]
    q = {3'b000, sum[16:8]};  // sum is 17b -> q max 511
    r = {1'b0, sum[7:0]};

    // default
    inc = 1'b0;

    // rounding decision based on ROUND_MODE
    // r compares to 128 (0x80)
    case (ROUND_MODE)
      0: begin
        // TRUNCATE: inc = 0
        inc = 1'b0;
      end
      1: begin
        // ROUND_NEAREST (ties away): r > 128 OR r == 128
        inc = (r[7:0] >= 8'h80);
      end
      2: begin
        // ROUND_AWAY: r >= 128 (same as above for unsigned, kept separate for clarity)
        inc = (r[7:0] >= 8'h80);
      end
      3: begin
        // ROUND_EVEN: r > 128 OR (r == 128 AND q is odd)
        inc = (r[7:0] > 8'h80) | ((r[7:0] == 8'h80) & q[0]);
      end
      default: inc = 1'b0;
    endcase

    q_inc   = {1'b0, q} + {{10{1'b0}}, inc}; // 11-bit
    y10_ext = q_inc; // this is the (rounded) result before clamp
  end

  // ------------------------------------------------------------
  // clamp + y8 generation (combinational)
  // ------------------------------------------------------------
  logic [9:0] y10_c;
  logic [7:0] y8_c;
  logic       ovf_c;

  always @* begin
    // y10 clamp to 0..1023 (10'h3FF)
    if (y10_ext[10] == 1'b1) begin
      // overflow beyond 10-bit
      y10_c = 10'h3FF;
      ovf_c = 1'b1;
    end else begin
      y10_c = y10_ext[9:0];
      ovf_c = 1'b0;
    end

    // y8 from y10 (after y10 clamp)
    case (Y8_MODE)
      0: begin
        // 10-bit -> 8-bit by >>2
        y8_c = y10_c[9:2];
        // no extra saturation needed here (already bounded)
      end
      1: begin
        // clamp to 255
        if (y10_c > 10'd255) begin
          y8_c = 8'hFF;
          ovf_c = 1'b1; // mark saturation too
        end else begin
          y8_c = y10_c[7:0];
        end
      end
      default: begin
        y8_c = y10_c[9:2];
      end
    endcase
  end

  // ------------------------------------------------------------
  // pipeline with stall (out_ready) and clock-enable (en)
  // ------------------------------------------------------------
  integer k;

  logic [DEPTH:0] v_sh;
  logic [9:0]     y10_pipe [0:DEPTH];
  logic [7:0]     y8_pipe  [0:DEPTH];
  logic           ovf_pipe [0:DEPTH];

  // can_accept means: pipeline can advance this cycle
  // - if output stage is invalid => can accept regardless of out_ready
  // - if output stage valid => need out_ready to move forward (consume)
  logic can_accept;
  always @* begin
    can_accept = (out_ready || !v_sh[DEPTH]);
    in_ready   = can_accept;
  end

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      v_sh <= '0;
      for (k = 0; k <= DEPTH; k = k + 1) begin
        y10_pipe[k] <= 10'h000;
        y8_pipe[k]  <= 8'h00;
        ovf_pipe[k] <= 1'b0;
      end
    end else if (en && can_accept) begin
      // stage0 load
      v_sh[0] <= v3_valid;

      if (ZERO_WHEN_INVALID && !v3_valid) begin
        y10_pipe[0] <= 10'h000;
        y8_pipe[0]  <= 8'h00;
        ovf_pipe[0] <= 1'b0;
      end else begin
        y10_pipe[0] <= y10_c;
        y8_pipe[0]  <= y8_c;
        ovf_pipe[0] <= ovf_c;
      end

      // shift
      for (k = 1; k <= DEPTH; k = k + 1) begin
        v_sh[k]      <= v_sh[k-1];
        y10_pipe[k]  <= y10_pipe[k-1];
        y8_pipe[k]   <= y8_pipe[k-1];
        ovf_pipe[k]  <= ovf_pipe[k-1];
      end
    end
    // else: hold all regs (stall) when !en or !can_accept
  end

  // outputs
  assign v4_valid = v_sh[DEPTH];
assign y10      = y10_pipe[DEPTH];
assign y8       = y8_pipe[DEPTH];
assign ovf      = ovf_pipe[DEPTH];


endmodule

`default_nettype wire
`endif
