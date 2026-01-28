// rgb_unpack_clip.sv
// iverilog-safe RGB unpack + clip/expand (no variable part-select)
`ifndef RGB_UNPACK_CLIP_SV
`define RGB_UNPACK_CLIP_SV

`timescale 1ns/1ps
`default_nettype none

module rgb_unpack_clip #(
  parameter int unsigned FORMAT = 0,   // 0:RGB888 1:RGB101010 2:RGB121212 3:RGB565
  parameter int unsigned LAT    = 1,   // valid/data delayed by LAT cycles
  parameter bit ZERO_WHEN_INVALID = 1'b1
)(
  input  logic clk,
  input  logic rst,

  input  logic pix_valid,
  input  logic [35:0] rgb_in,

  output logic v1_valid,
  output logic [7:0] r8,
  output logic [7:0] g8,
  output logic [7:0] b8
);

  localparam int unsigned F_RGB888    = 0;
  localparam int unsigned F_RGB101010 = 1;
  localparam int unsigned F_RGB121212 = 2;
  localparam int unsigned F_RGB565    = 3;

  logic [7:0] r_c, g_c, b_c;

  function automatic [7:0] exp5_to_8(input [4:0] x);
    begin exp5_to_8 = {x, x[4:2]}; end
  endfunction

  function automatic [7:0] exp6_to_8(input [5:0] x);
    begin exp6_to_8 = {x, x[5:4]}; end
  endfunction

  function automatic [7:0] msb10_to_8(input [9:0] x);
    begin msb10_to_8 = x[9:2]; end
  endfunction

  function automatic [7:0] msb12_to_8(input [11:0] x);
    begin msb12_to_8 = x[11:4]; end
  endfunction

  always @* begin
    r_c = 8'h00; g_c = 8'h00; b_c = 8'h00;

    if (FORMAT == F_RGB888) begin
      r_c = rgb_in[23:16];
      g_c = rgb_in[15:8];
      b_c = rgb_in[7:0];
    end else if (FORMAT == F_RGB101010) begin
      r_c = msb10_to_8(rgb_in[29:20]);
      g_c = msb10_to_8(rgb_in[19:10]);
      b_c = msb10_to_8(rgb_in[9:0]);
    end else if (FORMAT == F_RGB121212) begin
      r_c = msb12_to_8(rgb_in[35:24]);
      g_c = msb12_to_8(rgb_in[23:12]);
      b_c = msb12_to_8(rgb_in[11:0]);
    end else if (FORMAT == F_RGB565) begin
      r_c = exp5_to_8(rgb_in[15:11]);
      g_c = exp6_to_8(rgb_in[10:5]);
      b_c = exp5_to_8(rgb_in[4:0]);
    end
  end

  integer k;
  logic [LAT:0] v_sh;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      v_sh <= '0;
    end else begin
      v_sh[0] <= pix_valid;
      for (k = 1; k <= LAT; k = k + 1) begin
        v_sh[k] <= v_sh[k-1];
      end
    end
  end

  always @* v1_valid = v_sh[LAT];

  // stage0 choose zero-on-invalid
  logic [7:0] r0, g0, b0;
  always @* begin
    if (ZERO_WHEN_INVALID && !pix_valid) begin
      r0 = 8'h00; g0 = 8'h00; b0 = 8'h00;
    end else begin
      r0 = r_c;   g0 = g_c;   b0 = b_c;
    end
  end

  generate
    if (LAT == 0) begin : G_LAT0
      always @* begin
        r8 = r0;
        g8 = g0;
        b8 = b0;
      end
    end else begin : G_LATN
      logic [7:0] r_pipe [0:LAT];
      logic [7:0] g_pipe [0:LAT];
      logic [7:0] b_pipe [0:LAT];
      integer j;

      always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
          for (j = 0; j <= LAT; j = j + 1) begin
            r_pipe[j] <= 8'h00;
            g_pipe[j] <= 8'h00;
            b_pipe[j] <= 8'h00;
          end
        end else begin
          r_pipe[0] <= r0;
          g_pipe[0] <= g0;
          b_pipe[0] <= b0;
          for (j = 1; j <= LAT; j = j + 1) begin
            r_pipe[j] <= r_pipe[j-1];
            g_pipe[j] <= g_pipe[j-1];
            b_pipe[j] <= b_pipe[j-1];
          end
        end
      end

      always @* begin
        r8 = r_pipe[LAT];
        g8 = g_pipe[LAT];
        b8 = b_pipe[LAT];
      end
    end
  endgenerate

endmodule

`default_nettype wire
`endif
