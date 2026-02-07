// ============================================================================
// fp32_exp_no_lut_lanes.sv
// - Parallel lanes wrapper to increase throughput
// - Keeps original fp32_exp_no_lut unchanged
// - Adds in_ready so upstream won't drop inputs
// ============================================================================

`include "./src/EPU/attention_score/fp32_exp_no_lut.sv"
`timescale 1ns/1ps
`default_nettype none

module fp32_exp_no_lut_lanes #(
  parameter int unsigned LANES = 4
)(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        in_valid,
  output logic        in_ready,
  input  logic [31:0] in_fp32,

  output logic        out_valid,
  output logic [31:0] out_fp32
);

  // ----------------------------
  // Lanes
  // ----------------------------
  logic [LANES-1:0] lane_busy;     // busy = accepted but not yet returned out_valid
  logic [LANES-1:0] lane_in_fire;  // 1-cycle pulse to a selected lane

  logic [LANES-1:0] lane_out_valid;
  logic [31:0]      lane_out_fp32 [LANES];

  // instantiate lanes
  genvar g;
  generate
    for (g=0; g<LANES; g++) begin: G_LANE
      // feed lane only when selected
      fp32_exp_no_lut u (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (lane_in_fire[g]),
        .in_fp32  (in_fp32),
        .out_valid(lane_out_valid[g]),
        .out_fp32 (lane_out_fp32[g])
      );
    end
  endgenerate

  // ----------------------------
  // Select free lane (round-robin)
  // ----------------------------
  logic [$clog2(LANES)-1:0] rr_ptr;
  logic [$clog2(LANES)-1:0] sel_lane;
  logic                     sel_ok;

  function automatic int find_free_lane(input logic [$clog2(LANES)-1:0] start);
    int k;
    begin
      find_free_lane = -1;
      for (k=0; k<LANES; k++) begin
        int idx;
        idx = (start + k) % LANES;
        if (!lane_busy[idx]) begin
          find_free_lane = idx;
          disable for;
        end
      end
    end
  endfunction

  // combinational selection
  always_comb begin
    int f;
    f = find_free_lane(rr_ptr);
    sel_ok   = (f >= 0);
    sel_lane = (f >= 0) ? f[$clog2(LANES)-1:0] : '0;

    in_ready = sel_ok; // can accept if there is a free lane

    lane_in_fire = '0;
    if (in_valid && in_ready) begin
      lane_in_fire[sel_lane] = 1'b1;
    end
  end

  // ----------------------------
  // Track lane_busy, rr_ptr
  // ----------------------------
  integer i;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      lane_busy <= '0;
      rr_ptr    <= '0;
    end else begin
      // mark accepted lane busy
      if (in_valid && in_ready) begin
        lane_busy[sel_lane] <= 1'b1;
        rr_ptr <= sel_lane + 1'b1;
      end

      // clear busy when lane outputs
      for (i=0; i<LANES; i++) begin
        if (lane_out_valid[i]) begin
          lane_busy[i] <= 1'b0;
        end
      end
    end
  end

  // ----------------------------
  // Output mux: if multiple lanes finish same cycle, pick lowest index
  // ----------------------------
  always_comb begin
    out_valid = 1'b0;
    out_fp32  = 32'h0;
    for (int j=0; j<LANES; j++) begin
      if (lane_out_valid[j] && !out_valid) begin
        out_valid = 1'b1;
        out_fp32  = lane_out_fp32[j];
      end
    end
  end

endmodule

`default_nettype wire
