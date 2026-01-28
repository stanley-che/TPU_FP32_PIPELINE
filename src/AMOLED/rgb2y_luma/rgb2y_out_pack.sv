// rgb2y_out_pack.sv (upgrade)
// - ready/valid backpressure (elastic pipeline, depth = LAT+1)
// - generic sideband bus (SB_W)
// - optional sideband gating with valid
// - optional rounding/saturation/clip
// - bypass mode
// - debug pulses + optional counters
//
// Notes:
// - en=0 => freeze all regs, in_ready=0 (upstream must hold), output held
// - Data/sideband are treated as unsigned here (typical for luma)
// - LAT means "extra registered latency" AFTER this stage's pack/clip logic
//
// I/O summary:
//   in_valid/in_ready + y_in + sb_in
//   out_valid/out_ready + Y + sb_out
//
// If you want sof/eol, map them into sb_in bits.

`ifndef RGB2Y_OUT_PACK_SV
`define RGB2Y_OUT_PACK_SV

`timescale 1ns/1ps
`default_nettype none

module rgb2y_out_pack #(
  parameter int unsigned IN_W  = 10,   // input Y width (can be 10/12/14...)
  parameter int unsigned OUT_W = 10,   // output Y width (usually 10)
  parameter int unsigned SB_W  = 0,    // sideband width (0 ok)
  parameter int unsigned LAT   = 0,    // extra pipeline regs (elastic), depth=LAT+1
  parameter bit          ZERO_WHEN_INVALID = 1'b1,

  parameter bit          GATE_SIDEBAND_WITH_VALID = 1'b1,

  // optional processing (unsigned)
  parameter bit          USE_ROUND = 1'b0, // when IN_W>OUT_W, add 0.5 LSB before truncate
  parameter bit          USE_SAT   = 1'b1, // clamp to [0 .. 2^OUT_W-1] after rounding/trunc
  parameter bit          USE_CLIP  = 1'b0, // clamp to [clip_min .. clip_max]

  // debug
  parameter bit          COUNT_DBG = 1'b0
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 en,

  // bypass
  input  logic                 bypass,   // 1: bypass processing (still passes through pipeline)

  // input stream
  input  logic                 in_valid,
  output logic                 in_ready,
  input  logic [IN_W-1:0]       y_in,
  input  logic [SB_W-1:0]       sb_in,

  // clip controls (when USE_CLIP=1)
  input  logic                 clip_en,
  input  logic [OUT_W-1:0]      clip_min,
  input  logic [OUT_W-1:0]      clip_max,

  // output stream
  output logic                 out_valid,
  input  logic                 out_ready,
  output logic [OUT_W-1:0]      Y,
  output logic [SB_W-1:0]       sb_out,

  // debug pulses
  output logic                 drop_pulse,  // in_valid && !in_ready (only meaningful when en=1)
  output logic                 stall_pulse, // out_valid && !out_ready (only meaningful when en=1)

  // optional debug counters
  output logic [31:0]          drop_cnt,
  output logic [31:0]          stall_cnt
);

  localparam int unsigned STAGES = (LAT + 1);
  localparam int unsigned PAY_W  = (OUT_W + SB_W);

  // ----------------------------
  // pack/clip function (unsigned)
  // ----------------------------
  function automatic [OUT_W-1:0] do_pack_y(input logic [IN_W-1:0] yin);
    logic [IN_W:0]      rounded;   // +1 for carry
    logic [OUT_W-1:0]   ytrunc;
    logic [OUT_W-1:0]   ysat;
    logic [OUT_W-1:0]   yclip;
    begin
      // default: take LSBs (or exact if IN_W==OUT_W)
      ytrunc = yin[OUT_W-1:0];

      // rounding only makes sense when IN_W>OUT_W
      if (USE_ROUND && (IN_W > OUT_W)) begin
        // add 0.5 LSB at cut point: bit (IN_W-OUT_W-1)
        // Example IN_W=12 OUT_W=10 => add 1<<(12-10-1)=1<<1
        rounded = {1'b0, yin} + ({{IN_W{1'b0}},1'b1} << (IN_W-OUT_W-1));
        ytrunc  = rounded[OUT_W-1:0];
      end

      // saturation to OUT_W max (unsigned)
      if (USE_SAT) begin
        // if any upper bits beyond OUT_W are 1, clamp to max
        if (IN_W > OUT_W) begin
          if (|yin[IN_W-1:OUT_W]) ysat = {OUT_W{1'b1}};
          else                    ysat = ytrunc;
        end else begin
          ysat = ytrunc;
        end
      end else begin
        ysat = ytrunc;
      end

      // programmable clip
      if (USE_CLIP && clip_en) begin
        if (ysat < clip_min)      yclip = clip_min;
        else if (ysat > clip_max) yclip = clip_max;
        else                      yclip = ysat;
      end else begin
        yclip = ysat;
      end

      do_pack_y = yclip;
    end
  endfunction

  // ----------------------------
  // stage0 computed payload
  // ----------------------------
  logic [OUT_W-1:0] y0;
  logic [SB_W-1:0]  sb0;

  always @* begin
    // sideband gating policy
    if (SB_W == 0) sb0 = '0;
    else begin
      if (GATE_SIDEBAND_WITH_VALID) sb0 = sb_in & {SB_W{in_valid}};
      else                          sb0 = sb_in;
    end

    // Y processing / bypass
    if (bypass) y0 = y_in[OUT_W-1:0];
    else        y0 = do_pack_y(y_in);

    // optional zeroing when invalid (at capture point)
    if (ZERO_WHEN_INVALID && !in_valid) begin
      y0  = {OUT_W{1'b0}};
      if (SB_W != 0) sb0 = {SB_W{1'b0}};
    end
  end

  wire [PAY_W-1:0] pay0 = {sb0, y0};

  // ----------------------------
  // elastic pipeline (ready/valid)
  // ----------------------------
  logic [STAGES-1:0]     v_r;
  logic [PAY_W-1:0]      p_r [0:STAGES-1];
  logic [STAGES-1:0]     rdy;   // stage-ready to accept from previous stage
  integer i;

  // ready propagation (combinational)
  always @* begin
    // last stage ready if downstream ready or empty
    rdy[STAGES-1] = out_ready | ~v_r[STAGES-1];

    // backwards: stage i ready if empty or next stage ready
    for (i = STAGES-2; i >= 0; i = i - 1) begin
      rdy[i] = ~v_r[i] | rdy[i+1];
    end

    // external in_ready
    if (!en) in_ready = 1'b0;
    else     in_ready = rdy[0];
  end

  // debug pulses (combinational)
  always @* begin
    drop_pulse  = en && in_valid  && !in_ready;
    stall_pulse = en && out_valid && !out_ready;
  end

  // pipeline register update
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      v_r <= '0;
      for (i=0; i<STAGES; i=i+1) begin
        p_r[i] <= '0;
      end
    end else if (en) begin
      // move from stage i-1 -> i if stage i is ready
      for (i=STAGES-1; i>=1; i=i-1) begin
        if (rdy[i]) begin
          v_r[i] <= v_r[i-1];
          p_r[i] <= p_r[i-1];
        end
      end

      // stage0 capture new input when ready
      if (rdy[0]) begin
        v_r[0] <= in_valid;
        p_r[0] <= pay0;
      end
      // else hold stage0
    end
    // else en==0 hold all regs
  end

  // output decode + optional zeroing when out_valid=0
  wire [OUT_W-1:0] y_out_int  = p_r[STAGES-1][OUT_W-1:0];
  wire [SB_W-1:0]  sb_out_int = (SB_W==0) ? '0 : p_r[STAGES-1][PAY_W-1:OUT_W];

  always @* begin
    out_valid = v_r[STAGES-1];

    if (ZERO_WHEN_INVALID && !out_valid) begin
      Y      = {OUT_W{1'b0}};
      sb_out = {SB_W{1'b0}};
    end else begin
      Y      = y_out_int;
      sb_out = sb_out_int;
    end
  end

  // ----------------------------
  // optional debug counters
  // ----------------------------
  generate
    if (COUNT_DBG) begin : G_CNT
      always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
          drop_cnt  <= 32'd0;
          stall_cnt <= 32'd0;
        end else if (en) begin
          if (drop_pulse)  drop_cnt  <= drop_cnt  + 32'd1;
          if (stall_pulse) stall_cnt <= stall_cnt + 32'd1;
        end
      end
    end else begin : G_NO_CNT
      assign drop_cnt  = 32'd0;
      assign stall_cnt = 32'd0;

    end
  endgenerate

endmodule

`default_nettype wire
`endif
