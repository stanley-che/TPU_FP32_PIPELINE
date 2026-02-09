// ============================================================
// nb_feat_bundle_assembler_9.sv  (IVERILOG-SAFE) - TB(strong) PASS
// - Collect 9 tokens (idx 0..8) of same group_tag into kv0..kv8
// - out_valid holds stable under backpressure until out_ready
// - STRICT: after consume (out_fire), NEXT cycle out_valid must be 0
// - "cooldown" = exactly 1 cycle AFTER consume (stall upstream 1 cycle)
// - Single always_ff for out_valid / got_m / regs (avoid multi-assign hazards)
// ============================================================

`ifndef NB_FEAT_BUNDLE_ASSEMBLER_9_SV
`define NB_FEAT_BUNDLE_ASSEMBLER_9_SV

`timescale 1ns/1ps
`default_nettype none

module nb_feat_bundle_assembler_9 #(
  parameter int unsigned FEAT_W = 256,
  parameter int unsigned TAG_W  = 16,
  parameter int unsigned IDX_W  = 4,
  parameter bit          Q_FROM_CENTER_IDX4 = 1'b1
)(
  input  logic                 clk,
  input  logic                 rst,
  input  logic                 en,

  input  logic                 feat_valid,
  output logic                 feat_ready,
  input  logic [FEAT_W-1:0]    feat_vec,
  input  logic [TAG_W-1:0]     feat_tag,

  output logic                 out_valid,
  input  logic                 out_ready,

  output logic [FEAT_W-1:0]    kv0,
  output logic [FEAT_W-1:0]    kv1,
  output logic [FEAT_W-1:0]    kv2,
  output logic [FEAT_W-1:0]    kv3,
  output logic [FEAT_W-1:0]    kv4,
  output logic [FEAT_W-1:0]    kv5,
  output logic [FEAT_W-1:0]    kv6,
  output logic [FEAT_W-1:0]    kv7,
  output logic [FEAT_W-1:0]    kv8,

  output logic [9*FEAT_W-1:0]  kv_bus,
  output logic [FEAT_W-1:0]    q_vec,
  output logic [TAG_W-1:IDX_W] out_group_tag
);

  // ----------------------------
  // Internal regs
  // ----------------------------
  logic [FEAT_W-1:0] kv0_r,kv1_r,kv2_r,kv3_r,kv4_r,kv5_r,kv6_r,kv7_r,kv8_r;
  logic [8:0]        got_m;
  logic [TAG_W-1:IDX_W] cur_group;

  // ----------------------------
  // Tag decode
  // ----------------------------
  wire [IDX_W-1:0]       token_idx = feat_tag[IDX_W-1:0];
  wire [TAG_W-1:IDX_W]   group_tag = feat_tag[TAG_W-1:IDX_W];

  // ----------------------------
  // Handshakes
  // ----------------------------
  wire out_fire  = (en && !rst) && out_valid && out_ready;

  // pop_d is 1-cycle delayed out_fire
  logic pop_d;
  always_ff @(posedge clk) begin
    if (rst)      pop_d <= 1'b0;
    else if (en)  pop_d <= out_fire;
  end

  // EXACTLY 1 cycle AFTER pop
  wire pop_cooldown = pop_d;

  // upstream accept only when not holding output and not in cooldown
  always @* begin
    if (!en || rst) feat_ready = 1'b0;
    else           feat_ready = (!out_valid && !pop_cooldown);
  end
  wire feat_fire = (en && !rst) && feat_valid && feat_ready;

  // ----------------------------
  // Main sequential logic
  // ----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      out_valid     <= 1'b0;
      got_m         <= 9'b0;
      cur_group     <= '0;
      out_group_tag <= '0;

      kv0_r <= '0; kv1_r <= '0; kv2_r <= '0; kv3_r <= '0; kv4_r <= '0;
      kv5_r <= '0; kv6_r <= '0; kv7_r <= '0; kv8_r <= '0;

    end else if (en) begin
      // ---------------------------------------------------------
      // 1) POP has absolute priority: clear out_valid immediately
      // ---------------------------------------------------------
      if (out_fire) begin
        out_valid <= 1'b0;
        got_m     <= 9'b0;
        // keep kv regs (out_valid gates visibility)

      end else if (pop_cooldown) begin
        // ---------------------------------------------------------
        // 2) COOLDOWN cycle: stall upstream; keep out_valid low
        // ---------------------------------------------------------
        out_valid <= 1'b0;
        // got_m stays as-is (already cleared on out_fire)

      end else if (out_valid) begin
        // ---------------------------------------------------------
        // 3) HOLD: stable under backpressure until out_fire
        // ---------------------------------------------------------
        out_valid <= 1'b1;

      end else begin
        // ---------------------------------------------------------
        // 4) COLLECT (only when out_valid=0 and not cooldown)
        // ---------------------------------------------------------
        if (feat_fire) begin
          // group tracking / flush on group change
          if (got_m == 9'b0) begin
            cur_group     <= group_tag;
            out_group_tag <= group_tag;
          end else if (group_tag != cur_group) begin
            got_m         <= 9'b0;
            cur_group     <= group_tag;
            out_group_tag <= group_tag;

            kv0_r <= '0; kv1_r <= '0; kv2_r <= '0; kv3_r <= '0; kv4_r <= '0;
            kv5_r <= '0; kv6_r <= '0; kv7_r <= '0; kv8_r <= '0;
          end

          // store token + set mask bit
          case (token_idx)
            0: begin kv0_r <= feat_vec; got_m[0] <= 1'b1; end
            1: begin kv1_r <= feat_vec; got_m[1] <= 1'b1; end
            2: begin kv2_r <= feat_vec; got_m[2] <= 1'b1; end
            3: begin kv3_r <= feat_vec; got_m[3] <= 1'b1; end
            4: begin kv4_r <= feat_vec; got_m[4] <= 1'b1; end
            5: begin kv5_r <= feat_vec; got_m[5] <= 1'b1; end
            6: begin kv6_r <= feat_vec; got_m[6] <= 1'b1; end
            7: begin kv7_r <= feat_vec; got_m[7] <= 1'b1; end
            8: begin kv8_r <= feat_vec; got_m[8] <= 1'b1; end
            default: begin /* ignore */ end
          endcase
        end

        // ---------------------------------------------------------
        // 5) RAISE when complete
        //    - block re-raise on pop_d (means last cycle consumed)
        // ---------------------------------------------------------
        if (!pop_d && (got_m == 9'b111111111)) begin
          out_valid <= 1'b1;
        end else begin
          out_valid <= 1'b0;
        end
      end
    end
  end

  // ----------------------------
  // Outputs
  // ----------------------------
  always @* begin
    kv0 = kv0_r; kv1 = kv1_r; kv2 = kv2_r; kv3 = kv3_r; kv4 = kv4_r;
    kv5 = kv5_r; kv6 = kv6_r; kv7 = kv7_r; kv8 = kv8_r;

    kv_bus = {kv8_r, kv7_r, kv6_r, kv5_r, kv4_r, kv3_r, kv2_r, kv1_r, kv0_r};

    if (Q_FROM_CENTER_IDX4) q_vec = kv4_r;
    else                    q_vec = '0;
  end

endmodule

`default_nettype wire
`endif
