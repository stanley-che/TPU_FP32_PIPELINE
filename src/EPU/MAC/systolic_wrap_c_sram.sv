// systolic_wrap_c_sram.sv  (FULL FLATTEN OUTPUT VERSION)
`include "./src/EPU/MAC/sa_tile_driver.sv"
`include "./src/EPU/MAC/c_sram_ctrl_flat.sv"
`timescale 1ns/1ps
`default_nettype none


module systolic_wrap_c_sram #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY_C = 1,

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N)
)(
  input  logic clk,
  input  logic rst,

  // command
  input  logic        start,      // pulse
  input  logic [15:0] K_len,
  output logic        busy,
  output logic        done,

  // tiles (FLAT BUS)
  input  logic [M*KMAX*DATA_W-1:0] W_tile_flat,
  input  logic [KMAX*N*DATA_W-1:0] X_tile_flat,

  // C SRAM read (CPU)
  input  logic              c_rd_en,
  input  logic              c_rd_re,
  input  logic [ROW_W-1:0]  c_rd_row,
  input  logic [COL_W-1:0]  c_rd_col,
  output logic [DATA_W-1:0] c_rd_rdata,
  output logic              c_rd_rvalid,

  // FULL FLATTEN OUTPUT (debug/observe)
  output logic [M*N*DATA_W-1:0] c_out_flat_o,
  output logic [M*N-1:0]        c_valid_flat_o,
  output logic                  C_valid
);

  // ============================================================
  // 1) sa_tile_driver_flat -> systolic_array_os_flat
  // ============================================================
  logic              step_valid;
  logic [M*DATA_W-1:0] a_row_flat;
  logic [N*DATA_W-1:0] b_col_flat;
  logic              k_first;
  logic              k_last;
  logic              step_ready;

  // SA outputs
  logic [M*N*DATA_W-1:0] c_out_flat;
  logic [M*N-1:0]        c_valid_flat;
  logic [M*N*DATA_W-1:0] psum_out_flat; // optional debug
  logic c_start;

  always_ff @(posedge clk) begin
    if (rst)
      c_start <= 1'b0;
    else if (c_valid_flat != '0)
      c_start <= 1'b1;   // 第一次看到 SA valid
      
  end
  
  sa_tile_driver_flat #(
    .M(M), .N(N), .KMAX(KMAX)
  ) u_drv (
    .clk(clk),
    .rst(rst),

    .tile_start(start),
    .K_len(K_len),
    .tile_busy(busy),
    .tile_done(done),

    .W_tile_flat(W_tile_flat),
    .X_tile_flat(X_tile_flat),

    .step_valid(step_valid),
    .a_row_flat(a_row_flat),
    .b_col_flat(b_col_flat),
    .k_first(k_first),
    .k_last(k_last),
    .step_ready(step_ready),

    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_flat)
  );

  systolic_array_os_flat #(
    .M(M), .N(N)
  ) u_sa (
    .clk(clk),
    .rst(rst),

    .step_valid(step_valid),
    .a_row_flat(a_row_flat),
    .b_col_flat(b_col_flat),
    .k_first(k_first),
    .k_last(k_last),

    .step_ready(step_ready),

    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_flat),

    .psum_out_flat(psum_out_flat)
  );

  // ============================================================
  // 2) C SRAM controller (writer + SRAM + CPU read + debug exports)
  //    - start/flush：建議跟 start 同步（同一個 pulse）
  // ============================================================
  c_sram_ctrl_flat #(
    .M(M), .N(N), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY_C),
    .ROW_W(ROW_W), .COL_W(COL_W)
  ) u_cctrl (
    .clk(clk),
    .rst(rst),

    .start(start),
    .flush(start),   // 你也可以改成獨立 flush

    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_flat),

    // CPU read
    .cpu_c_en   (c_rd_en),
    .cpu_c_re   (c_rd_re),
    .cpu_c_row  (c_rd_row),
    .cpu_c_col  (c_rd_col),
    .cpu_c_rdata(c_rd_rdata),
    .cpu_c_rvalid(c_rd_rvalid),

    // debug export
    .c_out_flat_o  (c_out_flat_o),
    .c_valid_flat_o(c_valid_flat_o),
    .C_valid       (C_valid)
  );
//DEBUG
/*
always_ff @(posedge clk) begin
  if (!rst && step_valid) begin
    $write("[PSUM] ");
    for (int idx=0; idx<M*N; idx++) begin
      $write("%08h ", psum_out_flat[idx*DATA_W +: DATA_W]);
    end
    $write("\n");
  end
end

always_ff @(posedge clk) begin
  if (!rst) begin
    if (start) $display("[HS] start");
    if (step_valid || step_ready)
      $display("[HS] step_valid=%0b step_ready=%0b k_first=%0b k_last=%0b",
               step_valid, step_ready, k_first, k_last);
  end
end

always_ff @(posedge clk) begin
  if (!rst && step_valid) begin
    $write("[A_IN] ");
    for (int i=0; i<M; i++) $write("%08h ", a_row_flat[i*DATA_W +: DATA_W]);
    $write("\n");
  end
end
*/

endmodule

`default_nettype wire
