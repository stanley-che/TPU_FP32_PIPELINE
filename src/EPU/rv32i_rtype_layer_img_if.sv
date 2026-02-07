/*
| instr         | opcode | funct7 | funct3 | explain                                               |
| ------------- | ------ | ------ | ------ | ----------------------------------------------------- |
| `IM_WR`       | 0x33   | 0x06   | 3'b000 | 寫 IMG SRAM：rs1 packed(row,col)，rs2=data (fp32 bits) |
| `IM_RD`       | 0x33   | 0x06   | 3'b001 | 讀 IMG SRAM：rs1 packed(row,col) → rd（下一拍回）      |
| `IM_STAT`     | 0x33   | 0x06   | 3'b010 | 讀狀態：{.., rvalid, done, busy}                       |
*/
`include "./src/EPU/img/layer_img_if.sv"   // 你前面那個 layer_img_if

`timescale 1ns/1ps
`default_nettype none


module rv32i_rtype_layer_img_if #(
  parameter int unsigned IMG_H   = 255,
  parameter int unsigned IMG_W   = 255,
  parameter int unsigned ADDR_W  = 16,
  parameter int unsigned DATA_W  = 32,
  parameter int unsigned BYTE_W  = (DATA_W/8)
)(
  input  logic clk,
  input  logic rst,

  input  logic        instr_valid,
  output logic        instr_ready,
  input  logic [31:0] instr,
  input  logic [31:0] rs1_val,
  input  logic [31:0] rs2_val,
  input  logic [4:0]  rd_addr,

  output logic        rd_we,
  output logic [4:0]  rd_waddr,
  output logic [31:0] rd_wdata,

  output logic        accel_busy,
  output logic        accel_done,
  output logic        accel_C_valid
);

  // ----------------------------
  // decode
  // ----------------------------
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;

  assign opcode = instr[6:0];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];

  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_IMG    = 7'h06;

  logic is_img;
  assign is_img = (opcode == OPC_RTYPE) && (funct7 == F7_IMG);

  localparam logic [2:0] F3_IM_WR   = 3'b000;
  localparam logic [2:0] F3_IM_RD   = 3'b001;
  localparam logic [2:0] F3_IM_STAT = 3'b010;

  // ----------------------------
  // unpack rs1={row,col}
  // ----------------------------
  logic [15:0] row, col;
  assign row = rs1_val[31:16];
  assign col = rs1_val[15:0];

  // ----------------------------
  // wires to layer_img_if
  // ----------------------------
  logic              cpu_rd_req;
  logic [15:0]       cpu_row, cpu_col;
  logic [DATA_W-1:0] cpu_rdata;

  logic              seq_we;
  logic [15:0]       seq_row, seq_col;
  logic [DATA_W-1:0] seq_wdata;

  logic [15:0]       seq_row_q, seq_col_q;
  logic [DATA_W-1:0] seq_wdata_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      seq_row_q   <= '0;
      seq_col_q   <= '0;
      seq_wdata_q <= '0;
    end else begin
      if (instr_valid && instr_ready && is_img && (funct3 == F3_IM_WR)) begin
        seq_row_q   <= row;
        seq_col_q   <= col;
        seq_wdata_q <= rs2_val;
      end
    end
  end

  // ----------------------------
  // STAT pending (1-deep)
  // ----------------------------
  logic pending_stat;
  logic [4:0] pending_rdaddr;

  // ----------------------------
  // RD pipeline (2-cycle return)
  // ----------------------------
  logic        rd_p0, rd_p1;
  logic [4:0]  rdaddr_q;
  logic [15:0] rd_row_q, rd_col_q;

  // accept only if ours and no outstanding response
  assign instr_ready = is_img && !(pending_stat || rd_p0 || rd_p1);

  // generate 1-cycle pulses for WR and RD launch
  logic seq_we_q;
  logic cpu_rd_req_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      seq_we_q     <= 1'b0;
      cpu_rd_req_q <= 1'b0;
    end else begin
      seq_we_q     <= (instr_valid && instr_ready && is_img && (funct3 == F3_IM_WR));
      cpu_rd_req_q <= (instr_valid && instr_ready && is_img && (funct3 == F3_IM_RD));
    end
  end

  // RD pipe + latch addr
  always_ff @(posedge clk) begin
    if (rst) begin
      rd_p0    <= 1'b0;
      rd_p1    <= 1'b0;
      rdaddr_q <= '0;
      rd_row_q <= '0;
      rd_col_q <= '0;
    end else begin
      rd_p1 <= rd_p0;
      rd_p0 <= 1'b0;

      if (instr_valid && instr_ready && is_img && (funct3 == F3_IM_RD)) begin
        rd_p0    <= 1'b1;
        rdaddr_q <= rd_addr;
        rd_row_q <= row;
        rd_col_q <= col;
      end
    end
  end

  // STAT pending latch (1 cycle delayed response)
  always_ff @(posedge clk) begin
    if (rst) begin
      pending_stat   <= 1'b0;
      pending_rdaddr <= '0;
    end else begin
      // set
      if (instr_valid && instr_ready && is_img && (funct3 == F3_IM_STAT)) begin
        pending_stat   <= 1'b1;
        pending_rdaddr <= rd_addr;
      end else begin
        pending_stat <= 1'b0; // auto clear next cycle (1-deep)
      end
    end
  end

  // hold read address stable while RD in flight
  logic hold_rd;
  assign hold_rd = rd_p0 || rd_p1;

  assign cpu_row = hold_rd ? rd_row_q : row;
  assign cpu_col = hold_rd ? rd_col_q : col;

  assign cpu_rd_req = cpu_rd_req_q;

  assign seq_we = seq_we_q;
  assign seq_row   = seq_row_q;
  assign seq_col   = seq_col_q;
  assign seq_wdata = seq_wdata_q;
  // ----------------------------
  // outputs (registered pulses)
  // ----------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      rd_we         <= 1'b0;
      rd_waddr      <= '0;
      rd_wdata      <= '0;
      accel_busy    <= 1'b0;
      accel_done    <= 1'b0;
      accel_C_valid <= 1'b0;
    end else begin
      rd_we         <= 1'b0;
      accel_busy    <= 1'b0;
      accel_done    <= 1'b0;
      accel_C_valid <= 1'b0;

      // RD returns on rd_p1
      if (rd_p1) begin
        rd_we         <= 1'b1;
        rd_waddr      <= rdaddr_q;
        rd_wdata      <= cpu_rdata;
        accel_done    <= 1'b1;
        accel_C_valid <= 1'b1;
      end

      // STAT returns one cycle after accept (pending_stat==1)
      if (pending_stat) begin
        rd_we      <= 1'b1;
        rd_waddr   <= pending_rdaddr;
        rd_wdata   <= {29'h0, 1'b0 /*rvalid*/, 1'b1 /*done*/, 1'b0 /*busy*/};
        accel_done <= 1'b1;
      end

      // WR done pulse (same cycle as seq_we_q)
      if (seq_we_q) begin
        accel_done <= 1'b1;
      end
    end
  end

  // ----------------------------
  // DUT: layer_img_if
  // ----------------------------
  layer_img_if #(
    .IMG_H (IMG_H),
    .IMG_W (IMG_W),
    .ADDR_W(ADDR_W),
    .DATA_W(DATA_W)
  ) u_layer_img (
    .clk       (clk),
    .rst       (rst),

    .cpu_rd_req(cpu_rd_req),
    .cpu_row   (cpu_row),
    .cpu_col   (cpu_col),
    .cpu_rdata (cpu_rdata),

    .seq_we    (seq_we),
    .seq_row   (seq_row),
    .seq_col   (seq_col),
    .seq_wdata (seq_wdata)
  );

endmodule

`default_nettype wire
