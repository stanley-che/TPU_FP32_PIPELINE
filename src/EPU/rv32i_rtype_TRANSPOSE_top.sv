/*
|     inst      | opcode | funct7 | funct3 | func explain                                      |
| ------------- | ------ | ------ | ------ | --------------------------------------------------|
|   `TP_AWR`    | 0x33   | 0x02   | 3'b000 | write A SRAM：`rs1={row,col}`，`rs2=data`          |
|  `TP_START`   | 0x33   | 0x02   | 3'b001 | start transpose                                   |
|   `TP_STAT`   | 0x33   | 0x02   | 3'b010 | read status `{done,busy}`                         |
|   `TP_BRD`    | 0x33   | 0x02   | 3'b011 | read B SRAM：`rs1={row,col}` → `rd`               |
*/

`include "./src/EPU/transpose/transpose_crtl.sv"
`timescale 1ns/1ps
`default_nettype none


module transpose_custom_rtype #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 2
)(
  input  logic clk,
  input  logic rst,

  // "CPU custom instruction" interface
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
  // R-type decode fields
  // ----------------------------
  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;

  assign opcode = instr[6:0];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];

  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_TP     = 7'h02;

  localparam logic [2:0] F3_AWR   = 3'b000;
  localparam logic [2:0] F3_START = 3'b001;
  localparam logic [2:0] F3_STAT  = 3'b010;
  localparam logic [2:0] F3_BRD   = 3'b011;

  logic is_tp;
  assign is_tp = (opcode == OPC_RTYPE) && (funct7 == F7_TP);

  // rs1 packing: {row,col} = {rs1[31:16], rs1[15:0]}
  logic [31:0] row_u32, col_u32;
  assign row_u32 = {16'b0, rs1_val[31:16]};
  assign col_u32 = {16'b0, rs1_val[15:0]};

  // ----------------------------
  // DUT: your top_transpose_cpu
  // ----------------------------
  logic start;
  logic done;
  logic busy;

  logic              cpu_a_we;
  logic [31:0]       cpu_a_row, cpu_a_col;
  logic [DATA_W-1:0] cpu_a_wdata;

  logic              cpu_b_re;
  logic [31:0]       cpu_b_row, cpu_b_col;
  logic [DATA_W-1:0] cpu_b_rdata;
  logic              cpu_b_rvalid;

  top_transpose_cpu #(
    .NRows(M),
    .NCols(N),
    .NB(NB),
    .ADDR_W(ADDR_W),
    .Data_W(DATA_W),
    .M(6) // 你原本 bank_sram 用的 M 參數意義若不同，請改成你要的
  ) u_tp (
    .clk(clk),
    .rst_n(~rst),      // 注意：你的 top_transpose_cpu 是 rst_n
    .start(start),
    .done(done),
    .busy(busy),

    .cpu_a_we(cpu_a_we),
    .cpu_a_row(cpu_a_row),
    .cpu_a_col(cpu_a_col),
    .cpu_a_wdata(cpu_a_wdata),

    .cpu_b_re(cpu_b_re),
    .cpu_b_row(cpu_b_row),
    .cpu_b_col(cpu_b_col),
    .cpu_b_rdata(cpu_b_rdata),
    .cpu_b_rvalid(cpu_b_rvalid)
  );

  // status outputs
  assign accel_busy    = busy;
  assign accel_done    = done;
  assign accel_C_valid = cpu_b_rvalid; // 如果你想定義成 "transpose result valid" 也可改

  // ----------------------------
  // Wrapper control FSM
  //   - handles stall on BRD
  // ----------------------------
    typedef enum logic [0:0] {W_IDLE, W_WAIT_BRD} wstate_e;
  wstate_e wst;

  logic [4:0]  rd_hold;

  // accept condition (only when ready)
  logic accept;
  assign accept = instr_valid && instr_ready && is_tp;

  // ----------------------------
  // combinational outputs (single driver!)
  // ----------------------------
  always_comb begin
    // defaults
    instr_ready = (wst == W_IDLE);

    rd_we    = 1'b0;
    rd_waddr = rd_addr;
    rd_wdata = 32'b0;

    start    = 1'b0;
    cpu_a_we = 1'b0;
    cpu_b_re = 1'b0;

    cpu_a_row   = row_u32;
    cpu_a_col   = col_u32;
    cpu_a_wdata = rs2_val[DATA_W-1:0];

    cpu_b_row   = row_u32;
    cpu_b_col   = col_u32;

    // ----- IDLE: accept commands -----
    if (wst == W_IDLE && accept) begin
      case (funct3)
        F3_AWR: begin
          cpu_a_we = 1'b1; // 1-cycle pulse
        end

        F3_START: begin
          start = 1'b1;    // 1-cycle pulse
        end

        F3_STAT: begin
          rd_we    = 1'b1;
          rd_waddr = rd_addr;
          rd_wdata = {30'b0, busy, done}; // bit0=done, bit1=busy
        end

        F3_BRD: begin
          cpu_b_re = 1'b1; // issue read (1-cycle)
          // rd_we will be produced later when rvalid
        end

        default: begin end
      endcase
    end

    // ----- WAIT_BRD: return data when valid -----
    if (wst == W_WAIT_BRD && cpu_b_rvalid) begin
      rd_we    = 1'b1;
      rd_waddr = rd_hold;
      rd_wdata = cpu_b_rdata;
    end
  end

  // ----------------------------
  // state / hold regs
  // ----------------------------
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      wst     <= W_IDLE;
      rd_hold <= '0;
    end else begin
      case (wst)
        W_IDLE: begin
          if (accept && funct3 == F3_BRD) begin
            rd_hold <= rd_addr;   // remember destination reg
            wst     <= W_WAIT_BRD;
          end
        end

        W_WAIT_BRD: begin
          if (cpu_b_rvalid) begin
            wst <= W_IDLE;
          end
        end
      endcase
    end
  end

endmodule

`default_nettype wire
