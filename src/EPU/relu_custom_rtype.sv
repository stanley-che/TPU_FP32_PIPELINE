/*
|     inst       | opcode | funct7 | funct3 | func explain                                      |
| ---------------| ------ | ------ | ------ | --------------------------------------------------|
|   `RELU_AWR`   | 0x33   | 0x03   | 3'b000 | write X SRAM：rs1={row,col}，rs2=data             |
|  `RELU_START`  | 0x33   | 0x03   | 3'b001 | start relu (process tile/region)                  |
|   `RELU_STAT`  | 0x33   | 0x03   | 3'b010 | read status {done,busy}                           |
|   `RELU_YRD`   | 0x33   | 0x03   | 3'b011 | read Y SRAM：rs1={row,col} -> rd                  |
*/

`include "./src/EPU/ReLu/top_relu_cpu.sv"  
`timescale 1ns/1ps
`default_nettype none

module relu_custom_rtype #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
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

  // ------------------------------------------------------------
  // Instruction decode fields
  // ------------------------------------------------------------
  wire [6:0] opcode = instr[6:0];
  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];

  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_RELU   = 7'h03;

  // funct3 mapping
  localparam logic [2:0] F3_AWR   = 3'b000; // RELU_AWR
  localparam logic [2:0] F3_START = 3'b001; // RELU_START
  localparam logic [2:0] F3_STAT  = 3'b010; // RELU_STAT
  localparam logic [2:0] F3_YRD   = 3'b011; // RELU_YRD

  wire is_relu = (opcode == OPC_RTYPE) && (funct7 == F7_RELU);
  wire is_AWR  = is_relu && (funct3 == F3_AWR);
  wire is_START= is_relu && (funct3 == F3_START);
  wire is_STAT = is_relu && (funct3 == F3_STAT);
  wire is_YRD  = is_relu && (funct3 == F3_YRD);

  // ------------------------------------------------------------
  // Row/Col unpack from rs1_val
  // (Assumption: rs1[31:16]=row, rs1[15:0]=col)
  // ------------------------------------------------------------
  wire [31:0] row_u32 = {16'b0, rs1_val[31:16]};
  wire [31:0] col_u32 = {16'b0, rs1_val[15:0]};

  // ------------------------------------------------------------
  // Underlying accelerator (matrix_relu_cpu)
  // ------------------------------------------------------------
  logic relu_start, relu_done, relu_busy;

  logic              cpu_x_we;
  logic [31:0]       cpu_x_row, cpu_x_col;
  logic [DATA_W-1:0] cpu_x_wdata;

  logic              cpu_y_re;
  logic [31:0]       cpu_y_row, cpu_y_col;
  logic [DATA_W-1:0] cpu_y_rdata;
  logic              cpu_y_rvalid;

  matrix_relu_cpu_ReLu #(
    .NRows(M),
    .NCols(N),
    .NB(NB),
    .ADDR_W(ADDR_W),
    .Data_W(DATA_W)
  ) u_relu (
    .clk(clk),
    .rst_n(~rst),

    .start(relu_start),
    .done(relu_done),
    .busy(relu_busy),

    .cpu_x_we(cpu_x_we),
    .cpu_x_row(cpu_x_row),
    .cpu_x_col(cpu_x_col),
    .cpu_x_wdata(cpu_x_wdata),

    .cpu_y_re(cpu_y_re),
    .cpu_y_row(cpu_y_row),
    .cpu_y_col(cpu_y_col),
    .cpu_y_rdata(cpu_y_rdata),
    .cpu_y_rvalid(cpu_y_rvalid)
  );

  assign accel_busy    = relu_busy;
  assign accel_done    = relu_done;
  assign accel_C_valid = relu_done; // 你要的 valid：done pulse 當作 C_valid

  // ------------------------------------------------------------
  // Make done "sticky" for STAT so software won't miss the pulse
  // (Read STAT will clear sticky_done)
  // ------------------------------------------------------------
  logic sticky_done;
  always_ff @(posedge clk or posedge rst) begin
    if (rst) sticky_done <= 1'b0;
    else begin
      if (relu_done) sticky_done <= 1'b1;
      if (instr_valid && instr_ready && is_STAT) sticky_done <= 1'b0;
    end
  end

  // ------------------------------------------------------------
  // Wrapper control FSM for readback instructions (STAT/YRD)
  // ------------------------------------------------------------
  typedef enum logic [1:0] {W_IDLE, W_WAIT_Y} wstate_t;
  wstate_t wst;

  logic [4:0] rd_q;          // destination reg for delayed responses
  logic       pend_is_stat;  // 1 if pending STAT response, else YRD
    // ----------------------------
  // Handshake (accept every cycle)
  // ----------------------------
  wire accept = instr_valid && instr_ready;
  assign instr_ready = 1'b1;

  // ----------------------------
  // Combinational controls to accelerator (AWR/START/YRD request)
  // ----------------------------
  always_comb begin
    relu_start = 1'b0;

    cpu_x_we    = 1'b0;
    cpu_x_row   = '0;
    cpu_x_col   = '0;
    cpu_x_wdata = '0;

    cpu_y_re  = 1'b0;
    cpu_y_row = '0;
    cpu_y_col = '0;

    if (accept && is_AWR) begin
      cpu_x_we    = 1'b1;
      cpu_x_row   = row_u32;
      cpu_x_col   = col_u32;
      cpu_x_wdata = rs2_val;
    end

    if (accept && is_START) begin
      relu_start = 1'b1;
    end

    if (accept && is_YRD && (wst == W_IDLE)) begin
      cpu_y_re  = 1'b1;
      cpu_y_row = row_u32;
      cpu_y_col = col_u32;
    end
  end

  // ----------------------------
  // Sticky done (STAT read clears)
  // ----------------------------
  always_ff @(posedge clk or posedge rst) begin
    if (rst) sticky_done <= 1'b0;
    else begin
      if (relu_done) sticky_done <= 1'b1;
      if (accept && is_STAT) sticky_done <= 1'b0;
    end
  end

  // ----------------------------
  // Pending STAT response (1-cycle delayed so TB won't miss)
  // ----------------------------
  logic stat_pend;
  logic [4:0] stat_rd_q;


  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      wst      <= W_IDLE;
      rd_q     <= 5'd0;
      stat_pend<= 1'b0;
      stat_rd_q<= 5'd0;
    end else begin
      // launch STAT pending
      if (accept && is_STAT) begin
        stat_pend <= 1'b1;
        stat_rd_q <= rd_addr;
      end

      case (wst)
        W_IDLE: begin
          if (accept && is_YRD) begin
            rd_q <= rd_addr;
            wst  <= W_WAIT_Y;
          end
        end

        W_WAIT_Y: begin
          if (cpu_y_rvalid) begin
            wst <= W_IDLE;
          end
        end
      endcase
    end
  end

  // ----------------------------
  // RD writeback pulses (ONLY sequential, single driver)
  // ----------------------------
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rd_we    <= 1'b0;
      rd_waddr <= 5'd0;
      rd_wdata <= 32'h0;
    end else begin
      rd_we <= 1'b0; // default: no write

      // STAT response (delayed 1 cycle)
      if (stat_pend) begin
        rd_we    <= 1'b1;
        rd_waddr <= stat_rd_q;
        rd_wdata <= {30'b0, relu_busy, sticky_done}; // [1]=busy, [0]=done(sticky)
        stat_pend<= 1'b0;
      end

      // YRD response
      if (wst == W_WAIT_Y && cpu_y_rvalid) begin
        rd_we    <= 1'b1;
        rd_waddr <= rd_q;
        rd_wdata <= cpu_y_rdata;
      end
    end
  end


endmodule

`default_nettype wire
