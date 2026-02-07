/*
| instr         | opcode | funct7 | funct3 | explain                                              |
| ------------- | ------ | ------ | ------ | --------------------------------------------------- |
| `BA_XWR`      | 0x33   | 0x04   | 3'b000 | 寫 X SRAM：rs1 packed(row,col)，rs2=data (fp32 bits) |
| `BA_BWR`      | 0x33   | 0x04   | 3'b001 | 寫 Bias：rs1 packed(col) (低 bits)，rs2=data         |
| `BA_START`    | 0x33   | 0x04   | 3'b010 | 啟動：開始做 C = X + B（整個 MxN）                   |
| `BA_CRD`      | 0x33   | 0x04   | 3'b011 | 讀 C SRAM：rs1 packed(row,col) → rd                  |
| `BA_STAT`     | 0x33   | 0x04   | 3'b100 | 讀狀態：回傳 {.., C_valid, done, busy}               |
*/

`include "./src/EPU/bias_adder/bias_add_core_top.sv"
`timescale 1ns/1ps
`default_nettype none

module rv32i_rtype_BIAS_ADD_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N)
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

  // decode
  wire [6:0] opcode = instr[6:0];
  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];

  localparam logic [6:0] OPC_RTYPE = 7'b0110011;
  localparam logic [6:0] F7_BIAS   = 7'b0000100; // 0x04

  localparam logic [2:0] F3_XWR    = 3'b000;
  localparam logic [2:0] F3_BWR    = 3'b001;
  localparam logic [2:0] F3_START  = 3'b010;
  localparam logic [2:0] F3_CRD    = 3'b011;
  localparam logic [2:0] F3_STAT   = 3'b100;

  // core wires
  logic start_go;
  logic busy, done, C_valid;

  // cpu write X
  logic              cpu_x_we;
  logic [ROW_W-1:0]  cpu_x_row;
  logic [COL_W-1:0]  cpu_x_col;
  logic [DATA_W-1:0] cpu_x_wdata;
  logic [BYTE_W-1:0] cpu_x_wmask;

  // cpu write Bias (vector per column)
  logic              cpu_b_we;
  logic [COL_W-1:0]  cpu_b_col;
  logic [DATA_W-1:0] cpu_b_wdata;
  logic [BYTE_W-1:0] cpu_b_wmask;

  // c read ports
  logic              c_rd_en, c_rd_re;
  logic [ROW_W-1:0]  c_rd_row;
  logic [COL_W-1:0]  c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic              c_rd_rvalid;

  bias_add_core_top #(
    .M(M), .N(N),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .ROW_W(ROW_W), .COL_W(COL_W)
  ) u_core (
    .clk(clk),
    .rst(rst),

    .start(start_go),
    .busy(busy),
    .done(done),
    .C_valid(C_valid),

    .cpu_x_we(cpu_x_we),
    .cpu_x_row(cpu_x_row),
    .cpu_x_col(cpu_x_col),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .cpu_b_we(cpu_b_we),
    .cpu_b_col(cpu_b_col),
    .cpu_b_wdata(cpu_b_wdata),
    .cpu_b_wmask(cpu_b_wmask),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid)
  );

  assign accel_busy    = busy;
  assign accel_done    = done;
  assign accel_C_valid = C_valid;

  // FSM for CRD wait
  typedef enum logic [1:0] {S_IDLE, S_WAIT_C} state_t;
  state_t st;

  logic [ROW_W-1:0] crd_row_q;
  logic [COL_W-1:0] crd_col_q;
  logic [4:0]       rd_q;

  // ready only in idle
  always_comb begin
    instr_ready = (st == S_IDLE);
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;

      start_go <= 1'b0;
      cpu_x_we <= 1'b0;
      cpu_b_we <= 1'b0;
      c_rd_en  <= 1'b0;
      c_rd_re  <= 1'b0;

      cpu_x_row   <= '0;
      cpu_x_col   <= '0;
      cpu_x_wdata <= '0;
      cpu_x_wmask <= {BYTE_W{1'b1}};

      cpu_b_col   <= '0;
      cpu_b_wdata <= '0;
      cpu_b_wmask <= {BYTE_W{1'b1}};

      c_rd_row <= '0;
      c_rd_col <= '0;

      rd_we    <= 1'b0;
      rd_waddr <= 5'd0;
      rd_wdata <= 32'd0;

      crd_row_q <= '0;
      crd_col_q <= '0;
      rd_q      <= '0;

    end else begin
      // clear 1-cycle pulses
      start_go <= 1'b0;
      cpu_x_we <= 1'b0;
      cpu_b_we <= 1'b0;
      c_rd_en  <= 1'b0;
      c_rd_re  <= 1'b0;
      rd_we    <= 1'b0;

      // keep masks default 0xF
      cpu_x_wmask <= {BYTE_W{1'b1}};
      cpu_b_wmask <= {BYTE_W{1'b1}};

      case (st)
        S_IDLE: begin
          if (instr_valid && (opcode == OPC_RTYPE) && (funct7 == F7_BIAS)) begin
            case (funct3)

              // BA_XWR: rs1 packs {row,col} (row in low bits, col above it)
              F3_XWR: begin
                cpu_x_we    <= 1'b1;
                cpu_x_row   <= rs1_val[ROW_W-1:0];
                cpu_x_col   <= rs1_val[ROW_W +: COL_W];
                cpu_x_wdata <= rs2_val;
              end

              // BA_BWR: rs1 low bits = col (bias vector per column)
              F3_BWR: begin
                cpu_b_we    <= 1'b1;
                cpu_b_col   <= rs1_val[COL_W-1:0];
                cpu_b_wdata <= rs2_val;
              end

              // BA_START
              F3_START: begin
                start_go <= 1'b1;
              end

              // BA_CRD: capture addr then wait for rvalid
              F3_CRD: begin
                crd_row_q <= rs1_val[ROW_W-1:0];
                crd_col_q <= rs1_val[ROW_W +: COL_W];
                rd_q      <= rd_addr;
                st        <= S_WAIT_C;
              end

              // BA_STAT: immediate return {.., C_valid, done, busy}
              F3_STAT: begin
                rd_we    <= 1'b1;
                rd_waddr <= rd_addr;
                rd_wdata <= {29'd0, C_valid, done, busy};
              end

              default: begin end
            endcase
          end
        end

        S_WAIT_C: begin
          // Hold read request until rvalid
          c_rd_en  <= 1'b1;
          c_rd_re  <= 1'b1;
          c_rd_row <= crd_row_q;
          c_rd_col <= crd_col_q;

          if (c_rd_rvalid) begin
            rd_we    <= 1'b1;
            rd_waddr <= rd_q;
            rd_wdata <= c_rd_rdata;
            st       <= S_IDLE;
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end


endmodule

`default_nettype wire
