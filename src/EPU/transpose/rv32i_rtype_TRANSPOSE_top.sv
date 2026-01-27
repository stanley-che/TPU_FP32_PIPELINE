/*
| 指令         | funct3   | 說明                                  |
| ---------- | -------- | ----------------------------------- |
| `TP_AWR`   | `3'b000` | 寫 A SRAM：`rs1={row,col}`，`rs2=data` |
| `TP_START` | `3'b001` | 啟動 transpose                        |
| `TP_STAT`  | `3'b010` | 讀狀態 `{done,busy}`                   |
| `TP_BRD`   | `3'b011` | 讀 B SRAM：`rs1={row,col}` → `rd`     |

*/

// rv32i_rtype_TRANSPOSE_top.sv
`include "./src/EPU/transpose/transpose_ctrl.sv"
`timescale 1ns/1ps
`default_nettype none

module rv32i_rtype_TRANSPOSE_top #(
  parameter int unsigned NRows  = 8,
  parameter int unsigned NCols  = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,

  parameter int unsigned ROW_W  = (NRows<=1)?1:$clog2(NRows),
  parameter int unsigned COL_W  = (NCols<=1)?1:$clog2(NCols)
)(
  input  logic clk,
  input  logic rst,

  // ---------------- CPU custom instruction IF ----------------
  input  logic        instr_valid,
  output logic        instr_ready,
  input  logic [31:0] instr,
  input  logic [31:0] rs1_val,
  input  logic [31:0] rs2_val,
  input  logic [4:0]  rd_addr,

  output logic        rd_we,
  output logic [4:0]  rd_waddr,
  output logic [31:0] rd_wdata,

  // status
  output logic        accel_busy,
  output logic        accel_done
);

  // ==========================================================
  // Decode
  // ==========================================================
  wire [6:0] opcode = instr[6:0];
  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];

  localparam logic [6:0] OPC_RTYPE = 7'b0110011;
  localparam logic [6:0] F7_ACCEL  = 7'b0000001;

  localparam logic [2:0] F3_AWR    = 3'b000;
  localparam logic [2:0] F3_START  = 3'b001;
  localparam logic [2:0] F3_STAT   = 3'b010;
  localparam logic [2:0] F3_BRD    = 3'b011;

  // ==========================================================
  // Transpose controller IF
  // ==========================================================
  logic start_tr;
  logic busy, done;

  // A SRAM write (CPU preload)
  logic              a_we;
  logic [ROW_W-1:0]  a_row;
  logic [COL_W-1:0]  a_col;
  logic [DATA_W-1:0] a_wdata;

  // B SRAM read
  logic              b_rd_en;
  logic [ROW_W-1:0]  b_rd_row;
  logic [COL_W-1:0]  b_rd_col;
  logic [DATA_W-1:0] b_rd_rdata;
  logic              b_rd_rvalid;

  // ==========================================================
  // Instantiate transpose controller
  // ==========================================================
  transpose_ctrl #(
    .NRows(NRows),
    .NCols(NCols),
    .ADDR_W(ADDR_W),
    .Data_W(DATA_W)
  ) u_tr (
    .clk   (clk),
    .rst_n (!rst),

    .start (start_tr),
    .done  (done),
    .busy  (busy),

    // CPU preload A
    .cpu_a_we    (a_we),
    .cpu_a_row   (a_row),
    .cpu_a_col   (a_col),
    .cpu_a_wdata (a_wdata),

    // CPU read B
    .cpu_b_rd_en    (b_rd_en),
    .cpu_b_rd_row   (b_rd_row),
    .cpu_b_rd_col   (b_rd_col),
    .cpu_b_rd_rdata (b_rd_rdata),
    .cpu_b_rd_rvalid(b_rd_rvalid)
  );

  assign accel_busy = busy;
  assign accel_done = done;

  // ==========================================================
  // FSM (align with MAC style)
  // ==========================================================
  typedef enum logic [1:0] {S_IDLE, S_WAIT_B} state_t;
  state_t st;

  logic [ROW_W-1:0] b_row_q;
  logic [COL_W-1:0] b_col_q;
  logic [4:0]       rd_q;

  assign instr_ready = (st == S_IDLE);

  // ==========================================================
  // Main control
  // ==========================================================
  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;

      start_tr <= 1'b0;
      a_we     <= 1'b0;
      b_rd_en  <= 1'b0;

      rd_we    <= 1'b0;
      rd_waddr <= 5'd0;
      rd_wdata <= 32'd0;

      b_row_q  <= '0;
      b_col_q  <= '0;
      rd_q     <= '0;

    end else begin
      // default: clear 1-cycle pulses
      start_tr <= 1'b0;
      a_we     <= 1'b0;
      b_rd_en  <= 1'b0;
      rd_we    <= 1'b0;

      case (st)
        // ----------------------------------------------------
        S_IDLE: begin
          if (instr_valid && opcode==OPC_RTYPE && funct7==F7_ACCEL) begin
            case (funct3)

              // TP_AWR: preload A
              F3_AWR: begin
                a_we    <= 1'b1;
                a_row   <= rs1_val[ROW_W-1:0];
                a_col   <= rs1_val[ROW_W +: COL_W];
                a_wdata <= rs2_val;
              end

              // TP_START
              F3_START: begin
                start_tr <= 1'b1;
              end

              // TP_STAT
              F3_STAT: begin
                rd_we    <= 1'b1;
                rd_waddr <= rd_addr;
                rd_wdata <= {30'd0, done, busy};
              end

              // TP_BRD
              F3_BRD: begin
                b_row_q <= rs1_val[ROW_W-1:0];
                b_col_q <= rs1_val[ROW_W +: COL_W];
                rd_q    <= rd_addr;
                st      <= S_WAIT_B;
              end

              default: begin end
            endcase
          end
        end

        // ----------------------------------------------------
        S_WAIT_B: begin
          b_rd_en  <= 1'b1;
          b_rd_row <= b_row_q;
          b_rd_col <= b_col_q;

          if (b_rd_rvalid) begin
            rd_we    <= 1'b1;
            rd_waddr <= rd_q;
            rd_wdata <= b_rd_rdata;
            st       <= S_IDLE;
          end
        end
      endcase
    end
  end

endmodule

`default_nettype wire
