/*
| instr         | opcode | funct7 | funct3 | explain                                                        |
| ------------- | ------ | ------ | ------ | -------------------------------------------------------------- |
| `AM_XWR`      | 0x33   | 0x05   | 3'b000 | 寫 logits SRAM：rs1 packed(row,col)，rs2=data (fp32 bits)       |
| `AM_START`    | 0x33   | 0x05   | 3'b001 | 啟動 argmax：掃描指定 row 的 N 個 logits，算出 max_idx          |
| `AM_STAT`     | 0x33   | 0x05   | 3'b010 | 讀狀態：回傳 {.., result_valid, done, busy}                     |
| `AM_RIDX`     | 0x33   | 0x05   | 3'b011 | 讀結果 index：rd = max_idx (可選：也可把 max_val 一起回傳/另讀) |
| `AM_RMAX`     | 0x33   | 0x05   | 3'b100 | 讀 max_val：rd = max_val (fp32 bits) (可選)                     |
*/

`timescale 1ns/1ps
`default_nettype none

// 建議你另外做：./src/EPU/argmax/argmax_core_top.sv
// 這裡先把 core 直接寫在同檔也可以。

module rv32i_rtype_argmax #(
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
  output logic        accel_C_valid   // 這裡可改名 accel_result_valid；我先沿用你的訊號名
);

  // ----------------------------
  // decode
  // ----------------------------
  localparam logic [6:0] OPCODE_RTYPE = 7'h33;
  localparam logic [6:0] FUNCT7_AM    = 7'h05;

  logic [6:0] opcode;
  logic [2:0] funct3;
  logic [6:0] funct7;

  assign opcode = instr[6:0];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];

  wire is_am = instr_valid && (opcode == OPCODE_RTYPE) && (funct7 == FUNCT7_AM);

  wire cmd_xwr   = is_am && (funct3 == 3'b000);
  wire cmd_start = is_am && (funct3 == 3'b001);
  wire cmd_stat  = is_am && (funct3 == 3'b010);
  wire cmd_ridx  = is_am && (funct3 == 3'b011);
  wire cmd_rmax  = is_am && (funct3 == 3'b100);

  // 你的習慣：ready 通常在可接收新指令時拉高
  // 這裡做最簡：除非正在 busy 做掃描，不然都 ready
  assign instr_ready = 1'b1;


  // ----------------------------
  // logits SRAM (簡化：用 reg array 模擬)
  // - 真實版你可以換成你既有的 SRAM wrapper
  // ----------------------------
  logic [DATA_W-1:0] xmem [0:M-1][0:N-1];

  wire [ROW_W-1:0] rs1_row = rs1_val[ROW_W+COL_W-1:COL_W];
  wire [COL_W-1:0] rs1_col = rs1_val[COL_W-1:0];

  // ----------------------------
  // fp32 compare helper (ordered int trick)
  // ----------------------------
  function automatic logic [31:0] fp32_to_ordered(input logic [31:0] u);
    if (u[31]) fp32_to_ordered = ~u;
    else       fp32_to_ordered = u ^ 32'h8000_0000;
  endfunction

  function automatic logic fp32_gt(input logic [31:0] a, input logic [31:0] b);
    logic [31:0] ao, bo;
    begin
      ao = fp32_to_ordered(a);
      bo = fp32_to_ordered(b);
      fp32_gt = (ao > bo);
    end
  endfunction

  // ----------------------------
  // FSM for argmax scan
  // ----------------------------
  typedef enum logic [1:0] {IDLE, RUN, DONE} state_t;
  state_t st;

  logic [ROW_W-1:0] cur_row;
  logic [COL_W-1:0] cur_col;

  logic [31:0] max_val;
  logic [COL_W-1:0] max_idx;

  // status flags
  assign accel_busy    = (st == RUN);
  assign accel_done    = (st == DONE);
  assign accel_C_valid = (st == DONE); // 結果可讀

  // ----------------------------
  // rd return
  // ----------------------------
  always_comb begin
    rd_we    = 1'b0;
    rd_waddr = rd_addr;
    rd_wdata = 32'h0;

    if (is_am) begin
      if (cmd_stat) begin
        rd_we    = 1'b1;
        // {.., result_valid, done, busy}
        rd_wdata = {29'h0, accel_C_valid, accel_done, accel_busy};
      end
      else if (cmd_ridx) begin
        rd_we    = 1'b1;
        rd_wdata = {{(32-COL_W){1'b0}}, max_idx};
      end
      else if (cmd_rmax) begin
        rd_we    = 1'b1;
        rd_wdata = max_val;
      end
    end
  end

  // ----------------------------
  // main sequential
  // ----------------------------
  integer i;
  always_ff @(posedge clk) begin
    if (rst) begin
      st      <= IDLE;
      cur_row <= '0;
      cur_col <= '0;
      max_val <= 32'h0;
      max_idx <= '0;
    end else begin
      // writes always allowed when not busy (你也可允許 busy 時寫，但會變複雜)
      if (cmd_xwr && (st != RUN)) begin
        xmem[rs1_row][rs1_col] <= rs2_val;
      end

      case (st)
        IDLE: begin
          if (cmd_start) begin
            // 你可以選：row 由 rs1_val 帶，或固定 0
            cur_row <= rs1_val[ROW_W-1:0]; // 若你想固定 row=0，改成 '0
            cur_col <= '0;

            // init max from col=0
            max_val <= xmem[rs1_val[ROW_W-1:0]][0];
            max_idx <= '0;

            st <= RUN;
          end
        end

        RUN: begin
          // 下一個 col
          if (cur_col == (N-1)) begin
            st <= DONE;
          end else begin
            cur_col <= cur_col + 1'b1;

            // 比較 xmem[row][cur_col+1] vs max_val
            // 注意：這裡用的是「更新後的 cur_col」，所以取值寫成 cur_col+1
            if (fp32_gt(xmem[cur_row][cur_col + 1'b1], max_val)) begin
              max_val <= xmem[cur_row][cur_col + 1'b1];
              max_idx <= cur_col + 1'b1;
            end
          end
        end

        DONE: begin
          // 你可以讓 CPU 讀完 RIDX/RMAX 或重新 START 才清掉
          // 這裡最簡：任何新的 START 會重跑；或你也可提供 CLEAR 指令
          if (cmd_start) begin
            cur_row <= rs1_val[ROW_W-1:0];
            cur_col <= '0;
            max_val <= xmem[rs1_val[ROW_W-1:0]][0];
            max_idx <= '0;
            st <= RUN;
          end
          // 或者讀 RIDX/RMAX 後回 IDLE（看你想要的語意）
        end
      endcase
    end
  end

endmodule
`default_nettype wire
