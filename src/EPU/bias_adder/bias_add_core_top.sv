
/*
| instr         | opcode | funct7 | funct3 | explain                                              |
| ------------- | ------ | ------ | ------ | --------------------------------------------------- |
| `BA_XWR`      | 0x33   | 0x04   | 3'b000 | 寫 X SRAM：rs1 packed(row,col)，rs2=data (fp32 bits) |
| `BA_BWR`      | 0x33   | 0x04   | 3'b001 | 寫 Bias：rs1 packed(col) (低 bits)，rs2=data         |
| `BA_START`    | 0x33   | 0x04   | 3'b010 | 啟動：開始做 C = X + B（整個 MxN）                   |
| `BA_CRD`      | 0x33   | 0x04   | 3'b011 | 讀 C SRAM：rs1 packed(row,col) → rd                  |
| `BA_STAT`     | 0x33   | 0x04   | 3'b100 | 讀狀態：回傳 {.., C_valid, done, busy}               |
*/

`include "./src/EPU/bias_adder/fp_adder_driver_ba.sv"
`timescale 1ns/1ps
`default_nettype none

module bias_add_core_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N)
)(
  input  logic clk,
  input  logic rst,

  // start/busy/done
  input  logic start,
  output logic busy,
  output logic done,
  output logic C_valid,

  // CPU write X
  input  logic              cpu_x_we,
  input  logic [ROW_W-1:0]  cpu_x_row,
  input  logic [COL_W-1:0]  cpu_x_col,
  input  logic [DATA_W-1:0] cpu_x_wdata,
  input  logic [BYTE_W-1:0] cpu_x_wmask,

  // CPU write Bias (vector)
  input  logic              cpu_b_we,
  input  logic [COL_W-1:0]  cpu_b_col,
  input  logic [DATA_W-1:0] cpu_b_wdata,
  input  logic [BYTE_W-1:0] cpu_b_wmask,

  // C read port (1-cycle latency)
  input  logic              c_rd_en,
  input  logic              c_rd_re,
  input  logic [ROW_W-1:0]  c_rd_row,
  input  logic [COL_W-1:0]  c_rd_col,
  output logic [DATA_W-1:0] c_rd_rdata,
  output logic              c_rd_rvalid
);

  // ----------------------------
  // Simple memories (behavioral SRAM)
  // ----------------------------
  logic [DATA_W-1:0] X_mem [0:M-1][0:N-1];
  logic [DATA_W-1:0] C_mem [0:M-1][0:N-1];
  logic              C_vld [0:M-1][0:N-1];
  logic [DATA_W-1:0] B_vec [0:N-1];

  // byte-mask helper (active-high mask = write-enable per byte)
  function automatic [DATA_W-1:0] apply_wmask(
    input [DATA_W-1:0] oldv,
    input [DATA_W-1:0] newv,
    input [BYTE_W-1:0] wmask
  );
    logic [DATA_W-1:0] outv;
    outv = oldv;
    for (int i=0; i<BYTE_W; i++) begin
      if (wmask[i]) outv[i*8 +: 8] = newv[i*8 +: 8];
    end
    return outv;
  endfunction

  // CPU writes
  always_ff @(posedge clk) begin
    if (rst) begin
      for (int n=0; n<N; n++) begin
        B_vec[n] <= '0;
      end
      for (int r=0; r<M; r++) begin
        for (int c=0; c<N; c++) begin
          X_mem[r][c] <= '0;
          C_mem[r][c] <= '0;
          C_vld[r][c] <= 1'b0;
        end
      end
    end else begin
      if (cpu_x_we) begin
        X_mem[cpu_x_row][cpu_x_col] <= apply_wmask(
          X_mem[cpu_x_row][cpu_x_col], cpu_x_wdata, cpu_x_wmask
        );
      end
      if (cpu_b_we) begin
        B_vec[cpu_b_col] <= apply_wmask(B_vec[cpu_b_col], cpu_b_wdata, cpu_b_wmask);
      end
    end
  end

  // ----------------------------
  // C read: 1-cycle latency
  // ----------------------------
  logic              rd_pending;
  logic [ROW_W-1:0]  rd_row_q;
  logic [COL_W-1:0]  rd_col_q;

  // FF: latch request + address
always_ff @(posedge clk) begin
  if (rst) begin
    rd_pending <= 1'b0;
    rd_row_q   <= '0;
    rd_col_q   <= '0;
  end else begin
    rd_pending <= (c_rd_en && c_rd_re);
    if (c_rd_en && c_rd_re) begin
      rd_row_q <= c_rd_row;
      rd_col_q <= c_rd_col;
    end
  end
end

// COMB: output data/valid from latched addr
always_comb begin
  c_rd_rvalid = rd_pending;
  c_rd_rdata  = C_mem[rd_row_q][rd_col_q];
end

  // ----------------------------
  // FP adder driver
  // ----------------------------
  logic        add_start;
  logic        add_busy;
  logic        add_done;
  logic [31:0] add_z;

  logic [31:0] a_cur, b_cur;

  fp_adder_driver_ba u_add (
    .clk   (clk),
    .rst   (rst),
    .start (add_start),
    .a_bits(a_cur),
    .b_bits(b_cur),
    .busy  (add_busy),
    .done  (add_done),
    .z_bits(add_z)
  );

  // ----------------------------
  // Compute FSM: C = X + B
  // ----------------------------
  typedef enum logic [2:0] {C_IDLE, C_PREP, C_LAUNCH, C_WAIT, C_WRITE} cstate_t;
  cstate_t cst;

  logic [ROW_W-1:0] r_idx;
  logic [COL_W-1:0] c_idx;

  // status
  always_comb begin
    busy    = (cst != C_IDLE);
    C_valid = done; // 你原本的簡化：整張算完就 valid
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      cst       <= C_IDLE;
      done      <= 1'b0;
      add_start <= 1'b0;
      r_idx     <= '0;
      c_idx     <= '0;
      a_cur     <= '0;
      b_cur     <= '0;
      for (int r=0; r<M; r++) begin
        for (int c=0; c<N; c++) begin
          C_vld[r][c] <= 1'b0;
        end
      end
    end else begin
      add_start <= 1'b0;

      case (cst)
        C_IDLE: begin
          if (start) begin
            done  <= 1'b0;
            r_idx <= '0;
            c_idx <= '0;

            for (int r=0; r<M; r++) begin
              for (int c=0; c<N; c++) begin
                C_vld[r][c] <= 1'b0;
              end
            end

            cst <= C_PREP;
          end
        end

        // PREP: 先把 operand 放到 a_cur/b_cur
        C_PREP: begin
          a_cur <= X_mem[r_idx][c_idx];
          b_cur <= B_vec[c_idx];
          cst   <= C_LAUNCH;
        end

        // LAUNCH: 下一拍再發 start，確保 driver latch 到新 operand
        C_LAUNCH: begin
          add_start <= 1'b1;
          if (add_busy) begin
            add_start <= 1'b0;
            cst       <= C_WAIT;
          end
        end

        C_WAIT: begin
          if (add_done) begin
            cst <= C_WRITE;
          end
        end

        C_WRITE: begin
          C_mem[r_idx][c_idx] <= add_z;
          C_vld[r_idx][c_idx] <= 1'b1;

          if (c_idx == COL_W'(N-1)) begin
            c_idx <= '0;
            if (r_idx == ROW_W'(M-1)) begin
              done <= 1'b1;
              cst  <= C_IDLE;
            end else begin
              r_idx <= r_idx + 1'b1;
              cst   <= C_PREP;
            end
          end else begin
            c_idx <= c_idx + 1'b1;
            cst   <= C_PREP;
          end
        end

        default: cst <= C_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
