// rv32i_rtype_MAC_top.sv
/*
| instr       | opcode | funct7 | funct3 | explain                                      |
| ----------- | ------ | ------ | ------ | ---------------------------------------------|
| `ACC_WWR`   | 0x33   | 0x01   | 3'b000 | 寫 W SRAM：`rs1` packed(row,k)，`rs2`=data    |
| `ACC_XWR`   | 0x33   | 0x01   | 3'b001 | 寫 X SRAM：`rs1` packed(k,n)，`rs2`=data      |
| `ACC_START` | 0x33   | 0x01   | 3'b010 | 啟動：`rs1`=K_len（低 16）                    |
| `ACC_CRD`   | 0x33   | 0x01   | 3'b011 | 讀 C SRAM：`rs1` packed(row,col)，回傳到 `rd` |
| `ACC_STAT`  | 0x33   | 0x01   | 3'b100 | 讀狀態：回傳 `{..., C_valid, done, busy}`     |

*/
`include "./src/EPU/MAC/sramsa.sv"
`timescale 1ns/1ps
`default_nettype none


module rv32i_rtype_MAC_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  parameter int unsigned CONFLICT_POLICY_C = 1,

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N),
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX)
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

  // command constants
  localparam logic [6:0] OPC_RTYPE = 7'b0110011;
  localparam logic [6:0] F7_ACCEL  = 7'b0000001;

  localparam logic [2:0] F3_WWR    = 3'b000;
  localparam logic [2:0] F3_XWR    = 3'b001;
  localparam logic [2:0] F3_START  = 3'b010;
  localparam logic [2:0] F3_CRD    = 3'b011;
  localparam logic [2:0] F3_STAT   = 3'b100;

  // accel core wires
  logic        start_tile;
  logic [15:0] K_len_reg;
  logic        busy, done, C_valid;

  // cpu write ports
  logic                 cpu_w_we;
  logic [ROW_W-1:0]     cpu_w_row;
  logic [K_W-1:0]       cpu_w_k;
  logic [DATA_W-1:0]    cpu_w_wdata;
  logic [BYTE_W-1:0]    cpu_w_wmask;

  logic                 cpu_x_we;
  logic [K_W-1:0]       cpu_x_k;
  logic [N_W-1:0]       cpu_x_n;
  logic [DATA_W-1:0]    cpu_x_wdata;
  logic [BYTE_W-1:0]    cpu_x_wmask;

  // c read ports
  logic              c_rd_en, c_rd_re;
  logic [ROW_W-1:0]  c_rd_row;
  logic [COL_W-1:0]  c_rd_col;
  logic [DATA_W-1:0] c_rd_rdata;
  logic              c_rd_rvalid;

  // debug (unused)
  logic [M*N*DATA_W-1:0]    c_out_flat_o;
  logic [M*N-1:0]           c_valid_flat_o;
  logic [M*KMAX*DATA_W-1:0] W_tile_flat_dbg;
  logic [KMAX*N*DATA_W-1:0] X_tile_flat_dbg;

  tile_compute_system_top #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY_C(CONFLICT_POLICY_C),
    .ROW_W(ROW_W), .COL_W(COL_W), .N_W(N_W), .K_W(K_W)
  ) u_accel (
    .clk(clk),
    .rst(rst),

    .start(start_tile),
    .K_len(K_len_reg),
    .busy(busy),
    .done(done),

    .cpu_w_we(cpu_w_we),
    .cpu_w_row(cpu_w_row),
    .cpu_w_k(cpu_w_k),
    .cpu_w_wdata(cpu_w_wdata),
    .cpu_w_wmask(cpu_w_wmask),

    .cpu_x_we(cpu_x_we),
    .cpu_x_k(cpu_x_k),
    .cpu_x_n(cpu_x_n),
    .cpu_x_wdata(cpu_x_wdata),
    .cpu_x_wmask(cpu_x_wmask),

    .c_rd_en(c_rd_en),
    .c_rd_re(c_rd_re),
    .c_rd_row(c_rd_row),
    .c_rd_col(c_rd_col),
    .c_rd_rdata(c_rd_rdata),
    .c_rd_rvalid(c_rd_rvalid),

    .c_out_flat_o(c_out_flat_o),
    .c_valid_flat_o(c_valid_flat_o),
    .C_valid(C_valid),

    .W_tile_flat_dbg(W_tile_flat_dbg),
    .X_tile_flat_dbg(X_tile_flat_dbg)
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
  always @(*) begin
    instr_ready = (st == S_IDLE);
  end

  // main sequential controller
  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;

      // pulses default
      start_tile <= 1'b0;
      cpu_w_we   <= 1'b0;
      cpu_x_we   <= 1'b0;
      c_rd_en    <= 1'b0;
      c_rd_re    <= 1'b0;

      // defaults
      cpu_w_row   <= '0;
      cpu_w_k     <= '0;
      cpu_w_wdata <= '0;
      cpu_w_wmask <= {BYTE_W{1'b1}};

      cpu_x_k     <= '0;
      cpu_x_n     <= '0;
      cpu_x_wdata <= '0;
      cpu_x_wmask <= {BYTE_W{1'b1}};

      c_rd_row <= '0;
      c_rd_col <= '0;

      rd_we    <= 1'b0;
      rd_waddr <= 5'd0;
      rd_wdata <= 32'd0;

      K_len_reg <= 16'd0;

      crd_row_q <= '0;
      crd_col_q <= '0;
      rd_q      <= '0;

    end else begin
      // clear 1-cycle pulses every cycle
      start_tile <= 1'b0;
      cpu_w_we   <= 1'b0;
      cpu_x_we   <= 1'b0;
      c_rd_en    <= 1'b0;
      c_rd_re    <= 1'b0;
      rd_we      <= 1'b0;

      // keep masks default 0xF
      cpu_w_wmask <= {BYTE_W{1'b1}};
      cpu_x_wmask <= {BYTE_W{1'b1}};

      case (st)
        S_IDLE: begin
          if (instr_valid && (opcode == OPC_RTYPE) && (funct7 == F7_ACCEL)) begin
            case (funct3)

              // ACC_WWR: rs1 packs {row,k} in low bits
              F3_WWR: begin
                cpu_w_we    <= 1'b1;
                cpu_w_row   <= rs1_val[ROW_W-1:0];
                cpu_w_k     <= rs1_val[ROW_W +: K_W];
                cpu_w_wdata <= rs2_val;
              end

              // ACC_XWR: rs1 packs {k,n} in low bits
              F3_XWR: begin
                cpu_x_we    <= 1'b1;
                cpu_x_k     <= rs1_val[K_W-1:0];
                cpu_x_n     <= rs1_val[K_W +: N_W];
                cpu_x_wdata <= rs2_val;
              end

              // ACC_START: latch K_len and pulse start
              F3_START: begin
                K_len_reg  <= rs1_val[15:0];
                start_tile <= 1'b1;
              end

              // ACC_CRD: capture addr, then go wait
              F3_CRD: begin
                crd_row_q <= rs1_val[ROW_W-1:0];
                crd_col_q <= rs1_val[ROW_W +: COL_W];
                rd_q      <= rd_addr;
                st        <= S_WAIT_C;
              end

              // ACC_STAT: immediate return
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
          // keep driving read request
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
