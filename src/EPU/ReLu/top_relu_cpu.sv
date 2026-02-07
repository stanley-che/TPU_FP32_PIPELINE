
`include "./src/EPU/ReLu/bank_sram_ReLu.sv"
`timescale 1ns/1ps
`default_nettype none

module matrix_relu_cpu_ReLu #(
  parameter int unsigned NRows  = 8,
  parameter int unsigned NCols  = 8,
  parameter int unsigned NB     = 2,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned Data_W = 32
)(
  input  logic clk,
  input  logic rst_n,

  input  logic start,
  output logic done,
  output logic busy,

  // CPU write X
  input  logic              cpu_x_we,
  input  logic [31:0]       cpu_x_row, cpu_x_col,
  input  logic [Data_W-1:0] cpu_x_wdata,

  // CPU read Y
  input  logic              cpu_y_re,
  input  logic [31:0]       cpu_y_row, cpu_y_col,
  output logic [Data_W-1:0] cpu_y_rdata,
  output logic              cpu_y_rvalid
);

  // ------------------------------------------------------------
  // Helpers: linear address
  // addr = row*NCols + col
  // ------------------------------------------------------------
  function automatic [ADDR_W-1:0] lin_addr(input logic [31:0] r, input logic [31:0] c);
    logic [63:0] tmp;
    begin
      tmp = (r * NCols) + c;
      lin_addr = tmp[ADDR_W-1:0];
    end
  endfunction

  localparam int unsigned M   = 2; // 0=CPU, 1=ENGINE
  localparam int unsigned TOT = NRows * NCols;

  // ------------------------------------------------------------
  // Use packed arrays internally (Icarus-friendly)
  // ------------------------------------------------------------
  logic [M-1:0]             x_req_v, x_req_we;
  logic [M-1:0][ADDR_W-1:0] x_req_addr;
  logic [M-1:0][Data_W-1:0] x_req_wdata;
  logic [M-1:0]             x_req_ready;
  logic [M-1:0][Data_W-1:0] x_rsp_rdata;
  logic [M-1:0]             x_rsp_v;

  logic [M-1:0]             y_req_v, y_req_we;
  logic [M-1:0][ADDR_W-1:0] y_req_addr;
  logic [M-1:0][Data_W-1:0] y_req_wdata;
  logic [M-1:0]             y_req_ready;
  logic [M-1:0][Data_W-1:0] y_rsp_rdata;
  logic [M-1:0]             y_rsp_v;

  // ------------------------------------------------------------
  // If bank_sram ports are FLAT buses, pack/unpack here
  // (If your bank_sram is already array-ported, you can connect directly)
  // ------------------------------------------------------------
  wire [M*ADDR_W-1:0] x_req_addr_flat  = {x_req_addr[1],  x_req_addr[0]};
  wire [M*Data_W-1:0] x_req_wdata_flat = {x_req_wdata[1], x_req_wdata[0]};
  wire [M*Data_W-1:0] x_rsp_rdata_flat;

  wire [M*ADDR_W-1:0] y_req_addr_flat  = {y_req_addr[1],  y_req_addr[0]};
  wire [M*Data_W-1:0] y_req_wdata_flat = {y_req_wdata[1], y_req_wdata[0]};
  wire [M*Data_W-1:0] y_rsp_rdata_flat;

  // unpack responses (continuous assigns; OK for Icarus)
  assign x_rsp_rdata[0] = x_rsp_rdata_flat[Data_W-1:0];
  assign x_rsp_rdata[1] = x_rsp_rdata_flat[2*Data_W-1:Data_W];

  assign y_rsp_rdata[0] = y_rsp_rdata_flat[Data_W-1:0];
  assign y_rsp_rdata[1] = y_rsp_rdata_flat[2*Data_W-1:Data_W];

  bank_sram_ReLu #(.NB(NB), .ADDR_W(ADDR_W), .Data_W(Data_W), .M(M)) u_x_sram (
    .clk(clk),
    .rst_n(rst_n),
    .req_v(x_req_v),
    .req_we(x_req_we),
    .Req_addr(x_req_addr_flat),
    .Req_wData(x_req_wdata_flat),
    .req_ready(x_req_ready),
    .Rsp_rData(x_rsp_rdata_flat),
    .rsp_v(x_rsp_v)
  );

  bank_sram_ReLu #(.NB(NB), .ADDR_W(ADDR_W), .Data_W(Data_W), .M(M)) u_y_sram (
    .clk(clk),
    .rst_n(rst_n),
    .req_v(y_req_v),
    .req_we(y_req_we),
    .Req_addr(y_req_addr_flat),
    .Req_wData(y_req_wdata_flat),
    .req_ready(y_req_ready),
    .Rsp_rData(y_rsp_rdata_flat),
    .rsp_v(y_rsp_v)
  );

  // ------------------------------------------------------------
  // CPU address
  // ------------------------------------------------------------
  wire [ADDR_W-1:0] cpu_x_addr = lin_addr(cpu_x_row, cpu_x_col);
  wire [ADDR_W-1:0] cpu_y_addr = lin_addr(cpu_y_row, cpu_y_col);

  // ------------------------------------------------------------
  // CPU Y-read handshake (1 outstanding)
  // ------------------------------------------------------------
  logic              cpu_y_pending;
  logic [ADDR_W-1:0] cpu_y_addr_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cpu_y_pending <= 1'b0;
      cpu_y_addr_q  <= '0;
    end else begin
      if (cpu_y_re && !cpu_y_pending) begin
        cpu_y_pending <= 1'b1;
        cpu_y_addr_q  <= cpu_y_addr;
      end
      if (cpu_y_pending && y_req_ready[0]) begin
        cpu_y_pending <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------
  // Engine FSM
  // ------------------------------------------------------------
  typedef enum logic [1:0] {S_IDLE, S_READ, S_WAIT, S_WRITE} state_t;
  state_t st;

  logic [ADDR_W-1:0] idx, idx_hold;
  logic [Data_W-1:0] x_hold;
  logic [Data_W-1:0] y_relu;


  assign y_relu = x_hold[Data_W-1] ? '0 : x_hold;

  // ------------------------------------------------------------
  // SINGLE request generator (CPU + ENGINE)
  // ------------------------------------------------------------
  always_comb begin
    // defaults
    x_req_v     = '0;
    x_req_we    = '0;
    x_req_addr  = '0;
    x_req_wdata = '0;

    y_req_v     = '0;
    y_req_we    = '0;
    y_req_addr  = '0;
    y_req_wdata = '0;

    // ---- stream0 = CPU ----
    // X write
    x_req_v[0]     = cpu_x_we;
    x_req_we[0]    = cpu_x_we;
    x_req_addr[0]  = cpu_x_addr;
    x_req_wdata[0] = cpu_x_wdata;

    // Y read (held until accepted)
    y_req_v[0]     = cpu_y_pending;
    y_req_we[0]    = 1'b0;
    y_req_addr[0]  = cpu_y_addr_q;
    y_req_wdata[0] = '0;

    // ---- stream1 = ENGINE ----
    case (st)
      S_READ: begin
        x_req_v[1]    = 1'b1;
        x_req_we[1]   = 1'b0;
        x_req_addr[1] = idx;
      end

      S_WRITE: begin
        y_req_v[1]     = 1'b1;
        y_req_we[1]    = 1'b1;
        y_req_addr[1]  = idx_hold;
        y_req_wdata[1] = y_relu;
      end

      default: ;
    endcase
  end

  // ------------------------------------------------------------
  // CPU Y read response
  // ------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cpu_y_rdata  <= '0;
      cpu_y_rvalid <= 1'b0;
    end else begin
      cpu_y_rvalid <= y_rsp_v[0];
      cpu_y_rdata  <= y_rsp_rdata[0];
    end
  end

  // ------------------------------------------------------------
  // Engine state machine
  // ------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st       <= S_IDLE;
      idx      <= '0;
      idx_hold <= '0;
      x_hold   <= '0;
      busy     <= 1'b0;
      done     <= 1'b0;
    end else begin
      done <= 1'b0;

      case (st)
        S_IDLE: begin
          busy <= 1'b0;
          idx  <= '0;
          if (start) begin
            busy <= 1'b1;
            st   <= S_READ;
          end
        end

        S_READ: begin
          if (x_req_ready[1]) begin
            idx_hold <= idx;
            st       <= S_WAIT;
          end
        end

        S_WAIT: begin
          if (x_rsp_v[1]) begin
            x_hold <= x_rsp_rdata[1];
            st     <= S_WRITE;
          end
        end

        S_WRITE: begin
          if (y_req_ready[1]) begin
            if (idx == TOT-1) begin
              busy <= 1'b0;
              done <= 1'b1;
              st   <= S_IDLE;
            end else begin
              idx <= idx + 1;
              st  <= S_READ;
            end
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
