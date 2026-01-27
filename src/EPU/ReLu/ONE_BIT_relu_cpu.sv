`timescale 1ns/1ps
`default_nettype none

module ONE_BIT_relu_cpu #(
  parameter int unsigned NRows  = 8,
  parameter int unsigned NCols  = 8,
  parameter int unsigned NB    = 2,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned Data_W = 32
)(
  input  logic clk,
  input  logic rst_n,

  input  logic start,
  output logic done,
  output logic busy,

  input  logic              cpu_x_we,
  input  logic [31:0]       cpu_x_row, cpu_x_col,
  input  logic [Data_W-1:0] cpu_x_wdata,

  input  logic              cpu_y_re,
  input  logic [31:0]       cpu_y_row, cpu_y_col,
  output logic [Data_W-1:0] cpu_y_rdata,
  output logic              cpu_y_rvalid
);

  // super-minimal storage (replace with SRAM later)
  logic [Data_W-1:0] x_reg;

  // write
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x_reg <= '0;
    end else if (cpu_x_we) begin
      x_reg <= cpu_x_wdata;
    end
  end

  // start/busy/done (toy behavior)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0;
      done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start) begin
        busy <= 1'b1;
      end
      // pretend 1-cycle processing
      if (busy) begin
        busy <= 1'b0;
        done <= 1'b1;
      end
    end
  end

  // readback with 1-cycle latency and rvalid
  logic [Data_W-1:0] y_next;
  always@(*) begin
    // FP32 ReLU = sign bit check
    y_next = x_reg[Data_W-1] ? '0 : x_reg;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cpu_y_rdata  <= '0;
      cpu_y_rvalid <= 1'b0;
    end else begin
      cpu_y_rvalid <= 1'b0;
      if (cpu_y_re) begin
        cpu_y_rdata  <= y_next;
        cpu_y_rvalid <= 1'b1;
      end
    end
  end

endmodule

`default_nettype wire
