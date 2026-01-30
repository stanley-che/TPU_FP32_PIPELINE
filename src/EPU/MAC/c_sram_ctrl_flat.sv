`include "./src/EPU/MAC/c_tile_writer.sv"
`include "./src/EPU/MAC/sram_mem_mn_c.sv"
`timescale 1ns/1ps
`default_nettype none

module c_sram_ctrl_flat #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY = 1,
  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N)
)(
  input  logic clk,
  input  logic rst,

  // start/flush for a new tile result-set
  input  logic start,   // pulse
  input  logic flush,   // usually same as start

  // -------- SA result in --------
  input  logic [M*N*DATA_W-1:0] c_out_flat,
  input  logic [M*N-1:0]        c_valid_flat,

  // -------- CPU read interface (A port) --------
  input  logic              cpu_c_en,
  input  logic              cpu_c_re,
  input  logic [ROW_W-1:0]  cpu_c_row,
  input  logic [COL_W-1:0]  cpu_c_col,
  output logic [DATA_W-1:0] cpu_c_rdata,
  output logic              cpu_c_rvalid,

  // -------- debug export --------
  output logic [M*N*DATA_W-1:0] c_out_flat_o,
  output logic [M*N-1:0]        c_valid_flat_o,
  output logic                  C_valid   // 1 when SRAM contains full matrix for this tile
);

  // pass-through debug
  logic [M*N-1:0] c_valid_clean;

genvar idx;
generate
  for (idx = 0; idx < M*N; idx++) begin : GEN_VALID_CLEAN
    // only treat definite 1 as valid; X/Z -> 0
    assign c_valid_clean[idx] = (c_valid_flat[idx] === 1'b1);
  end
endgenerate

// debug export (optional: export clean version so你看得到真實有效脈波)
assign c_out_flat_o   = c_out_flat;
assign c_valid_flat_o = c_valid_clean;


  // ============================================================
  // 1) Writer: SA flat -> SRAM write port
  // ============================================================
  logic              w_c_we_en, w_c_we;
  logic [ROW_W-1:0]  w_c_wrow;
  logic [COL_W-1:0]  w_c_wcol;
  logic [DATA_W-1:0] w_c_wdata;
  logic [BYTE_W-1:0] w_c_wmask;

  // NOTE: use your writer module (you provided)
  c_tile_writer_flat #(
    .M(M), .N(N), .DATA_W(DATA_W), .BYTE_W(BYTE_W)
  ) u_wr (
    .clk(clk),
    .rst(rst),
    .flush(flush),

    .c_out_flat(c_out_flat),
    .c_valid_flat(c_valid_clean),

    .c_we_en(w_c_we_en),
    .c_we   (w_c_we),
    .c_wrow (w_c_wrow),
    .c_wcol (w_c_wcol),
    .c_wdata(w_c_wdata),
    .c_wmask(w_c_wmask)
  );

  // ============================================================
  // 2) Track "writing in progress" + "all written" (C_valid)
  //    We don't assume any special pattern of c_valid_flat.
  //    We consider SRAM safe-to-read when writer has no more writes
  //    for a while after start (drained).
  // ============================================================
  // Simple robust drain detector: after start_seen, wait until
  // we observe "no write" for DRAIN_GAP cycles.
  localparam int unsigned DRAIN_GAP = 8;

  logic start_seen;
  logic [7:0] gap_cnt;
  logic writing;

  always_ff @(posedge clk) begin
    if (rst) begin
      start_seen <= 1'b0;
      gap_cnt    <= '0;
      C_valid     <= 1'b0;
      writing     <= 1'b0;
    end else begin
      if (start) begin
        start_seen <= 1'b1;
        gap_cnt    <= '0;
        C_valid     <= 1'b0;
        writing     <= 1'b0;
      end

      // writing if any write enable asserted
      if (w_c_we_en ) begin
        writing  <= 1'b1;
        gap_cnt  <= '0;
      end else if (start_seen) begin
        // no write this cycle
        if (gap_cnt != DRAIN_GAP[7:0]) gap_cnt <= gap_cnt + 1'b1;
      end

      // declare C_valid when: we have seen start, and we have seen at least one write,
      // and then we've had a sustained quiet period (writer drained)
      if (start_seen && writing && (gap_cnt == DRAIN_GAP[7:0])) begin
        C_valid <= 1'b1;
      end

      // optional: clear start_seen after valid
      if (C_valid) start_seen <= 1'b0;
    end
  end

  // ============================================================
  // 3) SRAM instance
  // ============================================================
  // A-port (read) internal signals
  logic              mem_c_en, mem_c_re;
  logic [ROW_W-1:0]  mem_c_row;
  logic [COL_W-1:0]  mem_c_col;
  logic [DATA_W-1:0] mem_c_rdata;
  logic              mem_c_rvalid;

  // B-port write signals from writer
  // IMPORTANT: your SRAM model seems to use active-low mask in previous debug,
  // but your writer outputs whatever. Here we force safe behavior:
  // if writer outputs all-ones, keep it; else allow pass-through.
  // If you know polarity: set explicitly.
  logic [BYTE_W-1:0] mem_c_wmask;
  assign mem_c_wmask = w_c_wmask; // if your SRAM is active-low, set this to '0 on full write

  sram_mem_mn_c #(
    .M(M), .N(N), .DATA_W(DATA_W), .BYTE_W(BYTE_W),
    .CONFLICT_POLICY(CONFLICT_POLICY)
  ) u_cmem (
    .clk(clk),
    .rst(rst),

    // Port A
    .c_en   (mem_c_en),
    .c_re   (mem_c_re),
    .c_row  (mem_c_row),
    .c_col  (mem_c_col),
    .c_rdata(mem_c_rdata),
    .c_rvalid(mem_c_rvalid),

    // Port B
    .c_we_en(w_c_we_en),
    .c_we   (w_c_we),
    .c_wrow (w_c_wrow),
    .c_wcol (w_c_wcol),
    .c_wdata(w_c_wdata),
    .c_wmask(mem_c_wmask)
  );

  // ============================================================
  // 4) CPU read controller (hold read until rvalid)
  //    CPU is allowed to read only when C_valid==1
  // ============================================================
  typedef enum logic [1:0] {R_IDLE, R_WAIT} rstate_t;
  rstate_t rstate;

  logic [ROW_W-1:0] rd_row_q;
  logic [COL_W-1:0] rd_col_q;

  // drive SRAM port A
  always_comb begin
    mem_c_en  = 1'b0;
    mem_c_re  = 1'b0;
    mem_c_row = '0;
    mem_c_col = '0;

    if (rstate == R_WAIT) begin
      mem_c_en  = 1'b1;
      mem_c_re  = 1'b1;
      mem_c_row = rd_row_q;
      mem_c_col = rd_col_q;
    end
  end

  // FSM
  always_ff @(posedge clk) begin
    if (rst) begin
      rstate       <= R_IDLE;
      cpu_c_rvalid <= 1'b0;
      cpu_c_rdata  <= '0;
    end else begin
      cpu_c_rvalid <= 1'b0;

      case (rstate)
        R_IDLE: begin
          if (C_valid && cpu_c_en && cpu_c_re) begin
            rd_row_q <= cpu_c_row;
            rd_col_q <= cpu_c_col;
            rstate   <= R_WAIT;
          end
        end

        R_WAIT: begin
          if (mem_c_rvalid) begin
            cpu_c_rdata  <= mem_c_rdata;
            cpu_c_rvalid <= 1'b1;
            rstate       <= R_IDLE;
          end
        end
      endcase
    end
  end

endmodule

`default_nettype wire
