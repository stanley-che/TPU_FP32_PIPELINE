`timescale 1ns/1ps
`default_nettype none

module sram_word_ab #(
  parameter int unsigned ADDR_W = 10,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),
  parameter int unsigned CONFLICT_POLICY = 1   // 0=READ_FIRST, 1=WRITE_FIRST
)(
  input  logic                  clk,
  input  logic                  rst,

  // Port A (READ ONLY)
  input  logic                  a_en,
  input  logic                  a_re,
  input  logic [ADDR_W-1:0]     a_addr,
  output logic [DATA_W-1:0]     a_rdata,
  output logic                  a_rvalid,

  // Port B (WRITE ONLY)
  input  logic                  b_en,
  input  logic                  b_we,
  input  logic [ADDR_W-1:0]     b_addr,
  input  logic [DATA_W-1:0]     b_wdata,
  input  logic [BYTE_W-1:0]     b_wmask
);

  localparam int unsigned DEPTH = (1 << ADDR_W);

  // word-addressed memory (clean + tool-friendly)
  logic [DATA_W-1:0] mem [0:DEPTH-1];

  // registered read request
  logic              a_re_q;
  logic [ADDR_W-1:0] a_addr_q;

  function automatic [DATA_W-1:0] apply_mask(
    input [DATA_W-1:0] old_data,
    input [DATA_W-1:0] wdata,
    input [BYTE_W-1:0] wmask
  );
    automatic logic [DATA_W-1:0] tmp;
    begin
      tmp = old_data;
      for (int i = 0; i < BYTE_W; i++) begin
        if (wmask[i]) tmp[i*8 +: 8] = wdata[i*8 +: 8];
      end
      return tmp;
    end
  endfunction

  wire b_do_w    = b_en && b_we;
  wire same_addr = (b_addr == a_addr_q);

  // base read comes from mem at latched address
  wire [DATA_W-1:0] a_base_now = mem[a_addr_q];

  // WRITE_FIRST forwarding (if same addr in the cycle we output)
  wire [DATA_W-1:0] a_forwarded =
    (CONFLICT_POLICY == 1 && b_do_w && same_addr)
      ? apply_mask(a_base_now, b_wdata, b_wmask)
      : a_base_now;

  always_ff @(posedge clk) begin
    if (rst) begin
      a_re_q   <= 1'b0;
      a_addr_q <= '0;
      a_rvalid <= 1'b0;
      a_rdata  <= '0;
      // 可選：不要 reset 清 mem（大矩陣很慢）；要清也可加 for loop
    end else begin
      // output for previous request
      a_rvalid <= a_re_q;
      a_rdata  <= a_forwarded;

      // latch next request
      a_re_q   <= (a_en && a_re);
      if (a_en && a_re) a_addr_q <= a_addr;  // 只在真的讀時 latch

      // write
      if (b_do_w) begin
        mem[b_addr] <= apply_mask(mem[b_addr], b_wdata, b_wmask);
      end
    end
  end

endmodule

`default_nettype wire
