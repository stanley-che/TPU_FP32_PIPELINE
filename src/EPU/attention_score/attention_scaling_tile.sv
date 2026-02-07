// attention_scaling_tile.sv
// ------------------------------------------------------------
// Scaling for Attention Score using existing GEMM engine
// ScaledScore = (1/sqrt(D_len)) * Score
//
// Implemented as matrix multiply:
//   W = alpha * I_T   (T x T)
//   X = Score         (T x T)
//   C = W * X         (T x T)  => scaled score
//
// NOTE:
// - No FP multiplier inside this module.
// - All multiplication happens in tile_compute_system_top.
// - alpha is generated from D_len via LUT (FP32 bits).
// ------------------------------------------------------------

`include "./src/EPU/attention_score/sramsa.sv"

`timescale 1ns/1ps
`default_nettype none

module attention_scaling_tile #(
  parameter int unsigned T      = 8,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned BYTE_W = (DATA_W/8),

  // K dimension for GEMM; for scaling we only need K=T
  parameter int unsigned KMAX   = T,

  localparam int unsigned ROW_W = (T<=1)?1:$clog2(T),
  localparam int unsigned COL_W = (T<=1)?1:$clog2(T),
  localparam int unsigned N_W   = (T<=1)?1:$clog2(T),
  localparam int unsigned K_W   = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // ---------------- control ----------------
  input  logic        start,
  input  logic [15:0] D_len,     
  output logic        busy,
  output logic        done,

  // ---------------- upstream score read (from attention_score_Mod) ----------------
  output logic              in_score_re,
  output logic [ROW_W-1:0]  in_score_tq,
  output logic [COL_W-1:0]  in_score_tk,
  input  logic [DATA_W-1:0] in_score_rdata,
  input  logic              in_score_rvalid,

  // ---------------- scaled C SRAM CPU read ----------------
  input  logic              c_rd_en,
  input  logic              c_rd_re,
  input  logic [ROW_W-1:0]  c_rd_row,
  input  logic [COL_W-1:0]  c_rd_col,
  output logic [DATA_W-1:0] c_rd_rdata,
  output logic              c_rd_rvalid,

  output logic              C_valid
);

  // ------------------------------------------------------------
  // D_len -> alpha = 1/sqrt(D_len) (FP32 bits) LUT
  // Extend as needed.
  // ------------------------------------------------------------
  logic [DATA_W-1:0] alpha_fp32;

  always_comb begin
    case (D_len)
      16'd1:    alpha_fp32 = 32'h3F800000; // 1.0
      16'd2:    alpha_fp32 = 32'h3F3504F3; // 0.70710677
      16'd3:    alpha_fp32 = 32'h3F13CD3A; // 0.57735026
      16'd4:    alpha_fp32 = 32'h3F000000; // 0.5
      16'd5:    alpha_fp32 = 32'h3EE4F92E; // ~0.4472136
      16'd6:    alpha_fp32 = 32'h3EDDB3D7; // ~0.4082483
      16'd7:    alpha_fp32 = 32'h3ED069F3; // ~0.3779645
      16'd8:    alpha_fp32 = 32'h3EB504F3; // 0.35355338
      16'd9:    alpha_fp32 = 32'h3EAAAAAB; // 0.33333334
      16'd10:   alpha_fp32 = 32'h3EA35D8E; // ~0.31622776
      16'd12:   alpha_fp32 = 32'h3E942477; // ~0.28867513
      16'd16:   alpha_fp32 = 32'h3E800000; // 0.25
      16'd32:   alpha_fp32 = 32'h3E2AAAAB; // ~0.17677669
      16'd64:   alpha_fp32 = 32'h3E000000; // 0.125
      16'd128:  alpha_fp32 = 32'h3D800000; // 0.0625
      16'd256:  alpha_fp32 = 32'h3D000000; // 0.03125
      16'd512:  alpha_fp32 = 32'h3C800000; // 0.015625
      16'd1024: alpha_fp32 = 32'h3C000000; // 0.0078125
      default:  alpha_fp32 = 32'h3F800000; // fallback = 1.0
    endcase
  end

  // ------------------------------------------------------------
  // Instantiate tile_compute_system_top as GEMM scaling engine
  // M=T, N=T, KMAX=T, K_len=T
  // ------------------------------------------------------------
  logic tile_start, tile_busy, tile_done;

  // CPU write W SRAM
  logic                 cpu_w_we;
  logic [ROW_W-1:0]     cpu_w_row;
  logic [K_W-1:0]       cpu_w_k;
  logic [DATA_W-1:0]    cpu_w_wdata;
  logic [BYTE_W-1:0]    cpu_w_wmask;

  // CPU write X SRAM
  logic                 cpu_x_we;
  logic [K_W-1:0]       cpu_x_k;
  logic [N_W-1:0]       cpu_x_n;
  logic [DATA_W-1:0]    cpu_x_wdata;
  logic [BYTE_W-1:0]    cpu_x_wmask;

  // debug (unused)
  logic [T*T*DATA_W-1:0] c_out_flat_o;
  logic [T*T-1:0]        c_valid_flat_o;
  logic [T*KMAX*DATA_W-1:0] W_tile_flat_dbg;
  logic [KMAX*T*DATA_W-1:0] X_tile_flat_dbg;

  tile_compute_system_top #(
    .M(T), .N(T), .KMAX(KMAX),
    .DATA_W(DATA_W), .BYTE_W(BYTE_W)
  ) u_scale_gemm (
    .clk(clk),
    .rst(rst),

    .start(tile_start),
    .K_len(T[15:0]),
    .busy(tile_busy),
    .done(tile_done),

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

  // ------------------------------------------------------------
  // FSM: Write W=alpha*I, then copy Score -> X, then start GEMM
  // ------------------------------------------------------------
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_W_WRITE,
    ST_X_REQ,
    ST_X_WAIT,
    ST_START,
    ST_WAIT,
    ST_DONE
  } st_t;

  st_t st;

  logic [ROW_W-1:0] i;
  logic [COL_W-1:0] j;

  // outputs defaults (combinational)
  always_comb begin
  // mask：看你的 SRAM 定義（下面我會講）
    cpu_w_wmask = {BYTE_W{1'b1}};
    cpu_x_wmask = {BYTE_W{1'b1}};

    busy = (st != ST_IDLE) && (st != ST_DONE);
    done = (st == ST_DONE);
  end


  // FSM sequential
  always_ff @(posedge clk) begin
    if (rst) begin
      st <= ST_IDLE;
      i  <= '0;
      j  <= '0;
      cpu_w_we    <= 1'b0;
      cpu_x_we    <= 1'b0;
      in_score_re <= 1'b0;
      tile_start  <= 1'b0;
      cpu_w_row   <= '0;
      cpu_w_k     <= '0;
      cpu_w_wdata <= '0;

      cpu_x_k     <= '0;
      cpu_x_n     <= '0;
      cpu_x_wdata <= '0;

      in_score_tq <= '0;
      in_score_tk <= '0;
    end else begin
    cpu_w_we    <= 1'b0;
    cpu_x_we    <= 1'b0;
    in_score_re <= 1'b0;
    tile_start  <= 1'b0;
      case (st)
        ST_IDLE: begin
          i <= '0;
          j <= '0;
          if (start) st <= ST_W_WRITE;
        end

        // ----------------------------
        // Write W = alpha * I (T x T)
        // ----------------------------
        ST_W_WRITE: begin
          cpu_w_we    <= 1'b1;
          cpu_w_row   <= i;
          cpu_w_k     <= j;
          cpu_w_wdata <= (i == j) ? alpha_fp32 : 32'h0000_0000;

          if (j == T-1) begin
            j <= '0;
            if (i == T-1) begin
              i <= '0;
              st <= ST_X_REQ;
            end else begin
              i <= i + 1'b1;
            end
          end else begin
            j <= j + 1'b1;
          end
        end

        // ----------------------------
        // Request upstream score read: S[i][j]
        // ----------------------------
        ST_X_REQ: begin
          in_score_re <= 1'b1;
          in_score_tq <= i;
          in_score_tk <= j;
          st <= ST_X_WAIT;
        end

        // ----------------------------
        // Wait rvalid then write into X[k][n]
        // X[i][j] = S[i][j]
        // ----------------------------
        ST_X_WAIT: begin
          if (in_score_rvalid) begin
            cpu_x_we    <= 1'b1;
            cpu_x_k     <= i;
            cpu_x_n     <= j;
            cpu_x_wdata <= in_score_rdata;

            if (j == T-1) begin
              j <= '0;
              if (i == T-1) begin
                i <= '0;
                st <= ST_START;
              end else begin
                i <= i + 1'b1;
                st <= ST_X_REQ;
              end
            end else begin
              j <= j + 1'b1;
              st <= ST_X_REQ;
            end
          end
        end

        // ----------------------------
        // Start GEMM engine (1-cycle pulse)
        // ----------------------------
        ST_START: begin
          tile_start <= 1'b1;
          st <= ST_WAIT;
        end

        // ----------------------------
        // Wait for scaled result valid
        // ----------------------------
        ST_WAIT: begin
          if (C_valid) st <= ST_DONE;
        end

        // ----------------------------
        // Done; wait start deassert to re-arm (optional)
        // ----------------------------
        ST_DONE: begin
          if (!start) st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
