`include "./src/EPU/rv32i_rtype_argmax.sv"
`include "./src/EPU/rv32i_rtype_BIAS_ADD_top.sv"
`include "./src/EPU/rv32i_rtype_TRANSPOSE_top.sv"
`include "./src/EPU/rv32i_rtype_MAC_top.sv"
`include "./src/EPU/relu_custom_rtype.sv"
`timescale 1ns/1ps
`default_nettype none

module layer_sequencer #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8 ,
  parameter int unsigned KMAX   = 1024,
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 2,
  parameter int unsigned BYTE_W = (DATA_W/8),

  parameter int unsigned ROW_W  = (M<=1)?1:$clog2(M),
  parameter int unsigned COL_W  = (N<=1)?1:$clog2(N),
  parameter int unsigned N_W    = (N<=1)?1:$clog2(N),
  parameter int unsigned K_W    = (KMAX<=1)?1:$clog2(KMAX)
)(
  input  logic clk,
  input  logic rst,

  // CPU custom instruction interface
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

  // ------------------------------------------------------------
  // funct7 map (你現有的)
  // ------------------------------------------------------------
  localparam logic [6:0] OPC_RTYPE = 7'h33;
  localparam logic [6:0] F7_MAC    = 7'h01;
  localparam logic [6:0] F7_TP     = 7'h02;
  localparam logic [6:0] F7_RELU   = 7'h03;
  localparam logic [6:0] F7_BA     = 7'h04;
  localparam logic [6:0] F7_AM     = 7'h05;
  localparam logic [6:0] F7_LAYER  = 7'h06;

  wire [6:0] opcode = instr[6:0];
  wire [2:0] funct3 = instr[14:12];
  wire [6:0] funct7 = instr[31:25];

  // ------------------------------------------------------------
  // helper: make R-type instr with only funct7/funct3/opcode meaningful
  // ------------------------------------------------------------
  function automatic logic [31:0] mk_instr(input logic [6:0] f7, input logic [2:0] f3);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[6:0]   = OPC_RTYPE;
      t[14:12] = f3;
      t[31:25] = f7;
      mk_instr = t;
    end
  endfunction

  // packers:
  // (A) low-bit packing used by MAC/BA/AM: row in [ROW_W-1:0], col in [ROW_W +: COL_W]
  function automatic logic [31:0] pack_low_row_col(input int unsigned r, input int unsigned c);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]       = r[ROW_W-1:0];
      t[ROW_W +: COL_W]  = c[COL_W-1:0];
      pack_low_row_col = t;
    end
  endfunction

  // (B) hi16/lo16 packing used by TP/RELU wrappers: rs1[31:16]=row, rs1[15:0]=col
  function automatic logic [31:0] pack_16_row_col(input int unsigned r, input int unsigned c);
    pack_16_row_col = {r[15:0], c[15:0]};
  endfunction

  // MAC special packers
  function automatic logic [31:0] pack_mac_row_k(input int unsigned r, input int unsigned k);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[ROW_W-1:0]      = r[ROW_W-1:0];
      t[ROW_W +: K_W]   = k[K_W-1:0];
      pack_mac_row_k = t;
    end
  endfunction
  function automatic logic [31:0] pack_am_row_col(input int unsigned r, input int unsigned c);
  logic [31:0] t;
  begin
    t = 32'b0;
    t[COL_W-1:0]                  = c[COL_W-1:0];
    t[COL_W +: ROW_W]             = r[ROW_W-1:0];
    pack_am_row_col = t;
  end
  endfunction
  
  function automatic logic [31:0] pack_mac_k_n(input int unsigned k, input int unsigned nidx);
    logic [31:0] t;
    begin
      t = 32'b0;
      t[K_W-1:0]      = k[K_W-1:0];
      t[K_W +: N_W]   = nidx[N_W-1:0];
      pack_mac_k_n = t;
    end
  endfunction

  // ------------------------------------------------------------
  // Instantiate your existing wrappers
  // (compile 時把你的檔案一起丟給 iverilog)
  // ------------------------------------------------------------
  logic mac_iv, mac_ir;
  logic [31:0] mac_i, mac_rs1, mac_rs2; logic [4:0] mac_rd;
  logic mac_rd_we; logic [4:0] mac_rd_waddr; logic [31:0] mac_rd_wdata;
  logic mac_busy, mac_done, mac_cvalid;

  rv32i_rtype_MAC_top #(
    .M(M), .N(N), .KMAX(KMAX), .DATA_W(DATA_W)
  ) u_mac (
    .clk(clk), .rst(rst),
    .instr_valid(mac_iv), .instr_ready(mac_ir),
    .instr(mac_i), .rs1_val(mac_rs1), .rs2_val(mac_rs2), .rd_addr(mac_rd),
    .rd_we(mac_rd_we), .rd_waddr(mac_rd_waddr), .rd_wdata(mac_rd_wdata),
    .accel_busy(mac_busy), .accel_done(mac_done), .accel_C_valid(mac_cvalid)
  );

  logic ba_iv, ba_ir;
  logic [31:0] ba_i, ba_rs1, ba_rs2; logic [4:0] ba_rd;
  logic ba_rd_we; logic [4:0] ba_rd_waddr; logic [31:0] ba_rd_wdata;
  logic ba_busy, ba_done, ba_cvalid;

  rv32i_rtype_BIAS_ADD_top #(
    .M(M), .N(N), .DATA_W(DATA_W)
  ) u_ba (
    .clk(clk), .rst(rst),
    .instr_valid(ba_iv), .instr_ready(ba_ir),
    .instr(ba_i), .rs1_val(ba_rs1), .rs2_val(ba_rs2), .rd_addr(ba_rd),
    .rd_we(ba_rd_we), .rd_waddr(ba_rd_waddr), .rd_wdata(ba_rd_wdata),
    .accel_busy(ba_busy), .accel_done(ba_done), .accel_C_valid(ba_cvalid)
  );

  logic relu_iv, relu_ir;
  logic [31:0] relu_i, relu_rs1, relu_rs2; logic [4:0] relu_rd;
  logic relu_rd_we; logic [4:0] relu_rd_waddr; logic [31:0] relu_rd_wdata;
  logic relu_busy, relu_done, relu_cvalid;

  relu_custom_rtype #(
    .M(M), .N(N), .DATA_W(DATA_W), .ADDR_W(ADDR_W), .NB(NB)
  ) u_relu (
    .clk(clk), .rst(rst),
    .instr_valid(relu_iv), .instr_ready(relu_ir),
    .instr(relu_i), .rs1_val(relu_rs1), .rs2_val(relu_rs2), .rd_addr(relu_rd),
    .rd_we(relu_rd_we), .rd_waddr(relu_rd_waddr), .rd_wdata(relu_rd_wdata),
    .accel_busy(relu_busy), .accel_done(relu_done), .accel_C_valid(relu_cvalid)
  );

  logic tp_iv, tp_ir;
  logic [31:0] tp_i, tp_rs1, tp_rs2; logic [4:0] tp_rd;
  logic tp_rd_we; logic [4:0] tp_rd_waddr; logic [31:0] tp_rd_wdata;
  logic tp_busy, tp_done, tp_cvalid;

  transpose_custom_rtype #(
    .M(M), .N(N), .DATA_W(DATA_W), .ADDR_W(ADDR_W), .NB(NB)
  ) u_tp (
    .clk(clk), .rst(rst),
    .instr_valid(tp_iv), .instr_ready(tp_ir),
    .instr(tp_i), .rs1_val(tp_rs1), .rs2_val(tp_rs2), .rd_addr(tp_rd),
    .rd_we(tp_rd_we), .rd_waddr(tp_rd_waddr), .rd_wdata(tp_rd_wdata),
    .accel_busy(tp_busy), .accel_done(tp_done), .accel_C_valid(tp_cvalid)
  );

  logic am_iv, am_ir;
  logic [31:0] am_i, am_rs1, am_rs2; logic [4:0] am_rd;
  logic am_rd_we; logic [4:0] am_rd_waddr; logic [31:0] am_rd_wdata;
  logic am_busy, am_done, am_cvalid;

  rv32i_rtype_argmax #(
    .M(M), .N(N), .DATA_W(DATA_W)
  ) u_am (
    .clk(clk), .rst(rst),
    .instr_valid(am_iv), .instr_ready(am_ir),
    .instr(am_i), .rs1_val(am_rs1), .rs2_val(am_rs2), .rd_addr(am_rd),
    .rd_we(am_rd_we), .rd_waddr(am_rd_waddr), .rd_wdata(am_rd_wdata),
    .accel_busy(am_busy), .accel_done(am_done), .accel_C_valid(am_cvalid)
  );

  // ------------------------------------------------------------
  // layer config regs
  // ------------------------------------------------------------
  logic [15:0] K_len_reg;
  logic        use_tp_reg;
  logic [ROW_W-1:0] argmax_row_reg;

  // result regs
  logic [COL_W-1:0] class_idx_reg;
  logic [31:0]      max_val_reg;

  // ------------------------------------------------------------
  // Sequencer FSM
  // ------------------------------------------------------------
  typedef enum logic [5:0] {
    ST_IDLE,

    ST_MAC_START,
    ST_MAC_WAIT,

    ST_M2B_CRD, ST_M2B_WAITRD, ST_M2B_XWR,
    ST_BA_START, ST_BA_WAIT,

    ST_B2R_CRD, ST_B2R_WAITRD, ST_B2R_AWR,
    ST_RELU_START, ST_RELU_WAIT,

    ST_R2A_YRD, ST_R2A_WAITRD, ST_R2A_AMXWR,

    ST_AM_START, ST_AM_WAIT,
    ST_AM_RIDX, ST_AM_WAITRIDX,

    ST_DONE
  } st_t;

  st_t st;

  int unsigned r_cnt, c_cnt;
  logic [31:0] tmp_data;

  // one uop slot (target+payload)
  typedef enum logic [2:0] {T_NONE, T_MAC, T_BA, T_RELU, T_TP, T_AM} tgt_t;
  tgt_t u_tgt;
  logic u_valid;
  logic [31:0] u_instr, u_rs1, u_rs2;
  logic [4:0]  u_rd;

  // accept uop when target ready
  wire u_fire =
    u_valid && (
      (u_tgt==T_MAC  && mac_ir ) ||
      (u_tgt==T_BA   && ba_ir  ) ||
      (u_tgt==T_RELU && relu_ir) ||
      (u_tgt==T_TP   && tp_ir  ) ||
      (u_tgt==T_AM   && am_ir  )
    );

  // ------------------------------------------------------------
  // CPU dispatch vs sequencer ownership
  // ------------------------------------------------------------
  logic run_active;
  wire is_layer = (opcode==OPC_RTYPE) && (funct7==F7_LAYER);

  // While running: only allow LAY_STAT / LAY_RCLASS reads
  wire cpu_wants_stat   = is_layer && (funct3==3'b010);
  wire cpu_wants_rclass = is_layer && (funct3==3'b011);
  wire cpu_wants_cfg    = is_layer && (funct3==3'b000);
  wire cpu_wants_start  = is_layer && (funct3==3'b001);

  // direct decode for passthrough when NOT running
  typedef enum logic [2:0] {D_NONE, D_MAC, D_TP, D_RELU, D_BA, D_AM} dsel_t;
  dsel_t dsel;
  always_comb begin
    dsel = D_NONE;
    if (opcode==OPC_RTYPE) begin
      case (funct7)
        F7_MAC:  dsel = D_MAC;
        F7_TP:   dsel = D_TP;
        F7_RELU: dsel = D_RELU;
        F7_BA:   dsel = D_BA;
        F7_AM:   dsel = D_AM;
        default: dsel = D_NONE;
      endcase
    end
  end

  // ------------------------------------------------------------
  // Drive submodules (either CPU passthrough or sequencer uop)
  // ------------------------------------------------------------
  // default off
  always_comb begin
    // defaults
    mac_iv=0; mac_i='0; mac_rs1='0; mac_rs2='0; mac_rd='0;
    ba_iv =0; ba_i ='0; ba_rs1 ='0; ba_rs2 ='0; ba_rd ='0;
    relu_iv=0; relu_i='0; relu_rs1='0; relu_rs2='0; relu_rd='0;
    tp_iv=0; tp_i='0; tp_rs1='0; tp_rs2='0; tp_rd='0;
    am_iv=0; am_i='0; am_rs1='0; am_rs2='0; am_rd='0;

    if (run_active) begin
      // sequencer owns submodules
      case (u_tgt)
        T_MAC:  begin mac_iv=u_valid;  mac_i=u_instr;  mac_rs1=u_rs1;  mac_rs2=u_rs2;  mac_rd=u_rd; end
        T_BA:   begin ba_iv=u_valid;   ba_i=u_instr;   ba_rs1=u_rs1;   ba_rs2=u_rs2;   ba_rd=u_rd; end
        T_RELU: begin relu_iv=u_valid; relu_i=u_instr; relu_rs1=u_rs1; relu_rs2=u_rs2; relu_rd=u_rd; end
        T_TP:   begin tp_iv=u_valid;   tp_i=u_instr;   tp_rs1=u_rs1;   tp_rs2=u_rs2;   tp_rd=u_rd; end
        T_AM:   begin am_iv=u_valid;   am_i=u_instr;   am_rs1=u_rs1;   am_rs2=u_rs2;   am_rd=u_rd; end
        default: ;
      endcase
    end else begin
      // CPU passthrough
      case (dsel)
        D_MAC:  begin mac_iv=instr_valid;  mac_i=instr;  mac_rs1=rs1_val;  mac_rs2=rs2_val;  mac_rd=rd_addr; end
        D_TP:   begin tp_iv =instr_valid;  tp_i =instr;  tp_rs1 =rs1_val;  tp_rs2 =rs2_val;  tp_rd =rd_addr; end
        D_RELU: begin relu_iv=instr_valid; relu_i=instr; relu_rs1=rs1_val; relu_rs2=rs2_val; relu_rd=rd_addr; end
        D_BA:   begin ba_iv =instr_valid;  ba_i =instr;  ba_rs1 =rs1_val;  ba_rs2 =rs2_val;  ba_rd =rd_addr; end
        D_AM:   begin am_iv =instr_valid;  am_i =instr;  am_rs1 =rs1_val;  am_rs2 =rs2_val;  am_rd =rd_addr; end
        default: ;
      endcase
    end
  end

  // CPU instr_ready
  always_comb begin
    if (run_active) begin
      // only layer reads allowed
      instr_ready = cpu_wants_stat || cpu_wants_rclass;
      // allow cfg/start only when idle (not running)
      // (跑起來就不給改)
    end else begin
      if (is_layer) begin
        instr_ready = 1'b1; // cfg/start/stat/rclass 都可
      end else begin
        case (dsel)
          D_MAC:  instr_ready = mac_ir;
          D_TP:   instr_ready = tp_ir;
          D_RELU: instr_ready = relu_ir;
          D_BA:   instr_ready = ba_ir;
          D_AM:   instr_ready = am_ir;
          default: instr_ready = 1'b1;
        endcase
      end
    end
  end

  wire cpu_accept = instr_valid && instr_ready;

  // ------------------------------------------------------------
  // Writeback to CPU:
  // - passthrough when not running & not layer
  // - layer STAT/RCLASS are generated here
  // ------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      rd_we    <= 1'b0;
      rd_waddr <= 5'd0;
      rd_wdata <= 32'd0;
    end else begin
      rd_we <= 1'b0;

      if (!run_active && !is_layer) begin
        // passthrough
        case (dsel)
          D_MAC:  if (mac_rd_we)  begin rd_we<=1; rd_waddr<=mac_rd_waddr;  rd_wdata<=mac_rd_wdata; end
          D_TP:   if (tp_rd_we)   begin rd_we<=1; rd_waddr<=tp_rd_waddr;   rd_wdata<=tp_rd_wdata; end
          D_RELU: if (relu_rd_we) begin rd_we<=1; rd_waddr<=relu_rd_waddr; rd_wdata<=relu_rd_wdata; end
          D_BA:   if (ba_rd_we)   begin rd_we<=1; rd_waddr<=ba_rd_waddr;   rd_wdata<=ba_rd_wdata; end
          D_AM:   if (am_rd_we)   begin rd_we<=1; rd_waddr<=am_rd_waddr;   rd_wdata<=am_rd_wdata; end
          default: ;
        endcase
      end else if (is_layer && cpu_accept) begin
        // layer reads
        rd_we    <= 1'b1;
        rd_waddr <= rd_addr;
        if (funct3==3'b010) begin
          // STAT: {phase[7:0], done, busy}
          rd_wdata <= {22'd0, st[5:0], accel_done, accel_busy};
        end else if (funct3==3'b011) begin
          // RCLASS
          rd_wdata <= {24'd0, class_idx_reg};
        end else begin
          rd_wdata <= 32'd0;
        end
      end
    end
  end

  // ------------------------------------------------------------
  // accel status (for top-level)
  // ------------------------------------------------------------
  assign accel_busy    = run_active;
  //assign accel_done    = (st == ST_DONE);
  assign accel_C_valid = (st == ST_DONE);
  logic done_latched;

always_ff @(posedge clk) begin
  if (rst) begin
    done_latched <= 1'b0;
  end else begin
    // 新一輪 START 清掉 done
    if (!run_active && is_layer && cpu_accept && (funct3==3'b001)) begin
      done_latched <= 1'b0;
    end
    // 跑到 ST_DONE 就 latch 起來
    else if (st == ST_DONE) begin
      done_latched <= 1'b1;
    end
  end
end

assign accel_done = done_latched;

  // ------------------------------------------------------------
  // Sequencer main
  // ------------------------------------------------------------
  // uop slot management
  task automatic issue_uop(input tgt_t t, input logic [31:0] ii, input logic [31:0] a, input logic [31:0] b, input logic [4:0] rd);
    begin
      u_tgt   <= t;
      u_instr <= ii;
      u_rs1   <= a;
      u_rs2   <= b;
      u_rd    <= rd;
      u_valid <= 1'b1;
    end
  endtask

  always_ff @(posedge clk) begin
    if (rst) begin
      run_active     <= 1'b0;
      st             <= ST_IDLE;
      u_valid        <= 1'b0;
      u_tgt          <= T_NONE;
      u_instr        <= '0;
      u_rs1          <= '0;
      u_rs2          <= '0;
      u_rd           <= '0;

      K_len_reg      <= 16'd0;
      use_tp_reg     <= 1'b0;
      argmax_row_reg <= '0;

      r_cnt          <= 0;
      c_cnt          <= 0;
      tmp_data       <= 32'd0;
      class_idx_reg  <= '0;
      max_val_reg    <= 32'd0;
    end else begin
      // accept layer cfg/start when not running
      if (!run_active && is_layer && cpu_accept) begin
        if (funct3==3'b000) begin
          // CFG: rs1[15:0]=K_len, rs1[16]=use_tp, rs1[ROW_W-1:0] in low bits as argmax_row (你也可換成 rs1[23:20])
          K_len_reg      <= rs1_val[15:0];
          use_tp_reg     <= rs1_val[16];
          argmax_row_reg <= rs1_val[ROW_W-1:0];
        end else if (funct3==3'b001) begin
          // START
          run_active <= 1'b1;
          st         <= ST_MAC_START;
          r_cnt      <= 0;
          c_cnt      <= 0;
          u_valid    <= 1'b0;
        end
      end

      // uop fire: drop valid after accepted
      if (u_fire) begin
        u_valid <= 1'b0;
        u_tgt   <= T_NONE;
      end

      // ------------------------
      // sequencer FSM (only when run_active)
      // ------------------------
      if (run_active) begin
        case (st)

          ST_MAC_START: begin
            if (!u_valid) begin
              // ACC_START: funct3=010, funct7=01
              issue_uop(T_MAC, mk_instr(F7_MAC, 3'b010), {16'd0, K_len_reg}, 32'd0, 5'd0);
              st <= ST_MAC_WAIT;
            end
          end

          ST_MAC_WAIT: begin
            if (mac_done) begin
              r_cnt <= 0;
              c_cnt <= 0;
              st    <= ST_M2B_CRD;
            end
          end

          // move MAC C -> BA X
          ST_M2B_CRD: begin
            if (!u_valid) begin
              // ACC_CRD: funct3=011
              issue_uop(T_MAC, mk_instr(F7_MAC, 3'b011), pack_low_row_col(r_cnt,c_cnt), 32'd0, 5'd0);
              st <= ST_M2B_WAITRD;
            end
          end

          ST_M2B_WAITRD: begin
            if (mac_rd_we) begin
              tmp_data <= mac_rd_wdata;
              st <= ST_M2B_XWR;
            end
          end

          ST_M2B_XWR: begin
            if (!u_valid) begin
              // BA_XWR: funct3=000
              issue_uop(T_BA, mk_instr(F7_BA, 3'b000), pack_low_row_col(r_cnt,c_cnt), tmp_data, 5'd0);
              // next element
              if (c_cnt == N-1) begin
                c_cnt <= 0;
                if (r_cnt == M-1) begin
                  st <= ST_BA_START;
                end else begin
                  r_cnt <= r_cnt + 1;
                  st <= ST_M2B_CRD;
                end
              end else begin
                c_cnt <= c_cnt + 1;
                st <= ST_M2B_CRD;
              end
            end
          end

          ST_BA_START: begin
            if (!u_valid) begin
              issue_uop(T_BA, mk_instr(F7_BA, 3'b010), 32'd0, 32'd0, 5'd0);
              st <= ST_BA_WAIT;
            end
          end

          ST_BA_WAIT: begin
            if (ba_done) begin
              r_cnt <= 0;
              c_cnt <= 0;
              st <= ST_B2R_CRD;
            end
          end

          // move BA C -> RELU A
          ST_B2R_CRD: begin
            if (!u_valid) begin
              issue_uop(T_BA, mk_instr(F7_BA, 3'b011), pack_low_row_col(r_cnt,c_cnt), 32'd0, 5'd0);
              st <= ST_B2R_WAITRD;
            end
          end

          ST_B2R_WAITRD: begin
            if (ba_rd_we) begin
              tmp_data <= ba_rd_wdata;
              st <= ST_B2R_AWR;
            end
          end

          ST_B2R_AWR: begin
            if (!u_valid) begin
              // RELU_AWR: funct3=000 (注意 ReLU wrapper 用 16/16 pack)
              issue_uop(T_RELU, mk_instr(F7_RELU, 3'b000), pack_16_row_col(r_cnt,c_cnt), tmp_data, 5'd0);

              if (c_cnt == N-1) begin
                c_cnt <= 0;
                if (r_cnt == M-1) begin
                  st <= ST_RELU_START;
                end else begin
                  r_cnt <= r_cnt + 1;
                  st <= ST_B2R_CRD;
                end
              end else begin
                c_cnt <= c_cnt + 1;
                st <= ST_B2R_CRD;
              end
            end
          end

          ST_RELU_START: begin
            if (!u_valid) begin
              issue_uop(T_RELU, mk_instr(F7_RELU, 3'b001), 32'd0, 32'd0, 5'd0);
              st <= ST_RELU_WAIT;
            end
          end

          ST_RELU_WAIT: begin
            if (relu_done) begin
              // 最簡版先不走 TP，直接搬到 Argmax logits
              r_cnt <= 0;
              c_cnt <= 0;
              st <= ST_R2A_YRD;
            end
          end

          // move RELU Y -> Argmax logits
          ST_R2A_YRD: begin
            if (!u_valid) begin
              issue_uop(T_RELU, mk_instr(F7_RELU, 3'b011), pack_16_row_col(r_cnt,c_cnt), 32'd0, 5'd0);
              st <= ST_R2A_WAITRD;
            end
          end

          ST_R2A_WAITRD: begin
            if (relu_rd_we) begin
              tmp_data <= relu_rd_wdata;
              st <= ST_R2A_AMXWR;
            end
          end

          ST_R2A_AMXWR: begin
            if (!u_valid) begin
              // AM_XWR: funct3=000 (argmax 用 low-bit row/col pack)
              // 這裡把 relu 的 (r,c) 直接當 logits SRAM 的 (row,col)
              issue_uop(T_AM, mk_instr(F7_AM, 3'b000),pack_low_row_col(r_cnt,c_cnt),tmp_data, 5'd0);


              if (c_cnt == N-1) begin
                c_cnt <= 0;
                if (r_cnt == M-1) begin
                  st <= ST_AM_START;
                end else begin
                  r_cnt <= r_cnt + 1;
                  st <= ST_R2A_YRD;
                end
              end else begin
                c_cnt <= c_cnt + 1;
                st <= ST_R2A_YRD;
              end
            end
          end

          ST_AM_START: begin
            if (!u_valid) begin
              // AM_START: funct3=001, row in rs1_val[ROW_W-1:0]
              issue_uop(T_AM, mk_instr(F7_AM, 3'b001), {{(32-ROW_W){1'b0}}, argmax_row_reg}, 32'd0, 5'd0);
              st <= ST_AM_WAIT;
            end
          end

          ST_AM_WAIT: begin
            if (am_done) begin
              st <= ST_AM_RIDX;
            end
          end

          ST_AM_RIDX: begin
            if (!u_valid) begin
              issue_uop(T_AM, mk_instr(F7_AM, 3'b011), 32'd0, 32'd0, 5'd0);
              st <= ST_AM_WAITRIDX;
            end
          end

          ST_AM_WAITRIDX: begin
            if (am_rd_we) begin
              class_idx_reg <= am_rd_wdata[COL_W-1:0];
              st <= ST_DONE;
            end
          end

          ST_DONE: begin
            // 完成：保留結果，等 CPU 讀 LAY_RCLASS
            run_active <= 1'b0;
            st <= ST_IDLE;
          end

          default: st <= ST_IDLE;
        endcase
      end
    end
  end

endmodule

`default_nettype wire
