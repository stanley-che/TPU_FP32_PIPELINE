/*
31        25 24   20 19   15 14  12 11   7 6      0
+-----------+-------+-------+-------+------+--------+
|  funct7   |  rs2  |  rs1  | funct3|  rd  | opcode |
+-----------+-------+-------+-------+------+--------+

opcode = 0x33 (RV32I R-type)

MAC（Matrix Multiply / Accumulate）
funct7 = 0x01
| inst          | funct3 | rs1         | rs2           | rd | 硬體行為             
| ------------- | ------ | ----------- | ------------- | -- | -------------------- 
| **MAC_AWR**   | `000`  | `{row,col}` | `A[row][col]` | x  | 寫入 Matrix A SRAM  
| **MAC_BWR**   | `001`  | `{row,col}` | `B[row][col]` | x  | 寫入 Matrix B SRAM  
| **MAC_START** | `010`  | x           | x             | x  | 啟動 C = A × B      
| **MAC_CRD**   | `011`  | `{row,col}` | x             | rd | 讀取結果 C[row][col]  
| **MAC_STAT**  | `100`  | x           | x             | rd | 回傳 `{busy, done}` 

TRANSPOSE（Matrix Transpose）
funct7 = 0x02
| inst         | funct3 | rs1         | rs2           | rd | 硬體行為              
| ------------ | ------ | ----------- | ------------- | -- | ----------------- 
| **TP_AWR**   | `000`  | `{row,col}` | `A[row][col]` | x  | 寫入 A SRAM         
| **TP_START** | `001`  | x           | x             | x  | 啟動轉置 B = Aᵀ       
| **TP_STAT**  | `010`  | x           | x             | rd | 回傳 `{busy, done}` 
| **TP_BRD**   | `011`  | `{row,col}` | x             | rd | 讀取轉置後 B[row][col] 

ReLU（Activation）
funct7 = 0x03
| inst           | funct3 | rs1         | rs2           | rd | 硬體行為              
| -------------- | ------ | ----------- | ------------- | -- | ----------------- 
| **RELU_XWR**   | `000`  | `{row,col}` | `X[row][col]` | x  | 寫入輸入 X            
| **RELU_START** | `001`  | x           | x             | x  | 啟動 ReLU           
| **RELU_STAT**  | `010`  | x           | x             | rd | 回傳 `{busy, done}` 
| **RELU_YRD**   | `011`  | `{row,col}` | x             | rd | 讀取輸出 Y            


BIAS_ADD（Bias Addition）
funct7 = 0x04
| inst         | funct3 | rs1         | rs2           | rd | 硬體行為              
| ------------ | ------ | ----------- | ------------- | -- | ----------------- 
| **BA_XWR**   | `000`  | `{row,col}` | `X[row][col]` | x  | 寫入 X SRAM         
| **BA_BWR**   | `001`  | `{col}`     | `B[col]`      | x  | 寫入 Bias           
| **BA_START** | `010`  | x           | x             | x  | 啟動 C = X + B      
| **BA_CRD**   | `011`  | `{row,col}` | x             | rd | 讀取結果              
| **BA_STAT**  | `100`  | x           | x             | rd | 回傳 `{busy, done}` 

ARGMAX（Classification）
funct7 = 0x05
| inst          | funct3 | rs1     | rs2    | rd | 硬體行為              
| ------------- | ------ | ------- | ------ | -- | ----------------- 
| **ARG_WR**    | `000`  | `{idx}` | `data` | x  | 寫入向量資料           
| **ARG_START** | `001`  | x       | x      | x  | 啟動 argmax 搜尋      
| **ARG_STAT**  | `010`  | x       | x      | rd | 回傳 `{busy, done}` 
| **ARG_RD**    | `011`  | x       | x      | rd | 回傳最大值 index       

IMG_IF（Image SRAM Interface）
funct7 = 0x06
| inst        | funct3 | rs1         | rs2          | rd | 硬體行為                              
| ----------- | ------ | ----------- | ------------ | -- | ---------------------------------- ---------
| **IM_WR**   | `000`  | `{row,col}` | `data(fp32)` | x  | 將 `data` 寫入 IMG SRAM[row][col]       
| **IM_RD**   | `001`  | `{row,col}` | x            | rd | 從 IMG SRAM[row][col] 讀出資料（下一拍回 rd） 
| **IM_STAT** | `010`  | x           | x            | rd | 回傳狀態 `{busy, done, rvalid}`        

*/
// === include your wrappers (paths依你的專案調整) ===


`include "./src/EPU/layer_sequencer.sv"           // <== 你上面那個 layer_sequencer

`include "./src/EPU/rv32i_rtype_layer_img_if.sv"

`timescale 1ns/1ps
`default_nettype none

module rv32i_rtype_accel_top #(
  parameter int unsigned M      = 8,
  parameter int unsigned N      = 8,
  parameter int unsigned KMAX   = 1024,

  parameter int unsigned IMG_H  = 8,
  parameter int unsigned IMG_W  = 8,

  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 16,
  parameter int unsigned NB     = 2,
  parameter int unsigned BYTE_W = (DATA_W/8)
)(
  input  logic clk,
  input  logic rst,

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

  // ============================================================
  // Decode (CPU-visible)
  // ============================================================
  logic [6:0] opcode, funct7;
  logic [2:0] funct3;

  assign opcode = instr[6:0];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];

  localparam logic [6:0] OPC_RTYPE  = 7'h33;

  // CPU-visible mapping:
  localparam logic [6:0] F7_IMG_CPU   = 7'h06; // IMG
  localparam logic [6:0] F7_LAYER_CPU = 7'h07; // LAYER

  // layer_sequencer internal mapping (your existing file):
  localparam logic [6:0] F7_LAYER_INT = 7'h06; // do NOT change your layer_sequencer

  // ============================================================
  // Choose path: IMG vs LAYER/OTHERS
  // ============================================================
  wire is_img_cpu   = (opcode == OPC_RTYPE) && (funct7 == F7_IMG_CPU);
  wire is_layer_cpu = (opcode == OPC_RTYPE) && (funct7 == F7_LAYER_CPU);

  // gate valids
  wire v_img   = instr_valid && is_img_cpu;
  wire v_layer = instr_valid && !is_img_cpu; // everything else goes to layer_sequencer

  // ============================================================
  // Remap instr for layer_sequencer: 0x07 -> 0x06
  // ============================================================
  logic [31:0] instr_to_layer;
  always_comb begin
    instr_to_layer = instr;
    if (is_layer_cpu) begin
      instr_to_layer[31:25] = F7_LAYER_INT; // 0x07 (CPU) -> 0x06 (internal)
    end
  end

  // ============================================================
  // Child wires
  // ============================================================
  // ---- IMG ----
  logic        rdy_img;
  logic        we_img;
  logic [4:0]  waddr_img;
  logic [31:0] wdata_img;
  logic        busy_img, done_img, cval_img;

  // ---- LAYER (includes passthrough MAC/TP/RELU/BA/AM) ----
  logic        rdy_layer;
  logic        we_layer;
  logic [4:0]  waddr_layer;
  logic [31:0] wdata_layer;
  logic        busy_layer, done_layer, cval_layer;

  // ============================================================
  // Instantiate IMG_IF (funct7=0x06 in that module)
  // ============================================================
  rv32i_rtype_layer_img_if #(
    .IMG_H(IMG_H), .IMG_W(IMG_W),
    .ADDR_W(ADDR_W), .DATA_W(DATA_W), .BYTE_W(BYTE_W)
  ) u_img (
    .clk(clk), .rst(rst),
    .instr_valid(v_img),
    .instr_ready(rdy_img),
    .instr(instr),            // keep original (expects funct7=0x06)
    .rs1_val(rs1_val),
    .rs2_val(rs2_val),
    .rd_addr(rd_addr),
    .rd_we(we_img),
    .rd_waddr(waddr_img),
    .rd_wdata(wdata_img),
    .accel_busy(busy_img),
    .accel_done(done_img),
    .accel_C_valid(cval_img)
  );

  // ============================================================
  // Instantiate layer_sequencer (internal uses funct7=0x06 for LAYER)
  // ============================================================
  layer_sequencer #(
    .M(M), .N(N), .KMAX(KMAX),
    .DATA_W(DATA_W), .ADDR_W(ADDR_W),
    .NB(NB), .BYTE_W(BYTE_W)
  ) u_layer (
    .clk(clk), .rst(rst),
    .instr_valid(v_layer),
    .instr_ready(rdy_layer),
    .instr(instr_to_layer),   // <-- remapped for LAYER
    .rs1_val(rs1_val),
    .rs2_val(rs2_val),
    .rd_addr(rd_addr),
    .rd_we(we_layer),
    .rd_waddr(waddr_layer),
    .rd_wdata(wdata_layer),
    .accel_busy(busy_layer),
    .accel_done(done_layer),
    .accel_C_valid(cval_layer)
  );

  // ============================================================
  // Selection latch (avoid same-cycle STAT/RD response confusion)
  // ============================================================
  typedef enum logic [0:0] {SEL_LAY = 1'b0, SEL_IMG = 1'b1} sel_t;
  logic sel_q_valid;
  sel_t sel_q;
  sel_t sel_eff;

  // effective select:
  // - if we have a pending-response latch, use it
  // - else decode this cycle
  always @(*) begin
  if (sel_q_valid) begin
    sel_eff = sel_q;
  end else begin
    if (is_img_cpu) sel_eff = SEL_IMG;
    else            sel_eff = SEL_LAY;
  end
end


  // instr_ready mux
  always_comb begin
    instr_ready = (sel_eff == SEL_IMG) ? rdy_img : rdy_layer;
  end

  wire accept = instr_valid && instr_ready;

  // Does this accepted instruction expect a writeback later?
  // - IMG: IM_RD (f3=001), IM_STAT (f3=010) -> rd_we later
  // - Layer path: wrappers/stat/reads + LAYER_STAT/RCLASS -> rd_we later
  function automatic logic expects_wb_layer(input logic [31:0] iw);
    logic [6:0] op, f7;
    logic [2:0] f3;
    begin
      op = iw[6:0];
      f3 = iw[14:12];
      f7 = iw[31:25];

      // layer_sequencer sees LAYER as 0x06 internally
      if (op != OPC_RTYPE) expects_wb_layer = 1'b0;
      else begin
        case (f7)
          7'h01: expects_wb_layer = (f3==3'b011) || (f3==3'b100); // MAC_CRD / MAC_STAT
          7'h02: expects_wb_layer = (f3==3'b010) || (f3==3'b011); // TP_STAT / TP_BRD
          7'h03: expects_wb_layer = (f3==3'b010) || (f3==3'b011); // RELU_STAT / RELU_YRD
          7'h04: expects_wb_layer = (f3==3'b011) || (f3==3'b100); // BA_CRD / BA_STAT
          7'h05: expects_wb_layer = (f3==3'b010) || (f3==3'b011); // AM_STAT / AM_RD
          7'h06: expects_wb_layer = (f3==3'b010) || (f3==3'b011); // LAY_STAT / LAY_RCLASS
          default: expects_wb_layer = 1'b0;
        endcase
      end
    end
  endfunction

  logic expects_wb;
  always@(*) begin
    if (is_img_cpu) begin
      expects_wb = (funct3 == 3'b001) || (funct3 == 3'b010); // IM_RD / IM_STAT
    end else begin
      expects_wb = expects_wb_layer(instr_to_layer);
    end
  end

  // response pulse from selected output
  logic resp_pulse;
  always_comb begin
    resp_pulse = (sel_eff == SEL_IMG) ? we_img : we_layer;
  end

  // latch rules:
  // - if accept and expects_wb: latch selection until resp_pulse
  // - if accept and !expects_wb: do not latch (clear immediately)
  always_ff @(posedge clk) begin
    if (rst) begin
      sel_q_valid <= 1'b0;
      sel_q       <= SEL_LAY;
    end else begin
      // clear on response
      if (resp_pulse) begin
        sel_q_valid <= 1'b0;
      end

      // accept updates
      if (accept) begin
        if (expects_wb) begin
          sel_q_valid <= 1'b1;
          sel_q       <= is_img_cpu ? SEL_IMG : SEL_LAY;
        end else begin
          sel_q_valid <= 1'b0;
        end
      end
    end
  end

  // ============================================================
  // rd mux
  // ============================================================
  always_comb begin
    rd_we    = 1'b0;
    rd_waddr = '0;
    rd_wdata = '0;

    if (sel_eff == SEL_IMG) begin
      rd_we    = we_img;
      rd_waddr = waddr_img;
      rd_wdata = wdata_img;
    end else begin
      rd_we    = we_layer;
      rd_waddr = waddr_layer;
      rd_wdata = wdata_layer;
    end
  end

  // ============================================================
  // status OR
  // ============================================================
  always_comb begin
    accel_busy    = busy_layer | busy_img;
    accel_done    = done_layer | done_img;
    accel_C_valid = cval_layer | cval_img;
  end

endmodule

`default_nettype wire
