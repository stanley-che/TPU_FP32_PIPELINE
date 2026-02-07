`ifndef BURST_ENGINE_SV
`define BURST_ENGINE_SV

`timescale 1ns/1ps
`default_nettype none

module burst_engine #(
    parameter int unsigned SRAM_BUS_W     = 32,
    parameter int unsigned MEM_AW         = 18,
    parameter int unsigned FEAT_W         = 256,
    parameter int unsigned WORDS_PER_FEAT = 8,
    parameter int unsigned TAG_W          = 16,

    parameter int unsigned REQ_DEPTH  = 2,
    parameter int unsigned META_DEPTH = 16,
    parameter int unsigned BEAT_DEPTH = 16
)(
    input  logic clk,
    input  logic rst,

    // request
    input  logic               req_valid,
    output logic               req_ready,
    input  logic               req_is_wr,
    input  logic [FEAT_W-1:0]  req_wdata,
    input  logic [TAG_W-1:0]   req_tag,
    input  logic [MEM_AW-1:0]  req_addr,

    // memory command
    output logic               mem_req_valid,
    input  logic               mem_req_ready,
    output logic               mem_req_is_wr,
    output logic [MEM_AW-1:0]  mem_req_addr,
    output logic [SRAM_BUS_W-1:0] mem_req_wdata,
    output logic [SRAM_BUS_W/8-1:0] mem_req_wmask,

    // memory read return
    input  logic               mem_rvalid,
    input  logic [SRAM_BUS_W-1:0] mem_rdata,

    // beat output
    output logic               beat_out_valid,
    input  logic               beat_out_ready,
    output logic [SRAM_BUS_W-1:0] beat_out_data,
    output logic [TAG_W-1:0]      beat_out_tag,
    output logic [$clog2(WORDS_PER_FEAT)-1:0] beat_out_idx
  );

    localparam int BEAT_IDX_W = (WORDS_PER_FEAT <= 1) ? 1 : $clog2(WORDS_PER_FEAT);

    // ============================================================
    // 1) Request FIFO (flattened)
    // ============================================================
    logic req_is_wr_q [0:REQ_DEPTH-1];
    logic [TAG_W-1:0]  req_tag_q [0:REQ_DEPTH-1];
    logic [MEM_AW-1:0] req_addr_q[0:REQ_DEPTH-1];
    logic [FEAT_W-1:0] req_data_q[0:REQ_DEPTH-1];
    logic [BEAT_IDX_W-1:0] rd_resp_idx;
    logic [$clog2(REQ_DEPTH)-1:0] req_wptr, req_rptr;
    logic [$clog2(REQ_DEPTH+1)-1:0] req_cnt;

    assign req_ready = (req_cnt < REQ_DEPTH);

    // current request
    logic cur_valid;
    logic cur_is_wr;
    logic [TAG_W-1:0]  cur_tag;
    logic [MEM_AW-1:0] cur_addr;
    logic [FEAT_W-1:0] cur_data;
    logic [BEAT_IDX_W-1:0] cur_beat;
  
    // ============================================================
    // 2) META FIFO (tag + beat_idx)
    // ============================================================
    logic [TAG_W-1:0]      meta_tag [0:META_DEPTH-1];
    logic [BEAT_IDX_W-1:0] meta_idx [0:META_DEPTH-1];
    logic [$clog2(META_DEPTH)-1:0] meta_wptr, meta_rptr;
    logic [$clog2(META_DEPTH+1)-1:0] meta_cnt;

    wire meta_full  = (meta_cnt == META_DEPTH);
    wire meta_empty = (meta_cnt == 0);

    // ============================================================
    // 3) BEAT FIFO
    // ============================================================
    logic [TAG_W-1:0]      beat_tag_q [0:BEAT_DEPTH-1];
    logic [BEAT_IDX_W-1:0] beat_idx_q [0:BEAT_DEPTH-1];
    logic [SRAM_BUS_W-1:0] beat_data_q[0:BEAT_DEPTH-1];

    logic [$clog2(BEAT_DEPTH)-1:0] beat_wptr, beat_rptr;
    logic [$clog2(BEAT_DEPTH+1)-1:0] beat_cnt;

    wire beat_full  = (beat_cnt == BEAT_DEPTH);
    wire beat_empty = (beat_cnt == 0);

    // ============================================================
    // helpers
    // ============================================================
    function automatic [SRAM_BUS_W-1:0] get_word;
      input [FEAT_W-1:0] feat;
      input [BEAT_IDX_W-1:0] idx;
      begin
          get_word = feat >> (idx * SRAM_BUS_W);
      end
    endfunction
    always_ff @(posedge clk) begin
        if(rst)begin
            rd_resp_idx <= 0;
        end else if(!cur_valid) begin
            rd_resp_idx <= 0;
        end
    end
    // ============================================================
    // Request FIFO + current request
    // ============================================================
    wire req_push = req_valid && req_ready;
    wire req_pop_to_cur = (!cur_valid) && (req_cnt != 0);

    function automatic [$clog2(REQ_DEPTH)-1:0] inc_req_ptr(input [$clog2(REQ_DEPTH)-1:0] p);
        if (p == REQ_DEPTH-1) inc_req_ptr = '0;
        else                 inc_req_ptr = p + 1'b1;
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            req_wptr <= '0;
            req_rptr <= '0;
            req_cnt  <= '0;
            cur_valid<= 1'b0;
            cur_beat <= '0;
        end else begin
            if (req_push) begin
                req_is_wr_q[req_wptr] <= req_is_wr;
                req_tag_q [req_wptr]  <= req_tag;
                req_addr_q[req_wptr]  <= req_addr;
                req_data_q[req_wptr]  <= req_wdata;
                req_wptr <= inc_req_ptr(req_wptr);
            end

            if (req_pop_to_cur) begin
                cur_is_wr <= req_is_wr_q[req_rptr];
                cur_tag   <= req_tag_q [req_rptr];
                cur_addr  <= req_addr_q[req_rptr];
                cur_data  <= req_data_q[req_rptr];
                cur_valid <= 1'b1;
                cur_beat  <= '0;
                req_rptr  <= inc_req_ptr(req_rptr);
            end

            // cnt update (single assignment)
            case ({req_push, req_pop_to_cur})
                2'b10: req_cnt <= req_cnt + 1'b1;
                2'b01: req_cnt <= req_cnt - 1'b1;
                default: req_cnt <= req_cnt; // 00 or 11 => unchanged
            endcase

            // advance beat (同你原本)
            if (cur_valid && mem_req_valid && mem_req_ready) begin
                if (cur_beat == WORDS_PER_FEAT-1) cur_valid <= 1'b0;
                else                              cur_beat  <= cur_beat + 1'b1;
            end
        end
    end


    // ============================================================
    // Memory command
    // ============================================================
    assign mem_req_valid = cur_valid && (cur_is_wr || !meta_full);
    assign mem_req_is_wr = cur_is_wr;
    assign mem_req_addr  = cur_addr + cur_beat;
    assign mem_req_wdata = get_word(cur_data, cur_beat);
    assign mem_req_wmask = {SRAM_BUS_W/8{1'b1}};

    // ============================================================
    // META FIFO push/pop  (single-driver fix)
    // ============================================================
    wire meta_push = mem_req_valid && mem_req_ready && cur_valid && !cur_is_wr;

    // 只有當真的把 rdata 吃進 beat_fifo 時，才 pop meta
    wire meta_pop  = mem_rvalid && !beat_full && !meta_empty; // same as beat_push

    always_ff @(posedge clk) begin
        if (rst) begin
            meta_wptr <= 0;
            meta_rptr <= 0;
            meta_cnt  <= 0;
        end else begin
            // push only
            if (meta_push && !meta_pop) begin
                meta_tag[meta_wptr] <= cur_tag;
                meta_idx[meta_wptr] <= cur_beat;
                meta_wptr <= (meta_wptr + 1) % META_DEPTH;
                meta_cnt  <= meta_cnt + 1;
            end
            // pop only
            else if (!meta_push && meta_pop) begin
                meta_rptr <= (meta_rptr + 1) % META_DEPTH;
                meta_cnt  <= meta_cnt - 1;
            end
            // push+pop same cycle
            else if (meta_push && meta_pop) begin
                meta_tag[meta_wptr] <= cur_tag;
                meta_idx[meta_wptr] <= cur_beat;
                meta_wptr <= (meta_wptr + 1) % META_DEPTH;
                meta_rptr <= (meta_rptr + 1) % META_DEPTH;
            end
        end
    end


    // ============================================================
    // BEAT FIFO push/pop
    // ============================================================
    wire beat_push = mem_rvalid && !beat_full && !meta_empty;
    wire beat_pop  = beat_out_valid && beat_out_ready;


    function automatic [$clog2(BEAT_DEPTH)-1:0] inc_beat_ptr(input [$clog2(BEAT_DEPTH)-1:0] p);
        if (p == BEAT_DEPTH-1) inc_beat_ptr = '0;
        else                  inc_beat_ptr = p + 1'b1;
    endfunction

    always_ff @(posedge clk) begin
        if (rst) begin
            beat_wptr <= '0;
            beat_rptr <= '0;
            beat_cnt  <= '0;
        end else begin
            if (beat_push) begin
                beat_tag_q [beat_wptr]  <= meta_tag[meta_rptr];
                beat_idx_q [beat_wptr]  <= meta_idx[meta_rptr]; // << 用 meta_idx，不要用 rd_resp_idx
                beat_data_q[beat_wptr]  <= mem_rdata;
                beat_wptr <= inc_beat_ptr(beat_wptr);
            end
            if (beat_pop) 
                beat_rptr <= inc_beat_ptr(beat_rptr);

            case ({beat_push, beat_pop})
                2'b10: beat_cnt <= beat_cnt + 1'b1;
                2'b01: beat_cnt <= beat_cnt - 1'b1;
                default: beat_cnt <= beat_cnt; // 00 or 11
            endcase
        end
    end


    // ============================================================
    // Output
    // ============================================================
    assign beat_out_valid = !beat_empty;
    assign beat_out_tag   = beat_tag_q [beat_rptr];
    assign beat_out_idx   = beat_idx_q [beat_rptr];
    assign beat_out_data  = beat_data_q[beat_rptr];

endmodule

`default_nettype wire
`endif
