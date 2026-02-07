`ifndef BURST_REQ_IF_SV
`define BURST_REQ_IF_SV
`timescale 1ns/1ps
`default_nettype none
 module burst_req_if#(
    parameter int unsigned tile_x=320,
    parameter int unsigned tile_y=180,
    parameter int unsigned sram_bus=32,
    parameter int unsigned feat_dim=8,
    parameter int unsigned elen_W=32,
    parameter int unsigned tag_w=16,
    parameter bit isclamp=1'b0,//zero for unclamped, one for clamped  
    parameter int unsigned FEAT_BITS_P= feat_dim * elen_W,
    parameter int unsigned WORDS_PER_FEAT_P = (FEAT_BITS_P + sram_bus - 1) / sram_bus,
    parameter int unsigned TOTAL_WORDS_P    = tile_x * tile_y * WORDS_PER_FEAT_P,
    parameter int unsigned MEM_AW           = (TOTAL_WORDS_P <= 1) ? 1 : $clog2(TOTAL_WORDS_P)
 )(
    input clk,
    input rst,
    //feature write request
    input logic valid_wr,
    output logic ready_wr,
    input logic [$clog2(tile_y)-1:0] tile_i_wr,
    input logic [$clog2(tile_x)-1:0] tile_j_wr,
    input logic [feat_dim*elen_W-1:0] feat_in,
    //feature read request from neigh_fwetch
    input logic valid_rd,
    output logic ready_rd,
    input logic [$clog2(tile_y)-1:0] tile_i_rd,
    input logic [$clog2(tile_x)-1:0] tile_j_rd,
    input logic [tag_w-1:0] tag_rd,
    //internal sram interface for burst engine
    output logic req_valid,
    input logic  req_ready, 
    output logic req_is_wr, //1 for write, 0 for read
    output logic [MEM_AW-1:0] sram_addr,
    output logic [feat_dim*elen_W-1:0] sram_wdata,  
    output logic [tag_w-1:0] sram_tag,
    input  logic                  use_ext_addr,
    input  logic [MEM_AW-1:0]      ext_base_addr
 );
    //localparams
    localparam int unsigned FEAT_BITS = FEAT_BITS_P;
    localparam int unsigned WORDS_PER_FEAT = WORDS_PER_FEAT_P;
    //clamp for power 2
    function automatic logic [$clog2(tile_y)-1:0]clamp_i(input logic [$clog2(tile_y)-1:0] in);
        if(isclamp) begin
            if(in>tile_y[$clog2(tile_y)-1:0]-1)
                return tile_y[$clog2(tile_y)-1:0]-1;
            else
                return in;
        end
        else begin
            return in;
        end
    endfunction
    function automatic logic [$clog2(tile_x)-1:0]clamp_j(input logic [$clog2(tile_x)-1:0] in);
        if(isclamp) begin
            if(in>tile_x[$clog2(tile_x)-1:0]-1)
                return tile_x[$clog2(tile_x)-1:0]-1;
            else
                return in;
        end
        else begin
            return in;
        end
    endfunction
    //arbitation between read and write for one deep pending request
    logic pending_valid;
    logic pending_is_wr;
    logic [MEM_AW-1:0] pending_addr;
    logic [FEAT_BITS-1:0] pending_wdata;
    logic [tag_w-1:0] pending_tag;
    //arbitation logic write priority
    logic take_wr, take_rd;
    always_comb begin
        take_wr = 1'b0;
        take_rd = 1'b0;
        //ready when no pending and write has higher priority
        ready_wr = ~pending_valid;
        ready_rd = ~pending_valid&~valid_wr;
        if(~pending_valid) begin
            if(valid_wr) begin
                take_wr = 1'b1;
            end
            else if(valid_rd) begin
                take_rd = 1'b1;
            end
        end
    end
    
    //address cal for selected request
    //write address=TILE_ID*WORDS_PER_FEAT
    //TILE_ID= tile_i*tile_x + tile_j
    logic [$clog2(tile_y)-1:0] tile_i_sel;
    logic [$clog2(tile_x)-1:0] tile_j_sel;
    logic [31:0]               tile_id_u;
    logic [MEM_AW+31:0]        addr_calc_u;
    always_comb begin
        if(take_wr) begin
            tile_i_sel = clamp_i(tile_i_wr);
            tile_j_sel = clamp_j(tile_j_wr);
        end
        else begin
            tile_i_sel = clamp_i(tile_i_rd);
            tile_j_sel = clamp_j(tile_j_rd);
        end
        tile_id_u    = (tile_i_sel * tile_x) + tile_j_sel;
        addr_calc_u  = tile_id_u * WORDS_PER_FEAT;
    end
    logic [MEM_AW-1:0] addr_sel;
    logic [MEM_AW+31:0] addr_calc;
    assign addr_sel = addr_calc_u[MEM_AW-1:0];

    //pending register
    always_ff @(posedge clk) begin
        if(rst) begin
            pending_valid <= 1'b0;
            pending_is_wr <= 1'b0;
            pending_addr  <= '0;
            pending_wdata <= '0;
            pending_tag   <= '0;
        end
        else begin
            if(pending_valid & req_ready) begin
                pending_valid <= 1'b0;
            end
            if(~pending_valid) begin
                if(take_wr) begin
                    pending_valid <= 1'b1;
                    pending_is_wr <= 1'b1;
                    pending_addr <= (use_ext_addr) ? ext_base_addr : addr_sel;
                    pending_wdata <= feat_in;
                    pending_tag   <= '0;
                end
                else if(take_rd) begin
                    pending_valid <= 1'b1;
                    pending_is_wr <= 1'b0;
                    pending_addr <= (use_ext_addr) ? ext_base_addr : addr_sel;
                    pending_wdata <= '0;
                    pending_tag   <= tag_rd;
                end
            end
        end
    end
    //output assignment
    assign req_valid  = pending_valid;
    assign req_is_wr  = pending_is_wr;
    assign sram_addr  = pending_addr;
    assign sram_wdata = pending_wdata;
    assign sram_tag   = pending_tag;

 endmodule
 `default_nettype wire  
`endif