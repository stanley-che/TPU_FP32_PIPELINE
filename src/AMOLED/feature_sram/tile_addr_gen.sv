`ifndef TILE_ADDR_GEN_SV
`define TILE_ADDR_GEN_SV

`timescale 1ns/1ps
`default_nettype none
module tile_addr_gen#(
    parameter int unsigned tile_x=320,
    parameter int unsigned tile_y=180,
    parameter int unsigned sram_bus=32,
    parameter int unsigned feat_dim=8,
    parameter int unsigned elen_W=32
)
(
    input logic [$clog2(tile_y)-1:0] tile_i,
    input logic [$clog2(tile_x)-1:0] tile_j,
    output logic [$clog2(tile_x*tile_y)-1:0] tile_id,
    output logic [$clog2(tile_x*tile_y*(feat_dim*elen_W+sram_bus-1)/sram_bus)-1:0] sram_addr
);
    //local params
    localparam int unsigned feat_bits = feat_dim * elen_W;
    localparam int unsigned feat_sram_words = (feat_bits + sram_bus - 1) / sram_bus; //ceil division
    //tile id width
    localparam int unsigned tile_id_W = $clog2(tile_x * tile_y);
    //sram addr width
    localparam int unsigned sram_addr_W = $clog2(tile_x * tile_y * feat_sram_words);
    //clamp util
    function automatic logic [$clog2(tile_y)-1:0]clamp_i(input logic [$clog2(tile_y)-1:0] in);
        if(in>tile_y[$clog2(tile_y)-1:0]-1)
            return tile_y[$clog2(tile_y)-1:0]-1;
        else
            return in;
    endfunction
    function automatic logic [$clog2(tile_x)-1:0]clamp_j(input logic [$clog2(tile_x)-1:0] in);
        if(in>tile_x[$clog2(tile_x)-1:0]-1)
            return tile_x[$clog2(tile_x)-1:0]-1;
        else
            return in;
    endfunction
    //combinational address generation
    logic [$clog2(tile_y)-1:0] tile_i_clamped;
    logic [$clog2(tile_x)-1:0] tile_j_clamped;
    //usee wider temporary signals to avoid overflow durning  multiplication and add
    logic [$clog2(tile_x*tile_y)-1:0] tile_id_tmp;
    logic [$clog2(tile_x*tile_y*feat_sram_words)-1:0] sram_addr_tmp;
    always@(*)begin
        tile_i_clamped = clamp_i(tile_i);
        tile_j_clamped = clamp_j(tile_j);
        tile_id_tmp = tile_i_clamped * tile_x + tile_j_clamped;
        sram_addr_tmp = tile_id_tmp * feat_sram_words;
        tile_id = tile_id_tmp;
        sram_addr = sram_addr_tmp[sram_addr_W-1:0];
    end 
endmodule
`default_nettype wire
`endif 