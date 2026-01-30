//radix-4 booth multiplier-accumulator for unsigned inputs
//`timescale 1ns/1ps
`timescale 1ns/1ps
module mac_booth_fixed_unsigned #(
    parameter integer mul_len = 24,
    parameter integer out_len = 48,
    parameter integer add_len = 102
)(
    input  logic                  clk,
    input  logic                  rst,
    input  logic                  start,
    input  logic [mul_len-1:0]     a,      // unsigned
    input  logic [mul_len-1:0]     b,      // unsigned
    output logic [out_len-1:0]     product,
    output logic                  mac_done
);

    localparam integer W          = out_len + 2;      // a_sh/sum/carry width (default 50)
    localparam integer DIGITS     = mul_len / 2+1;      // radix-4 digits (default 12)
    localparam integer SCW        = 2*W;              // packed {sum,carry} width (default 100)
    localparam integer PAD        = W - (mul_len + 1);// zeros so {zeros,a,0} fits W
    localparam integer b_len=mul_len+1;
    // shift regs
    logic [W-1:0]          a_sh;
    logic [b_len:0]      b_sh;      // extra bit for b[-1]=0
    logic [$clog2(DIGITS+1)-1:0] count;


    // CSA accum regs
    logic [W-1:0] sum_r, carry_r;

    // temps (declare at module scope for iverilog friendliness)
    logic signed [2:0] b_rec;
    logic [W-1:0]      pp;
    logic [add_len-1:0] temp;
    logic [W-1:0]      sum_n, carry_n;
    logic [W-1:0]      full;

    // --- Proper CSA: returns {sum, carry_shifted} packed (with optional leading zeros if add_len>SCW)
    function automatic [add_len-1:0] csa3(
        input logic [W-1:0] in1,
        input logic [W-1:0] in2,
        input logic [W-1:0] in3
    );
        logic [W-1:0] s;
        logic [W-1:0] c;
        begin
            s = in1 ^ in2 ^ in3;
            c = ((in1 & in2) | (in1 & in3) | (in2 & in3)) << 1; // shifted carry
            if (add_len > SCW)
                csa3 = { {(add_len-SCW){1'b0}}, s, c };
            else
                csa3 = { s, c }; // assumes add_len==SCW
        end
    endfunction

    // --- Booth recode: (b2,b1,b0) -> {-2,-1,0,+1,+2}
    function automatic logic signed [2:0] booth_rec(input logic [2:0] bb);
        begin
            case (bb)
                3'b000, 3'b111: booth_rec =  3'sd0;
                3'b001, 3'b010: booth_rec =  3'sd1;
                3'b011:         booth_rec =  3'sd2;
                3'b100:         booth_rec = -3'sd2;
                3'b101, 3'b110: booth_rec = -3'sd1;
                default:        booth_rec =  3'sd0;
            endcase
        end
    endfunction

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            a_sh     <= '0;
            b_sh     <= '0;
            count    <= '0;
            sum_r    <= '0;
            carry_r  <= '0;
            product  <= '0;
            mac_done <= 1'b0;
        end else begin
            mac_done <= 1'b0;

            if (start) begin
                // W bits total: {PAD zeros, a, 1'b0}
                a_sh    <= { { (W-mul_len){1'b0} }, a };
                b_sh    <= {1'b0, b, 1'b0 };      // b[-1]=0
                sum_r   <= '0;
                carry_r <= '0;
                count   <= '0;
            end else begin
                if (count < DIGITS) begin
                    // booth window uses b_sh[2:0] = {b[2i+1], b[2i], b[2i-1]}
                    b_rec = booth_rec(b_sh[2:0]);

                    // generate partial product (two's complement negate for negative)
                    case (b_rec)
                        3'sd2:  pp = (a_sh << 1);
                        3'sd1:  pp = a_sh;
                        3'sd0:  pp = '0;
                        -3'sd1: pp = (~a_sh) + 1'b1;
                        -3'sd2: pp = (~(a_sh << 1)) + 1'b1;
                        default: pp = '0;
                    endcase

                    // CSA accumulate: get next sum/carry from packed result
                    temp    = csa3(sum_r, carry_r, pp);
                    sum_n   = temp[SCW-1:W];
                    carry_n = temp[W-1:0];

                    // write back regs
                    sum_r   <= sum_n;
                    carry_r <= carry_n;

                    // advance
                    a_sh  <= a_sh << 2;
                    b_sh  <= b_sh >> 2;
                    count <= count + 1'b1;

                    // if this was the last digit, finalize immediately (1-cycle done pulse)
                    if (count == DIGITS-1) begin
                        full     = sum_n + carry_n;           // final CPA once
                        product  <= full[out_len-1:0];
                        mac_done <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
