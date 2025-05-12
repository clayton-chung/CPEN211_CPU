module ALU(Ain,Bin,ALUop,out,Z, ovf);
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output reg [15:0] out;
    output reg Z;
    output reg ovf;

    // instantiate AddSub
    wire [15:0] sum;
    wire addSubOvf;
    AddSub add_sub(
        .Ain(Ain),
        .Bin(Bin),
        .sub(ALUop[0]), // since 00 = add, 01 = sub
        .s(sum),
        .ovf(addSubOvf)
    );

    always_comb begin
        ovf = 1'b0; // avoid inferred latch
        case (ALUop)
            2'b00: begin // Addition
                out = sum;
                ovf = addSubOvf;
            end
            2'b01: begin // Subtraction
                out = sum;
                ovf = addSubOvf;
            end
            2'b10: out = Ain & Bin; // Bitwise AND
            2'b11: out = ~Bin;      // Bitwise NOT Bin
            default: out = {16{1'bx}}; // for debugging
        endcase

        Z = (out == 0); // 16-bit result of out is zero
    end
endmodule

/* From Dally 10.3 Figure 10.14 */
// add Ain + Bin or subtract Ain - Bin, check for overflow
module AddSub (Ain,Bin,sub,s,ovf);
    parameter n = 16;
    input [n-1:0] Ain,Bin;
    input sub; // subtract if sub=1, otherwise add
    output [n-1:0] s;
    output ovf; // 1 if overflow
    wire c1, c2; // carry out of last two bits
    assign ovf = c1^c2; // overflow if signs don't match

    // add non sign bits
    assign {c1, s[n-2:0]} = Ain[n-2:0] + (Bin[n-2:0] ^ {n-1{sub}}) + sub;
    // add sign bits
    assign {c2, s[n-1]} = Ain[n-1] + (Bin[n-1] ^ sub) +c1;
endmodule 