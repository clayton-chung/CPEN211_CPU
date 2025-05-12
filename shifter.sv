module shifter(in,shift,sout);
    input [15:0] in;
    input [1:0] shift;
    output reg [15:0] sout;

    always_comb begin
        case (shift)
            2'b00: sout = in;      // Copy in to sout
            2'b01: sout = in << 1; // Left shift
            2'b10: sout = in >> 1; // Right shift, MSB 0
            2'b11: begin
                sout = in >> 1;    
                sout[15] = in[15]; // Right shift, MSB(sout) = MSB(in)
            end
            default: sout = {16{1'bx}}; // for debugging
        endcase
    end
    

endmodule