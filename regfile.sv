module regfile(data_in,writenum,write,readnum,clk,data_out);
    input [15:0] data_in;
    input [2:0] writenum, readnum;
    input write, clk;
    output reg [15:0] data_out;

    reg [15:0] R0 = 16'b0;
    reg [15:0] R1 = 16'b0;
    reg [15:0] R2 = 16'b0;
    reg [15:0] R3 = 16'b0;
    reg [15:0] R4 = 16'b0;
    reg [15:0] R5 = 16'b0;
    reg [15:0] R6 = 16'b0;
    reg [15:0] R7 = 16'b0;
    reg [15:0] out;
    assign data_out = out;


      always @(posedge clk) begin
        //write input number to the corresponding place when write is 1
        if (write == 1'b1) begin
            case(writenum)
                0: R0 = data_in;
                1: R1 = data_in;
                2: R2 = data_in;
                3: R3 = data_in;
                4: R4 = data_in;
                5: R5 = data_in;
                6: R6 = data_in;
                7: R7 = data_in;
                default begin
                    R0 = 16'b0;
                    R1 = 16'b0;
                    R2 = 16'b0;
                    R3 = 16'b0;
                    R4 = 16'b0;
                    R5 = 16'b0;
                    R6 = 16'b0;
                    R7 = 16'b0;
                end
            endcase
        end 
    end

    always @(*) begin
        //Assign the output value base on read number
        case(readnum) 
            0: out = R0;
            1: out = R1;
            2: out = R2;
            3: out = R3;
            4: out = R4;
            5: out = R5;
            6: out = R6;
            7: out = R7;
            default out = 16'bx;
        endcase
    end
endmodule
