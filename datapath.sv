`define mdata_sel 2'b00
`define sximm8_sel 2'b01
`define PC_sel 2'b10
`define datapath_out_sel 2'b11

module datapath(clk,
                readnum,
                vsel,
                loada,
                loadb,
                shift,
                asel,
                bsel,
                ALUop,
                loadc,
                loads,
                writenum,
                write,  
                Z_out,
                datapath_out,
                mdata, // the input from read data
                PC, // the Program Counter it is 9 bits width
                sximm8,
                sximm5
                );

    input clk;
    input write, loada, loadb, asel, bsel, loadc, loads;
    input [1:0] vsel; 
    input [15:0]  mdata; 
    input [8:0] PC; 
    input [2:0] readnum, writenum;
    input [1:0] shift, ALUop;

    output reg [15:0] datapath_out;
    output reg [2:0] Z_out; 

    reg [15:0] data_in,data_out,ALU_out;
    reg [15:0] sout;
    reg [15:0] Ain,Bin,A_register,B_register,Cout;
    reg [15:0] datapath_temout, A_register_temp, B_register_temp,Cout_tem;

    input [15:0] sximm8; 
    input [15:0] sximm5; 
    reg Z;
    reg [2:0] Z_tem; 
    reg ovf; 

    regfile REGFILE (data_in,writenum,write,readnum,clk,data_out); // (1)
    ALU ALU_D (Ain,Bin,ALUop,ALU_out,Z,ovf); // (2)
    shifter SHIFTER (B_register,shift,sout); // (8)

    //Register A (3)
    assign A_register_temp = A_register;
    always_ff @(posedge clk) begin
        if (loada == 1'b1) begin
            A_register = data_out;
        end else begin
            A_register = A_register_temp;
        end
    end
    //Register B (4)
    assign B_register_temp = B_register;
    always_ff @(posedge clk) begin
        if (loadb == 1'b1) begin
            B_register = data_out;
        end else begin
            B_register = B_register_temp;
        end
    end
    //Register C (5)
    assign Cout_tem = Cout;
    always_ff @(posedge clk) begin
        if (loadc == 1'b1) begin
            Cout = ALU_out;
        end else begin
            Cout = Cout_tem;
        end
    end

    //assign Ain (6)
    assign Ain = asel ? {16{1'b0}} : A_register;

    /* STAGE 1*/
    //assign Bin (7)
    assign Bin = bsel? sximm5 : sout;
 
    //assign datapath output 
    assign datapath_out = Cout;

    /* STAGE 1*/
    //Status Register (10)
    assign Z_tem = Z_out;
    always_ff @(posedge clk) begin
        if (loads == 1'b1) begin
            Z_out[2] = Z; // Zero flag
            Z_out[1] = ALU_out[15]; // Negative flag
            Z_out[0] = ovf; // Overflow flag
        end else begin
            Z_out = Z_tem;
        end
    end

    /* STAGE 1 */
    //data in (9)
    always_comb begin
        case(vsel)
            `mdata_sel: data_in = mdata; //vsel = 2'b00
            `sximm8_sel: data_in = sximm8; //vsel = 2'b01
            `PC_sel: data_in = {7'b0, PC}; //vsel = 2'b10
            `datapath_out_sel: data_in = datapath_out; //vsel = 2'b11
            default: data_in = {16{1'bx}};
        endcase
    end

endmodule