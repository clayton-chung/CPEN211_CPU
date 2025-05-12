`define RST 5'b00000
`define Decode 5'b00001
`define Move 5'b00010
`define GetBMove 5'b00011
`define ShiftMove 5'b00100
`define WriteRegRd 5'b00101
`define GetBALU 5'b00110
`define GetAALU 5'b00111
`define Add 5'b01000
`define Status 5'b01001
`define AND 5'b01010
`define MVN 5'b01011
`define IF1 5'b01100
`define IF2 5'b01101
`define UpdatePC 5'b01110

`define CalculateAddress 5'b01111
`define WriteMdata 5'b10000
`define Getaddress 5'b10001
`define Getdin 5'b10010
`define Writeaddress 5'b10011
`define HALT 5'b10100
`define GetALDR 5'b10101
`define GetASTR 5'b10110
`define WaitValue 5'b10111
`define WriteMdata2 5'b11000
`define Writeaddress2 5'b11001

`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

module cpu(clk,reset,in,out,N,V,Z,mem_cmd,mem_addr);
    input clk, reset;
    input [15:0] in;
    output [15:0] out;
    output N, V, Z;
    output [1:0] mem_cmd;
    output reg [8:0] mem_addr;

    wire load_ir, reset_pc, addr_sel, load_addr;
    reg load_pc;
    reg [15:0] decoder_input, decoder_input_temp;
    reg [2:0] opcode;
    reg [1:0] op, ALUop, shift;
    reg [4:0] imm5;
    reg [7:0] imm8;
    reg [2:0] Rn, Rd, Rm;
    reg [2:0] readnum, writenum;
    reg [15:0] sximm5, sximm8;
    reg [1:0] vsel;
    reg [2:0] nsel;
    reg [2:0] Z_out;
    reg [15:0] datapath_out, mdata;
    //reg [7:0] PC_old;
    reg write, loada, loadb, loadc, loads, w_out, asel, bsel;
    wire [4:0] present_state, state_next_reset, state_next;
    reg [23:0] datapathinstruction;
    reg [8:0] data_address_in, data_address_out;
    reg [8:0] PC;

    assign out = datapath_out;
    
    //mdata is the input data
    assign mdata = in;

    assign {state_next, vsel, nsel, asel, bsel, loada, loadb, loadc, loads, write, reset_pc, load_ir, load_pc, addr_sel, load_addr, mem_cmd} = datapathinstruction;

    //The DFF block for FSM, and the reset state
    vDFFcpu #(5) STATE(clk,state_next_reset,present_state);
    //new reset (1)
    assign state_next_reset = reset ? `RST : state_next;

    //assign N,V,Z flag base on Z_out
    assign N = Z_out[1];
    assign V = Z_out[0];
    assign Z = Z_out[2];

    //datapath instantiation 
    datapath DP (clk,readnum,vsel,loada,loadb,shift,asel,bsel,ALUop,loadc,loads,writenum,write,Z_out,datapath_out,mdata,PC,sximm8,sximm5);

    //FSM // With new change in last 6 bits of datapathinstruction
    always_comb begin
        casex ({present_state, opcode, op})
            //RST, IF1, IF2, UpdatePC
            {`RST,5'bxxxxx}: datapathinstruction = {`IF1,17'b00000000000010100,2'b00};
            {`IF1,5'bxxxxx}: datapathinstruction = {`IF2,17'b00000000000000010,`MREAD};
            {`IF2,5'bxxxxx}: datapathinstruction = {`UpdatePC,17'b00000000000001010,`MREAD};
            {`UpdatePC,5'bxxxxx}: datapathinstruction = {`Decode,17'b00000000000000100,2'b00};
            //Move im8
            {`Decode,5'b11010}: datapathinstruction = {`Move,17'b00000000000000000,2'b00}; 
            {`Move,5'bxxxxx}: datapathinstruction = {`IF1,17'b01100000000100000,2'b00}; //Instruction for Rn = sximm8
            //Move sh_Rm
            {`Decode,5'b11000}: datapathinstruction = {`GetBMove,17'b00000000000000000,2'b00};
            {`GetBMove,5'bxxxxx}: datapathinstruction = {`ShiftMove,17'b00001000100000000,2'b00}; //Get B for the move instruction 
            {`ShiftMove,5'bxxxxx}: datapathinstruction = {`WriteRegRd,17'b00000100010000000,2'b00}; //Shift for move instruction
            {`WriteRegRd,5'bxxxxx}: datapathinstruction = {`IF1,17'b11010000000100000,2'b00}; //Rd = datapath_out
            //ALU instruction 
            {`Decode,5'b101xx}: datapathinstruction = {`GetBALU,17'b00000000000000000,2'b00}; 
            {`GetBALU,5'bxxx11}: datapathinstruction = {`MVN,17'b00001000100000000,2'b00}; //Get B for ALU instruction 
            {`GetBALU,5'bxxx10}: datapathinstruction = {`GetAALU,17'b00001000100000000,2'b00};
            {`GetBALU,5'bxxx0x}: datapathinstruction = {`GetAALU,17'b00001000100000000,2'b00};
            {`GetAALU,5'bxxx00}: datapathinstruction = {`Add,17'b00100001000000000,2'b00}; //Get A for ALU instruction 
            {`GetAALU,5'bxxx01}: datapathinstruction = {`Status,17'b00100001000000000,2'b00};
            {`GetAALU,5'bxxx10}: datapathinstruction = {`AND,17'b00100001000000000,2'b00};
            {`Add,5'bxxxxx}: datapathinstruction = {`WriteRegRd,17'b00000000010000000,2'b00}; //Add A and B
            {`Status,5'bxxxxx}: datapathinstruction = {`IF1,17'b00000000001000000,2'b00}; //Status at SUB A and B
            {`AND,5'bxxxxx}: datapathinstruction = {`WriteRegRd,17'b00000000010000000,2'b00}; //AND A and B
            {`MVN,5'bxxxxx}: datapathinstruction = {`WriteRegRd,17'b00000100010000000,2'b00}; //MVN B
            //LDR
            {`Decode,5'b01100}: datapathinstruction = {`GetALDR,17'b00000000000000000,2'b00};
            {`GetALDR,5'b01100}: datapathinstruction = {`CalculateAddress,17'b00100001000000000,2'b00};
            {`CalculateAddress,5'bxxxxx}: datapathinstruction = {`WaitValue,17'b00000010010000000,2'b00};
            {`WaitValue,5'b01100}: datapathinstruction = {`WriteMdata,17'b00000000000000001,2'b00};
            {`WriteMdata,5'bxxxxx}: datapathinstruction = {`WriteMdata2,17'b00000000000000001,`MREAD};
            {`WriteMdata2,5'bxxxxx}: datapathinstruction = {`IF1,17'b00010000000100000,`MREAD};
            //STR
            {`Decode,5'b10000}: datapathinstruction = {`GetASTR,17'b00000000000000000,2'b00};
            {`GetASTR,5'b10000}: datapathinstruction = {`CalculateAddress,17'b00100001000000000,2'b00};
            {`CalculateAddress,5'bxxxxx}: datapathinstruction = {`WaitValue,17'b00000010010000000,`MNONE};
            {`WaitValue,5'b10000}: datapathinstruction = {`Getdin,17'b00000000000000001,2'b00};
            {`Getdin,5'bxxxxx}: datapathinstruction = {`Writeaddress,17'b00010000100000000,2'b00};
            {`Writeaddress,5'bxxxxx}: datapathinstruction = {`Writeaddress2,17'b00000100010000000,`MWRITE};
            {`Writeaddress2,5'bxxxxx}: datapathinstruction = {`IF1,17'b00000000000000000,`MWRITE};
            //HALT
            {`Decode,5'b111xx}: datapathinstruction = {`HALT,17'b00000000000000000,2'b00};
            {`HALT,5'bxxxxx}: datapathinstruction = {`HALT,17'b00000000000000000,2'b00};
            default: datapathinstruction = {{5{1'bx}},{19{1'bx}}};
        endcase
    end

    //PC (3)
    reg [8:0] next_pc;
    always_comb begin
        if (reset_pc == 1'b1)
            next_pc = 9'b000000000;
        else 
            next_pc = PC + 9'b000000001;
    end

    wire [8:0] pc_out_temp;
    assign pc_out_temp = PC;
    always_ff @(posedge clk) begin
        if (load_pc == 1'b1) 
            PC = next_pc;
        else 
            PC = pc_out_temp;
    end
 
    //mem_addr (5)
    always_comb begin
        if (addr_sel == 1'b1)
            mem_addr = PC;
        else 
            mem_addr = data_address_out;
    end

    //Data Address
    wire [8:0] data_address_out_tmp;
    assign data_address_out_tmp = data_address_out;
    assign data_address_in = datapath_out[8:0];
    always_ff @(posedge clk) begin
        if (load_addr == 1'b1) 
            data_address_out = data_address_in;
        else 
            data_address_out = data_address_out_tmp;
    end

    //Instruction Register
    assign decoder_input_temp = decoder_input;
    always_ff @(posedge clk) begin
        if (load_ir == 1'b1) 
            decoder_input = in;
        else 
            decoder_input = decoder_input_temp;
    end

    //Decoder
    always_comb begin
         opcode = decoder_input[15:13];
         op = decoder_input[12:11];
         ALUop = op;
         imm5 = decoder_input[4:0];
         imm8 = decoder_input[7:0];
         shift = decoder_input[4:3];
         Rn = decoder_input[10:8];
         Rd = decoder_input[7:5];
         Rm = decoder_input[2:0];
        //assign readnum and writenum base on nsel
        case (nsel) 
            3'b100: begin
                readnum = Rn;
                writenum = Rn;
            end
            3'b010: begin
                readnum = Rd;
                writenum = Rd;
            end
            3'b001: begin
                readnum = Rm;
                writenum = Rm;
            end
            default begin
                readnum = 3'bx;
                writenum = 3'bx;
            end
        endcase
        //extend imm5 and imm8 base on their most important bit
        if (imm5[4] == 1'b1) begin
             sximm5 = {11'b11111111111,imm5};
        end else begin
             sximm5 = {11'b00000000000,imm5};
        end

        if (imm8[7] == 1'b1) begin
             sximm8 = {8'b11111111,imm8};
        end else begin
             sximm8 = {8'b00000000,imm8};
        end
    end


endmodule

//The DFF block, use for flip-flop block for the FSM
module vDFFcpu(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q = D;
endmodule