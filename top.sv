`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

module top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5);
    input [3:0] KEY;
    input [9:0] SW;
    output reg [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    reg clk, write, reset;
    reg [7:0] write_address;
    reg [7:0] read_address;
    wire [15:0] din, dout;
    wire [15:0] write_data, read_data;
    reg msel, msel2, msel3, enable;

    assign clk = ~KEY[0];

    RAM #(16,8) MEM (clk,read_address,write_address,write,din,dout);

    wire [1:0] mem_cmd;
    wire [8:0] mem_addr;
    assign read_address = mem_addr[7:0];
    assign write_address = mem_addr[7:0];
    assign din = write_data;

    //Write signal, assign msel3, which is for MWRITE
    always_comb begin
      if (mem_cmd == `MWRITE) 
        msel3 = 1'b1;
      else 
        msel3 = 1'b0;   
    end
    assign write = msel & msel3;

    
    //(8) assign msel2, which is for MREAD
    always_comb begin
      if (mem_cmd == `MREAD) 
        msel2 = 1'b1;
      else 
        msel2 = 1'b0;   
    end

    //(9) assign msel 
    always_comb begin
      if (mem_addr[8] == 1'b0) 
        msel = 1'b1;
      else 
        msel = 1'b0;  
    end 

    //(7) Tri_state
    assign enable = msel & msel2;
    assign read_data = enable ? dout :  {16{1'bz}};

    wire Z, N, V; 
    cpu CPU(.clk   (~KEY[0]), // KEY0 is 1 when NOT pushed
          .reset (~KEY[1]), 
          .in    (read_data),
          .out   (write_data),
          .N     (N),
          .V     (V),
          .Z     (Z),
          .mem_cmd (mem_cmd),
          .mem_addr (mem_addr) ); 

    //Switch circuit 
    reg switchEnable;
    always_comb begin
      if (mem_cmd == `MREAD) begin
        if (mem_addr == 9'h140)
          switchEnable = 1'b1;
        else 
          switchEnable = 1'b0;
      end
      else 
        switchEnable = 1'b0;
    end
    assign read_data[15:8] = switchEnable ? {8'h00} : {8{1'bz}};
    assign read_data [7:0] = switchEnable ? SW[7:0] : {8{1'bz}};

    //LEDR circuit
    reg ledEnable;
    always_comb begin
      if (mem_cmd == `MWRITE) begin
        if (mem_addr == 9'h100)
          ledEnable = 1'b1;
        else 
          ledEnable = 1'b0;
      end
      else 
        ledEnable = 1'b0;
    end  
    //the DFF for ledr
    wire [7:0] led_temp;
    assign led_temp = LEDR[7:0]; 
    always_ff @(posedge clk) begin
      if (ledEnable == 1'b1)
        LEDR[7:0] = write_data[7:0];
      else 
        LEDR[7:0] = led_temp[7:0];
    end

  
    //HEX Light
    assign HEX5[0] = ~Z;
    assign HEX5[6] = ~N;
    assign HEX5[3] = ~V;

    // fill in sseg to display 4-bits in hexidecimal 0,1,2...9,A,B,C,D,E,F
    sseg H0(write_data[3:0],   HEX0);
    sseg H1(write_data[7:4],   HEX1);
    sseg H2(write_data[11:8],  HEX2);
    sseg H3(write_data[15:12], HEX3);
    assign HEX4 = 7'b1111111;
    assign {HEX5[2:1],HEX5[5:4]} = 4'b1111; // disabled
    assign LEDR[8] = 1'b0;
endmodule

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 32; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
        mem[write_address] <= din;
        dout <= mem[read_address]; 
  end 

endmodule

module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q <= D;
endmodule

module sseg(in,segs);
  input [3:0] in;
  output reg [6:0] segs;

  reg [6:0] out;
  assign segs = out;

  always_comb begin 
    case(in[3:0])
      0: out = 7'b1000000;
      1: out = 7'b1111001;
      2: out = 7'b0100100;
      3: out = 7'b0110000;
      4: out = 7'b0011001;
      5: out = 7'b0010010;
      6: out = 7'b0000010;
      7: out = 7'b1111000;
      8: out = 7'b0000000;
      9: out = 7'b0010000;
      10: out = 7'b0001000;
      11: out = 7'b0000011;
      12: out = 7'b1000110;
      13: out = 7'b0100001;
      14: out = 7'b0000110;
      15: out = 7'b0001110;
      default out = 7'b1000000;
    endcase
  end

endmodule
