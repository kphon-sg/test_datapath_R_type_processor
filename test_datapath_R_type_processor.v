module Testbench_R_type_processor;

reg clk,reset;
wire zero;
wire [31:0] W_ALUout;

always #5 clk=~clk;  

initial begin


$monitor($time,"clk=%b,reset=%b,zero=%b,W_ALUout=%h" ,clk,reset,zero,W_ALUout);
 
clk=0;
reset=0;
#5 reset=1;
#5 reset=0;
#50 $finish;
end
endmodule;

module datapath_R_type_processor(clk,reset,zero,W_ALUout);
input clk,reset;
output zero;
output [31:0] W_ALUout;
wire  [31:0] W_PC_in,W_PC_out,W_PC_plus_1,Instruction;
wire [4:0] W_m3;
wire [2:0] ALUop;
wire RegDst,RegWrite,ALUSrc,PCSrc,MemRead,MemWrite,MemtoReg,Jump;
wire  [31:0] W_RD1,W_RD2,W_RDm,W_MemtoReg,W_Branch_add,W_m1,W_m2;
Program_Counter    C1(.clk(clk), .reset(reset), .PC_in(W_PC_in), .PC_out(W_PC_out));

Adder32Bit         C2(.input1(32'b1), .input2(W_PC_out), .out(W_PC_plus_1));

Mux_32_bit         C3(.in0(W_PC_plus_1), .in1(W_m2), .mux_out(W_PC_in), .select(PCSrc));
Adder32Bit         C4(.input1(W_PC_plus_1), .input2(W_Branch_add), .out(W_m2));
Instruction_Memory C5(.read_address(W_PC_out), .instruction(Instruction),.reset(reset));


Mux_5_bit          C13(.in0(Instruction[20:16]), .in1(Instruction[15:11]), .mux_out(W_m3), .select(RegDst));

Register_File      C6(.clk(clk),.read_addr_1(Instruction[25:21]), .read_addr_2(Instruction[20:16]), .write_addr(W_m3), .read_data_1(W_RD1), .read_data_2(W_RD2), .write_data(W_MemtoReg), .RegWrite(RegWrite));

Sign_Extension     C7(.sign_in(Instruction[15:0]), .sign_out(W_Branch_add));

Mux_32_bit         C8(.in0(W_RD2), .in1(W_Branch_add),.mux_out(W_m1), .select(ALUSrc));

alu                C9(.alufn(ALUop),.ra(W_RD1),.rb_or_imm(W_m1),.aluout(W_ALUout),.zero(zero));

Data_Memory        C10(.addr(W_ALUout), .write_data(W_RD2), .read_data(W_RDm), .MemRead(MemRead), .MemWrite(MemWrite));

Mux_32_bit         C11(.in0(W_ALUout), .in1(W_RDm), .mux_out(W_MemtoReg), .select(MemtoReg));

Control            C12(.clk(clk),.Op_intstruct(Instruction[31:26]),.ints_function(Instruction[5:0]),.RegDst(RegDst),.PcScrc(PCSrc),.MemRead(MemRead),.MemtoReg(MemtoReg),.ALUOp(ALUop),.MemWrite(MemWrite),.ALUSrc(ALUSrc),.RegWrite(RegWrite),.Zero(zero),.Jump(Jump));


                                                             
endmodule














module Control(clk,Op_intstruct,ints_function,RegDst,PcScrc,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Zero,Jump);
    input clk,Zero;
    input [5:0] ints_function;
    input [5:0] Op_intstruct;
    output reg RegDst,PcScrc,MemRead,MemtoReg,Jump;
    output reg [2:0] ALUOp;
    output reg MemWrite,ALUSrc,RegWrite;
    always @(posedge clk or Op_intstruct or Zero or ints_function)
    begin
        case(Op_intstruct)
            6'b000000:   // R -Type Instruction
                begin
                    RegDst=1;
                    Jump=0;
                    PcScrc=0;
                    MemRead=0;
                    MemtoReg=0;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=1;
                    ALUOp =3'b000;
                    if(ints_function==6'b100000) // add
                            ALUOp =3'b000;
                                  
                    if(ints_function==6'b100010) // sub 
                            ALUOp =3'b001;
                                   
                    if(ints_function==6'b100100) // and
                            ALUOp =3'b010;
                                   
                    if(ints_function==6'b100101) // or
                            ALUOp =3'b010;
                end
          default:
                begin
                    RegDst=1;
                    PcScrc=0;
                    Jump=0;
                    MemRead=0;
                    MemRead=0;
                    MemtoReg=0;
                    ALUOp =3'b000;
                    MemWrite=0;
                    ALUSrc=0;
                    RegWrite=1;
                end
        endcase
    end



endmodule



module Program_Counter (clk, reset, PC_in, PC_out);
	input clk, reset;
	input [31:0] PC_in;
	output reg [31:0] PC_out;
	always @ (posedge clk or posedge reset)
	begin
		if(reset==1'b1)
			PC_out<=0;
		else
			PC_out<=PC_in;
	end
endmodule

module Adder32Bit(input1, input2, out);
    input [31:0] input1, input2;
    output [31:0] out;
    reg [31:0]out;
    always@( input1 or input2)
        begin
            out <= input1 + input2;
        end
endmodule

module Instruction_Memory (read_address, instruction, reset);
	input reset;
	input [31:0] read_address;
	output [31:0] instruction;
	reg [31:0] Imemory [63:0];
	integer k;
	// I-MEM in this case is addressed by word, not by byte
	assign instruction = Imemory[read_address];
	always @(posedge reset)
	begin
	for (k=16; k<32; k=k+1) 
		begin  
		  // here Out changes k=0 to k=16
		  Imemory[k] = 32'b0;
		end
    //add  $s5, $s1, $s2	// do the sum
    Imemory[0] = 32'b00000010001100101010100000100000;
    //sub  $s6, $s3, $s1	// do the sum
    Imemory[1] = 32'b00000010011100011011000000100010;
    //add  $s7, $s1, $s3	// do the sum
    Imemory[2] = 32'b00000010001100111011100000100000;
    //sub  $s4, $s2, $s1	// do the sum
    Imemory[3] = 32'b00000010010100011010000000100010; 
    //add  $s5, $s1, $s2	// do the sum
    Imemory[4] = 32'b00000010001100101010100000100000;    
    //add  $s5, $s1, $s2	// do the sum
    Imemory[5] = 32'b00000010001100101010100000100000;    
    //add  $s5, $s1, $s2	// do the sum
    Imemory[6] = 32'b00000010001100101010100000100000;
	end
endmodule




module Register_File (clk,read_addr_1, read_addr_2, write_addr, read_data_1, read_data_2, write_data, RegWrite);
	input [4:0] read_addr_1, read_addr_2, write_addr;
	input [31:0] write_data;
	input  clk,RegWrite;
	reg checkRegWrite;
	output reg [31:0] read_data_1, read_data_2;
	reg [31:0] Regfile [31:0];
	integer k;
	initial begin

    Regfile[17]=32'h11;  // $s1= 32'h11
    Regfile[18]=32'h22;  // $s2= 32'h22
    Regfile[19]=32'h33;  // $s3= 32'h33
    
    
	end
	
	//assign read_data_1 = Regfile[read_addr_1];
        always @(read_data_1 or Regfile[read_addr_1])
	        begin
	          if (read_addr_1 == 0) read_data_1 = 0;
	          else 
	          begin
	          read_data_1 = Regfile[read_addr_1];
	          //$display("read_addr_1=%d,read_data_1=%h",read_addr_1,read_data_1);
	          end
	        end
	//assign read_data_2 = Regfile[read_addr_2];
        always @(read_data_2 or Regfile[read_addr_2])
	        begin
	          if (read_addr_2 == 0) read_data_2 = 0;
	          else 
	          begin
	          read_data_2 = Regfile[read_addr_2];
	          //$display("read_addr_2=%d,read_data_2=%h",read_addr_2,read_data_2);
	          end
	        end
	always @(posedge clk)
	        begin
		      if (RegWrite == 1'b1)
		         begin 
		             Regfile[write_addr] = write_data;
		             $display("write_addr=%d write_data=%d",write_addr,write_data);
		         end
	        end
endmodule



module Mux_32_bit (in0, in1, mux_out, select);
	parameter N = 32;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule



module Mux_5_bit (in0, in1, mux_out, select);
	parameter N = 5;
	input [N-1:0] in0, in1;
	output [N-1:0] mux_out;
	input select;
	assign mux_out = select? in1: in0 ;
endmodule


module Sign_Extension (sign_in, sign_out);
	input [15:0] sign_in;
	output [31:0] sign_out;
	assign sign_out[15:0]=sign_in[15:0];
	assign sign_out[31:16]=sign_in[15]?16'b1111_1111_1111_1111:16'b0;
endmodule



module alu(
	input [2:0] alufn,
	input [31:0] ra,
	input [31:0] rb_or_imm,
	output reg [31:0] aluout,
	output reg zero);
	parameter	    ALU_OP_ADD	    = 3'b000,
			    ALU_OP_SUB	    = 3'b001,
			    ALU_OP_AND	    = 3'b010,
			    ALU_OP_OR		= 3'b011,
			    ALU_OP_XOR	    = 3'b100,
			    ALU_OP_LW		= 3'b101,
			    ALU_OP_SW		= 3'b110,
			    ALU_OP_BEQ	    = 3'b111;

always @(*) 
begin
		case(alufn)
			ALU_OP_ADD 	    : aluout = ra + rb_or_imm;
			ALU_OP_SUB 	    : aluout = ra - rb_or_imm;
			ALU_OP_AND 	    : aluout = ra & rb_or_imm;
			ALU_OP_OR	    : aluout = ra | rb_or_imm;
			ALU_OP_XOR	    : aluout = ra ^ rb_or_imm;
			ALU_OP_LW	    : aluout = ra + rb_or_imm;
			ALU_OP_SW	    : aluout = ra + rb_or_imm;
			ALU_OP_BEQ	    : begin
					            zero = (ra==rb_or_imm)?1'b1:1'b0;
					            aluout = ra - rb_or_imm;
					          end
		endcase
end
endmodule



module Data_Memory (addr, write_data, read_data, MemRead, MemWrite);
	input [31:0] addr;
	input [31:0] write_data;
	output [31:0] read_data;
	input MemRead, MemWrite;
	reg [31:0] DMemory [255:0];
	integer k;
	assign read_data = (MemRead) ? DMemory[addr] : 32'bx;
	initial begin
		for (k=0; k<64; k=k+1) 
            begin
		           DMemory[k] = 32'b0;
			end
		DMemory[11] = 99;
	end
	always @(*)
	begin
		if (MemWrite) DMemory[addr] = write_data;
	end
endmodule