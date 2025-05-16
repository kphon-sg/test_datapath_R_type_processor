// Top-level module
module ex(
    input [17:0] SW,
    input [3:0] KEY,
    output [0:6] HEX7, HEX6, HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    output [17:0] LEDR,
    output [7:0] LEDG
);
    wire [31:0] W_ALUout, W_PC_out;

    datapath_R_type_processor dut (
        .clk(KEY[0]),
        .reset(SW[0]),
        .zero(LEDG[0]),
        .W_PC_out(W_PC_out),
        .W_ALUout(W_ALUout)
    );

    hex_ssd H0 (.BIN(W_ALUout[3:0]), .SSD(HEX0));
    hex_ssd H1 (.BIN(W_ALUout[7:4]), .SSD(HEX1));
    hex_ssd H2 (.BIN(W_ALUout[11:8]), .SSD(HEX2));
    hex_ssd H3 (.BIN(W_ALUout[15:12]), .SSD(HEX3));
    hex_ssd H4 (.BIN(W_ALUout[19:16]), .SSD(HEX4));
    hex_ssd H5 (.BIN(W_ALUout[23:20]), .SSD(HEX5));
    hex_ssd H6 (.BIN(W_ALUout[27:24]), .SSD(HEX6));
    hex_ssd H7 (.BIN(W_ALUout[31:28]), .SSD(HEX7));

    assign LEDR[15:0] = W_PC_out[15:0];
    assign LEDR[17:16] = 2'b0;
endmodule

// Seven-segment display decoder
module hex_ssd (
    input [3:0] BIN,
    output reg [0:6] SSD
);
    always @(*) begin
        case (BIN)
            4'h0: SSD = 7'b0000001;
            4'h1: SSD = 7'b1001111;
            4'h2: SSD = 7'b0010010;
            4'h3: SSD = 7'b0000110;
            4'h4: SSD = 7'b1001100;
            4'h5: SSD = 7'b0100100;
            4'h6: SSD = 7'b0100000;
            4'h7: SSD = 7'b0001111;
            4'h8: SSD = 7'b0000000;
            4'h9: SSD = 7'b0001100;
            4'hA: SSD = 7'b0001000;
            4'hB: SSD = 7'b1100000;
            4'hC: SSD = 7'b0110001;
            4'hD: SSD = 7'b1000010;
            4'hE: SSD = 7'b0110000;
            4'hF: SSD = 7'b0111000;
            default: SSD = 7'b1111111;
        endcase
    end
endmodule

// Datapath for R-type processor
module datapath_R_type_processor (
    input clk, reset,
    output zero,
    output [31:0] W_ALUout, W_PC_out
);
    wire [31:0] W_PC_in, W_PC_plus_1, Instruction, W_RD1, W_RD2, W_RDm, W_Branch_add, W_m1, W_MemtoReg;
    wire [4:0] W_m3;
    wire [2:0] ALUop;
    wire RegDst, RegWrite, ALUSrc, MemRead, MemtoReg;

    Program_Counter C1 (
        .clk(clk),
        .reset(reset),
        .PC_in(W_PC_plus_1), // Simplified: no branch
        .PC_out(W_PC_out)
    );
    Adder32Bit C2 (
        .input1(32'b1),
        .input2(W_PC_out),
        .out(W_PC_plus_1)
    );
    Instruction_Memory C5 (
        .read_address(W_PC_out),
        .instruction(Instruction),
        .reset(reset)
    );
    Mux_5_bit C13 (
        .in0(Instruction[20:16]),
        .in1(Instruction[15:11]),
        .mux_out(W_m3),
        .select(RegDst)
    );
    Register_File C6 (
        .clk(clk),
        .read_addr_1(Instruction[25:21]),
        .read_addr_2(Instruction[20:16]),
        .write_addr(W_m3),
        .read_data_1(W_RD1),
        .read_data_2(W_RD2),
        .write_data(W_MemtoReg),
        .RegWrite(RegWrite)
    );
    Sign_Extension C7 (
        .sign_in(Instruction[15:0]),
        .sign_out(W_Branch_add)
    );
    Mux_32_bit C8 (
        .in0(W_RD2),
        .in1(W_Branch_add),
        .mux_out(W_m1),
        .select(ALUSrc)
    );
    alu C9 (
        .alufn(ALUop),
        .ra(W_RD1),
        .rb_or_imm(W_m1),
        .aluout(W_ALUout),
        .zero(zero)
    );
    Data_Memory C10 (
        .addr(W_ALUout),
        .write_data(W_RD2),
        .read_data(W_RDm),
        .MemRead(MemRead)
    );
    Mux_32_bit C11 (
        .in0(W_ALUout),
        .in1(W_RDm),
        .mux_out(W_MemtoReg),
        .select(MemtoReg)
    );
    Control C12 (
        .Op_intstruct(Instruction[31:26]),
        .ints_function(Instruction[5:0]),
        .RegDst(RegDst),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOp(ALUop),
        .ALUSrc(ALUSrc),
        .RegWrite(RegWrite)
    );
endmodule

// Control unit
module Control (
    input [5:0] Op_intstruct,
    input [5:0] ints_function,
    output reg RegDst, MemRead, MemtoReg, ALUSrc, RegWrite,
    output reg [2:0] ALUOp
);
    always @(*) begin
        case (Op_intstruct)
            6'b000000: begin // R-type
                RegDst = 1;
                MemRead = 0;
                MemtoReg = 0;
                ALUSrc = 0;
                RegWrite = 1;
                case (ints_function)
                    6'b100000: ALUOp = 3'b000; // ADD
                    6'b100010: ALUOp = 3'b001; // SUB
                    6'b100100: ALUOp = 3'b010; // AND
                    6'b100101: ALUOp = 3'b011; // OR
                    default:   ALUOp = 3'b000;
                endcase
            end
            6'b100011: begin // LW
                RegDst = 0;
                MemRead = 1;
                MemtoReg = 1;
                ALUSrc = 1;
                RegWrite = 1;
                ALUOp = 3'b000;
            end
            default: begin
                RegDst = 0;
                MemRead = 0;
                MemtoReg = 0;
                ALUSrc = 0;
                RegWrite = 0;
                ALUOp = 3'b000;
            end
        endcase
    end
endmodule

// Program Counter
module Program_Counter (
    input clk, reset,
    input [31:0] PC_in,
    output reg [31:0] PC_out
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            PC_out <= 0;
        else
            PC_out <= PC_in;
    end
endmodule

// 32-bit Adder
module Adder32Bit (
    input [31:0] input1, input2,
    output [31:0] out
);
    assign out = input1 + input2; // Changed to assign for simplicity
endmodule

// Instruction Memory
module Instruction_Memory (
    input reset,
    input [31:0] read_address,
    output [31:0] instruction
);
    reg [31:0] Imemory [63:0];
    assign instruction = Imemory[read_address[31:2]];

    always @(posedge reset) begin
        Imemory[0] = 32'b00000010001100101010100000100000; // add $s5, $s1, $s2
        Imemory[1] = 32'b00000010011100011011000000100010; // sub $s6, $s3, $s1
        Imemory[2] = 32'b00000010001100111011100000100000; // add $s7, $s1, $s3
        Imemory[3] = 32'b00000010010100011010000000100010; // sub $s4, $s2, $s1
        Imemory[4] = 32'b00000010001100101010100000100100; // and $s5, $s1, $s2
        Imemory[5] = 32'b00000010011100101010100000100101; // or $s5, $s3, $s2
        Imemory[6] = 32'b10001110010101100000000000000010; // lw $s6, 2($s2)
    end
endmodule

// Register File
module Register_File (
    input [4:0] read_addr_1, read_addr_2, write_addr,
    input [31:0] write_data,
    input clk, RegWrite,
    output wire [31:0] read_data_1, read_data_2
);
    reg [31:0] Regfile [31:0];

    initial begin
        Regfile[17] = 32'h11; // $s1 = 0x11
        Regfile[18] = 32'h22; // $s2 = 0x22
        Regfile[19] = 32'h33; // $s3 = 0x33
    end

    assign read_data_1 = (read_addr_1 == 0) ? 32'b0 : Regfile[read_addr_1];
    assign read_data_2 = (read_addr_2 == 0) ? 32'b0 : Regfile[read_addr_2];

    always @(posedge clk) begin
        if (RegWrite && write_addr != 0)
            Regfile[write_addr] = write_data;
    end
endmodule

// 32-bit Multiplexer
module Mux_32_bit (
    input [31:0] in0, in1,
    input select,
    output [31:0] mux_out
);
    assign mux_out = select ? in1 : in0;
endmodule

// 5-bit Multiplexer
module Mux_5_bit (
    input [4:0] in0, in1,
    input select,
    output [4:0] mux_out
);
    assign mux_out = select ? in1 : in0;
endmodule

// Sign Extension
module Sign_Extension (
    input [15:0] sign_in,
    output [31:0] sign_out
);
    assign sign_out = {{16{sign_in[15]}}, sign_in};
endmodule

// ALU
module alu (
    input [2:0] alufn,
    input [31:0] ra, rb_or_imm,
    output reg [31:0] aluout,
    output zero
);
    parameter ALU_OP_ADD = 3'b000,
              ALU_OP_SUB = 3'b001,
              ALU_OP_AND = 3'b010,
              ALU_OP_OR  = 3'b011,
              ALU_OP_LW  = 3'b101;

    assign zero = (aluout == 0); // Simplified zero logic

    always @(*) begin
        case (alufn)
            ALU_OP_ADD: aluout = ra + rb_or_imm;
            ALU_OP_SUB: aluout = ra - rb_or_imm;
            ALU_OP_AND: aluout = ra & rb_or_imm;
            ALU_OP_OR:  aluout = ra | rb_or_imm;
            ALU_OP_LW:  aluout = ra + rb_or_imm;
            default:    aluout = 32'b0;
        endcase
    end
endmodule

// Data Memory
module Data_Memory (
    input [31:0] addr,
    input [31:0] write_data,
    input MemRead,
    output [31:0] read_data
);
    reg [31:0] DMemory [39:0];

    initial begin
        DMemory[36] = 32'h99; // DMemory[0x24] = 0x99
    end

    assign read_data = MemRead ? DMemory[addr[31:2]] : 32'b0;
endmodule