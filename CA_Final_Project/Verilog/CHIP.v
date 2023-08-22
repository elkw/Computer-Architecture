// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I
    );
    //==== I/O Declaration ========================
    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;

    //==== Reg/Wire Declaration ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg    [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//
    // Todo: other wire/reg
    // PC
    wire [31:0] PC_nxt_4;       // PC+4
    wire [31:0] PC_nxt_imm;     // PC+(immediate<<1)
    // ImmediateGeneration
    wire [31:0] Immediate;      // immediate expansion
    // Control
	wire        Branch;
	wire        MemRead;
	wire        MemtoReg;
	wire [1:0]  ALUOp;
	wire        MemWrite;
	wire        ALUSrc;
    wire        Jump;
    // ALUControl
    wire [3:0]  ALUCntrol;
    // ALU
    wire [31:0] ALU_in1;
	wire [31:0] ALU_in2;
	wire [31:0] ALU_out;
	wire        ALU_zero;
    wire        ALU_ready;
    // Definition of states
    parameter   Standard    = 3'd0;
    parameter   MulDiv      = 3'd1;
    reg  [2:0]  state, state_nxt;

    assign  rs1 = mem_rdata_I[19:15];
    assign  rs2 = mem_rdata_I[24:20];
    assign  rd  = mem_rdata_I[11:7];
    //==== Submodule Connection ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    // Todo: other submodules
    ImmdiateGeneration i1(
        .in_imm(mem_rdata_I),
        .out_imm(Immediate));

    Control c1(
		.OpCode(mem_rdata_I[6:0]), 
        .Branch_w(Branch),
		.MemRead_w(MemRead), 
        .MemtoReg_w(MemtoReg),	
		.ALUOp_w(ALUOp), 
        .MemWrite_w(MemWrite),
        .RegWrite_w(regWrite), 
        .ALUSrc_w(ALUSrc),
        .Jump_w(Jump));

	ALUControl a1(
        .ALUOpt(ALUOp), 
        .Funct({mem_rdata_I[30],mem_rdata_I[14:12]}), 
        .ALUCtrl(ALUCntrol),
        .MD(mem_rdata_I[25]));
    
    ALU alu1(
        .clk(clk),
        .rst_n(rst_n),
        .in1(ALU_in1), 
        .in2(ALU_in2), 
        .ALUCtrol(ALUCntrol), 
        .out(ALU_out),
        .Fuct({mem_rdata_I[30],mem_rdata_I[14:12]}), 
        .zero_o(ALU_zero),
        .ready_o(ALU_ready));

    assign rd_data = ((Jump) ? PC_nxt_4 : ((MemtoReg) ? mem_rdata_D : ALU_out));
    assign PC_nxt_4 = (PC + 32'd4);
    assign PC_nxt_imm = (PC + (Immediate << 1));
	assign ALU_in1 = ((mem_rdata_I[6:0] == 7'b0010111) ? PC : rs1_data);
	assign ALU_in2 = (ALUSrc ? Immediate : rs2_data);
    //==== Combinational Part =====================
    // Todo: any combinational/sequential circuit
    always @(*)
    begin
        case(state)
            Standard:
            begin
                if((ALUCntrol == 4'b0011) || (ALUCntrol == 4'b0100))
                begin
                    state_nxt = MulDiv;
                    PC_nxt <= PC;
                end
                else
                begin
                    state_nxt = state;
                    PC_nxt <= (Jump && mem_rdata_I[3])? PC_nxt_imm : ((Jump && ~mem_rdata_I[3]) ? ALU_out : ((Branch && ALU_zero) ? PC_nxt_imm : PC_nxt_4));
                end
            end
            MulDiv :
            begin
                if(ALU_ready)
                begin
                    state_nxt = Standard;
                    PC_nxt <= (Jump && mem_rdata_I[3])? PC_nxt_imm : ((Jump && ~mem_rdata_I[3]) ? ALU_out : ((Branch && ALU_zero) ? PC_nxt_imm : PC_nxt_4));
                end
                else
                begin
                    state_nxt = state;
                    PC_nxt <= PC;
                end
            end
            default :
            begin
                state_nxt = state;
                PC_nxt <= PC;
            end
        endcase
    end

    //==== Sequential Part ========================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00400000; // Do not modify this value!!!
            state <= Standard;
        end
        else begin
            PC <= PC_nxt;
            state <= state_nxt;
        end
    end

    // OUTPUT
    assign mem_wen_D = MemWrite;
    assign mem_addr_D = ALU_out;
    assign mem_wdata_D = rs2_data;
    assign mem_addr_I = PC;

endmodule

module ImmdiateGeneration(in_imm, out_imm);

    input [31:0] in_imm;

	output reg [31:0] out_imm;

    always @(*)begin
        case(in_imm[6:0])
            //LB, LH, LW
            7'b0000011 : out_imm <= {{20{in_imm[31]}},in_imm[31:20]};
            //SB, SH, SW
            7'b0100011 : out_imm <= {{20{in_imm[31]}},in_imm[31:25],in_imm[11:7]};
            //BEQ, BNE, BLT, BGE
            7'b1100011 : out_imm <= {{20{in_imm[31]}},in_imm[31],in_imm[7],in_imm[30:25],in_imm[11:8]};
            //ADDI, SLTI, ...
            7'b0010011 : out_imm <= {{20{in_imm[31]}},in_imm[31:20]};
            //JALR
            7'b1100111 : out_imm <= {{20{in_imm[31]}},in_imm[31:20]};
            //JAL
            7'b1101111 : out_imm <= {{12{in_imm[31]}},in_imm[31],in_imm[19:12],in_imm[20],in_imm[30:21]};
            //AUIPC
            7'b0010111 : out_imm <= {in_imm[31:12],{12{1'd0}}};
            default : out_imm <= {32{1'd0}};
        endcase
    end 
endmodule

module Control(OpCode, Branch_w, MemRead_w, MemtoReg_w,
            ALUOp_w, MemWrite_w, ALUSrc_w, RegWrite_w, Jump_w);

    input [6:0] OpCode;

	output Branch_w;          //beq
	output MemRead_w;         //lw
    output MemtoReg_w;        //lw
    output [1:0] ALUOp_w;     //addi, slti, add, sub, xor
    output MemWrite_w;        //sw
    output ALUSrc_w;          //0: rs2, 1:immediate
    output RegWrite_w;
    output Jump_w;            //jal, jalr

    reg [8:0] control;
    
    assign {Branch_w, MemRead_w, MemtoReg_w, ALUOp_w, MemWrite_w, ALUSrc_w, RegWrite_w, Jump_w} = control;

    always@(*) begin
        case(OpCode)
            7'b0110011 : control <= 9'b000110010; //R-type
            7'b0010011 : control <= 9'b000100110; //I-type
            7'b0100011 : control <= 9'b00x001100; //S-type
            7'b1100011 : control <= 9'b10x010000; //B-type
            7'b0000011 : control <= 9'b011000110; //LW
            7'b1101111 : control <= 9'b001000111; //jal
            7'b1100111 : control <= 9'b001000111; //jalr
            7'b0010111 : control <= 9'b000000110; //auipc
            default: control <= 9'bxxxxxxxxx;
    endcase
    end
endmodule

module ALUControl(ALUOpt, Funct, ALUCtrl, MD);

    input [1:0] ALUOpt;
    input [3:0] Funct;
    input       MD;

    output reg [3:0] ALUCtrl;

    parameter AND = 4'b0000;
    parameter OR  = 4'b0001;
	parameter ADD = 4'b0010;
	parameter SUB = 4'b0110;
    parameter SLT = 4'b0111;
    parameter NOR = 4'b1100;
	parameter XOR = 4'b1101;
    parameter SLL = 4'b1000;
    parameter SRL = 4'b1001;
    parameter SRA = 4'b1011;
	parameter MUL = 4'b0011;
    parameter DIV = 4'b0100;

    always@(*)begin
        case(ALUOpt)
            2'b01: ALUCtrl <= SUB;
            2'b10: case(Funct[2:0])
                3'b000: ALUCtrl <= ADD;
                3'b010: ALUCtrl <= SLT;
                3'b100: ALUCtrl <= XOR;
                3'b110: ALUCtrl <= OR;
                3'b111: ALUCtrl <= AND;
                3'b001: ALUCtrl <= SLL;
                3'b101: ALUCtrl <= (Funct[3] == 0) ? SRL : SRA;
                default: ALUCtrl <= ADD;
            endcase
            2'b11: case(Funct)
                4'b0000: ALUCtrl <= (MD) ? MUL : ADD;
                4'b1000: ALUCtrl <= SUB;
                4'b0001: ALUCtrl <= SLL;
                4'b0010: ALUCtrl <= SLT;
                4'b0100: ALUCtrl <= (MD) ? DIV : XOR;
                4'b0101: ALUCtrl <= SRL;
                //4'b1101: ALUCtrl <= SRA;
                4'b0110: ALUCtrl <= OR;
                4'b0111: ALUCtrl <= AND;
                default: ALUCtrl <= ADD;
            endcase
            default: ALUCtrl <= ADD;
        endcase
    end
endmodule

module ALU(clk, rst_n, in1, in2, ALUCtrol, out, Fuct, zero_o, ready_o);
    
    input       clk;
    input       rst_n;
    input [31:0]in1, in2;
	input [3:0] ALUCtrol;
    input [3:0] Fuct;

	output reg [31:0] out;
	output reg        zero_o;
    output ready_o;

    reg         valid;
    reg  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: shift, 3: avg
    wire        ready_w;
    wire [63:0] out_w;

    always @(*) begin
        if((ALUCtrol == 4'b0011) || (ALUCtrol == 4'b0100))
        valid = 1;
        else
        valid = 0;

        if(ALUCtrol == 4'b0011)
        mode = 0;
        else
        mode = 1;
    end
    mulDiv md1(
        .clk(clk), 
        .rst_n(rst_n), 
        .valid(valid), 
        .ready(ready_w), 
        .mode(mode), 
        .in_A(in1), 
        .in_B(in2), 
        .out(out_w));

    always @(*)begin
		case (ALUCtrol)
			4'b0000: out <= in1 & in2;
			4'b0001: out <= in1 | in2;
			4'b0010: out <= in1 + in2;
			4'b0110: out <= in1 - in2;
			4'b0111: out <= {31'h00000000, ((in1 < in2)? 32'd1: 32'd0)};
			4'b1100: out <= ~(in1 | in2);
			4'b1101: out <= in1 ^ in2;
			4'b1000: out <= (in1 << in2);
			4'b1001: out <= (in1 >> in2);
			//4'b1011: out <= ({{32{in1[31]}}, in1} >> in2)[31:0];
            4'b0011: out <= out_w;         
            4'b0100: out <= out_w;
			default: out <= 32'h00000000;
		endcase
        case(Fuct[2:0])
            3'b000: zero_o <= ((in1 - in2) == 0);
            3'b001: zero_o <= ((in1 - in2) != 0);
            3'b100: zero_o <= (in1 < in2);
            3'b101: zero_o <= (in1 >= in2);
            default: zero_o <= 1'bx;
        endcase
    end
    assign ready_o = ready_w;
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0; // zero: hard-wired zero
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'h7fffeffc; // sp: stack pointer
                    32'd3: mem[i] <= 32'h10008000; // gp: global pointer
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: shift, 3: avg
    output        ready;
    input  [31:0] in_A, in_B;
    output [31:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter SHIFT = 3'd3;
    parameter AVG = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = (state == 5) ? 1 : 0;
    assign out = shreg[31:0];

    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                if(valid == 1)
                begin
                    case(mode)
                        2'b00:
                            state_nxt = MUL;
                        2'b01:
                            state_nxt = DIV;
                        2'b10:
                            state_nxt = SHIFT;
                        2'b11:
                            state_nxt = AVG;
                    endcase
                end
                else
                    state_nxt = state;
            end
            MUL :
            begin
                if(counter != 31)
                    state_nxt = state;
                else
                    state_nxt = OUT;
            end
            DIV :
            begin
                if(counter != 31)
                    state_nxt = state;
                else
                    state_nxt = OUT;
            end
            SHIFT :
                state_nxt = OUT;
            AVG :
                state_nxt = OUT;
            OUT :
            begin
                state_nxt = IDLE;
            end
            default :
                state_nxt = state;
        endcase
    end

    // Todo 2: Counter
    always @(*)
    begin
        if(state == 5)
            counter_nxt = 0;
        else if((state == 1) | (state == 2))
            counter_nxt = counter + 5'd1;
        else
            counter_nxt = counter;
    end

    // ALU input
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                if (valid)
                    alu_in_nxt = in_B;
                else
                    alu_in_nxt = 0;
            end
            OUT :
                alu_in_nxt = 0;
            default:
                alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*)
    begin
        case(state)
            IDLE:
                alu_out = 0;
            MUL:
            begin
                if(shreg[0] == 1)
                    alu_out = shreg[63:32] + alu_in;
                else
                    alu_out = shreg[63:32];
            end
            DIV:
            begin
                if(shreg[62:31] >= alu_in)
                    alu_out = {1'b1 , (shreg[62:31] - alu_in)};
                else
                    alu_out = {1'b0 , shreg[62:31]};
            end
            SHIFT:
                alu_out = (shreg[31:0] >> alu_in[2:0]);
            AVG:
                alu_out = (shreg[31:0] + alu_in) >> 1;
            default:
                alu_out = 0;
        endcase
    end
    // Todo 4: Shift register
    always @(*)
    begin
        case(state)
            IDLE:
            begin
                if (valid)
                    shreg_nxt = {32'd0,in_A};
                else
                    shreg_nxt = 0;
            end
            MUL:
                shreg_nxt = {alu_out,shreg[31:1]};
            DIV:
            begin
                if (alu_out[32] == 1)
                    shreg_nxt = {alu_out[31:0] , shreg[30:0] , 1'b1};
                else
                    shreg_nxt = {alu_out[31:0] , shreg[30:0] , 1'b0};
            end
            SHIFT:
                shreg_nxt = {32'd0, alu_out[31:0]};
            AVG:
                shreg_nxt = {32'd0, alu_out[31:0]};
            OUT :
                shreg_nxt = 0;
            default:
                shreg_nxt = shreg;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n)
    begin
        if (!rst_n)
        begin
            state <= IDLE;
            counter <= 5'd0;
            shreg <= 64'd0;
            alu_in <= 32'd0;
        end
        else
        begin
            state <= state_nxt;
            counter <= counter_nxt;
            shreg <= shreg_nxt;
            alu_in <= alu_in_nxt;
        end
    end
endmodule