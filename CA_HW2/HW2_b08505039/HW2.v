module ALU(
           clk,
           rst_n,
           valid,
           ready,
           mode,
           in_A,
           in_B,
           out
       );

// Definition of ports
input         clk, rst_n;
input         valid;
input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: shift, 3: avg
output        ready;
input  [31:0] in_A, in_B;
output [63:0] out;

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
assign out = shreg;

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
