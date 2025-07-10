`include "ctrl_encode_def.v"

module alu(A, B, ALUOp, C, Zero,PC,div_by_zero,overflow);
    input  signed [31:0] A, B;
    input        [4:0]  ALUOp;
    input [31:0] PC;
    output signed [31:0] C;
    output  Zero;
    output div_by_zero;           //除零异常
    output overflow;              //溢出异常
 
   reg [31:0] C;
   
   //除0异常
   wire is_div_op = (ALUOp == `ALUOp_div) || (ALUOp == `ALUOp_divu);
   assign div_by_zero = is_div_op & (B == 32'b0);
   
   //溢出异常
    wire [31:0] add_result = A + B;
    wire [31:0] sub_result = A - B;
    wire add_overflow = (ALUOp==`ALUOp_add) & (A[31] == B[31]) & (add_result[31] != A[31]);
    wire sub_overflow = (ALUOp==`ALUOp_sub) & (A[31] != B[31]) & (sub_result[31] != A[31]);
    assign overflow = add_overflow | sub_overflow;
   
   always @( * ) begin
      case ( ALUOp )
        `ALUOp_nop:C=A;
        `ALUOp_lui:C=B;
        `ALUOp_auipc:C=PC+B;
        `ALUOp_add:C=A+B;
        `ALUOp_sub:C=A-B;
        `ALUOp_bne:C={31'b0,(A==B)};
        `ALUOp_blt:C={31'b0,(A>=B)};
        `ALUOp_bge:C={31'b0,(A<B)};
        `ALUOp_bltu:C={31'b0,($unsigned(A)>=$unsigned(B))};
        `ALUOp_bgeu:C={31'b0,($unsigned(A)<$unsigned(B))};
        `ALUOp_slt:C={31'b0,(A<B)};
        `ALUOp_sltu:C={31'b0,($unsigned(A)<$unsigned(B))};
        `ALUOp_xor:C=A^B;
        `ALUOp_or:C=A|B;
        `ALUOp_and:C=A&B;
        `ALUOp_sll:C=A<<B;
        `ALUOp_srl:C=A>>B;
        `ALUOp_sra:C=A>>>B;
        `ALUOp_div: C = div_by_zero ? 32'hFFFFFFFF : ($signed(A) / $signed(B));
        `ALUOp_divu: C = div_by_zero ? 32'hFFFFFFFF : (A / B);
      endcase
   end // end always
   
   assign Zero = (C == 32'b0);

endmodule
    
