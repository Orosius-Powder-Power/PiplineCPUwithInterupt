// `include "ctrl_encode_def.v"

module ctrl(Op, Funct7, Funct3,
            RegWrite, MemWrite,
            EXTOp, ALUOp, NPCOp, 
            ALUSrc, GPRSel, WDSel,DMType,MemRead,
            is_eret,is_eretn,          
            is_ecall
            );
            
    input  [6:0] Op;       // opcode
    input  [6:0] Funct7;    // funct7
    input  [2:0] Funct3;    // funct3
           
    output       RegWrite; // control signal for register write
    output       MemWrite; // control signal for memory write
    output [5:0] EXTOp;    // control signal to signed extension
    output [4:0] ALUOp;    // ALU opertion
    output [2:0] NPCOp;    // next pc operation
    output       ALUSrc;   // ALU source for A
	output [2:0] DMType;
    output [1:0] GPRSel;   // general purpose register selection
    output [1:0] WDSel;    // (register) write data selection
    output       MemRead;
    
    //interrupt
    output      is_eret;
    output      is_eretn;
   
    output      is_ecall;   //系统调用
    
    //R型指令
    wire rtype  =  (Op == 7'b011_0011);  //0110011
    wire i_add  = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b000); // add 0000000 000
    wire i_sub  = rtype & (Funct7 == 7'b010_0000) & (Funct3 == 3'b000); // sub 0100000 000
    wire i_or   = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b110); // or  0000000 110
    wire i_and  = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b111); // and 0000000 111
	wire i_xor  = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b100); // xor 0000000 100
 	wire i_sll  = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b001);  //sll 0000000 001
	wire i_sra  = rtype & (Funct7 == 7'b010_0000) & (Funct3 == 3'b101);  //sra 0100000 101
	wire i_srl  = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b101);  //srl 0000000 101
	wire i_slt  = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b010); // slt 0000000 010
	wire i_sltu = rtype & (Funct7 == 7'b000_0000) & (Funct3 == 3'b011); //sltu 0000000 011
    wire i_div = rtype & (Funct7 == 7'b0000001) & (Funct3 == 3'b100);
    wire i_divu= rtype & (Funct7 == 7'b0000001) & (Funct3 == 3'b101);

    // I型load型指令
    wire itype_l  = (Op == 7'b000_0011);  //0000011
	wire i_lw = itype_l & (Funct3 == 3'b010);  //lw 010
	wire i_lb = itype_l & (Funct3 == 3'b000);  //lb 000
	wire i_lh = itype_l & (Funct3 == 3'b001);  //lh 001
	wire i_lbu = itype_l& (Funct3 == 3'b100);//lbu 100
	wire i_lhu = itype_l& (Funct3 == 3'b101);//lhu 101
	
    // I型运算指令
    wire itype_r  =  (Op == 7'b001_0011);  //0010011
    wire i_addi  =   itype_r & (Funct3 == 3'b000); // addi 000
    wire i_ori  =  itype_r& (Funct3 == 3'b110); // ori 110
    wire i_andi =  itype_r& (Funct3 == 3'b111);// andi 111
	wire i_xori =  itype_r& (Funct3 == 3'b100);// xori 100
	wire i_srai  = itype_r& (Funct7 == 7'b010_0000) & (Funct3 == 3'b101); //srai 0100000 101
	wire i_slti =  itype_r&(Funct3 == 3'b010);// slti 010
	wire i_sltiu = itype_r&(Funct3 == 3'b011);// sltiu 011
	wire i_slli  = itype_r& (Funct7 == 7'b000_0000) & (Funct3 == 3'b001); //slli 0000000 001
	wire i_srli  = itype_r& (Funct7 == 7'b000_0000) & (Funct3 == 3'b101); //srli 0000000 101

   //jal和jalr指令   5'b10010
    wire i_jal=  (Op == 7'b110_1111); //1101111
    wire i_jalr= (Op == 7'b110_0111) & (Funct3 == 3'b000);// 110 0111
  
    // S型指令
    wire stype  = (Op == 7'b010_0011);//0100011
    wire i_sw   =  stype& (Funct3 == 3'b010); // sw 010
	wire i_sb   =  stype& (Funct3 == 3'b000); // sb 000
	wire i_sh   =  stype& (Funct3 == 3'b001); // sh 001

    // SB型指令
    wire sbtype  = (Op == 7'b110_0011);//1100011
    wire i_beq  = sbtype& (Funct3 == 3'b000); // beq 000
	wire i_bne  = sbtype&(Funct3 == 3'b001); // bne 001
	wire i_blt  = sbtype&(Funct3 == 3'b100); // blt 100
	wire i_bge  = sbtype& (Funct3 == 3'b101); // bge 101
	wire i_bgeu  = sbtype& (Funct3 == 3'b111); // bgeu 111
	wire i_bltu  = sbtype& (Funct3 == 3'b110); // bltu 110
	
    // U型指令 
	wire i_lui = (Op == 7'b011_0111); //0110111
	wire i_auipc = (Op == 7'b001_0111); //0010111
    
    //中断/系统调用
    wire I_type_sys = (Op == 7'b1110011);
    assign is_eret = (I_type_sys) && (Funct3 == 3'b001);
    assign is_eretn = (I_type_sys) && (Funct3 == 3'b010);
    assign is_ecall = I_type_sys && (Funct3 == 3'b000) && (Funct7 == 7'b0000000);
    
    // 控制信号
    assign RegWrite   = rtype | itype_r | i_jalr | i_jal | i_lui | i_auipc |itype_l; 
    assign MemWrite   = stype; 
    assign ALUSrc     = itype_r | stype | i_jal | i_jalr |itype_l|i_lui|i_auipc;  //rs2/立即数

    // 立即数扩展信号
    // EXT_CTRL_ITYPE_SHAMT     6'b100000
    // EXT_CTRL_ITYPE	      6'b010000
    // EXT_CTRL_STYPE	      6'b001000
    // EXT_CTRL_BTYPE	      6'b000100
    // EXT_CTRL_UTYPE	      6'b000010
    // EXT_CTRL_JTYPE	      6'b000001
    assign EXTOp[5] = i_slli | i_srli | i_srai;
    assign EXTOp[4] = itype_l | i_addi | i_ori | i_andi | i_xori | i_jalr | i_slti | i_sltiu;
    assign EXTOp[3] = stype;
    assign EXTOp[2] = sbtype;
    assign EXTOp[1] = i_lui | i_auipc;
    assign EXTOp[0] = i_jal; 
    
    //写回方式
    // WDSel_FromALU 2'b00
    // WDSel_FromMEM 2'b01
    // WDSel_FromPC  2'b10 
    assign WDSel[0] = itype_l;
    assign WDSel[1] = i_jal | i_jalr;
   
    //NPC更新方式
    // NPC_PLUS4   3'b000
    // NPC_BRANCH  3'b001
    // NPC_JUMP    3'b010
    // NPC_JALR	   3'b100
    assign NPCOp[0] = sbtype;
    assign NPCOp[1] = i_jal;
	assign NPCOp[2]=i_jalr;
   
   /*
    //ALU Op
    assign ALUOp[0] = itype_l|stype|i_addi|i_ori|i_add|i_or|i_lui|i_slli|i_sll|i_sltu|i_sltiu|i_jalr|i_bne|i_bge|i_bgeu|i_sra|i_srai;
    assign ALUOp[1] = i_jalr|itype_l|stype|i_add|i_addi|i_and|i_andi|i_auipc|i_slli|i_sll|i_slt|i_sltu|i_slti|i_sltiu|i_blt|i_bge;
    assign ALUOp[2] = i_andi|i_and|i_ori|i_or|i_sub|i_xori|i_xor|i_slli|i_sll|i_blt|i_bne|i_bge|i_beq;
    assign ALUOp[3] = i_andi|i_and|i_ori|i_or|i_xori|i_slli|i_xor|i_sll|i_slt|i_sltu|i_slti|i_sltiu|i_bltu|i_bgeu;
    assign ALUOp[4] = i_sra|i_srl|i_srai|i_srli; 
    */
    
    //ALU Op
    reg [4:0] aluop_reg;
    always @(*) begin
        if (i_add | i_addi | itype_l | stype) aluop_reg = `ALUOp_add;
        else if (i_sub) aluop_reg = `ALUOp_sub;
        else if (i_lui) aluop_reg = `ALUOp_lui;
        else if (i_auipc) aluop_reg = `ALUOp_auipc;
        else if (i_bne) aluop_reg = `ALUOp_bne;
        //else if (i_beq) aluop_reg = `ALUOp_beq; 
        else if (i_blt) aluop_reg = `ALUOp_blt;
        else if (i_bge) aluop_reg = `ALUOp_bge;
        else if (i_bltu) aluop_reg = `ALUOp_bltu;
        else if (i_bgeu) aluop_reg = `ALUOp_bgeu;
        else if (i_slt | i_slti) aluop_reg = `ALUOp_slt;
        else if (i_sltu | i_sltiu) aluop_reg = `ALUOp_sltu;
        else if (i_xor | i_xori) aluop_reg = `ALUOp_xor;
        else if (i_or | i_ori) aluop_reg = `ALUOp_or;
        else if (i_and | i_andi) aluop_reg = `ALUOp_and;
        else if (i_sll | i_slli) aluop_reg = `ALUOp_sll;
        else if (i_srl | i_srli) aluop_reg = `ALUOp_srl;
        else if (i_sra | i_srai) aluop_reg = `ALUOp_sra;
        else if (i_div) aluop_reg = `ALUOp_div;
        else if (i_divu) aluop_reg = `ALUOp_divu;
        else aluop_reg = `ALUOp_nop;
    end
    assign ALUOp = aluop_reg;
    
    //访存方式
    //dm_word 3'b000
    //dm_halfword 3'b001
    //dm_halfword_unsigned 3'b010
    //dm_byte 3'b011
    //dm_byte_unsigned 3'b100
    assign DMType[0] = i_lh|i_sh|i_sb|i_lb;
    assign DMType[1] = i_lhu|i_sb|i_lb;
    assign DMType[2] = i_lbu;  
    
    assign MemRead = itype_l;
    
endmodule