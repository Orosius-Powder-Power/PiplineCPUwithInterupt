`define WDSel_FromALU 2'b00
`define WDSel_FromMEM 2'b01
`define WDSel_FromPC 2'b10
`define INT_HANDLER_ADDR 32'h1C090000

//新增中断类型：外部中断（INT）、除零异常、溢出异常、非法指令异常、ecall异常
module PCPU(
    input   clk,            // clock
    input   reset,          // reset
    input   INT,
    input   MIO_ready,
    input   [31:0]  inst_in,// instruction
    input   [31:0]  Data_in,// data from data memory
          
    output  mem_w,          // output: memory write signal
    output  [31:0] PC_out,  // PC address

    output  [31:0] Addr_out,// ALU output
    output  [31:0] Data_out,// data to data memory
           
    output  CPU_MIO,
    //input  [4:0] reg_sel,      // register selection (for debug use)
    //output [31:0] reg_data,    // selected register data (for debug use)
    output  [2:0] dm_ctrl
);
    wire        RegWrite;    // control signal to register write
    wire [5:0]  EXTOp;       // control signal to signed extension
    wire [4:0]  ALUOp;       // ALU opertion
    wire [2:0]  NPCOp;       // next PC operation

    wire [1:0]  WDSel;       // (register) write data selection
    wire [1:0]  GPRSel;      // general purpose register selection
   
    wire        ALUSrc;      // ALU source for A
    wire        Zero;        // ALU ouput zero

    wire [31:0] NPC;         // next PC
    wire [31:0] next_pc_val;         // final next PC （中断处理后）
    
    wire [4:0]  rs1;         // rs
    wire [4:0]  rs2;         // rt
    wire [4:0]  rd;          // rd
    wire [6:0]  Op;          // opcode
    wire [6:0]  Funct7;      // funct7
    wire [2:0]  Funct3;      // funct3
    wire [11:0] Imm12;       // 12-bit immediate
    wire [31:0] Imm32;       // 32-bit immediate
    wire [19:0] IMM;         // 20-bit immediate (address)
    wire [4:0]  A3;          // register address for write
    reg  [31:0] WD;          // register write data
    wire [31:0] RD1,RD2;     // register data specified by rs
    wire [31:0] B;           // operator for ALU B
	
	wire [4:0]  iimm_shamt;
	wire [11:0] iimm,simm,bimm;
	wire [19:0] uimm,jimm;
	wire [31:0] immout;
    wire [31:0] aluout;
    
    wire [255:0] MEM_WB_in;
    wire [255:0] MEM_WB_out;
    
    wire [255:0] EX_MEM_in;
    wire [255:0] EX_MEM_out;
    
    wire Stall;
    
    wire IF_ID_flush;
    wire ID_EX_flush;
    wire EX_MEM_flush;
    wire MEM_WB_flush;
    
    wire [2:0] True_NPCOp;
    
    wire [31:0] MEM_PC,MEM_RD2,MEM_aluout;
    wire [4:0] MEM_rd;
    wire [2:0] MEM_DMType,MEM_WDSel;
    wire MEM_RegWrite,MEM_Mem_w;
    
    wire flush;
    wire branch_flush;
    wire exception_flush;
    
    wire [255:0] IF_ID_in;
    wire [255:0] IF_ID_out;
    wire [31:0] ID_PC,ID_inst;
    
    wire [31:0] instruction;
    
    wire [2:0] DMType;
    wire Mem_w;
    wire Mem_r;
    
    wire [255:0] ID_EX_in;
    wire [255:0] ID_EX_out;
    wire [31:0] EX_PC,EX_RD1,EX_RD2,EX_immout;
    wire [4:0] EX_rs1,EX_rs2,EX_rd;
    wire EX_ALUSrc,EX_RegWrite,EX_Mem_w,EX_Mem_r;
    wire [4:0]EX_ALUOp;
    wire [2:0]EX_NPCOp,EX_DMType;
    wire [1:0]EX_WDSel;
    
    wire [1:0] forwardA;
    wire [1:0] forwardB;
    reg [31:0] ALU_A;
    reg [31:0] ALU_B;
    
    wire [31:0] WB_PC;
    wire [4:0] WB_rd;
    wire WB_RegWrite;
    wire [1:0] WB_WDSel;
    wire [31:0] WB_Data_in;
    wire [31:0] WB_aluout; 
    
    wire [4:0] True_ALUOp;
    
    //中断处理
    // New Control Signals for ERET/ERETN
    wire INT;
    wire is_eret, is_eretn;
    wire EX_is_eret, EX_is_eretn;
    
    //异常
    wire is_ecall;
    wire EX_is_ecall;
    wire div_by_zero;
    wire overflow;
    wire is_exception;
    
    // CSRs (Control and Status Registers)
    reg [31:0] SEPC;
    reg [7:0] SCAUSE;
    reg [7:0]  STATUS;   // bit 0: Global Interrupt Enable (GIE)
    reg [7:0]  INTMASK;  // bit-wise mask for interrupts
    
    // Internal state to indicate CPU is in exception handler
    reg In_Exception_Reg;
    
    // Interrupt detection logic in EX stage
    wire        Take_Interrupt;
    reg [7:0] EX_SCAUSE_val;
    wire        external_interrupt_req;
    
    
    //一、IF阶段
    assign IF_ID_in [31:0] = PC_out;     //存放PC
    assign IF_ID_in [63:32] = inst_in;    //存放指令（供译码）
    GRE_array #(256) IF_ID(
        .clk(clk),
        .rst(reset),
        .write_enable(~Stall),
        .flush(IF_ID_flush),
        .in(IF_ID_in),
        .out(IF_ID_out)
    );
    
    
    //二、ID阶段
    assign ID_PC = IF_ID_out [31:0];      //读取PC和指令
    assign ID_inst = IF_ID_out [63:32];
    
    assign instruction = ID_inst;      //读取指令
	
	//译码
	assign iimm_shamt = instruction[24:20];
	assign iimm = instruction[31:20];
	assign simm = {instruction[31:25],instruction[11:7]};
	assign bimm = {instruction[31],instruction[7],instruction[30:25],instruction[11:8]};
	assign uimm = instruction[31:12];
	assign jimm = {instruction[31],instruction[19:12],instruction[20],instruction[30:21]};
   
    assign Op = instruction[6:0];  // instruction
    assign Funct7 = instruction[31:25]; // funct7
    assign Funct3 = instruction[14:12]; // funct3
    assign rs1 = instruction[19:15];  // rs1
    assign rs2 = instruction[24:20];  // rs2
    assign rd = instruction[11:7];  // rd
    assign Imm12 = instruction[31:20];// 12-bit immediate
    assign IMM = instruction[31:12];  // 20-bit immediate
    
    //RF读写
    RF U_RF(
	   .clk(clk), .rst(reset),
	   .RFWr(WB_RegWrite), 
	   .A1(rs1), .A2(rs2), .A3(WB_rd), 
	   .WD(WD),
	   .RD1(RD1), .RD2(RD2)
		//.reg_sel(reg_sel),
		//.reg_data(reg_data)
	);
	
	//产生ctrl
	 ctrl U_ctrl(
		.Op(Op), .Funct7(Funct7), .Funct3(Funct3), 
		.RegWrite(RegWrite), .MemWrite(Mem_w),
		.EXTOp(EXTOp), .ALUOp(ALUOp), .NPCOp(NPCOp), 
		.ALUSrc(ALUSrc),.DMType(DMType), .GPRSel(GPRSel), .WDSel(WDSel), .MemRead(Mem_r),
		.is_eret(is_eret), .is_eretn(is_eretn),
        .is_ecall(is_ecall)
	);
    
    //立即数扩展
    EXT U_EXT(
		.iimm_shamt(iimm_shamt), .iimm(iimm), .simm(simm), .bimm(bimm),
		.uimm(uimm), .jimm(jimm),
		.EXTOp(EXTOp), .immout(immout)
	);
	
	//冒险检测单元（识别ld型数据冒险）
	Hazard_detecction_unit hazard_detecction_unit(
        .EX_Mem_r(EX_Mem_r),
        .rs1(rs1),
        .rs2(rs2),
        .EX_rd(EX_rd),
        .Stall(Stall)
    );
    
    //ID-EX reg:存放RF信息和ctrl
    assign ID_EX_in = {
        92'b0, // Unused bits to pad to 256
        is_ecall, is_eretn, is_eret, // 43bits
        Mem_r, Mem_w, WDSel, RegWrite, DMType, NPCOp, ALUOp, ALUSrc, // 18 bits
        rd, rs2, rs1, // 15 bits
        immout, // 32 bits
        RD2, RD1, ID_PC // 96 bits
    };
    
     GRE_array #(256) ID_EX(
        .clk(clk), .rst(reset),
        .write_enable(1'b1), .flush(ID_EX_flush),
        .in(ID_EX_in),
        .out(ID_EX_out)
    );
    
    
    //三、EX阶段
    //从寄存器中读取相应数据
    assign {EX_is_ecall, EX_is_eretn, EX_is_eret, // 3 bits
            EX_Mem_r, EX_Mem_w, EX_WDSel, EX_RegWrite, EX_DMType, EX_NPCOp, EX_ALUOp, EX_ALUSrc, // 18 bits
            EX_rd, EX_rs2, EX_rs1, // 15 bits
            EX_immout, // 32 bits
            EX_RD2, EX_RD1, EX_PC} = ID_EX_out[163:0];
    
    //前递单元
    Forwarding_unit forwarding_unit(
        .EX_rs1(EX_rs1),
        .EX_rs2(EX_rs2),
        .MEM_rd(MEM_rd),
        .WB_rd(WB_rd),
        .Mem_RegWrite(MEM_RegWrite),
        .WB_RegWrite(WB_RegWrite),
        .ForwardA(forwardA),
        .ForwardB(forwardB)
    );
    
    //根据前递单元确定A
    always @(*)
    begin
	   case(forwardA)
		  2'b00: ALU_A<=EX_RD1;        //不前递
		  2'b01: ALU_A<=WD;            //上上条指令rd
		  2'b10: ALU_A<=MEM_aluout;     //上条指令rd
	   endcase
	   case(forwardB)
		  2'b00: ALU_B<=EX_RD2;
		  2'b01: ALU_B<=WD;
		  2'b10: ALU_B<=MEM_aluout;
	   endcase
    end
    
    //根据ALUSrc确定B
    assign B = (EX_ALUSrc) ? EX_immout : ALU_B;
   
    //执行ALU运算
    alu U_alu(.A(ALU_A), .B(B), .ALUOp(EX_ALUOp), .C(aluout), .Zero(Zero), .PC(EX_PC), .div_by_zero(div_by_zero), .overflow(overflow));
    
   //EX阶段确定PC更新逻辑
    assign True_NPCOp = {EX_NPCOp[2:1], EX_NPCOp[0] & Zero};
    
    //更新NPC（并产生flush，当且仅当跳转时产生flush信号）
    NPC U_NPC(.PC(PC_out), .NPCOp(True_NPCOp), .IMM(EX_immout), .NPC(NPC),.JumpPC(EX_PC),.flush(branch_flush), .aluout(aluout));
    
    
    //region 中断处理
    // 外部中断信号
    assign external_interrupt_req = INT;
    
    //内部异常
    assign is_exception = EX_is_ecall || div_by_zero || overflow;
    
    //中断类型
    always @(*) begin
        if (overflow)           EX_SCAUSE_val = 8'd2;  // Arithmetic Overflow (custom code)
        else if (div_by_zero)        EX_SCAUSE_val = 8'd4;  // Divide by zero
        else if (EX_is_ecall)        EX_SCAUSE_val = 8'd8;  // ECALL from U-mode
        else if (INT)                EX_SCAUSE_val = 8'd16; // External Interrupt (Timer)
        else                         EX_SCAUSE_val = 8'd0;
    end

    // Instantiate the Exception Unit
    ExceptionUnit U_ExceptionUnit (
        .In_Exception   (In_Exception_Reg),
        .STATUS         (STATUS),
        .EX_SCAUSE      (EX_SCAUSE_val),  
        .INTMASK        (INTMASK),
        .Take_Interrupt (Take_Interrupt)
    );
    
    // Logic to update CSRs and exception state
   always @(posedge clk or posedge reset) begin
        if (reset) begin
            SEPC             <= 32'd0;
            SCAUSE           <= 32'd0;
            STATUS           <= 8'h01; // Enable interrupts by default for testing
            INTMASK          <= 8'h00; // Unmask all interrupts by default for testing
            In_Exception_Reg <= 1'b0;
        end else begin
            if (is_exception || Take_Interrupt) begin
                // Entering interrupt handler
                SEPC             <= EX_PC;
                SCAUSE           <= EX_SCAUSE_val;
                In_Exception_Reg <= 1'b1;
            end else if (EX_is_eret || EX_is_eretn) begin
                // Returning from interrupt
                In_Exception_Reg <= 1'b0;
            end
        end
    end
    //endregion
    
    
    //Flush control
    //（1）如果产生ld型数据冒险需要阻塞，则需要flush掉ID/EX reg
    //（2）如果分支预测失败，则需要flush掉IF/ID reg和ID/EX reg （EX阶段判断分支预测正确与否）
    //（3）如果产生中断，则应flush掉EX/Mem reg
    assign exception_flush = is_exception || Take_Interrupt || EX_is_eret || EX_is_eretn;
    assign flush = branch_flush || exception_flush;
    assign IF_ID_flush = flush; 
    assign ID_EX_flush = flush || Stall; // Stall flushes ID/EX with a bubble.
    assign EX_MEM_flush = is_exception || Take_Interrupt; // Flush EX/MEM on exception to inject bubble
    assign MEM_WB_flush = 1'b0; // This stage is never flushed
    
    // Final PC selection MUX
    assign next_pc_val = (is_exception || Take_Interrupt) ? `INT_HANDLER_ADDR :
                         (EX_is_eret)   ? SEPC :
                         (EX_is_eretn)  ? SEPC + 4 :
                         NPC;  
    
    //更新PC
    PC U_PC(.clk(clk), .rst(reset), .NPC(next_pc_val), .Stall(Stall), .PC(PC_out));

    //EX-Mem reg
    assign EX_MEM_in [31:0] = EX_PC; //PC
    assign EX_MEM_in [63:32] = aluout; //C
    assign EX_MEM_in [95:64] = ALU_B; //reg[rs2] (可能要存到Mem中)
    assign EX_MEM_in [100:96] = EX_rd; //rd
    assign EX_MEM_in [103:101] = EX_DMType; //dm_ctrl
    assign EX_MEM_in [104] = EX_RegWrite; //RegWrite
    assign EX_MEM_in [106:105] = EX_WDSel ;//WDSel
    assign EX_MEM_in [107] = EX_Mem_w; //mem_w
    
    GRE_array #(256) EX_MEM(
        .clk(clk),
        .rst(reset),
        .write_enable(1'b1),
        .flush(EX_MEM_flush),
        .in(EX_MEM_in),
        .out(EX_MEM_out)
    );
    
    
    //四、Mem阶段
    //读取EX-Mem寄存器信息
    assign MEM_PC = EX_MEM_out [31:0];
    assign MEM_aluout = EX_MEM_out [63:32];
    assign MEM_RD2 = EX_MEM_out [95:64];
    assign MEM_rd = EX_MEM_out [100:96];
    assign MEM_DMType = EX_MEM_out [103:101];
    assign MEM_RegWrite = EX_MEM_out [104];
    assign MEM_WDSel = EX_MEM_out [106:105];
    assign MEM_Mem_w = EX_MEM_out [107];
    
    //输出相应访存信息与RAM模块交互
    assign Addr_out = MEM_aluout;       //访存地址
	assign Data_out = MEM_RD2;       //存放数据
	assign dm_ctrl = MEM_DMType;      //读写方式
	assign mem_w = MEM_Mem_w;
    
    //Mem-WB reg
    assign MEM_WB_in [31:0] = MEM_PC; //PC
    assign MEM_WB_in [36:32] = MEM_rd; //rd
    assign MEM_WB_in [37] = MEM_RegWrite; //RegWrite
    assign MEM_WB_in [39:38] = MEM_WDSel; //WDSel
    assign MEM_WB_in [71:40] = Data_in;   //RAM中读取到的数据
    assign MEM_WB_in [103:72] = MEM_aluout; //C
    
    GRE_array #(256) MEM_WB(
        .clk(clk),
        .rst(reset),
        .write_enable(1'b1),
        .flush(MEM_WB_flush),
        .in(MEM_WB_in),
        .out(MEM_WB_out)
    );
    
    
    //五、WB阶段
    //Mem-WB reg
    assign WB_PC = MEM_WB_out [31:0];
    assign WB_rd = MEM_WB_out [36:32];
    assign WB_RegWrite = MEM_WB_out [37];
    assign WB_WDSel = MEM_WB_out [39:38];
    assign WB_Data_in = MEM_WB_out [71:40];
    assign WB_aluout = MEM_WB_out [103:72];
    
    //写回RF
    always @*
    begin
        case(WB_WDSel)
            `WDSel_FromALU: WD<=WB_aluout;
            `WDSel_FromMEM: WD<=WB_Data_in;
            `WDSel_FromPC: WD<= WB_PC+4;
        endcase
    end
    
endmodule