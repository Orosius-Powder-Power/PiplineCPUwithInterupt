`timescale 1ns / 1ps

//前递单元
//ID/EX rsi = EX/Mem rd  -->  MEM_Forward  (前提条件，对应的rd写信号要有效)
//ID/EX rsi = Mem/WB rd --> WB_Forward
//有一个特例，如果当前要用到的rs1/rs2和上一条指令的rd重了，且上一条指令的ld指令，此时上一条指令值还未取出，要阻塞一个周期
module Forwarding_unit(
    input [4:0] EX_rs1,
    input [4:0] EX_rs2,
    input [4:0] MEM_rd,
    input [4:0] WB_rd,
    input Mem_RegWrite,
    input WB_RegWrite,
    output [1:0] ForwardA,
    output [1:0] ForwardB
    );
    wire MEM_ForwardA = (MEM_rd == 0)? 1'b0:((~(|(MEM_rd ^ EX_rs1))) & (Mem_RegWrite));
    wire WB_ForwardA = (WB_rd == 0)? 1'b0:((~(|(WB_rd ^ EX_rs1))) & (WB_RegWrite) & (~MEM_ForwardA));
    assign ForwardA = {{MEM_ForwardA}, {WB_ForwardA}};
    wire MEM_ForwardB = (MEM_rd == 0)? 1'b0:((~(|(MEM_rd ^ EX_rs2))) & (Mem_RegWrite));
    wire WB_ForwardB = (WB_rd == 0)? 1'b0:((~(|(WB_rd ^ EX_rs2))) & (WB_RegWrite) & (~MEM_ForwardB));
    assign ForwardB = {{MEM_ForwardB}, {WB_ForwardB}};

    
endmodule
