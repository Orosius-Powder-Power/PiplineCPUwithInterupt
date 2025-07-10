`timescale 1ns / 1ps

module ExceptionUnit(
    input wire       In_Exception,      // CPU是否已在异常处理中 (来自 In_Exception_Reg)
    input wire [7:0] STATUS,            // 状态寄存器 (bit 0: 全局中断使能)
    input wire [7:0] EX_SCAUSE,         // EX阶段传来的中断/异常原因
    input wire [7:0] INTMASK,           // 中断屏蔽寄存器
    
    output wire      Take_Interrupt     // 最终决定：是否进入中断处理
);

    // 检查是否有未被屏蔽的中断请求
    // 假设EX_SCAUSE的每一位对应一个中断源，1表示请求，0表示无请求
    wire interrupt_pending = |(EX_SCAUSE & ~INTMASK);

    // 全局中断使能位 (假设在STATUS寄存器的bit 0)
    wire global_interrupt_enable = STATUS[0];

    // 最终决定是否接受中断的条件：
    // 1. 有未屏蔽的中断挂起
    // 2. 全局中断已使能
    // 3. 当前不处于另一个中断/异常处理过程中
    assign Take_Interrupt = interrupt_pending && global_interrupt_enable && !In_Exception;

endmodule
