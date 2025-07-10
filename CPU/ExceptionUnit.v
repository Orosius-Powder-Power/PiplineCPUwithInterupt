`timescale 1ns / 1ps

module ExceptionUnit(
    input wire       In_Exception,      // CPU�Ƿ������쳣������ (���� In_Exception_Reg)
    input wire [7:0] STATUS,            // ״̬�Ĵ��� (bit 0: ȫ���ж�ʹ��)
    input wire [7:0] EX_SCAUSE,         // EX�׶δ������ж�/�쳣ԭ��
    input wire [7:0] INTMASK,           // �ж����μĴ���
    
    output wire      Take_Interrupt     // ���վ������Ƿ�����жϴ���
);

    // ����Ƿ���δ�����ε��ж�����
    // ����EX_SCAUSE��ÿһλ��Ӧһ���ж�Դ��1��ʾ����0��ʾ������
    wire interrupt_pending = |(EX_SCAUSE & ~INTMASK);

    // ȫ���ж�ʹ��λ (������STATUS�Ĵ�����bit 0)
    wire global_interrupt_enable = STATUS[0];

    // ���վ����Ƿ�����жϵ�������
    // 1. ��δ���ε��жϹ���
    // 2. ȫ���ж���ʹ��
    // 3. ��ǰ��������һ���ж�/�쳣���������
    assign Take_Interrupt = interrupt_pending && global_interrupt_enable && !In_Exception;

endmodule
