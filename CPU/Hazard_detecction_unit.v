`timescale 1ns / 1ps

//����ð�գ������ǰҪ�õ���rs1/rs2����һ��ָ���rd���ˣ�����һ��ָ���ldָ���ʱ��һ��ָ��ֵ��δȡ����Ҫ����һ������
//ð�ռ�ⵥԪ�������һ����
//�߼���EX/Mem Memread = 1 && ID/EX rsi = EX/Mem rd 
module Hazard_detecction_unit(
    input EX_Mem_r,
    input [4:0] rs1,
    input [4:0] rs2,
    input [4:0] EX_rd,
    output reg Stall);
    
    always@(*) begin
        if(EX_rd != 0) begin
            if(EX_Mem_r && ((EX_rd == rs1)||(EX_rd == rs2))) begin
                Stall <= 1'b1;
            end
            else begin Stall <=1'b0; end
        end
        else begin Stall <=1'b0; end
    end

endmodule
