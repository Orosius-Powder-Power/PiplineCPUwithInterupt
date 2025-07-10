`timescale 1ns / 1ps

//数据冒险：如果当前要用到的rs1/rs2和上一条指令的rd重了，且上一条指令的ld指令，此时上一条指令值还未取出，要阻塞一个周期
//冒险检测单元来解决这一问题
//逻辑：EX/Mem Memread = 1 && ID/EX rsi = EX/Mem rd 
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
