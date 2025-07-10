`timescale 1ns / 1ps

module GRE_array #(parameter WIDTH = 256)(
    input clk, rst, write_enable, flush,
    input [WIDTH-1:0] in,
    output reg [WIDTH-1:0] out
    );
    
    localparam FLUSH_WIDTH = WIDTH - 32;
    
    always@(negedge clk or posedge rst)
    begin
        if(rst) begin out <= 0; end
        else if(write_enable)
        begin
            if(flush)
                out <= { {(FLUSH_WIDTH){1'b0}}, in[31:0] };
            else
                out <= in;
        end
    end
    
endmodule

/*
debug:ͨ������debug���֣������ж�ʱ������ID-EX�Ĵ�����EX_PC��flush���ˣ����´����SEPCΪ0���жϴ������ת����0������Ȼ�ǲ��Ե�
�����Ҵ��㣬��ˮ�߼Ĵ�����ǰ32λ�������PC������flush��
*/