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
debug:通过仿真debug发现，触发中断时，由于ID-EX寄存器的EX_PC被flush掉了，导致存入的SEPC为0，中断处理后跳转到了0，这显然是不对的
所以我打算，流水线寄存器的前32位（即存放PC）不被flush掉
*/