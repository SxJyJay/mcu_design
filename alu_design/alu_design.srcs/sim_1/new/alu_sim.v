`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/06/09 15:46:18
// Design Name: 
// Module Name: alu_sim
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module alu_sim;
//reg clk=0;
//always #5 clk <= ~clk;
    reg [2:0] ctrl;
    reg [31:0] a,b;
    reg [4:0] shamt=5'b00010;
//    wire [31:0] suibian;
//    wire k;
    wire [31:0] c;
    wire [31:0] d[6:4];
    
    
initial begin  
    a=32'b0010_0000_0001_0001;
    b=32'b0100_0010_0010_0000;
    ctrl=3'b000;
    #20
    ctrl=3'b100;
end
    
 shifter dut(.a(a),.b(shamt),.c1(d[5]),.c2(d[6]),.c3(d[4]));
    
//      mux shift(
//            .a(a), .b(b), .ctrl(ctrl),
//            .y(c)
//    );

//    prefix_adder jian(
//        .a(a), .b(b), .ctrl(ctrl[0]),
//        .s(c), .cout(k)
//);


//    assign suibian=32'b0000_0000_0000_0000;
    //test or
//    assign ctrl=3'b000;
//    mux huo(
//            .a(a), .b(b), .ctrl(ctrl),
//            .y(c)
//    );
    
    //test not
//    assign ctrl=3'b001;
//    mux fei(
//            .a(a), .b(suibian), .ctrl(ctrl),
//            .y(c)
//    );

    //test add

endmodule
