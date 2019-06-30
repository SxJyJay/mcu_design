`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/06/10 19:05:09
// Design Name: 
// Module Name: alu
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


//`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////////
//// Company: 
//// Engineer: 
//// 
//// Create Date: 2019/05/13 20:35:20
//// Design Name: 
//// Module Name: prefix_adder
//// Project Name: 
//// Target Devices: 
//// Tool Versions: 
//// Description: 
//// 
//// Dependencies: 
//// 
//// Revision:
//// Revision 0.01 - File Created
//// Additional Comments:
//// 
////////////////////////////////////////////////////////////////////////////////////



//module logic_and(
//    input [31:0] a,b,
//    output [31:0] c
//);
//    assign c=a&b;
//endmodule

module logic_or(
    input [31:0] a,b,
    output [31:0] c
);
    assign c=a|b;
endmodule

module logic_not(
    input [31:0] a,
    output c
);
    assign c=~a;
endmodule

module shifter(
     input [31:0] a,     // 32 位原始输入数据
     input [4:0] b,      // 5 位输入数据，控制移位的位数
     output reg [31:0] c1,c2,c3     // 32 位输出，由a 经过b 位通过aluc 指定的移位方式移位而得
    );
       reg [31:0] temp1;
       reg [31:0] temp2;
       reg [31:0] temp3;
//       reg [4:0] b=5'b00010;
       initial begin
               temp1 = b[0] ? {{a[31]}, a[31:1]} : a;
               temp1 = b[1] ? {{2{temp1[31]}}, temp1[31:2]} : temp1;
               temp1 = b[2] ? {{4{temp1[31]}}, temp1[31:4]} : temp1;
               temp1 = b[3] ? {{8{temp1[31]}}, temp1[31:8]} : temp1;
               temp1 = b[4] ? {{16{temp1[31]}}, temp1[31:16]} : temp1;
      
                temp2 = b[0] ? {32'b0, a[31:1]} : a;
                temp2 = b[1] ? {32'b0, temp2[31:2]} : temp2;
                temp2 = b[2] ? {32'b0, temp2[31:4]} : temp2;
                temp2= b[3] ? {32'b0, temp2[31:8]} : temp2;
                temp2= b[4] ? {32'b0, temp2[31:16]} : temp2;
          
                temp3= b[0] ? {{a[30:0]}, 1'b0} : a;
                temp3= b[1] ? {{temp3[29:0]}, 2'b0} : temp3;
                temp3 = b[2] ? {{temp3[27:0]}, 4'b0} : temp3;
                temp3 = b[3] ? {{temp3[23:0]}, 8'b0} : temp3;
                temp3 = b[4] ? {{temp3[15:0]}, 16'b0} : temp3;
             
    
        c1 = temp1;
        c2 = temp2;
        c3 = temp3;
       end

endmodule

module prefix_adder_low(
        input [15:0] a, bin, input ctrl,                   
        output [15:0] s, output cout
        );
    wire [14:0] p, g;  
    wire [7:0] pij_0, gij_0, pij_1, gij_1,             pij_2, gij_2, pij_3, gij_3;  
    wire [15:0] gen;
    reg [15:0] b;
    reg cin;
    
  always@(ctrl,bin)
    begin
    case (ctrl)
        1'b0:
            begin
                b=bin;
                cin=1'b0;
            end
        1'b1:
            begin
                b=~bin;
                cin=1'b1;
            end
    endcase
    cin=ctrl;
    end
    
  pgblock pgblock_top(a[14:0], b[14:0], p, g);
  //p[-1]=cin, g[-1]=0
  pgblackblock pgblackblock_0({p[14], p[12], p[10],    p[8], p[6], p[4], p[2], p[0]},  {g[14], g[12], g[10], g[8], g[6], g[4], g[2], g[0]},  {p[13], p[11], p[9], p[7], p[5], p[3], p[1], 1'b0},  {g[13], g[11], g[9], g[7], g[5], g[3], g[1], cin}, pij_0, gij_0);
  pgblackblock pgblackblock_1({pij_0[7], p[13],     pij_0[5], p[9], pij_0[3], p[5], pij_0[1], p[1]},   {gij_0[7], g[13], gij_0[5], g[9], gij_0[3],     g[5], gij_0[1], g[1]},   { {2{pij_0[6]}}, {2{pij_0[4]}}, {2{pij_0[2]}},      {2{pij_0[0]}} },    { {2{gij_0[6]}}, {2{gij_0[4]}}, {2{gij_0[2]}},      {2{gij_0[0]}} }, pij_1, gij_1);
  pgblackblock pgblackblock_2({pij_1[7], pij_1[6], pij_0[6], p[11], pij_1[3], pij_1[2], pij_0[2], p[3]}, {gij_1[7], gij_1[6], gij_0[6], g[11], gij_1[3], gij_1[2], gij_0[2], g[3]}, { {4{pij_1[5]}}, {4{pij_1[1]}} }, { {4{gij_1[5]}}, {4{gij_1[1]}} }, pij_2, gij_2);
  pgblackblock pgblackblock_3({pij_2[7], pij_2[6],     pij_2[5], pij_2[4], pij_1[5], pij_1[4],     pij_0[4], p[7]},   {gij_2[7], gij_2[6], gij_2[5],     gij_2[4], gij_1[5], gij_1[4], gij_0[4], g[7]},  { 8{pij_2[3]} },{ 8{gij_2[3]} }, pij_3, gij_3);
  sumblock sum_out(a, b, gen, s);
  assign gen  = {gij_3, gij_2[3:0], gij_1[1:0], gij_0[0], cin};  
  assign cout = (a[15] & b[15]) | (gen[15] & (a[15] | b[15]));
endmodule
 module prefix_adder_high(
                 input [15:0] a, bin, input cin, ctrl,                   
                 output [15:0] s, output cout
         //        output  [15:0] bout
                 );
             wire [14:0] p, g;  
             wire [7:0] pij_0, gij_0, pij_1, gij_1,             pij_2, gij_2, pij_3, gij_3;  
             wire [15:0] gen;
             reg [15:0] b;
             
               always@(ctrl,bin)
               begin
               case (ctrl)
                   1'b0:
                   begin
                    b=bin;
                   end
                   1'b1:
                   begin
                     b=~bin;
                   end
               endcase
               end
             
         //  assign ctrl=0;
         //  assign cin=ctrl;
             
         //    assign bout=b;
             
           pgblock pgblock_top(a[14:0], b[14:0], p, g);
           //p[-1]=cin, g[-1]=0
           pgblackblock pgblackblock_0({p[14], p[12], p[10],    p[8], p[6], p[4], p[2], p[0]},  {g[14], g[12], g[10], g[8], g[6], g[4], g[2], g[0]},  {p[13], p[11], p[9], p[7], p[5], p[3], p[1], 1'b0},  {g[13], g[11], g[9], g[7], g[5], g[3], g[1], cin}, pij_0, gij_0);
           pgblackblock pgblackblock_1({pij_0[7], p[13],     pij_0[5], p[9], pij_0[3], p[5], pij_0[1], p[1]},   {gij_0[7], g[13], gij_0[5], g[9], gij_0[3],     g[5], gij_0[1], g[1]},   { {2{pij_0[6]}}, {2{pij_0[4]}}, {2{pij_0[2]}},      {2{pij_0[0]}} },    { {2{gij_0[6]}}, {2{gij_0[4]}}, {2{gij_0[2]}},      {2{gij_0[0]}} }, pij_1, gij_1);
           pgblackblock pgblackblock_2({pij_1[7], pij_1[6], pij_0[6], p[11], pij_1[3], pij_1[2], pij_0[2], p[3]}, {gij_1[7], gij_1[6], gij_0[6], g[11], gij_1[3], gij_1[2], gij_0[2], g[3]}, { {4{pij_1[5]}}, {4{pij_1[1]}} }, { {4{gij_1[5]}}, {4{gij_1[1]}} }, pij_2, gij_2);
           pgblackblock pgblackblock_3({pij_2[7], pij_2[6],     pij_2[5], pij_2[4], pij_1[5], pij_1[4],     pij_0[4], p[7]},   {gij_2[7], gij_2[6], gij_2[5],     gij_2[4], gij_1[5], gij_1[4], gij_0[4], g[7]},  { 8{pij_2[3]} },{ 8{gij_2[3]} }, pij_3, gij_3);
           sumblock sum_out(a, b, gen, s);
           assign gen  = {gij_3, gij_2[3:0], gij_1[1:0], gij_0[0], cin};  
           assign cout = (a[15] & b[15]) | (gen[15] & (a[15] | b[15]));
         endmodule        
         module pgblock(
                 input [14:0] a, b,                
                 output [14:0] p, g);    assign p = a | b;  assign g = a & b;
         endmodule        
         module pgblackblock(
                 input [7:0] pik, gik, pk_1j, gk_1j,  
                 output [7:0] pij, gij);    assign pij = pik & pk_1j;  assign gij = gik | (pik & gk_1j);  
                 endmodule
         module sumblock(
                 input [15:0] a, b, g,        
                 output [15:0] s);
                  assign s = a ^ b ^ g;  
                  endmodule
                 
module prefix_adder(
        input [31:0] a,b,
        input ctrl,
        output [31:0] s,
        output cout
  );
  wire c_middle;
    prefix_adder_low lower_bits(
            .a(a[15:0]), .bin(b[15:0]), 
            .ctrl(ctrl),
            .s(s[15:0]),
            .cout(c_middle)
    );
    prefix_adder_high higher_bits(
            .a(a[31:16]), .bin(b[31:16]),
            .ctrl(ctrl),
            .cin(c_middle),
            .s(s[31:16]),
            .cout(cout)
    );
  endmodule
  
module comparision(
    input [31:0]a,
    input [31:0]b,
    input[31:0]sub,
    output reg slt
        );
always@(*)
    begin
        if(a[31]==1'b1&&b[31]==1'b0)
        slt=1'b1;
    else if(a[31]==1'b0&&b[31]==1'b1)
        slt=1'b0;
    else
        slt=sub[31];
    end
    endmodule


module ALU_Final(
    input [2:0] ctrl,
    input [31:0] a,b,
    output reg [31:0] y
);
    reg [4:0] shamt=5'b00010;
    wire [31:0] d[6:0];
    wire d_1;
    wire [31:0] suibian;
    
    //instantiation begin
    logic_or huo(
            .a(a), .b(b), .c(d[0])
    );
    logic_not fei(
        .a(a), .c(d[1])
);
    prefix_adder jia(
            .a(a), .b(b), .ctrl(1'b0),
            .s(d[2]), .cout(suibian)
    );
    prefix_adder jian(
            .a(a), .b(b), .ctrl(1'b1),
            .s(d[3]), .cout(suibian)
    );
    comparision slt(
             .a(a), .b(b), .sub(d[3]), .slt(d_1)
    );
      shifter dut(.a(a),.b(shamt),.c1(d[5]),.c2(d[6]),.c3(d[4]));//meiyoukonzhiyijiweide
      
//      booth_mult m1(d[7],a[15:0],b[15:0]);

    //instantiation end
    
    always@(ctrl)
        case(ctrl)
            3'b000:y=d[0]; //or
            3'b001:y=d[1]; //not
            3'b010:y=d[2]; //add
            3'b011:y=d[3]; //sub
            3'b100:y=d[4]; //shift left
            3'b101:y=d[5]; //algorithm shift right
            3'b110:y=d[6]; //logic shift right
            3'b111:y=d_1;     //mul       
        endcase
            
endmodule

module booth_mult_sm (p,xin,yin);
input [15:0] xin, yin;
logic [15:0] x,y;
output logic [31:0]p;
reg [2:0] cc[7:0];
reg [17:0] pp[7:0];
reg [31:0] spp[7:0];
logic [16:0] inv_x;
wire [31:0] suibian;
logic [31:0] prod1,prod2,prod3,prod4,prod5,prod6,prod7;
reg cout;
integer i,j;

always @ (xin or yin)
begin
//turn signed magnitude into signed decimal
        if ( xin[15] == 1 )
            x = {1'b1,~xin[14:0]}+1;
        if ( yin[15] == 1 )
            y = {1'b1,~yin[14:0]}+1;
          
inv_x = {~x[15],~x}+1;
cc[0] = {y[1],y[0],1'b0}; //generate Ck for each k, for k is not zero
for(j=1;j<8;j=j+1)
cc[j] = {y[2*j+1],y[2*j],y[2*j-1]};

for(j=0;j<8;j=j+1)//计算部分积
begin
case(cc[j])
3'b001 , 3'b010 : pp[j] = {x[15],x[15],x};
3'b011 : pp[j] = {x[15],x,1'b0};
3'b100 : pp[j] = {inv_x[16:0],1'b0};
3'b101 , 3'b110 : pp[j] = {inv_x[16],inv_x};
default : pp[j] = 0;
endcase

spp[j] = $signed(pp[j]);
for(i=0;i<j;i=i+1)
spp[j] = {spp[j],2'b00}; //multiply by 2 to the power x or shifting operation
end //'for(j=0;j<N;j=j+1)'
end
prefix_adder add0(
          spp[0], spp[1], 0,
            prod1,cout);
prefix_adder add1(
           spp[2], spp[3], 0,
            prod2);
prefix_adder add2(
           spp[4], spp[5], 0,
            prod3);
prefix_adder add3(
           spp[6], spp[7], 0,
            prod4);
 prefix_adder add4(
            prod1, prod2, 0,
            prod5);
prefix_adder add5(
            prod3, prod4, 0,
            prod6);
prefix_adder add6(
            prod5, prod6, 0,
            prod7);
always@(prod7)
    begin
        if (prod7[31] == 1)
            p = {1'b1,~prod7[31:16]}+1;
        else
            p = prod7[31:16];
    end
endmodule

module booth_mult (p,x,y);
input[15:0]x, y;
output logic [15:0]p;
reg [2:0] cc[7:0];
reg [17:0] pp[7:0];
reg [31:0] spp[7:0];
wire [16:0] inv_x;
wire [31:0] suibian;
logic [31:0] prod1,prod2,prod3,prod4,prod5,prod6,prod7;
reg cout;
integer i,j;
//generate two's complement of mutiplicand X(M)
assign inv_x = {~x[15],~x}+1;

always @ (x or y or inv_x)
begin
cc[0] = {y[1],y[0],1'b0}; //generate Ck for each k, for k is not zero
for(j=1;j<8;j=j+1)
cc[j] = {y[2*j+1],y[2*j],y[2*j-1]};

for(j=0;j<8;j=j+1)//计算部分积
begin
case(cc[j])
3'b001 , 3'b010 : pp[j] = {x[15],x[15],x};
3'b011 : pp[j] = {x[15],x,1'b0};
3'b100 : pp[j] = {inv_x[16:0],1'b0};
3'b101 , 3'b110 : pp[j] = {inv_x[16],inv_x};
default : pp[j] = 0;
endcase

spp[j] = $signed(pp[j]);
for(i=0;i<j;i=i+1)
spp[j] = {spp[j],2'b00}; //multiply by 2 to the power x or shifting operation
end //'for(j=0;j<N;j=j+1)'
end
prefix_adder add0(
          spp[0], spp[1], 0,
            prod1,cout);
prefix_adder add1(
           spp[2], spp[3], 0,
            prod2);
prefix_adder add2(
           spp[4], spp[5], 0,
            prod3);
prefix_adder add3(
           spp[6], spp[7], 0,
            prod4);
 prefix_adder add4(
            prod1, prod2, 0,
            prod5);
prefix_adder add5(
            prod3, prod4, 0,
            prod6);
prefix_adder add6(
            prod5, prod6, 0,
            prod7);
endmodule
