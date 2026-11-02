`timescale 1ns/1ps
// Five-stage control-path validator. No video latency is added.
// Fixed 1920x1080, fx=fy=1400, cx=960, cy=540 only.
// Weighted budget reserves margin for fixed-point rounding and bilinear rows.
module distortion_coeff_guard(
 input wire clk,rst_n,valid_in,
 input wire signed[31:0] k1,k2,p1,p2,
 output reg valid_out,accepted
);
 reg[3:0] vp,rp;
 reg[12:0] a,b;reg[8:0] c,d;
 reg[18:0] wa,wb,wc,wd;
 reg[19:0] s0,s1;
 reg[20:0] total;
 wire ranges_ok=k1>=-32'sd2621 && k1<=32'sd2621 &&
  k2>=-32'sd2621 && k2<=32'sd2621 &&
  p1>=-32'sd262 && p1<=32'sd262 && p2>=-32'sd262 && p2<=32'sd262;
 wire[31:0] ak1=k1[31] ? (~k1+32'd1) : k1;
 wire[31:0] ak2=k2[31] ? (~k2+32'd1) : k2;
 wire[31:0] ap1=p1[31] ? (~p1+32'd1) : p1;
 wire[31:0] ap2=p2[31] ? (~p2+32'd1) : p2;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   vp<=0;rp<=0;a<=0;b<=0;c<=0;d<=0;
   wa<=0;wb<=0;wc<=0;wd<=0;s0<=0;s1<=0;total<=0;valid_out<=0;accepted<=0;
  end else begin
   vp<={vp[2:0],valid_in};rp<={rp[2:0],ranges_ok};
   a<=ak1[12:0];b<=ak2[12:0];c<=ap1[8:0];d<=ap2[8:0];
   wa<=({6'd0,a}<<2)+({6'd0,a}<<1); // 6 |K1|
   wb<=({6'd0,b}<<2);                 // 4 |K2|
   wc<=({10'd0,c}<<4)+({10'd0,c}<<3); //24 |P1|
   wd<=({10'd0,d}<<4)-({10'd0,d}<<1); //14 |P2|
   s0<={1'b0,wa}+{1'b0,wb};s1<={1'b0,wc}+{1'b0,wd};
   total<={1'b0,s0}+{1'b0,s1};
   valid_out<=vp[3];accepted<=rp[3] && total<=21'd15726;
  end
 end
endmodule
