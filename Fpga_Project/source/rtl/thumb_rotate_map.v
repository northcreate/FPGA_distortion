`timescale 1ns/1ps
// Seven registered stages. 160x90 thumbnail shown at 4x, clockwise configurable integer degrees.
// Inverse transform: (R(-15)*(output-center))/4 + (80,45).
module thumb_rotate_map(
 input wire clk,rst_n,valid_in,
 input wire [11:0] x,y,
 input wire signed [15:0] translate_x,translate_y,
 input wire signed [17:0] cos_q,sin_q,
 output reg valid_out,inside_out,
 output reg [13:0] addr_out,
 output reg [7:0] u_out,v_out,
 output reg [15:0] wx_out,wy_out
);
 reg [5:0] vp;
 reg signed [17:0] dx,dy,cos_s1,sin_s1;
 reg signed [35:0] xc,ys,xs,yc;
 reg signed [36:0] rx,ry;
 reg signed [37:0] uq,vq;
 reg [7:0] ui,vi,u6,v6;
 reg [15:0] wx5,wy5,wx6,wy6;
 reg inside5,inside6;
 reg [13:0] row128_plus_x,row32;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   u_out<=0;v_out<=0;wx_out<=0;wy_out<=0;u6<=0;v6<=0;wx5<=0;wy5<=0;wx6<=0;wy6<=0;
   vp<=0;dx<=0;dy<=0;cos_s1<=18'sd65536;sin_s1<=0;xc<=0;ys<=0;xs<=0;yc<=0;rx<=0;ry<=0;
   uq<=0;vq<=0;ui<=0;vi<=0;inside5<=0;inside6<=0;
   row128_plus_x<=0;row32<=0;valid_out<=0;inside_out<=0;addr_out<=0;
  end else begin
   vp<={vp[4:0],valid_in};
   // S1
   dx<=$signed({6'b0,x})-18'sd960-$signed({{2{translate_x[15]}},translate_x});
   dy<=$signed({6'b0,y})-18'sd540-$signed({{2{translate_y[15]}},translate_y});
   cos_s1<=cos_q;sin_s1<=sin_q;
   // S2: one small multiplier per product, separate from adders.
   xc<=dx*cos_s1;ys<=dy*sin_s1;
   xs<=dx*sin_s1;yc<=dy*cos_s1;
   // S3
   rx<=$signed({xc[35],xc})+$signed({ys[35],ys});
   ry<=$signed({yc[35],yc})-$signed({xs[35],xs});
   // S4: divide by 4 is arithmetic wiring, no multiplier.
   uq<=($signed({rx[36],rx})>>>2)+38'sd5242880;
   vq<=($signed({ry[36],ry})>>>2)+38'sd2949120;
   // S5: bounds before truncation, floor sampling.
   inside5<=uq>=0 && uq<38'sd10485760 && vq>=0 && vq<38'sd5898240;
   if(uq>=0 && uq<38'sd10485760 && vq>=0 && vq<38'sd5898240)begin
    ui<=uq[23:16];vi<=vq[23:16];wx5<=uq[15:0];wy5<=vq[15:0];
   end else begin ui<=0;vi<=0;wx5<=0;wy5<=0;end
   // S6 / S7: 160*y+x, registered adder tree.
   row128_plus_x<=({6'd0,vi}<<7)+{6'd0,ui};row32<=({6'd0,vi}<<5);
   inside6<=inside5;u6<=ui;v6<=vi;wx6<=wx5;wy6<=wy5;
   u_out<=u6;v_out<=v6;wx_out<=wx6;wy_out<=wy6;
   addr_out<=row128_plus_x+row32;
   inside_out<=inside6;valid_out<=vp[5];
  end
 end
endmodule
