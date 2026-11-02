`timescale 1ns/1ps
// Streaming RGB565 12x12 box average, one output per 144 input pixels.
// x_last/y_last mark the last column/row in a 12x12 block; block_x=0..159.
// flush discards an incomplete frame. Accumulator RAM is initialized by row_zero.
// Each output channel is round(sum/144), with exact bounded reciprocal division.
module thumb_box12(
 input wire clk,rst_n,flush,accept,
 input wire x_first,x_last,row_zero,y_last,
 input wire [7:0] block_x,
 input wire [15:0] pixel_in,
 output reg valid_out,
 output reg [15:0] pixel_out
);
 reg [8:0] hr,hb;
 reg [9:0] hg;
 wire [8:0] nr=(x_first?9'd0:hr)+{4'd0,pixel_in[15:11]};
 wire [9:0] ng=(x_first?10'd0:hg)+{4'd0,pixel_in[10:5]};
 wire [8:0] nb=(x_first?9'd0:hb)+{4'd0,pixel_in[4:0]};
 // R:13 bits, G:14 bits, B:13 bits; 160 columns. No RAM reset.
 reg [39:0] sums[0:159];
 reg [39:0] previous;
 reg [8:0] row_r,row_b;
 reg [9:0] row_g;
 reg [7:0] address1,address2;
 reg zero1,last1,v1,v2,last2;
 reg [12:0] total_r,total_b;
 reg [13:0] total_g;
 wire finish_row=accept && x_last && block_x<160;
 always @(posedge clk)begin
  if(finish_row && !flush)previous<=sums[block_x];
  if(v2 && !flush)sums[address2]<={total_r,total_g,total_b};
 end
 reg v3,v4;
 reg [13:0] rounded_r,rounded_g,rounded_b;
 reg [27:0] product_r,product_g,product_b;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   hr<=0;hg<=0;hb<=0;row_r<=0;row_g<=0;row_b<=0;
   address1<=0;address2<=0;zero1<=0;last1<=0;last2<=0;
   total_r<=0;total_g<=0;total_b<=0;
   v1<=0;v2<=0;v3<=0;v4<=0;valid_out<=0;pixel_out<=0;
   rounded_r<=0;rounded_g<=0;rounded_b<=0;
   product_r<=0;product_g<=0;product_b<=0;
  end else if(flush)begin
   hr<=0;hg<=0;hb<=0;
   v1<=0;v2<=0;v3<=0;v4<=0;valid_out<=0;pixel_out<=0;
  end else begin
   v1<=finish_row;v2<=v1;v3<=v2&&last2;v4<=v3;valid_out<=v4;
   if(accept)begin hr<=nr;hg<=ng;hb<=nb;end
   if(finish_row)begin
    row_r<=nr;row_g<=ng;row_b<=nb;
    address1<=block_x;zero1<=row_zero;last1<=y_last;
   end
   if(v1)begin
    total_r<=(zero1?13'd0:previous[39:27])+{4'd0,row_r};
    total_g<=(zero1?14'd0:previous[26:13])+{4'd0,row_g};
    total_b<=(zero1?13'd0:previous[12:0])+{4'd0,row_b};
    address2<=address1;last2<=last1;
   end
   if(v2&&last2)begin
    rounded_r<={1'b0,total_r}+14'd72;
    rounded_g<=total_g+14'd72;
    rounded_b<={1'b0,total_b}+14'd72;
   end
   // ceil(2^20/144)=7282. Exact floor(N/144) for all N=0..9144.
   // These are registered constant products, not a combinational multiply chain.
   if(v3)begin
    product_r<=rounded_r*14'd7282;
    product_g<=rounded_g*14'd7282;
    product_b<=rounded_b*14'd7282;
   end
   if(v4)pixel_out<={product_r[24:20],product_g[25:20],product_b[24:20]};
  end
 end
endmodule
