`timescale 1ns/1ps
// Independent thumbnail branch; configurable padding matches the full-screen pipeline.
module thumbnail_rotation #(parameter integer TOTAL_LATENCY=31)(
 input wire clk,rst_n,
 input wire capture_vs,capture_de,capture_valid,capture_enable,
 input wire [15:0] capture_pixel,
 input wire display_de,
 input wire [11:0] display_x,display_y,
 input wire signed [15:0] translate_x,translate_y,
 input wire signed [17:0] cos_q,sin_q,
 output wire pixel_valid,
 output wire [15:0] pixel_out,
 output wire frame_ready
);
 wire mv,mi,sv,iv;wire [13:0] unused_addr;
 wire [7:0] mu,mv_coord;wire [15:0] mx,my;
 wire [15:0] p00,p01,p10,p11,sp;
 thumb_rotate_map map(.clk(clk),.rst_n(rst_n),.valid_in(display_de),
  .x(display_x),.y(display_y),.translate_x(translate_x),.translate_y(translate_y),
  .cos_q(cos_q),.sin_q(sin_q),.valid_out(mv),.inside_out(mi),.addr_out(unused_addr),
  .u_out(mu),.v_out(mv_coord),.wx_out(mx),.wy_out(my));
 thumb_frame_store store(.clk(clk),.rst_n(rst_n),.capture_vs(capture_vs),.capture_de(capture_de),
  .capture_valid(capture_valid),.capture_enable(capture_enable),.capture_pixel(capture_pixel),
  .rd_valid(mv),.rd_inside(mi),.rd_u(mu),.rd_v(mv_coord),.pixel_valid(sv),
  .p00(p00),.p01(p01),.p10(p10),.p11(p11),.frame_ready(frame_ready));
 reg [15:0] wx_delay[0:5],wy_delay[0:5];integer w;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)for(w=0;w<6;w=w+1)begin wx_delay[w]<=0;wy_delay[w]<=0;end
  else begin
   wx_delay[0]<=mx;wy_delay[0]<=my;
   for(w=1;w<6;w=w+1)begin wx_delay[w]<=wx_delay[w-1];wy_delay[w]<=wy_delay[w-1];end
  end
 end
 bilinear_interp_rgb565 interp(.clk(clk),.rst_n(rst_n),.valid_in(sv),
  .p00(p00),.p01(p01),.p10(p10),.p11(p11),.wx(wx_delay[5]),.wy(wy_delay[5]),
  .valid_out(iv),.pixel_out(sp));
 // 7 map + 6 neighborhood read + 8 interpolation + padding = TOTAL_LATENCY.
 // TOTAL_LATENCY must be >=23; current top explicitly supplies 32.
 localparam integer PAD=TOTAL_LATENCY-21;
 reg [PAD-1:0] vp;
 reg [15:0] pp[0:PAD-1];integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin vp<=0;for(i=0;i<PAD;i=i+1)pp[i]<=0;end
  else begin
   vp<={vp[PAD-2:0],iv};pp[0]<=sp;
   for(i=1;i<PAD;i=i+1)pp[i]<=pp[i-1];
  end
 end
 assign pixel_valid=vp[PAD-1];assign pixel_out=pp[PAD-1];
endmodule
