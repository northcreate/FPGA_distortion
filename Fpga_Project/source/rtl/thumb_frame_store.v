`timescale 1ns/1ps
// All ports use pix_clk. Capture 12x12 area averages of the processed stream.
// Two complete frames; publish only at capture VS rising. No CDC.
// Six-cycle read: split addresses, RAM, chunk mux, frame mux, neighbor ordering.
// Four parity banks per frame permit four neighbor reads without frame replication.
module thumb_frame_store(
 input wire clk,rst_n,
 input wire capture_vs,capture_de,capture_valid,capture_enable,
 input wire [15:0] capture_pixel,
 input wire rd_valid,rd_inside,
 input wire [7:0] rd_u,rd_v,
 output reg pixel_valid,
 output reg [15:0] p00,p01,p10,p11,
 output reg frame_ready
);
 wire write_commit;reg write_last;
 reg vs_d,cap_started,read_bank,write_bank,pending,bad_frame;
 reg [10:0] cap_x,cap_y;
 reg [3:0] mod_x,mod_y;
 reg [13:0] wr_count;
 wire frame_start=capture_vs && !vs_d;
 reg [7:0] block_x;
 wire average_valid;wire [15:0] average_pixel;
 wire average_flush=!capture_enable || frame_start || bad_frame;
 wire average_accept=cap_started && capture_de && capture_valid && cap_y<1080;
 thumb_box12 average(
  .clk(clk),.rst_n(rst_n),.flush(average_flush),.accept(average_accept),
  .x_first(mod_x==0),.x_last(mod_x==11),.row_zero(mod_y==0),.y_last(mod_y==11),
  .block_x(block_x),.pixel_in(capture_pixel),
  .valid_out(average_valid),.pixel_out(average_pixel));
 wire wr_fire=capture_enable && average_valid && cap_started && !pending &&
              !bad_frame && !frame_start && wr_count<14'd14400;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)block_x<=0;
  else if(!capture_enable || frame_start)block_x<=0;
  else if(cap_started && capture_de)begin
   if(cap_x==1919)block_x<=0;
   else if(mod_x==11)block_x<=block_x+1'b1;
  end
 end
 // RAM contents are never reset. frame_ready hides them until a complete frame.
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   vs_d<=0;cap_started<=0;read_bank<=0;write_bank<=1;pending<=0;
   bad_frame<=0;cap_x<=0;cap_y<=0;mod_x<=0;mod_y<=0;wr_count<=0;frame_ready<=0;
  end else begin
   vs_d<=capture_vs;
   if(!capture_enable)begin
    cap_started<=0;pending<=0;frame_ready<=0;bad_frame<=0;
    cap_x<=0;cap_y<=0;mod_x<=0;mod_y<=0;wr_count<=0;
   end else if(frame_start)begin
    cap_started<=1;cap_x<=0;cap_y<=0;mod_x<=0;mod_y<=0;wr_count<=0;
    pending<=0;bad_frame<=0;
    if(pending && !bad_frame && cap_y==1080 && cap_x==0)begin
     read_bank<=write_bank;write_bank<=read_bank;frame_ready<=1;
    end
   end else begin
    if(cap_started && capture_de)begin
     if(cap_x==1919)begin
      cap_x<=0;mod_x<=0;cap_y<=cap_y+1'b1;
      mod_y<=(mod_y==11)?4'd0:mod_y+1'b1;
     end else begin cap_x<=cap_x+1'b1;mod_x<=(mod_x==11)?4'd0:mod_x+1'b1;end
     if(!capture_valid)bad_frame<=1;
     if(cap_y>=1080)bad_frame<=1;
    end
    if(write_commit && write_last)pending<=1;
    if(wr_fire)begin
     wr_count<=wr_count+1'b1;

    end
   end
  end
 end

 // Repack into parity banks: parity={y[0],x[0]}, address=(y/2)*80+x/2.
 // Each bank has 80*45=3600 pixels, implemented with four 1024x16 RAMs.
 reg [7:0] write_x;
 reg [6:0] write_y;
 reg [11:0] write_row_base,write_addr;
 reg [1:0] write_parity;
 reg write_enable,write_frame;
 reg [15:0] write_data;
 assign write_commit=write_enable && capture_enable && !frame_start && !bad_frame;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   write_x<=0;write_y<=0;write_row_base<=0;write_addr<=0;
   write_parity<=0;write_enable<=0;write_frame<=0;write_last<=0;write_data<=0;
  end else if(!capture_enable || frame_start)begin
   write_x<=0;write_y<=0;write_row_base<=0;write_enable<=0;write_last<=0;
  end else begin
   write_enable<=wr_fire;
   if(wr_fire)begin
    write_addr<=write_row_base+{5'd0,write_x[7:1]};
    write_parity<={write_y[0],write_x[0]};write_frame<=write_bank;
    write_data<=average_pixel;write_last<=wr_count==14399;
    if(write_x==159)begin
     write_x<=0;write_y<=write_y+1'b1;
     if(write_y[0])write_row_base<=write_row_base+12'd80;
    end else write_x<=write_x+1'b1;
   end
  end
 end
 // Stage A: select the even/odd members of each clamped neighborhood.
 // Out-of-frame input is forced to address zero before reaching memory.
 wire coord_ok=rd_u<160 && rd_v<90;
 wire [7:0] su=coord_ok?rd_u:8'd0;
 wire [7:0] sv=coord_ok?rd_v:8'd0;
 wire [6:0] ye=sv[7:1]+((sv[0] && sv<89)?7'd1:7'd0);
 wire [6:0] yo=sv[7:1];
 reg [6:0] xe1,xo1;
 reg [11:0] re1,ro1;
 reg [11:0] address2[0:3];
 reg [4:0] valids,insides;
 reg [3:0] select_pipe[0:4];
 reg [4:0] frame_pipe;
 reg [1:0] chunk3[0:3];
 wire [15:0] raw0[0:3][0:3],raw1[0:3][0:3];
 reg [15:0] chunk0[0:3],chunk1[0:3],selected5[0:3];
 genvar p,k;
 generate for(p=0;p<4;p=p+1)begin:G_PARITY
  for(k=0;k<4;k=k+1)begin:G_CHUNK
   line_cache_ram16 r0(clk,write_commit && !write_frame && write_parity==p && write_addr[11:10]==k,
    write_addr[9:0],write_data,address2[p][9:0],raw0[p][k]);
   line_cache_ram16 r1(clk,write_commit && write_frame && write_parity==p && write_addr[11:10]==k,
    write_addr[9:0],write_data,address2[p][9:0],raw1[p][k]);
  end
 end endgenerate
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   xe1<=0;xo1<=0;re1<=0;ro1<=0;valids<=0;insides<=0;frame_pipe<=0;
   pixel_valid<=0;p00<=0;p01<=0;p10<=0;p11<=0;
   for(i=0;i<4;i=i+1)begin
    address2[i]<=0;chunk3[i]<=0;chunk0[i]<=0;chunk1[i]<=0;selected5[i]<=0;
   end
   for(i=0;i<5;i=i+1)select_pipe[i]<=0;
  end else begin
   valids<={valids[3:0],rd_valid};
   insides<={insides[3:0],rd_inside && coord_ok && frame_ready};
   frame_pipe<={frame_pipe[3:0],read_bank};
   // Selection order: {v1 parity,v0 parity,u1 parity,u0 parity}.
   select_pipe[0]<={(sv==89?sv[0]:~sv[0]),sv[0],(su==159?su[0]:~su[0]),su[0]};
   for(i=1;i<5;i=i+1)select_pipe[i]<=select_pipe[i-1];
   xe1<=su[7:1]+((su[0] && su<159)?7'd1:7'd0);xo1<=su[7:1];
   re1<=({5'd0,ye}<<6)+({5'd0,ye}<<4);
   ro1<=({5'd0,yo}<<6)+({5'd0,yo}<<4);
   // B: four independent addresses, C: synchronous RAM read.
   address2[0]<=re1+{5'd0,xe1};address2[1]<=re1+{5'd0,xo1};
   address2[2]<=ro1+{5'd0,xe1};address2[3]<=ro1+{5'd0,xo1};
   for(i=0;i<4;i=i+1)begin
    chunk3[i]<=address2[i][11:10];
    // D: chunk mux; E: frame mux.
    chunk0[i]<=raw0[i][chunk3[i]];chunk1[i]<=raw1[i][chunk3[i]];
    selected5[i]<=frame_pipe[3]?chunk1[i]:chunk0[i];
   end
   // F: order neighbors; at right/bottom edges duplicate the edge pixel.
   pixel_valid<=valids[4];
   if(insides[4])begin
    p00<=selected5[{select_pipe[4][2],select_pipe[4][0]}];
    p01<=selected5[{select_pipe[4][2],select_pipe[4][1]}];
    p10<=selected5[{select_pipe[4][3],select_pipe[4][0]}];
    p11<=selected5[{select_pipe[4][3],select_pipe[4][1]}];
   end else begin p00<=0;p01<=0;p10<=0;p11<=0;end
  end
 end
endmodule
