`timescale 1ns/1ps
// Two registered stages from camera RGB565/VS/DE to DDR input.
// Gamma definition: output = max * (input/max)^gamma.
// Mode 0: gamma 0.65 (brighten); 1: exact identity; 2: gamma 1.50 (darken).
// Statistics are taken BEFORE gamma, avoiding an enhancement feedback loop.
module adaptive_gamma_rgb565 #(
 parameter integer H_ACTIVE=1920,
 parameter integer V_ACTIVE=1080,
 parameter BOOT_AUTO_ENABLE=1'b1
)(
 input wire clk,rst_n,
 input wire auto_enable_async,
 input wire vs_in,de_in,
 input wire [15:0] pixel_in,
 output reg vs_out,de_out,
 output reg [15:0] pixel_out,
 output wire auto_active_out,
 output wire [1:0] gamma_mode_out,
 output reg frame_stats_valid
);
 localparam [31:0] FRAME_PIXELS=H_ACTIVE*V_ACTIVE;
 localparam [31:0] DARK_ENTER=H_ACTIVE*V_ACTIVE*64;
 localparam [31:0] DARK_LEAVE=H_ACTIVE*V_ACTIVE*88;
 localparam [31:0] BRIGHT_ENTER=H_ACTIVE*V_ACTIVE*192;
 localparam [31:0] BRIGHT_LEAVE=H_ACTIVE*V_ACTIVE*168;
 // Async assert, locally synchronous reset release.
 (* async_reg="true" *) reg [1:0] reset_pipe;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)reset_pipe<=0;
  else reset_pipe<={reset_pipe[0],1'b1};
 end
 wire local_rst_n=reset_pipe[1];
 (* async_reg="true" *) reg [1:0] enable_sync;
 reg auto_active;
 reg vs_d,frame_started,frame_bad;
 reg [31:0] pixel_count,luma_sum;
 reg [1:0] adaptive_mode,candidate,pending_mode;
 reg pending_confirm;
 wire [7:0] r8={pixel_in[15:11],pixel_in[15:13]};
 wire [7:0] g8={pixel_in[10:5],pixel_in[10:9]};
 wire [7:0] b8={pixel_in[4:0],pixel_in[4:2]};
 // Approximate luma in 0..255: (R + 2G + B)/4, no DSP or divider.
 wire [9:0] luma_weighted={2'b0,r8}+{1'b0,g8,1'b0}+{2'b0,b8};
 wire [7:0] luma=luma_weighted[9:2];
 wire frame_edge=vs_in && !vs_d;
 assign auto_active_out=auto_active;
 assign gamma_mode_out=auto_active?adaptive_mode:2'd1;
 // Hysteresis thresholds, evaluated on the just-completed raw-input frame.
 always @* begin
  candidate=adaptive_mode;
  case(adaptive_mode)
   0:if(luma_sum>=DARK_LEAVE)candidate=1;
   2:if(luma_sum<=BRIGHT_LEAVE)candidate=1;
   default:begin
    if(luma_sum<=DARK_ENTER)candidate=0;
    else if(luma_sum>=BRIGHT_ENTER)candidate=2;
    else candidate=1;
   end
  endcase
 end
 always @(posedge clk or negedge local_rst_n)begin
  if(!local_rst_n)begin
   enable_sync<={2{BOOT_AUTO_ENABLE}};auto_active<=BOOT_AUTO_ENABLE;
   vs_d<=0;frame_started<=0;frame_bad<=0;pixel_count<=0;luma_sum<=0;
   adaptive_mode<=1;pending_mode<=1;pending_confirm<=0;frame_stats_valid<=0;
  end else begin
   enable_sync<={enable_sync[0],auto_enable_async};vs_d<=vs_in;
   frame_stats_valid<=0;
   if(frame_edge)begin
    // Commit ONLY at VS, never during an active frame.
    auto_active<=enable_sync[1];frame_started<=1;
    pixel_count<=0;luma_sum<=0;frame_bad<=0;
    if(frame_started && !frame_bad && pixel_count==FRAME_PIXELS)begin
     frame_stats_valid<=1;
     if(candidate==adaptive_mode)pending_confirm<=0;
     else if(pending_confirm && pending_mode==candidate)begin
      adaptive_mode<=candidate;pending_confirm<=0;
     end else begin pending_mode<=candidate;pending_confirm<=1;end
    end else pending_confirm<=0;
   end else if(frame_started && de_in && !vs_in)begin
    if(pixel_count<FRAME_PIXELS)begin
     pixel_count<=pixel_count+1'b1;luma_sum<=luma_sum+{24'd0,luma};
    end else frame_bad<=1;
   end
  end
 end
 function [4:0] gamma5;
  input [4:0] value; input [1:0] mode;
  begin
   gamma5=value;
   case(mode)
    2'd0:case(value)
     5'd0:gamma5=5'd0;
     5'd1:gamma5=5'd3;
     5'd2:gamma5=5'd5;
     5'd3:gamma5=5'd7;
     5'd4:gamma5=5'd8;
     5'd5:gamma5=5'd9;
     5'd6:gamma5=5'd11;
     5'd7:gamma5=5'd12;
     5'd8:gamma5=5'd13;
     5'd9:gamma5=5'd14;
     5'd10:gamma5=5'd15;
     5'd11:gamma5=5'd16;
     5'd12:gamma5=5'd17;
     5'd13:gamma5=5'd18;
     5'd14:gamma5=5'd18;
     5'd15:gamma5=5'd19;
     5'd16:gamma5=5'd20;
     5'd17:gamma5=5'd21;
     5'd18:gamma5=5'd22;
     5'd19:gamma5=5'd23;
     5'd20:gamma5=5'd23;
     5'd21:gamma5=5'd24;
     5'd22:gamma5=5'd25;
     5'd23:gamma5=5'd26;
     5'd24:gamma5=5'd26;
     5'd25:gamma5=5'd27;
     5'd26:gamma5=5'd28;
     5'd27:gamma5=5'd28;
     5'd28:gamma5=5'd29;
     5'd29:gamma5=5'd30;
     5'd30:gamma5=5'd30;
     5'd31:gamma5=5'd31;
     default:gamma5=value;
    endcase
    2'd2:case(value)
     5'd0:gamma5=5'd0;
     5'd1:gamma5=5'd0;
     5'd2:gamma5=5'd1;
     5'd3:gamma5=5'd1;
     5'd4:gamma5=5'd1;
     5'd5:gamma5=5'd2;
     5'd6:gamma5=5'd3;
     5'd7:gamma5=5'd3;
     5'd8:gamma5=5'd4;
     5'd9:gamma5=5'd5;
     5'd10:gamma5=5'd6;
     5'd11:gamma5=5'd7;
     5'd12:gamma5=5'd7;
     5'd13:gamma5=5'd8;
     5'd14:gamma5=5'd9;
     5'd15:gamma5=5'd10;
     5'd16:gamma5=5'd11;
     5'd17:gamma5=5'd13;
     5'd18:gamma5=5'd14;
     5'd19:gamma5=5'd15;
     5'd20:gamma5=5'd16;
     5'd21:gamma5=5'd17;
     5'd22:gamma5=5'd19;
     5'd23:gamma5=5'd20;
     5'd24:gamma5=5'd21;
     5'd25:gamma5=5'd22;
     5'd26:gamma5=5'd24;
     5'd27:gamma5=5'd25;
     5'd28:gamma5=5'd27;
     5'd29:gamma5=5'd28;
     5'd30:gamma5=5'd30;
     5'd31:gamma5=5'd31;
     default:gamma5=value;
    endcase
    default:gamma5=value;
   endcase
  end
 endfunction
 function [5:0] gamma6;
  input [5:0] value; input [1:0] mode;
  begin
   gamma6=value;
   case(mode)
    2'd0:case(value)
     6'd0:gamma6=6'd0;
     6'd1:gamma6=6'd4;
     6'd2:gamma6=6'd7;
     6'd3:gamma6=6'd9;
     6'd4:gamma6=6'd10;
     6'd5:gamma6=6'd12;
     6'd6:gamma6=6'd14;
     6'd7:gamma6=6'd15;
     6'd8:gamma6=6'd16;
     6'd9:gamma6=6'd18;
     6'd10:gamma6=6'd19;
     6'd11:gamma6=6'd20;
     6'd12:gamma6=6'd21;
     6'd13:gamma6=6'd23;
     6'd14:gamma6=6'd24;
     6'd15:gamma6=6'd25;
     6'd16:gamma6=6'd26;
     6'd17:gamma6=6'd27;
     6'd18:gamma6=6'd28;
     6'd19:gamma6=6'd29;
     6'd20:gamma6=6'd30;
     6'd21:gamma6=6'd31;
     6'd22:gamma6=6'd32;
     6'd23:gamma6=6'd33;
     6'd24:gamma6=6'd34;
     6'd25:gamma6=6'd35;
     6'd26:gamma6=6'd35;
     6'd27:gamma6=6'd36;
     6'd28:gamma6=6'd37;
     6'd29:gamma6=6'd38;
     6'd30:gamma6=6'd39;
     6'd31:gamma6=6'd40;
     6'd32:gamma6=6'd41;
     6'd33:gamma6=6'd41;
     6'd34:gamma6=6'd42;
     6'd35:gamma6=6'd43;
     6'd36:gamma6=6'd44;
     6'd37:gamma6=6'd45;
     6'd38:gamma6=6'd45;
     6'd39:gamma6=6'd46;
     6'd40:gamma6=6'd47;
     6'd41:gamma6=6'd48;
     6'd42:gamma6=6'd48;
     6'd43:gamma6=6'd49;
     6'd44:gamma6=6'd50;
     6'd45:gamma6=6'd51;
     6'd46:gamma6=6'd51;
     6'd47:gamma6=6'd52;
     6'd48:gamma6=6'd53;
     6'd49:gamma6=6'd54;
     6'd50:gamma6=6'd54;
     6'd51:gamma6=6'd55;
     6'd52:gamma6=6'd56;
     6'd53:gamma6=6'd56;
     6'd54:gamma6=6'd57;
     6'd55:gamma6=6'd58;
     6'd56:gamma6=6'd58;
     6'd57:gamma6=6'd59;
     6'd58:gamma6=6'd60;
     6'd59:gamma6=6'd60;
     6'd60:gamma6=6'd61;
     6'd61:gamma6=6'd62;
     6'd62:gamma6=6'd62;
     6'd63:gamma6=6'd63;
     default:gamma6=value;
    endcase
    2'd2:case(value)
     6'd0:gamma6=6'd0;
     6'd1:gamma6=6'd0;
     6'd2:gamma6=6'd0;
     6'd3:gamma6=6'd1;
     6'd4:gamma6=6'd1;
     6'd5:gamma6=6'd1;
     6'd6:gamma6=6'd2;
     6'd7:gamma6=6'd2;
     6'd8:gamma6=6'd3;
     6'd9:gamma6=6'd3;
     6'd10:gamma6=6'd4;
     6'd11:gamma6=6'd5;
     6'd12:gamma6=6'd5;
     6'd13:gamma6=6'd6;
     6'd14:gamma6=6'd7;
     6'd15:gamma6=6'd7;
     6'd16:gamma6=6'd8;
     6'd17:gamma6=6'd9;
     6'd18:gamma6=6'd10;
     6'd19:gamma6=6'd10;
     6'd20:gamma6=6'd11;
     6'd21:gamma6=6'd12;
     6'd22:gamma6=6'd13;
     6'd23:gamma6=6'd14;
     6'd24:gamma6=6'd15;
     6'd25:gamma6=6'd16;
     6'd26:gamma6=6'd17;
     6'd27:gamma6=6'd18;
     6'd28:gamma6=6'd19;
     6'd29:gamma6=6'd20;
     6'd30:gamma6=6'd21;
     6'd31:gamma6=6'd22;
     6'd32:gamma6=6'd23;
     6'd33:gamma6=6'd24;
     6'd34:gamma6=6'd25;
     6'd35:gamma6=6'd26;
     6'd36:gamma6=6'd27;
     6'd37:gamma6=6'd28;
     6'd38:gamma6=6'd30;
     6'd39:gamma6=6'd31;
     6'd40:gamma6=6'd32;
     6'd41:gamma6=6'd33;
     6'd42:gamma6=6'd34;
     6'd43:gamma6=6'd36;
     6'd44:gamma6=6'd37;
     6'd45:gamma6=6'd38;
     6'd46:gamma6=6'd39;
     6'd47:gamma6=6'd41;
     6'd48:gamma6=6'd42;
     6'd49:gamma6=6'd43;
     6'd50:gamma6=6'd45;
     6'd51:gamma6=6'd46;
     6'd52:gamma6=6'd47;
     6'd53:gamma6=6'd49;
     6'd54:gamma6=6'd50;
     6'd55:gamma6=6'd51;
     6'd56:gamma6=6'd53;
     6'd57:gamma6=6'd54;
     6'd58:gamma6=6'd56;
     6'd59:gamma6=6'd57;
     6'd60:gamma6=6'd59;
     6'd61:gamma6=6'd60;
     6'd62:gamma6=6'd62;
     6'd63:gamma6=6'd63;
     default:gamma6=value;
    endcase
    default:gamma6=value;
   endcase
  end
 endfunction
 reg vs_s1,de_s1;
 reg [15:0] pixel_s1;
 reg [1:0] mode_s1;
 always @(posedge clk or negedge local_rst_n)begin
  if(!local_rst_n)begin
   vs_s1<=0;de_s1<=0;pixel_s1<=0;mode_s1<=1;
   vs_out<=0;de_out<=0;pixel_out<=0;
  end else begin
   // Carry the selected mode with its pixel, including pixels draining at VS.
   vs_s1<=vs_in;de_s1<=de_in;pixel_s1<=pixel_in;
   mode_s1<=gamma_mode_out;
   vs_out<=vs_s1;de_out<=de_s1;
   pixel_out<={gamma5(pixel_s1[15:11],mode_s1),
               gamma6(pixel_s1[10:5],mode_s1),gamma5(pixel_s1[4:0],mode_s1)};
  end
 end
endmodule
