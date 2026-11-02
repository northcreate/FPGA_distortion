`timescale 1ns/1ps
// Legacy bytes 01..05 plus packet A5 5A 10 flags angleLE txLE tyLE XOR 0D.
// Type 11: A5 5A 11 00 K1[7:0] K1[15:8] K1[23:16] K1[31:24] 00 00 XOR 0D.
// Type 12: A5 5A 12 00 K1_LE32 K2_LE32 P1_LE32 P2_LE32 00 00 XOR 0D (24 bytes).
// Packet payload never leaks into legacy command handling. All state commits at VS.
module video_cmd_ctrl #(
 parameter BOOT_CORRECTION=1'b1,BOOT_GAMMA=1'b1,
 parameter RESET_CORRECTION_ON_04=1'b0,
 parameter signed[31:0] MODEL_K1_Q=32'sd655,
 parameter signed[31:0] MODEL_K2_Q=0,MODEL_P1_Q=0,MODEL_P2_Q=0,
 parameter integer TIMEOUT_CYCLES=1485000
)(
 input wire clk,rst_n,vsync,
 input wire[7:0] rx_data,input wire rx_valid,
 output reg correction_on,thumbnail_on,gamma_on,
 output reg osd_enable,
 output reg overlay_enable,
 output reg [1:0] overlay_mode, // 0:center cross 1:50px box 2:100px grid
 output reg signed[15:0] tx,ty,
 output reg[8:0] angle_deg,
 output reg signed[17:0] cos_q,sin_q,
 output wire signed[31:0] k1_active_q,
 output reg command_seen,command_error,
 output wire signed[31:0] k2_active_q,p1_active_q,p2_active_q,
 output wire signed[31:0] model_k1_q,model_k2_q,model_p1_q,model_p2_q
);
 reg pc,pr,pg,vs_d;
 reg pending_osd_enable;
 reg pending_overlay_enable;
 reg [1:0] pending_overlay_mode;
 reg signed [31:0] pending_k1,committed_k1;
 reg signed [31:0] pending_k2,pending_p1,pending_p2,committed_k2,committed_p1,committed_p2;
 reg signed [31:0] group_k1,group_k2,group_p1,group_p2;
 reg signed [31:0] candidate_k1,candidate_k2,candidate_p1,candidate_p2;
 reg[15:0] reserved;
 reg guard_start;wire guard_done,guard_ok;
 distortion_coeff_guard guard(.clk(clk),.rst_n(rst_n),.valid_in(guard_start),
  .k1(candidate_k1),.k2(candidate_k2),.p1(candidate_p1),.p2(candidate_p2),
  .valid_out(guard_done),.accepted(guard_ok));
 reg signed[15:0] ptx,pty;
 reg[8:0] pa;
 reg[4:0] state;
 reg[31:0] timer;
 reg[7:0] flags,check_xor,packet_type;
 reg checksum_ok;
 reg[15:0] angle_word,tx_word,ty_word;
 wire signed [31:0] packet_k1=$signed({tx_word,angle_word});
 wire lut_valid;wire[8:0]lut_tag;wire signed[17:0]lc,ls;
 angle_lut_deg lut(clk,rst_n,pa,lut_valid,lut_tag,lc,ls);
 wire affine_packet_ok=checksum_ok && packet_type==8'h10 && (flags[7:3]==0) && angle_word<360 &&
   $signed(tx_word)>=-16'sd960 && $signed(tx_word)<=16'sd960 &&
   $signed(ty_word)>=-16'sd540 && $signed(ty_word)<=16'sd540 &&
   (flags[0] || (angle_word==0 && ty_word==0));
 wire k1_packet_ok=checksum_ok && packet_type==8'h11 && flags==0 && ty_word==0 &&
   packet_k1>=-32'sd2621 && packet_k1<=32'sd2621;
 wire packet_ok=affine_packet_ok || k1_packet_ok;
 // A command arriving on VS waits until the following VS: no partial commit.
 wire config_write=rx_valid && ((state==0 && rx_data>=1 && rx_data<=8) ||
                               (state==11 && packet_type!=8'h12 && rx_data==8'h0d && affine_packet_ok)) ||
                               (state==24 && guard_done && guard_ok);
 assign model_k1_q=committed_k1;assign model_k2_q=committed_k2;
 assign model_p1_q=committed_p1;assign model_p2_q=committed_p2;
 assign k1_active_q=correction_on?committed_k1:32'sd0;
 assign k2_active_q=correction_on?committed_k2:32'sd0;
 assign p1_active_q=correction_on?committed_p1:32'sd0;
 assign p2_active_q=correction_on?committed_p2:32'sd0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   pending_k1<=MODEL_K1_Q;committed_k1<=MODEL_K1_Q;
   pending_k2<=MODEL_K2_Q;committed_k2<=MODEL_K2_Q;
   pending_p1<=MODEL_P1_Q;committed_p1<=MODEL_P1_Q;
   pending_p2<=MODEL_P2_Q;committed_p2<=MODEL_P2_Q;
   group_k1<=0;group_k2<=0;group_p1<=0;group_p2<=0;reserved<=0;
   candidate_k1<=0;candidate_k2<=0;candidate_p1<=0;candidate_p2<=0;guard_start<=0;
   pc<=BOOT_CORRECTION;pr<=0;pg<=BOOT_GAMMA;ptx<=0;pty<=0;pa<=0;vs_d<=0;
   pending_osd_enable<=1'b1;
   pending_overlay_enable<=1'b0;pending_overlay_mode<=2'd0;
   osd_enable<=1'b1;
   overlay_enable<=1'b0;overlay_mode<=2'd0;
   correction_on<=BOOT_CORRECTION;thumbnail_on<=0;gamma_on<=BOOT_GAMMA;
   tx<=0;ty<=0;angle_deg<=0;cos_q<=18'sd65536;sin_q<=0;
   state<=0;timer<=0;flags<=0;check_xor<=0;packet_type<=0;checksum_ok<=0;angle_word<=0;tx_word<=0;ty_word<=0;
   command_seen<=0;command_error<=0;
  end else begin
   vs_d<=vsync;command_seen<=0;command_error<=0;guard_start<=0;
   if(vsync && !vs_d && !config_write && lut_valid && lut_tag==pa)begin
    committed_k1<=pending_k1;committed_k2<=pending_k2;
    committed_p1<=pending_p1;committed_p2<=pending_p2;
    correction_on<=pc;thumbnail_on<=pr;gamma_on<=pg;tx<=ptx;ty<=pty;
    osd_enable<=pending_osd_enable;
    overlay_enable<=pending_overlay_enable;
    overlay_mode<=pending_overlay_mode;
    angle_deg<=pa;cos_q<=lc;sin_q<=ls;
   end
   if(state==24 && guard_done)begin
    state<=0;
    if(guard_ok)begin
     pending_k1<=candidate_k1;pending_k2<=candidate_k2;
     pending_p1<=candidate_p1;pending_p2<=candidate_p2;command_seen<=1;
    end else command_error<=1;
   end
   if(state!=0 && state!=24 && !rx_valid)begin
    if(timer>=TIMEOUT_CYCLES-1)begin state<=0;timer<=0;command_error<=1;end
    else timer<=timer+1'b1;
   end
   if(rx_valid)begin
    timer<=0;
    if(state==24)begin
     // UART byte spacing is >=12890 clocks; validator completes in <10 clocks.
     // An impossible early direct rx_valid byte is rejected, never a legacy command.
     command_error<=1;
    end else if(packet_type==8'h12 && state>=4 && state<=23)begin
     if(state<=21)begin check_xor<=check_xor^rx_data;state<=state+1'b1;end
     case(state)
      4:group_k1[7:0]<=rx_data;
      5:group_k1[15:8]<=rx_data;
      6:group_k1[23:16]<=rx_data;
      7:group_k1[31:24]<=rx_data;
      8:group_k2[7:0]<=rx_data;
      9:group_k2[15:8]<=rx_data;
      10:group_k2[23:16]<=rx_data;
      11:group_k2[31:24]<=rx_data;
      12:group_p1[7:0]<=rx_data;
      13:group_p1[15:8]<=rx_data;
      14:group_p1[23:16]<=rx_data;
      15:group_p1[31:24]<=rx_data;
      16:group_p2[7:0]<=rx_data;
      17:group_p2[15:8]<=rx_data;
      18:group_p2[23:16]<=rx_data;
      19:group_p2[31:24]<=rx_data;
      20:reserved[7:0]<=rx_data;
      21:reserved[15:8]<=rx_data;
      22:begin checksum_ok<=(rx_data==check_xor);state<=23;end
      23:begin
       if(rx_data==8'h0d && checksum_ok && flags==0 && reserved==0)begin
        candidate_k1<=group_k1;candidate_k2<=group_k2;
        candidate_p1<=group_p1;candidate_p2<=group_p2;
        guard_start<=1;state<=24;
       end else begin state<=0;command_error<=1;end
      end
      default:begin state<=0;command_error<=1;end
     endcase
    end else case(state)
     0:case(rx_data)
      8'h01:begin pc<=~pc;command_seen<=1;end
      8'h02:begin ptx<=100;command_seen<=1;end
      8'h03:begin pr<=1;pa<=15;command_seen<=1;end
      8'h04:begin pr<=0;pa<=0;ptx<=0;pty<=0;command_seen<=1;
       if(RESET_CORRECTION_ON_04)pc<=BOOT_CORRECTION;end
      8'h05:begin pg<=~pg;command_seen<=1;end
      8'h06:begin pending_overlay_enable<=~pending_overlay_enable;command_seen<=1;end
      8'h07:begin pending_overlay_mode <= (pending_overlay_mode==2'd2)?2'd0:(pending_overlay_mode+2'd1);command_seen<=1;end
      8'h08:begin pending_osd_enable<=~pending_osd_enable;command_seen<=1;end
      8'ha5:state<=1;
      default:command_error<=1;
     endcase
     1:if(rx_data==8'h5a)state<=2;
       else begin state<=(rx_data==8'ha5)?1:0;command_error<=1;end
     2:begin packet_type<=rx_data;state<=3;check_xor<=rx_data;checksum_ok<=0;end
     3:begin flags<=rx_data;check_xor<=check_xor^rx_data;state<=4;end
     4:begin angle_word[7:0]<=rx_data;check_xor<=check_xor^rx_data;state<=5;end
     5:begin angle_word[15:8]<=rx_data;check_xor<=check_xor^rx_data;state<=6;end
     6:begin tx_word[7:0]<=rx_data;check_xor<=check_xor^rx_data;state<=7;end
     7:begin tx_word[15:8]<=rx_data;check_xor<=check_xor^rx_data;state<=8;end
     8:begin ty_word[7:0]<=rx_data;check_xor<=check_xor^rx_data;state<=9;end
     9:begin ty_word[15:8]<=rx_data;check_xor<=check_xor^rx_data;state<=10;end
     10:begin checksum_ok<=(rx_data==check_xor);state<=11;end
     11:begin
      state<=0;
      if(rx_data==8'h0d && packet_ok)begin
       if(k1_packet_ok)begin
        candidate_k1<=packet_k1;candidate_k2<=pending_k2;
        candidate_p1<=pending_p1;candidate_p2<=pending_p2;
        guard_start<=1;state<=24;
       end
       else begin
        pc<=flags[1];pr<=flags[0];pg<=flags[2];pa<=angle_word[8:0];
        ptx<=tx_word;pty<=ty_word;
       end
       if(affine_packet_ok)command_seen<=1;
      end else command_error<=1;
     end
     default:begin state<=0;command_error<=1;end
    endcase
   end
  end
 end
endmodule
