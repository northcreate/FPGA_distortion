`timescale 1ns/1ps
// 704x128 panel at (16,16). ASCII labels avoid a large Chinese glyph ROM.
// Dynamic four-corner model metrics. No measured residual/error is claimed.
// Parameterized OSD pipeline (32 cycles in current system); video timing unchanged.
module video_status_osd #(parameter integer TOTAL_LATENCY=32)(
 input wire clk,rst_n,vsync,de_in,
 input wire enable_in, // 08 toggles OSD ink; pipelined from Stage 1
 input wire[11:0] x_in,y_in,
 input wire correction_on,
 input wire [31:0] camera_period_ticks, // Compatibility with optional camera diagnostic top
 input wire camera_period_valid, // Not displayed in this metrics panel
 input wire signed[31:0] k1_q,k2_q,p1_q,p2_q,
 output wire cover_out,output wire[15:0] pixel_out
);
 localparam integer PAD=TOTAL_LATENCY-5;
 // Controller commits on VS. Snapshot one clock later so all displayed fields
 // represent that same committed frame, including when a command meets VS.
 reg vs_d,snapshot;
 reg active;
 reg [2:0] error_profile;
 reg[31:0]k1,k2,p1,p2;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   vs_d<=0;snapshot<=0;active<=0;error_profile<=4;
   k1<=0;k2<=0;p1<=0;p2<=0;
  end else begin
   vs_d<=vsync;snapshot<=vsync&&!vs_d;
   if(snapshot)begin
    active<=correction_on;k1<=k1_q;k2<=k2_q;p1<=p1_q;p2<=p2_q;
    if(!correction_on)error_profile<=4;
    else if(k1_q==0&&k2_q==0&&p1_q==0&&p2_q==0)error_profile<=3;
    else if(k1_q==655&&k2_q==16&&p1_q==8&&p2_q==8)error_profile<=1;
    else if(k1_q==655&&k2_q==0&&p1_q==0&&p2_q==0)error_profile<=2;
    else error_profile<=0;
   end
  end
 end
 wire[11:0]rx=x_in-12'd16,ry=y_in-12'd16;
 reg inside1,inside2,inside3,inside4,inside5;
 reg[3:0]line1;reg[5:0]col1;reg[2:0]gx1,gy1,gx2,gy2,gx3,gy3,gx4;
 reg header1,header2,header3,header4,on1,on2,on3,on4;
 reg[7:0]character2;
 reg[34:0]glyph3;
 reg[4:0]bits4;
 reg[15:0]pixel5;
 function[7:0]hexchar;
 input[3:0]v;
 begin hexchar=(v<10)?(8'd48+{4'd0,v}):(8'd65+{4'd0,v}-8'd10);end
 endfunction
 function[7:0]fixed_char;
 input[3:0]line;input[5:0]col;
 begin
 fixed_char=8'd32;
 case({line,col})
  10'd0:fixed_char=8'd67;
  10'd1:fixed_char=8'd79;
  10'd2:fixed_char=8'd82;
  10'd3:fixed_char=8'd82;
  10'd4:fixed_char=8'd69;
  10'd5:fixed_char=8'd67;
  10'd6:fixed_char=8'd84;
  10'd7:fixed_char=8'd73;
  10'd8:fixed_char=8'd79;
  10'd9:fixed_char=8'd78;
  10'd10:fixed_char=8'd58;
  10'd64:fixed_char=8'd75;
  10'd65:fixed_char=8'd49;
  10'd66:fixed_char=8'd32;
  10'd67:fixed_char=8'd81;
  10'd68:fixed_char=8'd49;
  10'd69:fixed_char=8'd54;
  10'd70:fixed_char=8'd32;
  10'd71:fixed_char=8'd72;
  10'd72:fixed_char=8'd69;
  10'd73:fixed_char=8'd88;
  10'd74:fixed_char=8'd58;
  10'd128:fixed_char=8'd75;
  10'd129:fixed_char=8'd50;
  10'd130:fixed_char=8'd32;
  10'd131:fixed_char=8'd81;
  10'd132:fixed_char=8'd49;
  10'd133:fixed_char=8'd54;
  10'd134:fixed_char=8'd32;
  10'd135:fixed_char=8'd72;
  10'd136:fixed_char=8'd69;
  10'd137:fixed_char=8'd88;
  10'd138:fixed_char=8'd58;
  10'd192:fixed_char=8'd80;
  10'd193:fixed_char=8'd49;
  10'd194:fixed_char=8'd32;
  10'd195:fixed_char=8'd81;
  10'd196:fixed_char=8'd49;
  10'd197:fixed_char=8'd54;
  10'd198:fixed_char=8'd32;
  10'd199:fixed_char=8'd72;
  10'd200:fixed_char=8'd69;
  10'd201:fixed_char=8'd88;
  10'd202:fixed_char=8'd58;
  10'd256:fixed_char=8'd80;
  10'd257:fixed_char=8'd50;
  10'd258:fixed_char=8'd32;
  10'd259:fixed_char=8'd81;
  10'd260:fixed_char=8'd49;
  10'd261:fixed_char=8'd54;
  10'd262:fixed_char=8'd32;
  10'd263:fixed_char=8'd72;
  10'd264:fixed_char=8'd69;
  10'd265:fixed_char=8'd88;
  10'd266:fixed_char=8'd58;
  10'd320:fixed_char=8'd77;
  10'd321:fixed_char=8'd79;
  10'd322:fixed_char=8'd68;
  10'd323:fixed_char=8'd69;
  10'd324:fixed_char=8'd76;
  10'd325:fixed_char=8'd32;
  10'd326:fixed_char=8'd67;
  10'd327:fixed_char=8'd79;
  10'd328:fixed_char=8'd82;
  10'd329:fixed_char=8'd78;
  10'd330:fixed_char=8'd69;
  10'd331:fixed_char=8'd82;
  10'd332:fixed_char=8'd32;
  10'd333:fixed_char=8'd83;
  10'd334:fixed_char=8'd72;
  10'd335:fixed_char=8'd73;
  10'd336:fixed_char=8'd70;
  10'd337:fixed_char=8'd84;
  10'd338:fixed_char=8'd58;
  10'd384:fixed_char=8'd65;
  10'd385:fixed_char=8'd67;
  10'd386:fixed_char=8'd84;
  10'd387:fixed_char=8'd73;
  10'd388:fixed_char=8'd86;
  10'd389:fixed_char=8'd69;
  10'd390:fixed_char=8'd32;
  10'd391:fixed_char=8'd77;
  10'd392:fixed_char=8'd65;
  10'd393:fixed_char=8'd80;
  10'd394:fixed_char=8'd32;
  10'd395:fixed_char=8'd83;
  10'd396:fixed_char=8'd72;
  10'd397:fixed_char=8'd73;
  10'd398:fixed_char=8'd70;
  10'd399:fixed_char=8'd84;
  10'd400:fixed_char=8'd58;
  10'd448:fixed_char=8'd83;
  10'd449:fixed_char=8'd73;
  10'd450:fixed_char=8'd77;
  10'd451:fixed_char=8'd32;
  10'd452:fixed_char=8'd77;
  10'd453:fixed_char=8'd65;
  10'd454:fixed_char=8'd88;
  10'd455:fixed_char=8'd32;
  10'd456:fixed_char=8'd77;
  10'd457:fixed_char=8'd65;
  10'd458:fixed_char=8'd80;
  10'd459:fixed_char=8'd32;
  10'd460:fixed_char=8'd69;
  10'd461:fixed_char=8'd82;
  10'd462:fixed_char=8'd82;
  10'd463:fixed_char=8'd79;
  10'd464:fixed_char=8'd82;
  10'd465:fixed_char=8'd58;
  default:fixed_char=8'd32;
 endcase
 end
 endfunction

 // Calibrated camera error CANNOT be inferred from loaded model coefficients.
 // The model-only, idealized display reports corner geometry, not measured error.
 // Numerals are preconverted to BCD by a once-per-frame calculator.
 wire [15:0] metric_bcd,vertical_bcd;
 wire metric_ready;
 osd_corner_shift_calc u_shift_calc (
   .clk(clk),.rst_n(rst_n),.start(snapshot),
   .k1(k1_q),.k2(k2_q),.p1(p1_q),.p2(p2_q),
   .shift_digits(metric_bcd),.vertical_digits(vertical_bcd),.ready(metric_ready)
 );
 function [7:0] digit;
  input [3:0] value;
  begin digit=8'd48+{4'b0,value}; end
 endfunction
 function [7:0] metric_char;
  input [5:0] index;
  input [15:0] dec;
  input valid;
  begin
   metric_char=8'd32;
   if (!valid) begin
     case(index)
      0:metric_char=8'd67; // C
      1:metric_char=8'd65; // A
      2:metric_char=8'd76; // L
      3:metric_char=8'd67; // C
      default:metric_char=8'd32;
     endcase
   end else begin
     case(index)
      0:metric_char=(dec[15:12]==0)?8'd32:digit(dec[15:12]);
      1:metric_char=digit(dec[11:8]);
      2:metric_char=8'd46;
      3:metric_char=digit(dec[7:4]);
      4:metric_char=digit(dec[3:0]);
      6:metric_char=8'd80;
      7:metric_char=8'd88;
      default:metric_char=8'd32;
     endcase
   end
  end
 endfunction
 // Offline RTL-versus-double validation, FULL INTEGER INPUT GRID only.
 // Profile values are conservative upper bounds for the exact tested RTL hash.
 // Not a camera residual, interpolation error, or live measurement.
 function [7:0] error_char;
 input [2:0] profile;input [5:0] index;
 begin
 case({profile,index})
 9'd0:error_char=8'd78;
 9'd1:error_char=8'd79;
 9'd2:error_char=8'd84;
 9'd3:error_char=8'd32;
 9'd4:error_char=8'd84;
 9'd5:error_char=8'd69;
 9'd6:error_char=8'd83;
 9'd7:error_char=8'd84;
 9'd8:error_char=8'd69;
 9'd9:error_char=8'd68;
 9'd64:error_char=8'd60;
 9'd65:error_char=8'd48;
 9'd66:error_char=8'd46;
 9'd67:error_char=8'd49;
 9'd68:error_char=8'd52;
 9'd69:error_char=8'd32;
 9'd70:error_char=8'd80;
 9'd71:error_char=8'd88;
 9'd72:error_char=8'd32;
 9'd73:error_char=8'd71;
 9'd74:error_char=8'd82;
 9'd75:error_char=8'd73;
 9'd76:error_char=8'd68;
 9'd128:error_char=8'd60;
 9'd129:error_char=8'd48;
 9'd130:error_char=8'd46;
 9'd131:error_char=8'd48;
 9'd132:error_char=8'd56;
 9'd133:error_char=8'd32;
 9'd134:error_char=8'd80;
 9'd135:error_char=8'd88;
 9'd136:error_char=8'd32;
 9'd137:error_char=8'd71;
 9'd138:error_char=8'd82;
 9'd139:error_char=8'd73;
 9'd140:error_char=8'd68;
 9'd192:error_char=8'd48;
 9'd193:error_char=8'd46;
 9'd194:error_char=8'd48;
 9'd195:error_char=8'd48;
 9'd196:error_char=8'd32;
 9'd197:error_char=8'd80;
 9'd198:error_char=8'd88;
 9'd199:error_char=8'd32;
 9'd200:error_char=8'd69;
 9'd201:error_char=8'd88;
 9'd202:error_char=8'd65;
 9'd203:error_char=8'd67;
 9'd204:error_char=8'd84;
 9'd256:error_char=8'd66;
 9'd257:error_char=8'd89;
 9'd258:error_char=8'd80;
 9'd259:error_char=8'd65;
 9'd260:error_char=8'd83;
 9'd261:error_char=8'd83;
 default:error_char=8'd32;
 endcase
 end
 endfunction
 function[34:0]font_glyph;
 input[7:0]ch;
 begin
 case(ch)
  8'd65:font_glyph=35'b01110100011000111111100011000110001;
  8'd66:font_glyph=35'b11110100011000111110100011000111110;
  8'd67:font_glyph=35'b01111100001000010000100001000001111;
  8'd68:font_glyph=35'b11110100011000110001100011000111110;
  8'd69:font_glyph=35'b11111100001000011110100001000011111;
  8'd70:font_glyph=35'b11111100001000011110100001000010000;
  8'd71:font_glyph=35'b01111100001000010111100011000101111;
  8'd72:font_glyph=35'b10001100011000111111100011000110001;
  8'd73:font_glyph=35'b01110001000010000100001000010001110;
  8'd74:font_glyph=35'b00111000100001000010100101001001100;
  8'd75:font_glyph=35'b10001100101010011000101001001010001;
  8'd76:font_glyph=35'b10000100001000010000100001000011111;
  8'd77:font_glyph=35'b10001110111010110101100011000110001;
  8'd78:font_glyph=35'b10001110011010110011100011000110001;
  8'd79:font_glyph=35'b01110100011000110001100011000101110;
  8'd80:font_glyph=35'b11110100011000111110100001000010000;
  8'd81:font_glyph=35'b01110100011000110001101011001001101;
  8'd82:font_glyph=35'b11110100011000111110101001001010001;
  8'd83:font_glyph=35'b01111100001000001110000010000111110;
  8'd84:font_glyph=35'b11111001000010000100001000010000100;
  8'd85:font_glyph=35'b10001100011000110001100011000101110;
  8'd86:font_glyph=35'b10001100011000110001100010101000100;
  8'd87:font_glyph=35'b10001100011000110101101011010101010;
  8'd88:font_glyph=35'b10001100010101000100010101000110001;
  8'd89:font_glyph=35'b10001100010101000100001000010000100;
  8'd90:font_glyph=35'b11111000010001000100010001000011111;
  8'd48:font_glyph=35'b01110100011001110101110011000101110;
  8'd49:font_glyph=35'b00100011000010000100001000010001110;
  8'd50:font_glyph=35'b01110100010000100010001000100011111;
  8'd51:font_glyph=35'b11110000010000101110000010000111110;
  8'd52:font_glyph=35'b00010001100101010010111110001000010;
  8'd53:font_glyph=35'b11111100001000011110000010000111110;
  8'd54:font_glyph=35'b01110100001000011110100011000101110;
  8'd55:font_glyph=35'b11111000010001000100010000100001000;
  8'd56:font_glyph=35'b01110100011000101110100011000101110;
  8'd57:font_glyph=35'b01110100011000101111000010000101110;
  8'd58:font_glyph=35'b00000001000010000000001000010000000;
  8'd46:font_glyph=35'b00000000000000000000000000010000100;
  8'd47:font_glyph=35'b00001000100001000100010000100010000;
  8'd45:font_glyph=35'b00000000000000011111000000000000000;
  8'd60:font_glyph={5'b00010,5'b00100,5'b01000,5'b10000,5'b01000,5'b00100,5'b00010};
  default:font_glyph=0;
 endcase
 end
 endfunction
 function[4:0]font_row;
 input[34:0]glyph;input[2:0]row;
 begin
 case(row)
 0:font_row=glyph[34:30];1:font_row=glyph[29:25];2:font_row=glyph[24:20];
 3:font_row=glyph[19:15];4:font_row=glyph[14:10];5:font_row=glyph[9:5];
 6:font_row=glyph[4:0];default:font_row=0;
 endcase
 end
 endfunction
 reg[31:0]word_selected,shifted_word;
 reg[7:0]character_next;
 reg[4:0]shift_bits;
 always @* begin
  word_selected=0;shifted_word=0;shift_bits=0;
  character_next=fixed_char(line1,col1);
  case(line1)
   0:case(col1)
    12:character_next=8'd79;
    13:character_next=active?8'd78:8'd70;
    14:character_next=active?8'd32:8'd70;
    default:begin end
   endcase
   1,2,3,4:begin
    case(line1)
     1:word_selected=k1;2:word_selected=k2;3:word_selected=p1;default:word_selected=p2;
    endcase
    if(col1>=12 && col1<20)begin
     shift_bits={~(col1[2:0]-3'd4),2'b00};
     // Equivalent to (19-col1)*4 for columns 12..19; low 3 bits wrap intentionally.
     shifted_word=word_selected >> shift_bits;
     character_next=hexchar(shifted_word[3:0]);
    end
   end
   5:if(col1>=20)character_next=metric_char(col1-6'd20,metric_bcd,metric_ready);
   6:if(col1>=18)character_next=metric_char(col1-6'd18,active?metric_bcd:16'h0000,metric_ready);
   7:if(col1>=20)character_next=error_char(error_profile,col1-6'd20);
   default:begin end
  endcase
 end
 integer i;
 reg[PAD-1:0]cover_delay;
 reg[15:0]pixel_delay[0:PAD-1];
 assign cover_out=cover_delay[PAD-1];assign pixel_out=pixel_delay[PAD-1];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   inside1<=0;inside2<=0;inside3<=0;inside4<=0;inside5<=0;
   col1<=0;line1<=0;gx1<=0;gy1<=0;gx2<=0;gy2<=0;gx3<=0;gy3<=0;gx4<=0;
   header1<=0;header2<=0;header3<=0;header4<=0;on1<=0;on2<=0;on3<=0;on4<=0;
   character2<=32;glyph3<=0;bits4<=0;pixel5<=0;cover_delay<=0;
   for(i=0;i<PAD;i=i+1)pixel_delay[i]<=0;
  end else begin
   // Stage 1: coordinate decomposition (all powers of two).
   inside1<=de_in && enable_in && x_in>=16 && x_in<720 && y_in>=16 && y_in<144;
   col1<=rx[9:4];line1<=ry[7:4];gx1<=rx[3:1];gy1<=ry[3:1];
   header1<=ry[7:4]==0;on1<=active;
   // Stage 2: text/parameter selection.
   inside2<=inside1;character2<=character_next;gx2<=gx1;gy2<=gy1;
   header2<=header1;on2<=on1;
   // Stage 3: registered glyph ROM.
   inside3<=inside2;glyph3<=font_glyph(character2);gx3<=gx2;gy3<=gy2;
   header3<=header2;on3<=on2;
   // Stage 4: registered font row selection.
   inside4<=inside3;bits4<=font_row(glyph3,gy3);gx4<=gx3;
   header4<=header3;on4<=on3;
   // Stage 5: foreground/background RGB565.
   // Glyph-only overlay: transparent outside actual 5x7 font ink.
   // Registered mask stays cycle-aligned with pixel5 and the PAD delay.
   inside5<=inside4 && (gx4 < 3'd5) && bits4[4-gx4];
   if(gx4<5 && bits4[4-gx4])pixel5<=header4?(on4?16'h07e0:16'hfd20):16'hffff;
   else pixel5<=16'h0843;
   // Parallel padding to match the existing coordinate/cache/interpolation path.
   cover_delay<={cover_delay[PAD-2:0],inside5};pixel_delay[0]<=pixel5;
   for(i=1;i<PAD;i=i+1)pixel_delay[i]<=pixel_delay[i-1];
  end
 end
endmodule


// A frame-rate 4-corner geometric shift estimator.
// This is NOT a calibration accuracy meter. Implements the OpenCV-style
// k1,k2,p1,p2 forward model at the four visible image corners, using the
// engineering assumptions fx=fy=1400 and (cx,cy)=(960,540).
// Hardware is one serial shift/add multiplier + serial square/root + BCD.
// 16-bit Q8 component and 32-bit square widths assume FPGA coeff_guard limits.
// No BRAM, no DSP, no changes to the 32-cycle video/OSD pixel pipeline.
// Recalculates in < 700 pix_clk cycles after a frame commit.
module osd_corner_shift_calc (
 input wire clk,rst_n,start,
 input wire signed [31:0] k1,k2,p1,p2,
 output reg [15:0] shift_digits,vertical_digits,
 output reg ready
);
  localparam [2:0] S_IDLE=0,S_ACC=1,S_SQUARE=2,S_COMPARE=3,
                   S_ROOT=4,S_PRE_BCD=5,S_BCD=6;
  reg [2:0]state;
  reg signed [31:0] ck1,ck2,cp1,cp2;
  reg [15:0] max_dy,shift_pending;
  reg bcd_vertical;
  reg [1:0]corner,term;
  reg axis;
  reg [3:0]mul_bit;
  reg signed [31:0]acc;
  reg signed [15:0] dx_q8,dy_q8;
  reg [15:0]sq_bits;
  reg [31:0]sq_base,sq_acc,x_square,maximum_sq,mag_sq_hold;
  reg sq_axis;
  reg [3:0]sq_bit;
  reg [31:0]radicand;
  reg [19:0]remainder;
  reg [15:0]root,root_final;
  reg [4:0]root_bit;
  reg [13:0]bcd_binary;
  reg [15:0]bcd_work;
  reg [3:0]bcd_bit;

  function signed [17:0] geom_weight;
    input [4:0] idx;
    begin case(idx)
   5'd0: geom_weight=-18'sd9508;
   5'd1: geom_weight=-18'sd5885;
   5'd2: geom_weight=18'sd11849;
   5'd3: geom_weight=18'sd34930;
   5'd4: geom_weight=-18'sd5348;
   5'd5: geom_weight=-18'sd3310;
   5'd6: geom_weight=18'sd20530;
   5'd7: geom_weight=18'sd11849;
   5'd8: geom_weight=18'sd9483;
   5'd9: geom_weight=18'sd5860;
   5'd10: geom_weight=-18'sd11837;
   5'd11: geom_weight=18'sd34864;
   5'd12: geom_weight=-18'sd5340;
   5'd13: geom_weight=-18'sd3300;
   5'd14: geom_weight=18'sd20508;
   5'd15: geom_weight=-18'sd11837;
   5'd16: geom_weight=-18'sd9499;
   5'd17: geom_weight=-18'sd5875;
   5'd18: geom_weight=-18'sd11827;
   5'd19: geom_weight=18'sd34918;
   5'd20: geom_weight=18'sd5333;
   5'd21: geom_weight=18'sd3298;
   5'd22: geom_weight=18'sd20493;
   5'd23: geom_weight=-18'sd11827;
   5'd24: geom_weight=18'sd9474;
   5'd25: geom_weight=18'sd5850;
   5'd26: geom_weight=18'sd11815;
   5'd27: geom_weight=18'sd34852;
   5'd28: geom_weight=18'sd5325;
   5'd29: geom_weight=18'sd3288;
   5'd30: geom_weight=18'sd20471;
   5'd31: geom_weight=18'sd11815;
     default:geom_weight=18'sd0;
    endcase end
  endfunction

  wire signed [31:0] src_coeff = (term==0)?ck1:
                                    (term==1)?ck2:(term==2)?cp1:cp2;
  wire [11:0] coef_mag = src_coeff[31] ? -src_coeff[11:0] : src_coeff[11:0];
  wire signed [17:0] w = geom_weight({corner,axis,term});
  wire signed [31:0] addend = $signed({{14{w[17]}},w}) <<< mul_bit;
  wire signed [31:0] acc_next = !coef_mag[mul_bit] ? acc :
                                     src_coeff[31] ? acc-addend : acc+addend;
  wire signed [15:0] component_finish=acc_next >>> 12; // Q8 pixel units
  wire [15:0] dx_abs = dx_q8[15] ? -dx_q8 : dx_q8;
  wire [15:0] dy_abs = dy_q8[15] ? -dy_q8 : dy_q8;
  wire [31:0] sq_next = sq_bits[0] ? sq_acc+sq_base : sq_acc;
  wire [31:0] mag_square = x_square + sq_next;
  wire [31:0] next_max = mag_sq_hold>maximum_sq ? mag_sq_hold:maximum_sq;
  wire [19:0] rem_shift={remainder[17:0],radicand[31:30]};
  wire [19:0] trial={2'b00,root,2'b01};
  wire root_take = rem_shift >= trial;
  wire [15:0] root_next = {root[14:0],root_take};
  wire [19:0] rem_next = root_take ? rem_shift-trial:rem_shift;
  // Convert Q8 pixel -> hundredths, rounding to 2 decimals.
  wire [31:0] centi_raw=((({16'd0,root_final}<<6)+({16'd0,root_final}<<5)+({16'd0,root_final}<<2))+32'd128)>>8;
  wire [13:0] centi=(centi_raw>9999)?14'd9999:centi_raw[13:0];
  reg [15:0]bcd_adjust;
  always @* begin
    bcd_adjust=bcd_work;
    if(bcd_adjust[3:0]>=5) bcd_adjust[3:0]=bcd_adjust[3:0]+4'd3;
    if(bcd_adjust[7:4]>=5) bcd_adjust[7:4]=bcd_adjust[7:4]+4'd3;
    if(bcd_adjust[11:8]>=5) bcd_adjust[11:8]=bcd_adjust[11:8]+4'd3;
    if(bcd_adjust[15:12]>=5) bcd_adjust[15:12]=bcd_adjust[15:12]+4'd3;
  end
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      state<=S_IDLE;ready<=0;shift_digits<=0;vertical_digits<=0;
      ck1<=0;ck2<=0;cp1<=0;cp2<=0;max_dy<=0;shift_pending<=0;bcd_vertical<=0;
      corner<=0;axis<=0;term<=0;mul_bit<=0;acc<=0;
      dx_q8<=0;dy_q8<=0;sq_bits<=0;
      sq_base<=0;sq_acc<=0;x_square<=0;maximum_sq<=0;mag_sq_hold<=0;
      sq_axis<=0;sq_bit<=0;radicand<=0;remainder<=0;
      root<=0;root_final<=0;root_bit<=0;bcd_binary<=0;bcd_work<=0;bcd_bit<=0;
    end else if(start)begin
      // All four inputs are already committed values; these are sampled by
      // the caller at the SAME frame boundary as its OSD register update.
      state<=S_ACC;ready<=0;corner<=0;axis<=0;term<=0;mul_bit<=0;acc<=0;
      maximum_sq<=0;dx_q8<=0;dy_q8<=0;
      ck1<=k1;ck2<=k2;cp1<=p1;cp2<=p2;max_dy<=0;bcd_vertical<=0;
    end else begin
      case(state)
      S_IDLE:begin end
      S_ACC:begin
        acc<=acc_next;
        if(mul_bit==4'd11)begin
          mul_bit<=0;
          if(term<3)term<=term+1'b1;
          else begin
            term<=0;acc<=0;
            if(!axis)begin dx_q8<=component_finish;axis<=1;end
            else begin
              dy_q8<=component_finish;
              state<=S_SQUARE;
              sq_axis<=0;sq_bit<=0;
              sq_bits<=dx_abs;sq_base<={16'd0,dx_abs};sq_acc<=0;
            end
          end
        end else mul_bit<=mul_bit+1'b1;
      end
      S_SQUARE:begin
        sq_acc<=sq_next;sq_bits<=sq_bits>>1;sq_base<=sq_base<<1;
        if(sq_bit==4'd15)begin
          sq_bit<=0;
          if(!sq_axis)begin
            x_square<=sq_next;sq_axis<=1;
            sq_bits<=dy_abs;sq_base<={16'd0,dy_abs};sq_acc<=0;
          end else begin
            mag_sq_hold<=mag_square;
            state<=S_COMPARE;
          end
        end else sq_bit<=sq_bit+1'b1;
      end
      S_COMPARE:begin
        if(dy_abs>max_dy)max_dy<=dy_abs;
        maximum_sq<=next_max;
        if(corner<3)begin
          corner<=corner+1'b1;axis<=0;term<=0;mul_bit<=0;acc<=0;
          state<=S_ACC;
        end else begin
          radicand<=next_max;remainder<=0;root<=0;root_bit<=0;
          state<=S_ROOT;
        end
      end
      S_ROOT:begin
        radicand<=radicand<<2;remainder<=rem_next;root<=root_next;
        if(root_bit==5'd15)begin
          root_final<=root_next;
          state<=S_PRE_BCD;
        end else root_bit<=root_bit+1'b1;
      end
      S_PRE_BCD:begin
        bcd_binary<=centi;bcd_work<=0;bcd_bit<=0;state<=S_BCD;
      end
      S_BCD:begin
        bcd_work<={bcd_adjust[14:0],bcd_binary[13]};
        bcd_binary<=bcd_binary<<1;
        if(bcd_bit==4'd13)begin
          if(!bcd_vertical)begin
            shift_pending<={bcd_adjust[14:0],bcd_binary[13]};
            root_final<=max_dy;bcd_vertical<=1;state<=S_PRE_BCD;
          end else begin
            shift_digits<=shift_pending;
            vertical_digits<={bcd_adjust[14:0],bcd_binary[13]};
            ready<=1;state<=S_IDLE;
          end
        end else bcd_bit<=bcd_bit+1'b1;
      end
      default:state<=S_IDLE;
      endcase
    end
  end
endmodule
