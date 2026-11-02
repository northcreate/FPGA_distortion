`timescale 1ns/1ps
// SIX registered stages. Screen x increases right, y increases down.
// Forward display transform: out = Rcw*(corrected-center)+center+(tx,0).
// Inverse lookup: dx=u-960-tx, dy=v-540;
// x'= cos*dx + sin*dy + 960; y'= -sin*dx + cos*dy + 540.
// Fractional source coordinates are kept in signed Q16.16 (no early rounding).
module affine_inverse_map(
    input wire clk,rst_n,valid_in,
    input wire [15:0] u_in,v_in,
    input wire correction_on,rotate_cw15,
    input wire signed [15:0] translate_x,translate_y,
    output reg valid_out,bounds_out,correction_out,
    output reg signed [31:0] u_q_out,v_q_out
);
    localparam signed [17:0] COS15_Q=18'sd63303;
    localparam signed [17:0] SIN15_Q=18'sd16962;
    localparam signed [33:0] CX_Q=34'sd62914560,CY_Q=34'sd35389440;
    localparam signed [33:0] UMAX_Q=34'sd125763584; // 1919<<16
    localparam signed [33:0] VMAX_Q=34'sd70713344;  // 1079<<16
    reg [4:0] vp,cp;
    reg rot_s1;
    reg signed [17:0] dx_s1,dy_s1;
    reg signed [35:0] xc_s2,ys_s2,xs_s2,yc_s2;
    reg signed [36:0] xr_s3,yr_s3;
    reg signed [37:0] uq_s4,vq_s4;
    reg signed [31:0] uq_s5,vq_s5;
    reg inside_s5;
    integer i;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            vp<=0;cp<=0;rot_s1<=0;dx_s1<=0;dy_s1<=0;
            xc_s2<=0;ys_s2<=0;xs_s2<=0;yc_s2<=0;
            xr_s3<=0;yr_s3<=0;uq_s4<=0;vq_s4<=0;
            uq_s5<=0;vq_s5<=0;inside_s5<=0;
            valid_out<=0;bounds_out<=0;correction_out<=0;u_q_out<=0;v_q_out<=0;
        end else begin
            vp<={vp[3:0],valid_in};cp<={cp[3:0],correction_on};
            // S1: integer translation/centering. Widen BEFORE subtraction.
            dx_s1<=$signed({2'b00,u_in})-18'sd960-$signed({{2{translate_x[15]}},translate_x});
            dy_s1<=$signed({2'b00,v_in})-18'sd540-$signed({{2{translate_y[15]}},translate_y});
            rot_s1<=rotate_cw15;
            // S2: four independent multiplier output registers. No add here.
            if(rot_s1)begin
                xc_s2<=dx_s1*COS15_Q;ys_s2<=dy_s1*SIN15_Q;
                xs_s2<=dx_s1*SIN15_Q;yc_s2<=dy_s1*COS15_Q;
            end else begin
                xc_s2<=$signed({{18{dx_s1[17]}},dx_s1})<<<16;
                yc_s2<=$signed({{18{dy_s1[17]}},dy_s1})<<<16;
                xs_s2<=0;ys_s2<=0;
            end
            // S3: sum/subtract registered products; sign-extend before addition.
            xr_s3<=$signed({xc_s2[35],xc_s2})+$signed({ys_s2[35],ys_s2});
            yr_s3<=$signed({yc_s2[35],yc_s2})-$signed({xs_s2[35],xs_s2});
            // S4: restore the center, still full signed precision.
            uq_s4<=$signed({xr_s3[36],xr_s3})+CX_Q;
            vq_s4<=$signed({yr_s3[36],yr_s3})+CY_Q;
            // S5: reject negative/overflow coordinates BEFORE truncation.
            inside_s5<=(uq_s4>=0 && uq_s4<=UMAX_Q && vq_s4>=0 && vq_s4<=VMAX_Q);
            if(uq_s4>=0 && uq_s4<=UMAX_Q && vq_s4>=0 && vq_s4<=VMAX_Q)begin
                uq_s5<=uq_s4[31:0];vq_s5<=vq_s4[31:0];
            end else begin uq_s5<=0;vq_s5<=0;end
            // S6: registered safe interface; invalid pixels keep their valid token.
            valid_out<=vp[4];bounds_out<=vp[4] && inside_s5;correction_out<=cp[4];
            u_q_out<=uq_s5;v_q_out<=vq_s5;
        end
    end
endmodule
