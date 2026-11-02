`timescale 1ns/1ps
// 16-stage, fractional Q16.16 input mapper for fixed 1920x1080 / f=1400.
// Upstream affine stage guarantees nonnegative in-frame inputs or bounds_in=0.
// Normalization uses registered 14x12/14x11 partial products, paired sums,
// then a registered 51-bit sum. Legacy s2/s3 names are now stages 4/5.
// Optional runtime coefficients: signed Q16.16, -2621..2621, fixed default intrinsics.
// Registered 18x18 products for runtime coefficients; no added video stages.
// All-zero runtime coefficients select exact identity, as does correction_on=0.
// Runtime mode requires fixed default intrinsics and the coefficient guard limits.

module distortion_dynamic_q16 #(
    parameter integer H_ACTIVE = 1920,
    parameter integer V_ACTIVE = 1080,
    parameter integer ENABLE_DISTORTION = 1,
    parameter integer USE_NARROW_RADIAL_MUL = 1,
    parameter integer RUNTIME_K1 = 0,
    parameter integer RUNTIME_COEFF = 0,

    parameter signed [31:0] CX_Q     = 32'sd62914560,  // 960.0, Q16.16
    parameter signed [31:0] CY_Q     = 32'sd35389440,  // 540.0, Q16.16
    parameter signed [31:0] FX_Q     = 32'sd91750400,  // 1400.0, Q16.16
    parameter signed [31:0] FY_Q     = 32'sd91750400,  // 1400.0, Q16.16
    parameter signed [31:0] INV_FX_Q = 32'sd3067834,   // (1/1400) * 2^32
    parameter signed [31:0] INV_FY_Q = 32'sd3067834,   // (1/1400) * 2^32
    parameter signed [31:0] K1_Q     = 32'sd655,       // k1 ~= 0.01, Q16.16
    parameter signed [31:0] K2_Q     = 32'sd0,         // k2 = 0 for first DDR test
    parameter signed [31:0] P1_Q     = 32'sd0,
    parameter signed [31:0] P2_Q     = 32'sd0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input wire signed [31:0] u_q_in,v_q_in,
    input wire bounds_in,correction_on,
    input wire signed [31:0] k1_q_in,k2_q_in,p1_q_in,p2_q_in,
    output reg         valid_out,
    output reg  [15:0] u_int,
    output reg  [15:0] v_int,
    output reg  [15:0] wx,
    output reg  [15:0] wy,
    output reg         in_bounds
);
    // Coefficient snapshot follows each input pixel through the extra S4d register stage.
    reg signed [17:0] k1_pipe [0:7];
    reg signed [17:0] k2_pipe [0:8];
    reg signed [17:0] p1_pipe [0:7],p2_pipe [0:7];
    integer ki;
    wire zero_runtime_model = (RUNTIME_COEFF != 0) ?
        (k1_q_in==0 && k2_q_in==0 && p1_q_in==0 && p2_q_in==0) :
        ((RUNTIME_K1 != 0) && k1_q_in==0 && K2_Q==0 && P1_Q==0 && P2_Q==0);
    localparam signed [31:0] ONE_Q16 = 32'sd65536;
    localparam signed [63:0] U_LIMIT_Q = (H_ACTIVE - 1) * 65536;
    localparam signed [63:0] V_LIMIT_Q = (V_ACTIVE - 1) * 65536;

    function signed [63:0] mul_s32;
        input signed [31:0] a;
        input signed [31:0] b;
        begin
            mul_s32 = $signed(a) * $signed(b);
        end
    endfunction

    // Explicit parallel-prefix carry network: no arithmetic '+' operator here.
    // Keep these short coordinate sums in fabric rather than DSP Z/post-adder paths.
    // Result is the exact unsigned sum modulo 2^18; valid geometry fits 18 bits.
    function [17:0] add18_fabric;
        input [17:0] a,b;
        reg [17:0] p0,p1,p2,p3,p4,p5,g0,g1,g2,g3,g4,g5;
        begin
            p0=a^b;g0=a&b;
            g1=g0|(p0&(g0<<1));p1=p0&(p0<<1);
            g2=g1|(p1&(g1<<2));p2=p1&(p1<<2);
            g3=g2|(p2&(g2<<4));p3=p2&(p2<<4);
            g4=g3|(p3&(g3<<8));p4=p3&(p3<<8);
            g5=g4|(p4&(g4<<16));p5=p4&(p4<<16);
            add18_fabric=p0^(g5<<1);
        end
    endfunction
    function [17:0] sum3_fabric;
        input [17:0] a,b,c;
        reg [17:0] sum_bits,carry_bits;
        begin
            sum_bits=a^b^c;
            carry_bits=((a&b)|(a&c)|(b&c))<<1;
            sum3_fabric=add18_fabric(sum_bits,carry_bits);
        end
    endfunction

    // Two extra stages inserted between legacy s1 and s2.
    // All partial products are unsigned and at most 14 x 12 bits.
    reg norm_pp_valid, norm_pair_valid;
    reg [25:0] xpp00,xpp01,xpp10,xpp11;
    reg [25:0] ypp00,ypp01,ypp10,ypp11;
    reg [39:0] xpair_lo,xpair_hi,ypair_lo,ypair_hi;
    reg xneg_pp,yneg_pp,xneg_pair,yneg_pair;
    reg signed [31:0] dx_pp,dy_pp,dx_pair,dy_pair;

    reg valid_s1, valid_s2, valid_s3, valid_s4, valid_s4d, valid_s5, valid_s6;
    reg valid_s7, valid_s8, valid_s9, valid_s10, valid_s11, valid_s12;

    reg signed [31:0] dx_s1, dy_s1;
    wire signed [31:0] dx_delta_s0 = u_q_in - CX_Q;
    wire [31:0] dx_abs_s0 = dx_delta_s0[31]
                              ? (~dx_delta_s0 + 32'd1)
                              : dx_delta_s0;
    reg  [27:0] dx_abs_s1;
    reg         dx_neg_s1;
    wire signed [31:0] dy_delta_s0 = v_q_in - CY_Q;
    wire [31:0] dy_abs_s0 = dy_delta_s0[31]
                              ? (~dy_delta_s0 + 32'd1)
                              : dy_delta_s0;
    reg  [27:0] dy_abs_s1;
    reg         dy_neg_s1;
    reg [50:0] x_norm_abs_product_s2, y_norm_abs_product_s2;
    reg dx_neg_s2, dy_neg_s2;
    reg signed [31:0] dx_s2, dy_s2;
    reg signed [31:0] xn_s3, yn_s3;
    reg [15:0] xn_mag_s3, yn_mag_s3;
    reg signed [31:0] xn_s4, yn_s4, x2_s4, y2_s4, xy_s4;
    localparam integer USE_EXACT_TEST_MODEL =
         (H_ACTIVE == 1920) && (V_ACTIVE == 1080) &&
         (CX_Q == 32'sd62914560) && (CY_Q == 32'sd35389440) &&
         (FX_Q == 32'sd91750400) && (FY_Q == 32'sd91750400) &&
         (INV_FX_Q == 32'sd3067834) && (INV_FY_Q == 32'sd3067834);

    wire signed [34:0] x_norm_product_short =
        $signed(dx_s1[27:16]) * $signed(INV_FX_Q[22:0]);
    wire signed [34:0] y_norm_product_short =
        $signed(dy_s1[27:16]) * $signed(INV_FY_Q[22:0]);

    wire signed [31:0] x_norm_abs_integer =
        $signed({13'd0, x_norm_abs_product_s2[50:32]});
    wire x_norm_has_fraction = |x_norm_abs_product_s2[31:0];
    wire signed [31:0] x_norm_signsafe = dx_neg_s2
        ? (-x_norm_abs_integer - (x_norm_has_fraction ? 32'sd1 : 32'sd0))
        : x_norm_abs_integer;
    wire x_norm_round_up = dx_neg_s2 && x_norm_has_fraction;
    wire [15:0] x_norm_mag_q;
    genvar xmag_i;
    generate
        for (xmag_i = 0; xmag_i < 16; xmag_i = xmag_i + 1) begin: G_XMAG_PREFIX
            if (xmag_i == 0) begin
                assign x_norm_mag_q[xmag_i] = x_norm_abs_integer[xmag_i] ^ x_norm_round_up;
            end else begin
                assign x_norm_mag_q[xmag_i] = x_norm_abs_integer[xmag_i] ^
                    (x_norm_round_up && (&x_norm_abs_integer[xmag_i-1:0]));
            end
        end
    endgenerate



    wire signed [31:0] y_norm_abs_integer =
        $signed({13'd0, y_norm_abs_product_s2[50:32]});

    wire signed [31:0] y_norm_signsafe =
        dy_neg_s2 ?
        (-y_norm_abs_integer -
         ((|y_norm_abs_product_s2[31:0]) ?
          32'sd1 : 32'sd0)) :
        y_norm_abs_integer;

    wire y_norm_round_up = dy_neg_s2 && (|y_norm_abs_product_s2[31:0]);
    wire [15:0] y_norm_mag_q;
    genvar ymag_i;
    generate
        for (ymag_i = 0; ymag_i < 16; ymag_i = ymag_i + 1) begin: G_YMAG_PREFIX
            if (ymag_i == 0) begin
                assign y_norm_mag_q[ymag_i] = y_norm_abs_integer[ymag_i] ^ y_norm_round_up;
            end else begin
                assign y_norm_mag_q[ymag_i] = y_norm_abs_integer[ymag_i] ^
                    (y_norm_round_up && (&y_norm_abs_integer[ymag_i-1:0]));
            end
        end
    endgenerate


    wire signed [35:0] x2_prod18 = $signed(xn_s3[17:0]) * $signed(xn_s3[17:0]);
    wire signed [35:0] y2_prod18 = $signed(yn_s3[17:0]) * $signed(yn_s3[17:0]);
    wire signed [35:0] xy_prod18 = $signed(xn_s3[17:0]) * $signed(yn_s3[17:0]);
    wire [31:0] x2_prod16 = $unsigned(xn_mag_s3) * $unsigned(xn_mag_s3);
    wire [31:0] y2_prod16 = $unsigned(yn_mag_s3) * $unsigned(yn_mag_s3);
    // New stage between square multipliers (S4) and radial coefficient DSPs.
    // Register both the radial sum and all companion per-pixel values.
    reg signed [31:0] xn_s4d, yn_s4d, x2_s4d, y2_s4d, xy_s4d;
    reg signed [31:0] r2_s4d;
    reg signed [17:0] tangent_x_s4d, tangent_y_s4d;
    reg signed [31:0] xn_s5, yn_s5, x2_s5, y2_s5, xy_s5, r2_s5;
    wire signed [35:0] runtime_k1_product =
        $signed(k1_pipe[7]) * $signed({2'b00,r2_s5[15:0]});
    // Register tangential factors one stage before multiplication. No adder-multiplier chain.
    reg signed [17:0] tangent_x_s5,tangent_y_s5;
    wire[17:0] x2_factor={2'b0,x2_s4[15:0]};
    wire[17:0] y2_factor={2'b0,y2_s4[15:0]};
    wire[31:0] r4_product16=$unsigned(r2_s5[15:0])*$unsigned(r2_s5[15:0]);
    wire signed[35:0] runtime_tx1=$signed(p1_pipe[7])*$signed(xy_s5[17:0]);
    wire signed[35:0] runtime_tx2=$signed(p2_pipe[7])*$signed(tangent_x_s5);
    wire signed[35:0] runtime_ty1=$signed(p1_pipe[7])*$signed(tangent_y_s5);
    wire signed[35:0] runtime_ty2=$signed(p2_pipe[7])*$signed(xy_s5[17:0]);
    reg signed [31:0] xn_s6, yn_s6, r4_s6, k1r2_s6;
    wire signed[35:0] runtime_k2_product=$signed(k2_pipe[8])*$signed({2'b0,r4_s6[15:0]});
    reg signed [31:0] tx1_s6, tx2_s6, ty1_s6, ty2_s6;
    reg signed [31:0] xn_s7, yn_s7, k2r4_s7, radial_base_s7;
    reg signed [31:0] tx_s7, ty_s7;
    reg signed [31:0] xn_s8, yn_s8, radial_s8, tx_s8, ty_s8;
    wire signed [35:0] radial_x_product18 =
        $signed(xn_s8[17:0]) * $signed(radial_s8[17:0]);
    wire signed [35:0] radial_y_product18 =
        $signed(yn_s8[17:0]) * $signed(radial_s8[17:0]);
    reg signed [63:0] radial_x_product_s9, radial_y_product_s9;
    reg signed [31:0] xn_bypass_s9, yn_bypass_s9, tx_s9, ty_s9;
    reg signed [31:0] xd_s10, yd_s10;
    wire signed [31:0] xscale_pair_a = (xd_s10 <<< 10) + (xd_s10 <<< 8);
    wire signed [31:0] xscale_pair_b = (xd_s10 <<< 7) - (xd_s10 <<< 3);
    wire signed [31:0] yscale_pair_a = (yd_s10 <<< 10) + (yd_s10 <<< 8);
    wire signed [31:0] yscale_pair_b = (yd_s10 <<< 7) - (yd_s10 <<< 3);
    wire signed [31:0] xscale_1400 = xscale_pair_a + xscale_pair_b;
    wire signed [31:0] yscale_1400 = yscale_pair_a + yscale_pair_b;

    reg signed [31:0] x_delta_s11, y_delta_s11;
    reg signed [63:0] u_pix_s12, v_pix_s12;

    reg [31:0] identity_u_pipe [0:14];
    reg [31:0] identity_v_pipe [0:14];
    integer identity_i;
    reg [14:0] correction_pipe,bounds_pipe;
    wire use_identity = !correction_pipe[14] || (ENABLE_DISTORTION == 0);
    wire signed [63:0] selected_u_q = use_identity ? $signed({32'd0,identity_u_pipe[14]}) : u_pix_s12;
    wire signed [63:0] selected_v_q = use_identity ? $signed({32'd0,identity_v_pipe[14]}) : v_pix_s12;
    wire selected_inside = bounds_pipe[14] &&
        selected_u_q>=0 && selected_u_q<=U_LIMIT_Q &&
        selected_v_q>=0 && selected_v_q<=V_LIMIT_Q;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for(ki=0;ki<8;ki=ki+1)begin k1_pipe[ki]<=0;p1_pipe[ki]<=0;p2_pipe[ki]<=0;end
            for(ki=0;ki<9;ki=ki+1)k2_pipe[ki]<=0;
            tangent_x_s5<=0;tangent_y_s5<=0;
            norm_pp_valid<=0; norm_pair_valid<=0;
            xpp00<=0;xpp01<=0;xpp10<=0;xpp11<=0;
            ypp00<=0;ypp01<=0;ypp10<=0;ypp11<=0;
            xpair_lo<=0;xpair_hi<=0;ypair_lo<=0;ypair_hi<=0;
            xneg_pp<=0;yneg_pp<=0;xneg_pair<=0;yneg_pair<=0;
            dx_pp<=0;dy_pp<=0;dx_pair<=0;dy_pair<=0;
            correction_pipe<=0; bounds_pipe<=0;
            valid_s1 <= 1'b0; valid_s2 <= 1'b0; valid_s3 <= 1'b0; valid_s4 <= 1'b0; valid_s4d <= 1'b0;
            valid_s5 <= 1'b0; valid_s6 <= 1'b0; valid_s7 <= 1'b0;
            valid_s8 <= 1'b0; valid_s9 <= 1'b0; valid_s10 <= 1'b0;
            valid_s11 <= 1'b0; valid_s12 <= 1'b0; valid_out <= 1'b0;
            dx_s1 <= 0; dy_s1 <= 0;
            dx_abs_s1 <= 12'd0; dx_neg_s1 <= 1'b0;
            dy_abs_s1 <= 12'd0; dy_neg_s1 <= 1'b0;
            x_norm_abs_product_s2 <= 35'd0;
            y_norm_abs_product_s2 <= 35'd0;
            dx_neg_s2 <= 1'b0; dy_neg_s2 <= 1'b0;
            dx_s2 <= 32'sd0; dy_s2 <= 32'sd0;
            xn_s3 <= 0; yn_s3 <= 0;
            xn_mag_s3 <= 16'd0; yn_mag_s3 <= 16'd0;
            xn_s4 <= 0; yn_s4 <= 0; x2_s4 <= 0; y2_s4 <= 0; xy_s4 <= 0;
            xn_s4d<=0;yn_s4d<=0;x2_s4d<=0;y2_s4d<=0;xy_s4d<=0;
            r2_s4d<=0;tangent_x_s4d<=0;tangent_y_s4d<=0;
            xn_s5 <= 0; yn_s5 <= 0; x2_s5 <= 0; y2_s5 <= 0; xy_s5 <= 0; r2_s5 <= 0;
            xn_s6 <= 0; yn_s6 <= 0; r4_s6 <= 0; k1r2_s6 <= 0;
            tx1_s6 <= 0; tx2_s6 <= 0; ty1_s6 <= 0; ty2_s6 <= 0;
            xn_s7 <= 0; yn_s7 <= 0; k2r4_s7 <= 0; radial_base_s7 <= ONE_Q16;
            tx_s7 <= 0; ty_s7 <= 0;
            xn_s8 <= 0; yn_s8 <= 0; radial_s8 <= ONE_Q16; tx_s8 <= 0; ty_s8 <= 0;
            radial_x_product_s9 <= 0; radial_y_product_s9 <= 0;
            xn_bypass_s9 <= 0; yn_bypass_s9 <= 0;
            tx_s9 <= 0; ty_s9 <= 0;
            xd_s10 <= 0; yd_s10 <= 0;
            x_delta_s11 <= 0; y_delta_s11 <= 0;
            u_pix_s12 <= 0; v_pix_s12 <= 0;
            u_int <= 0; v_int <= 0; wx <= 0; wy <= 0; in_bounds <= 1'b0;
            for (identity_i = 0; identity_i < 15; identity_i = identity_i + 1) begin
                identity_u_pipe[identity_i] <= 16'd0;
                identity_v_pipe[identity_i] <= 16'd0;
            end
        end else begin
            k1_pipe[0]<=k1_q_in[17:0];
            p1_pipe[0]<=p1_q_in[17:0];p2_pipe[0]<=p2_q_in[17:0];k2_pipe[0]<=k2_q_in[17:0];
            for(ki=1;ki<8;ki=ki+1)begin
                k1_pipe[ki]<=k1_pipe[ki-1];p1_pipe[ki]<=p1_pipe[ki-1];p2_pipe[ki]<=p2_pipe[ki-1];
            end
            for(ki=1;ki<9;ki=ki+1)k2_pipe[ki]<=k2_pipe[ki-1];
            correction_pipe<={correction_pipe[13:0],(correction_on && !zero_runtime_model)};
            bounds_pipe<={bounds_pipe[13:0],(bounds_in && u_q_in>=0 && v_q_in>=0 &&
                $signed(u_q_in)<=U_LIMIT_Q && $signed(v_q_in)<=V_LIMIT_Q)};
            identity_u_pipe[0] <= u_q_in;
            identity_v_pipe[0] <= v_q_in;
            for (identity_i = 1; identity_i < 15; identity_i = identity_i + 1) begin
                identity_u_pipe[identity_i] <= identity_u_pipe[identity_i-1];
                identity_v_pipe[identity_i] <= identity_v_pipe[identity_i-1];
            end
            valid_s1 <= valid_in;
            norm_pp_valid <= valid_s1;
            norm_pair_valid <= norm_pp_valid;
            valid_s2 <= norm_pair_valid;
            valid_s3 <= valid_s2;
            valid_s4 <= valid_s3;
            valid_s4d <= valid_s4;
            valid_s5 <= valid_s4d;
            valid_s6 <= valid_s5;
            valid_s7 <= valid_s6;
            valid_s8 <= valid_s7;
            valid_s9 <= valid_s8;
            valid_s10 <= valid_s9;
            valid_s11 <= valid_s10;
            valid_s12 <= valid_s11;
            valid_out <= valid_s12;

            if (valid_in) begin
                dx_s1 <= u_q_in - CX_Q;
                dy_s1 <= v_q_in - CY_Q;
                dx_abs_s1 <= dx_abs_s0[27:0];
                dx_neg_s1 <= dx_delta_s0[31];
                dy_abs_s1 <= dy_abs_s0[27:0];
                dy_neg_s1 <= dy_delta_s0[31];
            end

            if (valid_s1) begin
                xpp00 <= dx_abs_s1[13:0] * INV_FX_Q[11:0];
                xpp01 <= dx_abs_s1[13:0] * INV_FX_Q[22:12];
                xpp10 <= dx_abs_s1[27:14] * INV_FX_Q[11:0];
                xpp11 <= dx_abs_s1[27:14] * INV_FX_Q[22:12];
                ypp00 <= dy_abs_s1[13:0] * INV_FY_Q[11:0];
                ypp01 <= dy_abs_s1[13:0] * INV_FY_Q[22:12];
                ypp10 <= dy_abs_s1[27:14] * INV_FY_Q[11:0];
                ypp11 <= dy_abs_s1[27:14] * INV_FY_Q[22:12];
                xneg_pp<=dx_neg_s1; yneg_pp<=dy_neg_s1;
                dx_pp<=dx_s1; dy_pp<=dy_s1;
            end
            if (norm_pp_valid) begin
                xpair_lo <= {14'd0,xpp00} + {xpp10,14'd0};
                xpair_hi <= {14'd0,xpp01} + {xpp11,14'd0};
                ypair_lo <= {14'd0,ypp00} + {ypp10,14'd0};
                ypair_hi <= {14'd0,ypp01} + {ypp11,14'd0};
                xneg_pair<=xneg_pp; yneg_pair<=yneg_pp;
                dx_pair<=dx_pp; dy_pair<=dy_pp;
            end
            if (norm_pair_valid) begin
                // Upper pair is < 2^39: 28-bit input times 11-bit coefficient.
                // Widen to 51 bits BEFORE addition; no fractional truncation.
                x_norm_abs_product_s2 <= {11'd0,xpair_lo} + {xpair_hi[38:0],12'd0};
                y_norm_abs_product_s2 <= {11'd0,ypair_lo} + {ypair_hi[38:0],12'd0};
                dx_neg_s2 <= xneg_pair; dy_neg_s2 <= yneg_pair;
                dx_s2 <= dx_pair; dy_s2 <= dy_pair;
            end

            if (valid_s2) begin
                if (USE_EXACT_TEST_MODEL != 0) begin
                    xn_s3 <= x_norm_signsafe;
                    yn_s3 <= y_norm_signsafe;
                    xn_mag_s3 <= x_norm_mag_q;
                    yn_mag_s3 <= y_norm_mag_q;
                end else begin
                    xn_s3 <= mul_s32(dx_s2, INV_FX_Q) >>> 32;
                    yn_s3 <= mul_s32(dy_s2, INV_FY_Q) >>> 32;
                    xn_mag_s3 <= 16'd0;
                    yn_mag_s3 <= 16'd0;
                end
            end

            if (valid_s3) begin
                xn_s4 <= xn_s3;
                yn_s4 <= yn_s3;
                if (USE_EXACT_TEST_MODEL != 0) begin
                    x2_s4 <= {16'd0, x2_prod16[31:16]};
                    y2_s4 <= {16'd0, y2_prod16[31:16]};
                end else begin
                    x2_s4 <= {{12{x2_prod18[35]}}, x2_prod18[35:16]};
                    y2_s4 <= {{12{y2_prod18[35]}}, y2_prod18[35:16]};
                end
                xy_s4 <= {{12{xy_prod18[35]}}, xy_prod18[35:16]};
            end

            // S4d: isolate square-product output from the downstream
            // coefficient multipliers by inserting a registered radial sum.
            if (valid_s4) begin
                xn_s4d <= xn_s4; yn_s4d <= yn_s4;
                x2_s4d <= x2_s4; y2_s4d <= y2_s4; xy_s4d <= xy_s4;
                if (USE_EXACT_TEST_MODEL != 0)
                    r2_s4d <= {14'd0,add18_fabric(x2_factor,y2_factor)};
                else r2_s4d <= x2_s4 + y2_s4;
                tangent_x_s4d <= sum3_fabric(x2_factor<<1,x2_factor,y2_factor);
                tangent_y_s4d <= sum3_fabric(y2_factor<<1,y2_factor,x2_factor);
            end

            // S5: all operands advance together. The arithmetic values are
            // unchanged; the added stage changes latency by exactly one clock.
            if (valid_s4d) begin
                xn_s5 <= xn_s4d; yn_s5 <= yn_s4d;
                x2_s5 <= x2_s4d; y2_s5 <= y2_s4d; xy_s5 <= xy_s4d;
                r2_s5 <= r2_s4d;
                tangent_x_s5 <= tangent_x_s4d;
                tangent_y_s5 <= tangent_y_s4d;
            end

            if (valid_s5) begin
                xn_s6 <= xn_s5; yn_s6 <= yn_s5;
                if(RUNTIME_COEFF != 0)r4_s6<={16'd0,r4_product16[31:16]};
                else r4_s6 <= mul_s32(r2_s5, r2_s5) >>> 16;
                if(RUNTIME_K1 != 0 || RUNTIME_COEFF != 0)
                    k1r2_s6 <= {{12{runtime_k1_product[35]}},runtime_k1_product[35:16]};
                else k1r2_s6 <= mul_s32(K1_Q, r2_s5) >>> 16;
                if(RUNTIME_COEFF != 0)begin
                    tx1_s6<={{11{runtime_tx1[35]}},runtime_tx1[35:15]};
                    tx2_s6<={{12{runtime_tx2[35]}},runtime_tx2[35:16]};
                    ty1_s6<={{12{runtime_ty1[35]}},runtime_ty1[35:16]};
                    ty2_s6<={{11{runtime_ty2[35]}},runtime_ty2[35:15]};
                end else begin
                tx1_s6  <= mul_s32(P1_Q, xy_s5) >>> 15;
                tx2_s6  <= mul_s32(P2_Q, r2_s5 + (x2_s5 <<< 1)) >>> 16;
                ty1_s6  <= mul_s32(P1_Q, r2_s5 + (y2_s5 <<< 1)) >>> 16;
                ty2_s6  <= mul_s32(P2_Q, xy_s5) >>> 15;
                end
            end

            if (valid_s6) begin
                xn_s7 <= xn_s6; yn_s7 <= yn_s6;
                if(RUNTIME_COEFF != 0)
                    k2r4_s7<={{12{runtime_k2_product[35]}},runtime_k2_product[35:16]};
                else k2r4_s7 <= mul_s32(K2_Q, r4_s6) >>> 16;
                radial_base_s7 <= ONE_Q16 + k1r2_s6;
                tx_s7 <= tx1_s6 + tx2_s6;
                ty_s7 <= ty1_s6 + ty2_s6;
            end

            if (valid_s7) begin
                xn_s8 <= xn_s7; yn_s8 <= yn_s7;
                radial_s8 <= radial_base_s7 + k2r4_s7;
                tx_s8 <= tx_s7; ty_s8 <= ty_s7;
            end

            if (valid_s8) begin
                if (USE_NARROW_RADIAL_MUL != 0) begin
                    radial_x_product_s9 <= {{28{radial_x_product18[35]}}, radial_x_product18};
                    radial_y_product_s9 <= {{28{radial_y_product18[35]}}, radial_y_product18};
                end else begin
                    radial_x_product_s9 <= mul_s32(xn_s8, radial_s8);
                    radial_y_product_s9 <= mul_s32(yn_s8, radial_s8);
                end
                xn_bypass_s9 <= xn_s8;
                yn_bypass_s9 <= yn_s8;
                tx_s9 <= tx_s8;
                ty_s9 <= ty_s8;
            end

            if (valid_s9) begin
                if (ENABLE_DISTORTION != 0) begin
                    xd_s10 <= $signed(radial_x_product_s9[47:16]) + tx_s9;
                    yd_s10 <= $signed(radial_y_product_s9[47:16]) + ty_s9;
                end else begin
                    xd_s10 <= xn_bypass_s9;
                    yd_s10 <= yn_bypass_s9;
                end
            end

            if (valid_s10) begin
                if (USE_EXACT_TEST_MODEL != 0) begin
                    x_delta_s11 <= xscale_1400;
                    y_delta_s11 <= yscale_1400;
                end else begin
                    x_delta_s11 <= mul_s32(xd_s10, FX_Q) >>> 16;
                    y_delta_s11 <= mul_s32(yd_s10, FY_Q) >>> 16;
                end
            end

            if (valid_s11) begin
                u_pix_s12 <= $signed(x_delta_s11) + CX_Q;
                v_pix_s12 <= $signed(y_delta_s11) + CY_Q;
            end

            valid_out <= valid_s12;
            if(valid_s12 && selected_inside)begin
                u_int<=selected_u_q[31:16];v_int<=selected_v_q[31:16];
                wx<=selected_u_q[15:0];wy<=selected_v_q[15:0];in_bounds<=1'b1;
            end else begin
                u_int<=0;v_int<=0;wx<=0;wy<=0;in_bounds<=1'b0;
            end
        end
    end
endmodule
