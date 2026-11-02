`timescale 1ns/1ps
// 1080p inverse-map distortion coordinate engine.
// Fixed-point formats:
//   CX/CY/FX/FY, normalized x/y, k1/k2/p1/p2: signed Q16.16
//   INV_FX/INV_FY: signed Q0.32 (reciprocal focal length)
//   u/v input and integer outputs: pixels; wx/wy: unsigned Q0.16 fractions
//
// This version splits the multiply chain into 13 registered stages. Coordinates,
// bounds, and valid_out are aligned on the same output cycle.
module distortion_correction #(
    parameter integer H_ACTIVE = 1920,
    parameter integer V_ACTIVE = 1080,
    parameter integer ENABLE_DISTORTION = 1,
    // 1 for validated 1080p synthetic model: Q16.16 radial operands fit
    // in signed 18 bits. Set 0 for future models until their ranges are checked.
    parameter integer USE_NARROW_RADIAL_MUL = 1,

    // Synthetic 1080p test model. K1=2621/65536 ~= 0.04 with fx=fy=1400
    // has a maximum vertical source-row displacement of about 13.4 pixels.
    // The integrated distortion-mode cache uses 32 slots with radius=15.
    // Recalibrate and re-check source-row spread for the eventual OV5640 lens.
    parameter signed [31:0] CX_Q     = 32'sd62914560,  // 960.0, Q16.16
    parameter signed [31:0] CY_Q     = 32'sd35389440,  // 540.0, Q16.16
    parameter signed [31:0] FX_Q     = 32'sd91750400,  // 1400.0, Q16.16
    parameter signed [31:0] FY_Q     = 32'sd91750400,  // 1400.0, Q16.16
    parameter signed [31:0] INV_FX_Q = 32'sd3067834,   // (1/1400) * 2^32
    parameter signed [31:0] INV_FY_Q = 32'sd3067834,   // (1/1400) * 2^32
    parameter signed [31:0] K1_Q     = 32'sd2621,      // k1 ~= 0.04, Q16.16
    parameter signed [31:0] K2_Q     = 32'sd0,         // k2 = 0 for first DDR test
    parameter signed [31:0] P1_Q     = 32'sd0,
    parameter signed [31:0] P2_Q     = 32'sd0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire [15:0] u_in,
    input  wire [15:0] v_in,
    output reg         valid_out,
    output reg  [15:0] u_int,
    output reg  [15:0] v_int,
    output reg  [15:0] wx,
    output reg  [15:0] wy,
    output reg         in_bounds
);
    localparam signed [31:0] ONE_Q16 = 32'sd65536;
    localparam signed [63:0] U_LIMIT_Q = (H_ACTIVE - 1) * 65536;
    localparam signed [63:0] V_LIMIT_Q = (V_ACTIVE - 1) * 65536;

    // Exact 32x32 -> 64 signed product. Do not widen both operands to
    // 64 bits BEFORE multiplying: that may obscure narrow DSP inference.
    function signed [63:0] mul_s32;
        input signed [31:0] a;
        input signed [31:0] b;
        begin
            mul_s32 = $signed(a) * $signed(b);
        end
    endfunction

    // 13-stage valid pipeline: an extra registered radial-product stage.
    reg valid_s1, valid_s2, valid_s3, valid_s4, valid_s5, valid_s6;
    reg valid_s7, valid_s8, valid_s9, valid_s10, valid_s11, valid_s12;

    reg signed [31:0] dx_s1, dy_s1;
    // Timing experiment: register Y delta magnitude and sign at S1.
    // For the exact synthetic model CY_Q=540<<16, so v_in[11:0]-540
    // matches the signed 12-bit dy_s1[27:16] used by the old S2.
    // Do not affect the general-model fallback path (dy_s1 kept).
    // S1: register X magnitude/sign before the S2 multiplier (1080p model).
    wire [11:0] dx_delta_s0 = u_in[11:0] - 12'd960;
    wire [11:0] dx_abs_s0 = dx_delta_s0[11]
                              ? (~dx_delta_s0 + 12'd1)
                              : dx_delta_s0;
    reg  [11:0] dx_abs_s1;
    reg         dx_neg_s1;
    wire [11:0] dy_delta_s0 = v_in[11:0] - 12'd540;
    wire [11:0] dy_abs_s0 = dy_delta_s0[11]
                              ? (~dy_delta_s0 + 12'd1)
                              : dy_delta_s0;
    reg  [11:0] dy_abs_s1;
    reg         dy_neg_s1;
    // S2: register products, signs and generic fallback deltas.
    reg [34:0] x_norm_abs_product_s2, y_norm_abs_product_s2;
    reg dx_neg_s2, dy_neg_s2;
    reg signed [31:0] dx_s2, dy_s2;
    // S3: sign restore / floor rounding / magnitude registers.
    reg signed [31:0] xn_s3, yn_s3;
    // K1=0.04 test model: keep unsigned absolute magnitudes in S2 so
    // S3 x^2/y^2 DSP operands do not need sign conversion after S2.
    // |xn|, |yn| < 1.0 (Q16.16), hence 16 bits hold their magnitudes.
    reg [15:0] xn_mag_s3, yn_mag_s3;
    reg signed [31:0] xn_s4, yn_s4, x2_s4, y2_s4, xy_s4;
    // Faster exact normalization for THIS synthetic 1920x1080 model only.
    // dx_s1 = (u_in-960)*65536, dy_s1 = (v_in-540)*65536.
    // (dx_s1 * INV_FX_Q) >>> 32 = ((u_in-960) * INV_FX_Q) >>> 16.
    // The signed pixel deltas fit 12 bits, reciprocals fit signed 23 bits.
    // Fallback preserves general parameter behavior for future camera lenses.
    localparam integer USE_EXACT_TEST_MODEL =
         (H_ACTIVE == 1920) && (V_ACTIVE == 1080) &&
         (CX_Q == 32'sd62914560) && (CY_Q == 32'sd35389440) &&
         (FX_Q == 32'sd91750400) && (FY_Q == 32'sd91750400) &&
         (INV_FX_Q == 32'sd3067834) && (INV_FY_Q == 32'sd3067834);

    wire signed [34:0] x_norm_product_short =
        $signed(dx_s1[27:16]) * $signed(INV_FX_Q[22:0]);
    wire signed [34:0] y_norm_product_short =
        $signed(dy_s1[27:16]) * $signed(INV_FY_Q[22:0]);

    // S2: X sign-safe unsigned magnitude multiply, matching the Y path.
    // Negative arithmetic right shift uses floor(-a/2^16), not truncation.
    // S2 output: raw magnitude product registered BEFORE any rounding.
    wire signed [31:0] x_norm_abs_integer =
        $signed({13'd0, x_norm_abs_product_s2[34:16]});
    wire x_norm_has_fraction = |x_norm_abs_product_s2[15:0];
    wire signed [31:0] x_norm_signsafe = dx_neg_s2
        ? (-x_norm_abs_integer - (x_norm_has_fraction ? 32'sd1 : 32'sd0))
        : x_norm_abs_integer;
    // Exact +1 increment, expressed as independent prefix carries.
    // This removes the explicit 16-bit ripple incrementer from the
    // product -> magnitude -> next DSP-input path, without adding latency.
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

    // ===========================================
    // ★ 新增：S2 纵坐标归一化：无符号幅值乘法实验
    // 仅针对当前1920x1080测试模型
    // ===========================================
    // Magnitude and sign are now registered at S1, rather than
    // being derived combinationally from dy_s1 on the S1->S2 path.

    // 绝对值 × 正数倒数

    // 乘积右移16位
    wire signed [31:0] y_norm_abs_integer =
        $signed({13'd0, y_norm_abs_product_s2[34:16]});

    // 恢复正负符号
    // 对负数采用向负无穷方向取整，
    // 保持与原来的算术右移 >>>16 完全一致
    wire signed [31:0] y_norm_signsafe =
        dy_neg_s2 ?
        (-y_norm_abs_integer -
         ((|y_norm_abs_product_s2[15:0]) ?
          32'sd1 : 32'sd0)) :
        y_norm_abs_integer;

    // If negative, abs(-ceil(product/65536)) is ceil(product/65536).
    // If positive, abs(floor(product/65536)) is floor(product/65536).
    // Same exact parallel-prefix increment for Y normalization.
    wire y_norm_round_up = dy_neg_s2;
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


    // For the selected 1080p model xn/yn range is about [-0.686,+0.685]
    // in Q16.16, well inside signed 18-bit range [-2,+2).
    // S4 uses S3 normalized magnitude registers, adding exactly 1 coordinate cycle.
    wire signed [35:0] x2_prod18 = $signed(xn_s3[17:0]) * $signed(xn_s3[17:0]);
    wire signed [35:0] y2_prod18 = $signed(yn_s3[17:0]) * $signed(yn_s3[17:0]);
    wire signed [35:0] xy_prod18 = $signed(xn_s3[17:0]) * $signed(yn_s3[17:0]);
    // Same x^2/y^2 values with smaller, unsigned DSP inputs.
    wire [31:0] x2_prod16 = $unsigned(xn_mag_s3) * $unsigned(xn_mag_s3);
    wire [31:0] y2_prod16 = $unsigned(yn_mag_s3) * $unsigned(yn_mag_s3);
    reg signed [31:0] xn_s5, yn_s5, x2_s5, y2_s5, xy_s5, r2_s5;
    reg signed [31:0] xn_s6, yn_s6, r4_s6, k1r2_s6;
    reg signed [31:0] tx1_s6, tx2_s6, ty1_s6, ty2_s6;
    reg signed [31:0] xn_s7, yn_s7, k2r4_s7, radial_base_s7;
    reg signed [31:0] tx_s7, ty_s7;
    reg signed [31:0] xn_s8, yn_s8, radial_s8, tx_s8, ty_s8;
    // Narrow radial products are exact for the current 1080p test model.
    // x,y in [-0.686,0.686], radial in [1,~1.025] Q16.16.
    wire signed [35:0] radial_x_product18 =
        $signed(xn_s8[17:0]) * $signed(radial_s8[17:0]);
    wire signed [35:0] radial_y_product18 =
        $signed(yn_s8[17:0]) * $signed(radial_s8[17:0]);
    reg signed [63:0] radial_x_product_s9, radial_y_product_s9;
    reg signed [31:0] xn_bypass_s9, yn_bypass_s9, tx_s9, ty_s9;
    reg signed [31:0] xd_s10, yd_s10;
    // FX_Q=FY_Q=(1400<<16), hence the Q16.16 pixel delta
    // (xd_s10 * FX_Q) >>> 16 = 1400*xd_s10 exactly (low 32 bits).
    // 1400=1024+256+128-8: two balanced add/sub groups.
    wire signed [31:0] xscale_pair_a = (xd_s10 <<< 10) + (xd_s10 <<< 8);
    wire signed [31:0] xscale_pair_b = (xd_s10 <<< 7) - (xd_s10 <<< 3);
    wire signed [31:0] yscale_pair_a = (yd_s10 <<< 10) + (yd_s10 <<< 8);
    wire signed [31:0] yscale_pair_b = (yd_s10 <<< 7) - (yd_s10 <<< 3);
    wire signed [31:0] xscale_1400 = xscale_pair_a + xscale_pair_b;
    wire signed [31:0] yscale_1400 = yscale_pair_a + yscale_pair_b;

    reg signed [31:0] x_delta_s11, y_delta_s11;
    reg signed [63:0] u_pix_s12, v_pix_s12;

    // Debug-only identity path for ENABLE_DISTORTION == 0.
    // Input sampled in stage S1 arrives at stage S13 after 13 clock edges.
    // 12 registers here + S13 output register preserve that latency.
    // Deliberately bypasses normalization AND pixel reconstruction.
    reg [15:0] identity_u_pipe [0:11];
    reg [15:0] identity_v_pipe [0:11];
    integer identity_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0; valid_s2 <= 1'b0; valid_s3 <= 1'b0; valid_s4 <= 1'b0;
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
            for (identity_i = 0; identity_i < 12; identity_i = identity_i + 1) begin
                identity_u_pipe[identity_i] <= 16'd0;
                identity_v_pipe[identity_i] <= 16'd0;
            end
        end else begin
            identity_u_pipe[0] <= u_in;
            identity_v_pipe[0] <= v_in;
            for (identity_i = 1; identity_i < 12; identity_i = identity_i + 1) begin
                identity_u_pipe[identity_i] <= identity_u_pipe[identity_i-1];
                identity_v_pipe[identity_i] <= identity_v_pipe[identity_i-1];
            end
            valid_s1 <= valid_in;
            valid_s2 <= valid_s1;
            valid_s3 <= valid_s2;
            valid_s4 <= valid_s3;
            valid_s5 <= valid_s4;
            valid_s6 <= valid_s5;
            valid_s7 <= valid_s6;
            valid_s8 <= valid_s7;
            valid_s9 <= valid_s8;
            valid_s10 <= valid_s9;
            valid_s11 <= valid_s10;
            valid_s12 <= valid_s11;
            valid_out <= valid_s12;

            // S1: pixel delta from principal point, in Q16.16.
            if (valid_in) begin
                dx_s1 <= ($signed({16'd0, u_in}) <<< 16) - CX_Q;
                dy_s1 <= ($signed({16'd0, v_in}) <<< 16) - CY_Q;
                // Both normalized-coordinate magnitudes/signs are registered at S1.
                dx_abs_s1 <= dx_abs_s0;
                dx_neg_s1 <= dx_delta_s0[11];
                dy_abs_s1 <= dy_abs_s0;
                dy_neg_s1 <= dy_delta_s0[11];
            end

            // S2: register the normalization multiplications ONLY.
            // Separates DSP output from signed rounding/magnitude used by S4 DSP.
            if (valid_s1) begin
                x_norm_abs_product_s2 <= $unsigned(dx_abs_s1) * $unsigned(INV_FX_Q[22:0]);
                y_norm_abs_product_s2 <= $unsigned(dy_abs_s1) * $unsigned(INV_FY_Q[22:0]);
                dx_neg_s2 <= dx_neg_s1;
                dy_neg_s2 <= dy_neg_s1;
                dx_s2 <= dx_s1;
                dy_s2 <= dy_s1;
            end

            // S3: Q16.16 normalize/floor rounding, register signs and magnitudes.
            if (valid_s2) begin
                if (USE_EXACT_TEST_MODEL != 0) begin
                    xn_s3 <= x_norm_signsafe;
                    yn_s3 <= y_norm_signsafe;
                    xn_mag_s3 <= x_norm_mag_q;
                    yn_mag_s3 <= y_norm_mag_q;
                end else begin
                    // Generic parameters: preserve the original math with +1 latency.
                    xn_s3 <= mul_s32(dx_s2, INV_FX_Q) >>> 32;
                    yn_s3 <= mul_s32(dy_s2, INV_FY_Q) >>> 32;
                    xn_mag_s3 <= 16'd0;
                    yn_mag_s3 <= 16'd0;
                end
            end

            // S4: x^2, y^2 and xy, Q16.16.
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

            // S5: r^2 and operands forwarded to the next stage.
            if (valid_s4) begin
                xn_s5 <= xn_s4; yn_s5 <= yn_s4;
                x2_s5 <= x2_s4; y2_s5 <= y2_s4; xy_s5 <= xy_s4;
                r2_s5 <= x2_s4 + y2_s4;
            end

            // S6: independent products (no serial multiplier chain in one stage).
            if (valid_s5) begin
                xn_s6 <= xn_s5; yn_s6 <= yn_s5;
                r4_s6   <= mul_s32(r2_s5, r2_s5) >>> 16;
                k1r2_s6 <= mul_s32(K1_Q, r2_s5) >>> 16;
                tx1_s6  <= mul_s32(P1_Q, xy_s5) >>> 15;
                tx2_s6  <= mul_s32(P2_Q, r2_s5 + (x2_s5 <<< 1)) >>> 16;
                ty1_s6  <= mul_s32(P1_Q, r2_s5 + (y2_s5 <<< 1)) >>> 16;
                ty2_s6  <= mul_s32(P2_Q, xy_s5) >>> 15;
            end

            // S7: second radial multiplier and tangential sums.
            if (valid_s6) begin
                xn_s7 <= xn_s6; yn_s7 <= yn_s6;
                k2r4_s7 <= mul_s32(K2_Q, r4_s6) >>> 16;
                radial_base_s7 <= ONE_Q16 + k1r2_s6;
                tx_s7 <= tx1_s6 + tx2_s6;
                ty_s7 <= ty1_s6 + ty2_s6;
            end

            // S8: radial scale.
            if (valid_s7) begin
                xn_s8 <= xn_s7; yn_s8 <= yn_s7;
                radial_s8 <= radial_base_s7 + k2r4_s7;
                tx_s8 <= tx_s7; ty_s8 <= ty_s7;
            end

            // S9: REGISTER THE MULTIPLIER OUTPUTS. No final addition here.
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

            // S10: fixed-point slice and tangential add (no multiplier).
            if (valid_s9) begin
                if (ENABLE_DISTORTION != 0) begin
                    xd_s10 <= $signed(radial_x_product_s9[47:16]) + tx_s9;
                    yd_s10 <= $signed(radial_y_product_s9[47:16]) + ty_s9;
                end else begin
                    xd_s10 <= xn_bypass_s9;
                    yd_s10 <= yn_bypass_s9;
                end
            end

            // S11: normalized -> pixel offsets.
            if (valid_s10) begin
                if (USE_EXACT_TEST_MODEL != 0) begin
                    x_delta_s11 <= xscale_1400;
                    y_delta_s11 <= yscale_1400;
                end else begin
                    x_delta_s11 <= mul_s32(xd_s10, FX_Q) >>> 16;
                    y_delta_s11 <= mul_s32(yd_s10, FY_Q) >>> 16;
                end
            end

            // S12: signed Q16.16 source coordinates.
            if (valid_s11) begin
                u_pix_s12 <= $signed(x_delta_s11) + CX_Q;
                v_pix_s12 <= $signed(y_delta_s11) + CY_Q;
            end

            // S13: coordinates, fractions, bounds and valid are aligned.
            if (valid_s12) begin
                if (ENABLE_DISTORTION == 0) begin
                    u_int <= identity_u_pipe[11];
                    v_int <= identity_v_pipe[11];
                    wx <= 16'd0;
                    wy <= 16'd0;
                    in_bounds <= (identity_u_pipe[11] < H_ACTIVE) &&
                                 (identity_v_pipe[11] < V_ACTIVE);
                end else begin
                    // ★ 修改：恢复正常的数学坐标输出
                    u_int <= u_pix_s12[31:16];
                    v_int <= v_pix_s12[31:16];
                    
                    wx <= u_pix_s12[15:0];
                    wy <= v_pix_s12[15:0];

                    in_bounds <= (u_pix_s12 >= 64'sd0) &&
                                 (v_pix_s12 >= 64'sd0) &&
                                 (u_pix_s12 < U_LIMIT_Q) &&
                                 (v_pix_s12 < V_LIMIT_Q);
                end
            end else begin
                in_bounds <= 1'b0;
            end
        end
    end
endmodule