`timescale 1ns/1ps
// RGB565 bilinear interpolation, 8 registered stages.
// DIAGNOSTIC CANDIDATE: horizontal S3 signed differences are multiplied as
// unsigned magnitudes; S4 restores exact signed arithmetic shift rounding.
// Vertical S6 remains as in the known 8-stage implementation.
//
// Uses separable interpolation: horizontal on two source rows, then vertical.
// Horizontal results retain 8 fractional bits (Q8), while wx/wy stay Q0.16.
// Nine small signed multiplies replace four 17x17 weight multiplies plus
// twelve 34x(5/6) color multiplies in the previous implementation.
//
// Input accepted at edge N -> output/valid_out updated at edge N+7.
// The earlier 7-stage version updated the output at edge N+6; therefore
// top-level sync/DE timing delay must increase by exactly ONE clock.
// Pixel component output error versus full Q0.32 bilinear interpolation
// is at most one RGB565 channel LSB for the tested input range. Revalidate
// when changing formats/weights/bit widths.
module bilinear_interp_rgb565 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire [15:0] p00,
    input  wire [15:0] p01,
    input  wire [15:0] p10,
    input  wire [15:0] p11,
    input  wire [15:0] wx,
    input  wire [15:0] wy,
    output reg         valid_out,
    output reg  [15:0] pixel_out
);
    reg [5:0] r00_s1,r01_s1,r10_s1,r11_s1;
    reg [5:0] g00_s1,g01_s1,g10_s1,g11_s1;
    reg [5:0] b00_s1,b01_s1,b10_s1,b11_s1;
    reg [15:0] wx_s1,wy_s1;
    reg valid_s1;

    // S2: signed differences only; keep row base pixels alongside.
    reg signed [6:0] dr0_s2,dr1_s2,dg0_s2,dg1_s2,db0_s2,db1_s2;
    reg [5:0] r00_s2,r10_s2,g00_s2,g10_s2,b00_s2,b10_s2;
    reg [15:0] wx_s2,wy_s2;
    reg valid_s2;

    // S3: unsigned magnitude x unsigned Q0.16 weight.
    // Negative differences are carried as separate 1-bit signs, not fed to DSP.
    // Magnitude of each 7-bit signed difference fits in 7 unsigned bits.
    reg [22:0] r0prod_s3,r1prod_s3;
    reg [22:0] g0prod_s3,g1prod_s3;
    reg [22:0] b0prod_s3,b1prod_s3;
    reg r0neg_s3,r1neg_s3,g0neg_s3,g1neg_s3,b0neg_s3,b1neg_s3;
    wire [6:0] r0abs_s2 = dr0_s2[6] ? $unsigned(-dr0_s2) : $unsigned(dr0_s2);
    wire [6:0] r1abs_s2 = dr1_s2[6] ? $unsigned(-dr1_s2) : $unsigned(dr1_s2);
    wire [6:0] g0abs_s2 = dg0_s2[6] ? $unsigned(-dg0_s2) : $unsigned(dg0_s2);
    wire [6:0] g1abs_s2 = dg1_s2[6] ? $unsigned(-dg1_s2) : $unsigned(dg1_s2);
    wire [6:0] b0abs_s2 = db0_s2[6] ? $unsigned(-db0_s2) : $unsigned(db0_s2);
    wire [6:0] b1abs_s2 = db1_s2[6] ? $unsigned(-db1_s2) : $unsigned(db1_s2);
    reg [5:0] r00_s3,r10_s3,g00_s3,g10_s3,b00_s3,b10_s3;
    reg [15:0] wy_s3;
    reg valid_s3;

    // S4: horizontal blend, held at eight fractional bits.
    // 63*256 = 16128, which fits signed 15-bit Q8.
    reg signed [14:0] rh0_s4,rh1_s4,gh0_s4,gh1_s4,bh0_s4,bh1_s4;

    // S4: one horizontal Q8 addition with conditional carry-in.
    // Old negative rule: base - hi - (fraction != 0).
    // Since ~hi = -hi - 1 (two's complement), rewrite it as:
    //    base + (~hi) + (fraction == 0).
    // Positive rule is simply base + hi.
    // This keeps the exact floor-toward-minus-infinity rounding of the
    // original sign-safe implementation while eliminating the cascaded
    // +round, conditional negate, and +base adders.
    // All arithmetic is 15-bit modular; for valid RGB565 endpoints and
    // wx in [0,65535], the true horizontal result fits signed Q8[14:0].
    function [14:0] horiz_blend_q8;
        input [5:0]  base_component;
        input [22:0] abs_product;
        input        negative;
        reg [14:0]  base_q8;
        reg [14:0]  hi_adjusted;
        reg         carry_in;
        begin
            base_q8   = {1'b0,base_component,8'b0};
            hi_adjusted = abs_product[22:8] ^ {15{negative}};
            carry_in = negative && !(|abs_product[7:0]);
            horiz_blend_q8 = base_q8 + hi_adjusted + carry_in;
        end
    endfunction

    reg [15:0] wy_s4;
    reg valid_s4;

    // S5: vertical differences only.
    reg signed [14:0] rbase_s5,gbase_s5,bbase_s5;
    reg signed [14:0] dr_s5,dg_s5,db_s5;
    reg [15:0] wy_s5;
    reg valid_s5;

    // S6: signed 15x17 multiplies (vertical weight remains unsigned Q0.16).
    reg signed [31:0] rprod_s6,gprod_s6,bprod_s6;
    reg signed [14:0] rbase_s6,gbase_s6,bbase_s6;
    reg valid_s6;

    // S7: break the DSP-output -> final-adder timing path.
    // Take only the low 16 bits of the arithmetic right shift:
    // (product >>> 16)[15:0] == product[31:16], including negatives.
    // Final sum is 16-bit modular arithmetic, identical to old truncation.
    reg signed [15:0] rdelta_s7, gdelta_s7, bdelta_s7;
    reg signed [14:0] rbase_s7,  gbase_s7,  bbase_s7;
    reg valid_s7;

    // S8: do the small final add and RGB565 packing one clock later.
    wire signed [15:0] rfinal_q8 =
        $signed({rbase_s7[14],rbase_s7}) + $signed(rdelta_s7);
    wire signed [15:0] gfinal_q8 =
        $signed({gbase_s7[14],gbase_s7}) + $signed(gdelta_s7);
    wire signed [15:0] bfinal_q8 =
        $signed({bbase_s7[14],bbase_s7}) + $signed(bdelta_s7);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0; valid_s2 <= 1'b0; valid_s3 <= 1'b0;
            valid_s4 <= 1'b0; valid_s5 <= 1'b0; valid_s6 <= 1'b0;
            valid_s7 <= 1'b0;
            valid_out <= 1'b0; pixel_out <= 16'd0;
            r00_s1<=0;r01_s1<=0;r10_s1<=0;r11_s1<=0;
            g00_s1<=0;g01_s1<=0;g10_s1<=0;g11_s1<=0;
            b00_s1<=0;b01_s1<=0;b10_s1<=0;b11_s1<=0;
            wx_s1<=0;wy_s1<=0;
            dr0_s2<=0;dr1_s2<=0;dg0_s2<=0;dg1_s2<=0;db0_s2<=0;db1_s2<=0;
            r00_s2<=0;r10_s2<=0;g00_s2<=0;g10_s2<=0;b00_s2<=0;b10_s2<=0;
            wx_s2<=0;wy_s2<=0;
            r0prod_s3<=0;r1prod_s3<=0;g0prod_s3<=0;g1prod_s3<=0;
            b0prod_s3<=0;b1prod_s3<=0;
            r0neg_s3<=0;r1neg_s3<=0;g0neg_s3<=0;g1neg_s3<=0;b0neg_s3<=0;b1neg_s3<=0;
            r00_s3<=0;r10_s3<=0;g00_s3<=0;g10_s3<=0;b00_s3<=0;b10_s3<=0;
            wy_s3<=0;
            rh0_s4<=0;rh1_s4<=0;gh0_s4<=0;gh1_s4<=0;bh0_s4<=0;bh1_s4<=0;
            wy_s4<=0;
            rbase_s5<=0;gbase_s5<=0;bbase_s5<=0;
            dr_s5<=0;dg_s5<=0;db_s5<=0;wy_s5<=0;
            rprod_s6<=0;gprod_s6<=0;bprod_s6<=0;
            rbase_s6<=0;gbase_s6<=0;bbase_s6<=0;
            rdelta_s7<=0;gdelta_s7<=0;bdelta_s7<=0;
            rbase_s7<=0;gbase_s7<=0;bbase_s7<=0;
        end else begin
            valid_s1 <= valid_in;
            if (valid_in) begin
                r00_s1<={1'b0,p00[15:11]};r01_s1<={1'b0,p01[15:11]};
                r10_s1<={1'b0,p10[15:11]};r11_s1<={1'b0,p11[15:11]};
                g00_s1<=p00[10:5];g01_s1<=p01[10:5];
                g10_s1<=p10[10:5];g11_s1<=p11[10:5];
                b00_s1<={1'b0,p00[4:0]};b01_s1<={1'b0,p01[4:0]};
                b10_s1<={1'b0,p10[4:0]};b11_s1<={1'b0,p11[4:0]};
                wx_s1<=wx;wy_s1<=wy;
            end

            valid_s2 <= valid_s1;
            if (valid_s1) begin
                dr0_s2<=$signed({1'b0,r01_s1})-$signed({1'b0,r00_s1});
                dr1_s2<=$signed({1'b0,r11_s1})-$signed({1'b0,r10_s1});
                dg0_s2<=$signed({1'b0,g01_s1})-$signed({1'b0,g00_s1});
                dg1_s2<=$signed({1'b0,g11_s1})-$signed({1'b0,g10_s1});
                db0_s2<=$signed({1'b0,b01_s1})-$signed({1'b0,b00_s1});
                db1_s2<=$signed({1'b0,b11_s1})-$signed({1'b0,b10_s1});
                r00_s2<=r00_s1;r10_s2<=r10_s1;
                g00_s2<=g00_s1;g10_s2<=g10_s1;
                b00_s2<=b00_s1;b10_s2<=b10_s1;
                wx_s2<=wx_s1;wy_s2<=wy_s1;
            end

            valid_s3 <= valid_s2;
            if (valid_s2) begin
                r0prod_s3<=r0abs_s2*wx_s2;
                r1prod_s3<=r1abs_s2*wx_s2;
                g0prod_s3<=g0abs_s2*wx_s2;
                g1prod_s3<=g1abs_s2*wx_s2;
                b0prod_s3<=b0abs_s2*wx_s2;
                b1prod_s3<=b1abs_s2*wx_s2;
                r0neg_s3<=dr0_s2[6];r1neg_s3<=dr1_s2[6];
                g0neg_s3<=dg0_s2[6];g1neg_s3<=dg1_s2[6];
                b0neg_s3<=db0_s2[6];b1neg_s3<=db1_s2[6];
                r00_s3<=r00_s2;r10_s3<=r10_s2;
                g00_s3<=g00_s2;g10_s3<=g10_s2;
                b00_s3<=b00_s2;b10_s3<=b10_s2;
                wy_s3<=wy_s2;
            end

            valid_s4 <= valid_s3;
            if (valid_s3) begin
                rh0_s4 <= $signed(horiz_blend_q8(r00_s3, r0prod_s3, r0neg_s3));
                rh1_s4 <= $signed(horiz_blend_q8(r10_s3, r1prod_s3, r1neg_s3));
                gh0_s4 <= $signed(horiz_blend_q8(g00_s3, g0prod_s3, g0neg_s3));
                gh1_s4 <= $signed(horiz_blend_q8(g10_s3, g1prod_s3, g1neg_s3));
                bh0_s4 <= $signed(horiz_blend_q8(b00_s3, b0prod_s3, b0neg_s3));
                bh1_s4 <= $signed(horiz_blend_q8(b10_s3, b1prod_s3, b1neg_s3));
                wy_s4<=wy_s3;
            end

            valid_s5 <= valid_s4;
            if (valid_s4) begin
                rbase_s5<=rh0_s4;gbase_s5<=gh0_s4;bbase_s5<=bh0_s4;
                dr_s5<=rh1_s4-rh0_s4;
                dg_s5<=gh1_s4-gh0_s4;
                db_s5<=bh1_s4-bh0_s4;
                wy_s5<=wy_s4;
            end

            valid_s6 <= valid_s5;
            if (valid_s5) begin
                rprod_s6<=dr_s5*$signed({1'b0,wy_s5});
                gprod_s6<=dg_s5*$signed({1'b0,wy_s5});
                bprod_s6<=db_s5*$signed({1'b0,wy_s5});
                rbase_s6<=rbase_s5;gbase_s6<=gbase_s5;bbase_s6<=bbase_s5;
            end

            // Separate registered transfer from the DSP outputs.
            valid_s7 <= valid_s6;
            if (valid_s6) begin
                rdelta_s7 <= rprod_s6[31:16];
                gdelta_s7 <= gprod_s6[31:16];
                bdelta_s7 <= bprod_s6[31:16];
                rbase_s7 <= rbase_s6;
                gbase_s7 <= gbase_s6;
                bbase_s7 <= bbase_s6;
            end

            valid_out <= valid_s7;
            if (valid_s7) begin
                // Convex interpolation of valid RGB565 components is bounded;
                // no saturation logic is necessary for 0<=wx,wy<=65535.
                pixel_out <= {rfinal_q8[12:8],gfinal_q8[13:8],bfinal_q8[12:8]};
            end
        end
    end
endmodule
