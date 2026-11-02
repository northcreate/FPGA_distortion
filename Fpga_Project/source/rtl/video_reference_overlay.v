`timescale 1ns/1ps
// Fully synthetic HDMI reference lines. No BRAM, DDR, frame-store, or modulo.
// Input raster coordinates are the same sync_vg coordinates used by OSD.
// Output mask is delayed to align with the existing 32-clock video path;
// final HDMI output flip-flops remain unchanged.
module video_reference_overlay #(
    parameter integer TOTAL_LATENCY = 32
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        de_in,
    input  wire [11:0] x_in,
    input  wire [11:0] y_in,
    input  wire        enable_in,
    input  wire [1:0]  mode_in, // 0:cross, 1:border, 2:grid
    output wire        cover_out
);
    // 1920x1080, active-area coordinates x=[0..1919], y=[0..1079].
    wire cross_hit = (x_in == 12'd960) || (y_in == 12'd540);
    wire box_hit =
        (((x_in == 12'd50) || (x_in == 12'd1869)) &&
         (y_in >= 12'd50) && (y_in <= 12'd1029)) ||
        (((y_in == 12'd50) || (y_in == 12'd1029)) &&
         (x_in >= 12'd50) && (x_in <= 12'd1869));
    // Constant equality comparators, not x % 100 / y % 100 (no dividers).
    wire grid_x = (x_in == 12'd100)  || (x_in == 12'd200)  ||
                  (x_in == 12'd300)  || (x_in == 12'd400)  ||
                  (x_in == 12'd500)  || (x_in == 12'd600)  ||
                  (x_in == 12'd700)  || (x_in == 12'd800)  ||
                  (x_in == 12'd900)  || (x_in == 12'd1000) ||
                  (x_in == 12'd1100) || (x_in == 12'd1200) ||
                  (x_in == 12'd1300) || (x_in == 12'd1400) ||
                  (x_in == 12'd1500) || (x_in == 12'd1600) ||
                  (x_in == 12'd1700) || (x_in == 12'd1800) ||
                  (x_in == 12'd1900);
    wire grid_y = (y_in == 12'd100)  || (y_in == 12'd200) ||
                  (y_in == 12'd300)  || (y_in == 12'd400) ||
                  (y_in == 12'd500)  || (y_in == 12'd600) ||
                  (y_in == 12'd700)  || (y_in == 12'd800) ||
                  (y_in == 12'd900)  || (y_in == 12'd1000);
    wire grid_hit = grid_x || grid_y;

    wire hit_now = de_in && enable_in &&
                   ((mode_in == 2'd0 && cross_hit) ||
                    (mode_in == 2'd1 && box_hit)   ||
                    (mode_in == 2'd2 && grid_hit));
    reg [TOTAL_LATENCY-1:0] cover_pipe;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cover_pipe <= {TOTAL_LATENCY{1'b0}};
        else
            cover_pipe <= {cover_pipe[TOTAL_LATENCY-2:0], hit_now};
    end
    assign cover_out = cover_pipe[TOTAL_LATENCY-1];
endmodule
