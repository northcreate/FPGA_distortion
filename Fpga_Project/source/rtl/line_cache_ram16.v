`timescale 1ns / 1ps
// 1024 x 16-bit synchronous simple-dual-port line memory.
// One port writes a pixel lane while the other port performs a synchronous read.
// For 1920-pixel RGB565 lines, only addresses 0..959 are used (960 pixels/lane).
// The even/odd pixel split permits simultaneous access to adjacent source pixels
// without duplicating the full 32-bit row RAM.
module line_cache_ram16 (
    input             clk,
    input             wr_en,
    input      [9:0]  wr_addr,
    input      [15:0] wr_data,
    input      [9:0]  rd_addr,
    output reg [15:0] rd_data
);
    reg [15:0] mem [0:1023];

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
endmodule
