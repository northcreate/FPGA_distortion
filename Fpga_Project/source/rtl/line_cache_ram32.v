`timescale 1ns / 1ps
// Phase 2 row-cache storage: 32-bit x 1024-word, synchronous simple dual-port RAM.
// One port writes while the other port reads. Both ports use the pixel clock.
// This module is intentionally kept separate so PDS can infer embedded RAM.
module line_cache_ram32 (
    input             clk,
    input             wr_en,
    input      [9:0]  wr_addr,
    input      [31:0] wr_data,
    input      [9:0]  rd_addr,
    output reg [31:0] rd_data
);
    reg [31:0] mem [0:1023];

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end
endmodule
