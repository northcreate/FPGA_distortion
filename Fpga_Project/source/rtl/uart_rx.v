`timescale 1ns/1ps
// 8N1, LSB first. rx_valid/error are one-clock pulses. RX is asynchronous.
module uart_rx #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD = 115200
)(
    input wire clk, input wire rst_n, input wire rx_i,
    output reg [7:0] rx_data,
    output reg rx_valid, output reg framing_error
);
    function integer clog2;
        input integer value;
        integer v;
        begin v=value-1;clog2=0;while(v>0)begin clog2=clog2+1;v=v>>1;end
        if(clog2<1)clog2=1;end
    endfunction
    localparam integer BIT_CYCLES=(CLK_HZ+BAUD/2)/BAUD;
    localparam integer HALF_CYCLES=BIT_CYCLES/2;
    localparam integer CW=clog2(BIT_CYCLES);
    localparam [2:0] IDLE=0, START=1, DATA=2, STOP=3, WAIT_HIGH=4;
    (* ASYNC_REG="TRUE" *) reg rx_meta,rx_sync;
    reg [2:0] state,bit_index;
    reg [CW-1:0] ticks;
    reg [7:0] shift;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin rx_meta<=1'b1;rx_sync<=1'b1;end
        else begin rx_meta<=rx_i;rx_sync<=rx_meta;end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin
            state<=IDLE;ticks<=0;bit_index<=0;shift<=0;
            rx_data<=0;rx_valid<=0;framing_error<=0;
        end else begin
            rx_valid<=0;framing_error<=0;
            case(state)
            IDLE: begin
                ticks<=0;
                if(!rx_sync)begin ticks<=HALF_CYCLES-1;state<=START;end
            end
            START: if(ticks!=0)ticks<=ticks-1'b1;
                else if(rx_sync)state<=IDLE; // Reject a short false start.
                else begin ticks<=BIT_CYCLES-1;bit_index<=0;state<=DATA;end
            DATA: if(ticks!=0)ticks<=ticks-1'b1;
                else begin
                    shift[bit_index]<=rx_sync;ticks<=BIT_CYCLES-1;
                    if(bit_index==7)state<=STOP;
                    else bit_index<=bit_index+1'b1;
                end
            STOP: if(ticks!=0)ticks<=ticks-1'b1;
                else if(rx_sync)begin rx_data<=shift;rx_valid<=1;state<=IDLE;end
                else begin framing_error<=1;state<=WAIT_HIGH;end
            WAIT_HIGH: if(rx_sync)state<=IDLE; // A break is not a stream of bytes.
            default: state<=IDLE;
            endcase
        end
    end
endmodule
