/** 
 * @file    baud_gen.v
 * @brief   Parameterized baud rate generator for a UART core.
 *
 * Produces 2 single-cycle tick strobes:
 *   - o_baud_tick     : 1 pulse per bit period     (used by the transmitter)
 *   - o_baud_tick_16x : 16 pulses per bit period   (used by the receiving oversampler)
 */

 module baud_gen #(
    parameter integer CLK_FREQ  = 50_000_000, // Input clock frequency in Hz
    parameter integer BAUD_RATE = 115_200    // Target baud rate
 ) (
    input wire i_clk,
    input wire i_rst_n,        // Active-low synchronous reset
    output reg o_baud_tick,    // 1x bit-rate strobe (TX)
    output reg o_baud_tick_16x // 16x bit-rate strobe (RX)
 );

    // ----- Divisor Math -----

    // Use rounded integer division to minimize clock drift
    localparam integer DIV_1X  = (CLK_FREQ + BAUD_RATE/2) / BAUD_RATE;
    localparam integer DIV_16X = (CLK_FREQ + (BAUD_RATE*16)/2) / (BAUD_RATE*16);

    // Terminal count values
    localparam integer WRAP_1X  = DIV_1X - 1;  // e.g. 433 @ 50MHz/115200
    localparam integer WRAP_16X = DIV_16X - 1; // e.g.  26 @ 50MHz/(115200*16)

    // Compute minimum bit width needed to hold the largest count value
    localparam integer W1  = $clog2(DIV_1X);
    localparam integer W16 = $clog2(DIV_16X);

    reg [W1-1:0]  r_count_1x;
    reg [W16-1:0] r_count_16x;

    // ----- 1x Baud Tick Generator -----
    always @(posedge i_clk) 
    begin
        if(!i_rst_n) 
        begin
            r_count_1x  <= 0;
            o_baud_tick <= 1'b0;
        end
        else if (r_count_1x == WRAP_1X) 
        begin
            r_count_1x  <= 0;
            o_baud_tick <= 1'b1;
        end
        else 
        begin
            r_count_1x  <= r_count_1x + 1;
            o_baud_tick <= 1'b0;
        end
    end

    // ----- 16x Baud Tick -----
    always @(posedge i_clk)
    begin
        if (!i_rst_n)
        begin
            r_count_16x     <= 0;
            o_baud_tick_16x <= 1'b0;
        end
        else if (r_count_16x == WRAP_16X)
        begin
            r_count_16x     <= 0;
            o_baud_tick_16x <= 1'b1;
        end
        else
        begin
            r_count_16x     <= r_count_16x + 1;
            o_baud_tick_16x <= 1'b0;
        end
    end

 endmodule
 