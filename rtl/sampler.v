/**
 * @file    sampler.v
 * @brief   Sample-rate divider and capture strobe for the logic analyzer.
 *
 * Latches the synchronized probe channels once every (i_div + 1) clocks and
 * emits a 1-cycle strobe alongside the data.
 *
 * Sample rate = CLK_FREQ / (i_div + 1)
 *
 * Rate table @ 50 MHz:
 *      i_div |  rate
 *      ------+-----------
 *          0 |  50   MS/s   (strobe every cycle)
 *          1 |  25   MS/s
 *          4 |  10   MS/s
 *         49 |   1   MS/s
 *        499 | 100   kS/s
 *       4999 |  10   kS/s
 *      49999 |   1   kS/s
 */

module sampler #(
    parameter integer CHANNELS  = 8, // Number of probe channels
    parameter integer DIV_WIDTH = 24 // Divider width (24 bits supports down to ~3 S/s)
)(
    input  wire                 i_clk,
    input  wire                 i_rst_n,        // Active-low synchronous reset
    input  wire                 i_en,           // Active-high enable
    input  wire [DIV_WIDTH-1:0] i_div,          // Sample every (i_div + 1) clocks
    input  wire [CHANNELS-1:0]  i_probes,       // Synchronized channels from sync_in
    output reg  [CHANNELS-1:0]  o_sample,       // Captured channel state
    output reg                  o_sample_valid  // 1-cycle strobe for a valid sample
);

    reg [DIV_WIDTH-1:0] r_count;

    always @(posedge i_clk)
    begin
        if (!i_rst_n)
        begin
            r_count        <= 0;
            o_sample       <= 0;
            o_sample_valid <= 1'b0;
        end
        else if (!i_en)
        begin
            r_count        <= 0;   
            o_sample_valid <= 1'b0;
        end
        else if (r_count == i_div)
        begin
            r_count        <= 0;
            o_sample       <= i_probes;
            o_sample_valid <= 1'b1;
        end
        else
        begin
            r_count        <= r_count + 1'b1;
            o_sample_valid <= 1'b0;
        end
    end

endmodule