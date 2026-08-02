/**
 * @file   uart_tx.v
 * @brief  UART Transmitter
 *
 * Frame: 1 start bit (0), DATA_BITS data (LSB first), 1 stop bit (1).
 * Shifts output on each i_baud_tick pulse.
 *
 * Protocol:
 *   - Pulse i_tx_start for 1 cycle with valid i_tx_data (latched immediately).
 *   - o_tx_busy stays high during transmission; i_tx_start is ignored while busy.
 *   - o_tx_done pulses for 1 cycle when the stop bit completes.
 */

module uart_tx #(
    parameter integer DATA_BITS = 8
)(
    input wire                 i_clk,
    input wire                 i_rst_n,     // Active-low synchronous reset
    input wire                 i_baud_tick, // 1x bit-rate strobe
    input wire                 i_tx_start,  // Pulse to begin a transfer
    input wire [DATA_BITS-1:0] i_tx_data,   // Data to send (latched on start)
    output reg                 o_tx,        // Serial line (idle high)
    output reg                 o_tx_busy,   // Transmission in progress
    output reg                 o_tx_done    // Frame complete pulse
);

    // ----- State Encoding -----
    localparam S_IDLE  = 2'b00;
    localparam S_START = 2'b01;
    localparam S_DATA  = 2'b10;
    localparam S_STOP  = 2'b11;

    reg [1:0]                   r_state;
    reg [DATA_BITS-1:0]         r_shift;   // Shift register, LSB shifted out first
    reg [$clog2(DATA_BITS)-1:0] r_bit_idx; // Counts data bits sent

    always @(posedge i_clk)
    begin
        if(!i_rst_n)
        begin
            r_state   <= S_IDLE;
            o_tx      <= 1'b1;
            o_tx_busy <= 1'b0;
            o_tx_done <= 1'b0;
            r_shift   <= 0;
            r_bit_idx <= 0;
        end
        else
        begin
            o_tx_done <= 1'b0; // Overridden below when frame ends

            // ----- FSM -----
            case (r_state)

                // Wait for i_tx_start pulse to latch data and begin a frame
                S_IDLE:
                begin
                    o_tx      <= 1'b1;
                    o_tx_busy <= 1'b0;
                    if (i_tx_start)
                    begin
                        r_shift   <= i_tx_data; 
                        r_bit_idx <= 0;
                        o_tx_busy <= 1'b1;
                        r_state   <= S_START;
                    end
                end

                // Drive the start bit for 1 baud tick
                S_START:
                begin
                    if (i_baud_tick)
                    begin
                        o_tx    <= 1'b0; 
                        r_state <= S_DATA;
                    end
                end

                // Shift out each data bit, LSB first, 1 per baud tick
                S_DATA:
                begin
                    if (i_baud_tick)
                    begin
                        o_tx    <= r_shift[0];
                        r_shift <= r_shift >> 1;
                        if (r_bit_idx == DATA_BITS-1)
                        begin
                            r_state <= S_STOP;
                        end
                        else
                        begin
                            r_bit_idx <= r_bit_idx + 1;
                        end
                    end
                end

                // Drive the stop bit, signal completion, and return to IDLE
                S_STOP:
                begin
                    if (i_baud_tick)
                    begin
                        o_tx      <= 1'b1; 
                        o_tx_done <= 1'b1;
                        r_state   <= S_IDLE;
                    end
                end

                default: r_state <= S_IDLE;
            endcase
        end
    end

endmodule