/**
 * @file    uart.v
 * @brief   Top-level UART core.
 *
 * Ties together the baud generator, transmitter, and receiver into a single
 * reusable IP block. All timing is derived from CLK_FREQ / BAUD_RATE, so the
 * core drops into any clock domain by overriding the parameters.
 */

module uart #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter integer DATA_BITS = 8 
)(
    input wire                  i_clk,
    input wire                  i_rst_n,    // Active-low synchronous reset
 
    // TX interface
    input wire                  i_tx_start, // Pulse to send data
    input wire [DATA_BITS-1:0]  i_tx_data,
    output wire                 o_tx_busy,
    output wire                 o_tx_done,
    output wire                 o_tx,       // Serial line out

    // RX interface    
    input wire                  i_rx,       // Serial line in
    output wire [DATA_BITS-1:0] o_rx_data,
    output wire                 o_rx_valid,
    output wire                 o_frame_err
);

    // ----- Shared Baud Timing -----
    wire w_baud_tick, w_baud_tick_16x;

    baud_gen #(
        .CLK_FREQ        (CLK_FREQ),
        .BAUD_RATE       (BAUD_RATE)
    ) u_baud (
        .i_clk           (i_clk),
        .i_rst_n         (i_rst_n),
        .o_baud_tick     (w_baud_tick),
        .o_baud_tick_16x (w_baud_tick_16x)
    );

    // ----- Transmitter -----
    uart_tx #(
        .DATA_BITS   (DATA_BITS)
    ) u_tx (
        .i_clk       (i_clk),
        .i_rst_n     (i_rst_n),
        .i_baud_tick (w_baud_tick),
        .i_tx_start  (i_tx_start),
        .i_tx_data   (i_tx_data),
        .o_tx        (o_tx),
        .o_tx_busy   (o_tx_busy),
        .o_tx_done   (o_tx_done)
    );

    // ----- Receiver -----
    uart_rx #(
        .DATA_BITS       (DATA_BITS)
    ) u_rx (
        .i_clk           (i_clk),
        .i_rst_n         (i_rst_n),
        .i_baud_tick_16x (w_baud_tick_16x),
        .i_rx            (i_rx),
        .o_rx_data       (o_rx_data),
        .o_rx_valid      (o_rx_valid),
        .o_frame_err     (o_frame_err)
    );

endmodule