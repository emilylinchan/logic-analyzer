/**
 * @file    sync_in.v
 * @brief   Asynchronous input synchronizer for logic analyzer probe channels.
 *
 * Passes each channel through a 2-stage flip-flop synchronizer to resolve
 * metastability from CDC before the samples reach the capture logic.
 */

module sync_in #(
    parameter integer CHANNELS = 8       // Number of probe channels
)(
    input  wire                i_clk,
    input  wire                i_rst_n,  // Active-low synchronous reset
    input  wire [CHANNELS-1:0] i_async,  // Raw GPIO probe inputs
    output wire [CHANNELS-1:0] o_sync    // Synchronized samples, i_clk domain
);
    
    // CDC Synchronizer Attributes:
    // - SYNCHRONIZER_IDENTIFICATION: Enables MTBF calculation and tight placement of registers in Quartus
    // - preserve: Prevents synthesis from merging registers or retiming logic
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
    (* preserve *) reg [CHANNELS-1:0] r_meta, r_stable;

    always @(posedge i_clk)
    begin
        if (!i_rst_n)
        begin
            r_meta   <= 0;
            r_stable <= 0;
        end
        else
        begin
            r_meta   <= i_async;
            r_stable <= r_meta;
        end
    end

    assign o_sync = r_stable;

endmodule