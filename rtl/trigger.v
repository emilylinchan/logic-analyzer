/**
 * @file    trigger.v
 * @brief   Mask/value trigger comparator for the logic analyzer.
 *
 * A sample matches when every channel selected by i_mask equals the
 * corresponding bit of i_value.
 *
 * Modes:
 *   - Level (i_edge_mode = 0): fire as soon as the condition holds, even if it
 *     was already true when armed.
 *   - Edge (i_edge_mode = 1): require at least one non-matching sample first,
 *     so the trigger lands on the transition into the condition.
 */

module trigger #(
    parameter integer CHANNELS = 8
)(
    input  wire                i_clk,
    input  wire                i_rst_n,        // Active-low synchronous reset
    input  wire                i_arm,          // Pulse to reset trigger state
    input  wire                i_sample_valid, // Sampler strobe
    input  wire [CHANNELS-1:0] i_sample,       // Sampler data
    input  wire [CHANNELS-1:0] i_mask,         // 1 = channel participates
    input  wire [CHANNELS-1:0] i_value,        // Expected level on masked channels
    input  wire                i_edge_mode,    // 0 = level, 1 = edge
    output reg                 o_trigger       // 1-cycle pulse
);

    wire w_match = (((i_sample ^ i_value) & i_mask) == 0);

    reg r_fired;   // Latches after firing so only one pulse is issued per arm
    reg r_primed;  // Edge mode: set once a non-matching sample has been seen

    always @(posedge i_clk)
    begin
        if (!i_rst_n)
        begin
            o_trigger <= 1'b0;
            r_fired   <= 1'b0;
            r_primed  <= 1'b0;
        end
        else
        begin
            o_trigger <= 1'b0; // Overridden below when the trigger fires

            if (i_arm)
            begin
                r_fired  <= 1'b0;
                r_primed <= ~i_edge_mode; // Level mode (0) starts primed so a match fires immediately
                                          // Edge mode (1) starts unprimed until a mismatch is seen 
            end
            else if (i_sample_valid && !r_fired)
            begin
                if (!w_match)
                begin
                    r_primed <= 1'b1;
                end
                else if (r_primed)
                begin
                    o_trigger <= 1'b1;
                    r_fired   <= 1'b1;
                end
            end
        end
    end

endmodule