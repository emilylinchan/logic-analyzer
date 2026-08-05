/**
 * @file    la_top.v
 * @brief   Top-level logic analyzer for the DE10-Lite.
 *
 * Flow:  GPIO probes -> sync_in -> sampler -> trigger -> buffer_ram -> UART -> PC
 *
 * The host configures the capture over UART RX, arms it, and then reads back
 * DEPTH bytes over UART TX.
 *
 * ----- Host command set (bytes, LSB-first arguments) -----
 *   0x01 <lo> <hi>   SET_DIV    : sample rate = CLK_FREQ / (div + 1)
 *   0x02 <mask>      SET_MASK   : 1 = channel participates in the trigger
 *   0x03 <value>     SET_VALUE  : expected level on masked channels
 *   0x04 <mode>      SET_MODE   : bit0, 0 = level trigger, 1 = edge trigger
 *   0x05             ARM        : begin capture; DEPTH bytes stream back when full
 */

module la_top #(
    parameter integer CLK_FREQ   = 50_000_000,
    parameter integer BAUD_RATE  = 1_000_000,
    parameter integer CHANNELS   = 8,         // Probe channels
    parameter integer DEPTH      = 8192,      // Samples per capture
    parameter integer DIV_WIDTH  = 16,
    parameter integer ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                i_clk,     // MAX10_CLK1_50
    input  wire                i_rst_n,   // KEY[0], active low
    input  wire [CHANNELS-1:0] i_probe,   // GPIO probe channels (3.3 V)
    input  wire                i_uart_rx, // From USB-serial module
    output wire                o_uart_tx  // To USB-serial module
);

    // ----- Command Opcodes -----
    localparam [7:0] CMD_SET_DIV   = 8'h01;
    localparam [7:0] CMD_SET_MASK  = 8'h02;
    localparam [7:0] CMD_SET_VALUE = 8'h03;
    localparam [7:0] CMD_SET_MODE  = 8'h04;
    localparam [7:0] CMD_ARM       = 8'h05;

    // ----- Capture FSM states -----
    localparam [2:0] S_IDLE    = 3'd0;
    localparam [2:0] S_TRIG    = 3'd1; // Sampling, waiting for the trigger
    localparam [2:0] S_CAP     = 3'd2; // Sampling and writing to the buffer
    localparam [2:0] S_ADDR    = 3'd3; // Present read address
    localparam [2:0] S_SEND    = 3'd4; // Issue the UART start pulse
    localparam [2:0] S_TXWAIT  = 3'd5; // Wait for the transmitter to pick it up
    localparam [2:0] S_TXDONE  = 3'd6; // Wait for the byte to clear, then advance

    // ----- Reset Synchronizer -----
    (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
    (* preserve *) reg r_rst_meta, r_rst_stable;

    always @(posedge i_clk)
    begin
        r_rst_meta   <= i_rst_n;
        r_rst_stable <= r_rst_meta;
    end

    // ----- Interconnect -----
    wire [CHANNELS-1:0] w_probes;
    wire [CHANNELS-1:0] w_sample;
    wire                w_sample_valid;
    wire                w_trigger;
    wire                w_full;
    wire [ADDR_WIDTH:0] w_count;
    wire [CHANNELS-1:0] w_rd_data;
    wire [7:0]          w_rx_data;
    wire                w_rx_valid;
    wire                w_tx_busy;

    // ----- Configuration Registers -----
    reg [DIV_WIDTH-1:0] r_div;
    reg [CHANNELS-1:0]  r_mask;
    reg [CHANNELS-1:0]  r_value;
    reg                 r_edge_mode;

    // ----- Command Decoder -----
    reg [7:0] r_opcode;
    reg [1:0] r_arg_count; // Argument bytes still expected
    reg       r_arm_req;   // 1-cycle request from the host

    always @(posedge i_clk)
    begin
        if (!r_rst_stable)
        begin
            r_div       <= 16'd49; // Power-up default: 1 MS/s at 50 MHz
            r_mask      <= 0;      // Power-up default: match everything/ trigger immediately
            r_value     <= 0;
            r_edge_mode <= 1'b0;
            r_opcode    <= 3'd0;
            r_arg_count <= 0;
            r_arm_req   <= 1'b0;
        end
        else
        begin
            r_arm_req  <= 1'b0; // Overridden below on ARM

            if (w_rx_valid)
            begin
                if (r_arg_count == 2'd0)
                begin
                    // Opcode byte
                    r_opcode <= w_rx_data;
                    case (w_rx_data)
                        CMD_SET_DIV:  r_arg_count <= 2'd2;
                        CMD_SET_MASK,
                        CMD_SET_VALUE,
                        CMD_SET_MODE: r_arg_count <= 2'd1;
                        CMD_ARM:      r_arm_req   <= 1'b1;
                        default: ; // Unknown opcode, ignore
                    endcase
                end
                else
                begin
                    // Argument byte
                    case (r_opcode)
                        CMD_SET_DIV:
                            if (r_arg_count == 2'd2) r_div[7:0]  <= w_rx_data; // Low byte 
                            else                     r_div[15:8] <= w_rx_data; // High byte
                        CMD_SET_MASK:                r_mask      <= w_rx_data;
                        CMD_SET_VALUE:               r_value     <= w_rx_data;
                        CMD_SET_MODE:                r_edge_mode <= w_rx_data[0];
                        default: ;
                    endcase
                    r_arg_count <= r_arg_count - 1;
                end
            end
        end
    end

    // ----- Capture / Dump FSM -----
    reg [2:0]            r_state;
    reg [ADDR_WIDTH-1:0] r_rd_addr;
    reg                  r_clear;
    reg                  r_arm;
    reg                  r_tx_start;
    reg [7:0]            r_tx_data;

    wire w_capturing = (r_state == S_TRIG) || (r_state == S_CAP);

    always @(posedge i_clk)
    begin
        if (!r_rst_stable)
        begin
            r_state    <= S_IDLE;
            r_rd_addr  <= 0;
            r_clear    <= 1'b0;
            r_arm      <= 1'b0;
            r_tx_start <= 1'b0;
            r_tx_data  <= 0;
        end
        else
        begin
            r_clear    <= 1'b0;
            r_arm      <= 1'b0;
            r_tx_start <= 1'b0;

            case (r_state)

                // Wait for the host's ARM command before capturing
                S_IDLE:
                begin
                    if (r_arm_req)
                    begin
                        r_clear <= 1'b1;  // Rewind the buffer write pointer
                        r_arm   <= 1'b1;  // Reset trigger state
                        r_state <= S_TRIG;
                    end
                end

                // Sampler runs so the trigger can watch the line, but nothing is stored yet
                S_TRIG:
                begin
                    if (w_trigger)
                    begin
                        r_state <= S_CAP;
                    end
                end

                // Capture samples until the buffer fills, then rewind the read address
                S_CAP:
                begin
                    if (w_full)
                    begin
                        r_rd_addr <= 0;
                        r_state   <= S_ADDR;
                    end
                end

                // Read address from buffer RAM (1 clock data valid latency)
                S_ADDR:
                begin
                    r_state <= S_SEND;
                end

                // Latch valid read data and pulse the UART start strobe to transmit the sample byte
                S_SEND:
                begin
                    r_tx_data  <= w_rd_data;
                    r_tx_start <= 1'b1;
                    r_state    <= S_TXWAIT;
                end

                // Wait for the transmitter to acknowledge the start pulse by raising busy, confirming the byte was accepted
                S_TXWAIT:
                begin
                    if (w_tx_busy)
                    begin
                        r_state <= S_TXDONE;
                    end
                end

                // Wait for busy to drop when the byte has finished shifting out
                S_TXDONE:
                begin
                    if (!w_tx_busy)
                    begin
                        // Last sample
                        if (r_rd_addr == DEPTH-1)
                        begin
                            r_state <= S_IDLE;
                        end
                        // Advance to next sample
                        else
                        begin
                            r_rd_addr <= r_rd_addr + 1;
                            r_state   <= S_ADDR;
                        end
                    end
                end

                default: r_state <= S_IDLE;
            endcase
        end
    end

    // ----- Probe Synchronizer -----
    sync_in #(
        .CHANNELS (CHANNELS)
    ) u_sync (
        .i_clk    (i_clk),
        .i_rst_n  (r_rst_stable),
        .i_async  (i_probe),
        .o_sync   (w_probes)
    );

    // ----- Sample Rate Divider -----
    sampler #(
        .CHANNELS       (CHANNELS),
        .DIV_WIDTH      (DIV_WIDTH)
    ) u_sampler (
        .i_clk          (i_clk),
        .i_rst_n        (r_rst_stable),
        .i_en           (w_capturing),
        .i_div          (r_div),
        .i_probes       (w_probes),
        .o_sample       (w_sample),
        .o_sample_valid (w_sample_valid)
    );

    // ----- Trigger -----
    trigger #(
        .CHANNELS       (CHANNELS)
    ) u_trigger (
        .i_clk          (i_clk),
        .i_rst_n        (r_rst_stable),
        .i_arm          (r_arm),
        .i_sample_valid (w_sample_valid),
        .i_sample       (w_sample),
        .i_mask         (r_mask),
        .i_value        (r_value),
        .i_edge_mode    (r_edge_mode),
        .o_trigger      (w_trigger)
    );

    // ----- Capture Buffer -----
    buffer_ram #(
        .CHANNELS   (CHANNELS),
        .DEPTH   (DEPTH)
    ) u_buf (
        .i_clk     (i_clk),
        .i_rst_n   (r_rst_stable),
        .i_clear   (r_clear),
        .i_wr_en   (w_sample_valid && (r_state == S_CAP)),
        .i_wr_data (w_sample),
        .o_full    (w_full),
        .o_count   (w_count),
        .i_rd_addr (r_rd_addr),
        .o_rd_data (w_rd_data)
    );

    // ----- UART Core -----
    uart #(
        .CLK_FREQ    (CLK_FREQ),
        .BAUD_RATE   (BAUD_RATE),
        .DATA_BITS   (8)
    ) u_uart (
        .i_clk       (i_clk),
        .i_rst_n     (r_rst_stable),
        .i_tx_start  (r_tx_start),
        .i_tx_data   (r_tx_data),
        .o_tx_busy   (w_tx_busy),
        .o_tx_done   (),            // Unused
        .o_tx        (o_uart_tx),
        .i_rx        (i_uart_rx),
        .o_rx_data   (w_rx_data),
        .o_rx_valid  (w_rx_valid),
        .o_frame_err ()             // Unused
    );

endmodule