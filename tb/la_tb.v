/**
 * @file    la_tb.v
 * @brief   Self-checking testbench for the logic analyzer top level.
 *
 * Test Coverage:
 *   1. Idle          : line parks high and nothing is sent before an ARM
 *   2. Bad opcode    : unknown command bytes are ignored
 *   3. Capture       : DEPTH bytes come back in order and then the line goes idle
 *   4. SET_DIV       : sample spacing and capture duration follow the 24-bit divider
 *   5. Level trigger : capture is held off until the masked channels match
 *   6. Edge trigger  : an already-true condition does not fire without a transition
 *   7. Re-arm        : a second capture runs cleanly after the first
 */

`timescale 1ns / 1ps

module la_tb();

    // ----- DUT Configuration -----
    localparam integer CLK_FREQ  = 50_000_000;
    localparam integer BAUD_RATE = 1_000_000;
    localparam integer CHANNELS  = 8;
    localparam integer DEPTH     = 16; // Shortened from 8192 to keep the simulation quick
    localparam integer DIV_WIDTH = 24;

    localparam integer CLK_PERIOD = 1_000_000_000 / CLK_FREQ;  //   20 ns
    localparam integer BIT_PERIOD = 1_000_000_000 / BAUD_RATE; // 1000 ns

    localparam integer FIRST_BYTE_TIMEOUT_NS = 400 * BIT_PERIOD;
    localparam integer BYTE_TIMEOUT_NS       =  40 * BIT_PERIOD;
    localparam integer TIMEOUT_NS            = 5_000_000; // Whole-simulation watchdog

    localparam integer MSG_W = 8 * 64; // Width of the message argument on check tasks

    // ----- Host Command Opcodes -----
    localparam [7:0] CMD_SET_DIV   = 8'h01;
    localparam [7:0] CMD_SET_MASK  = 8'h02;
    localparam [7:0] CMD_SET_VALUE = 8'h03;
    localparam [7:0] CMD_SET_MODE  = 8'h04;
    localparam [7:0] CMD_ARM       = 8'h05;

    // ----- DUT I/O -----
    reg  r_clk     = 1'b0;
    reg  r_rst_n   = 1'b0;

    reg  r_uart_rx = 1'b1; // Idle high
    wire w_uart_tx;

    reg                r_use_counter = 1'b0;  // Flag for probe stimulus type
    reg [CHANNELS-1:0] r_counter     = 0;     // Every sample is unique and capture order can be checked
    reg [CHANNELS-1:0] r_probe_hold  = 0;     // Held value

    wire [CHANNELS-1:0] w_probe = r_use_counter ? r_counter : r_probe_hold;

    always @(posedge r_clk) r_counter <= r_counter + 1;

    // ----- DUT -----
    la_top #(
        .CLK_FREQ   (CLK_FREQ),
        .BAUD_RATE  (BAUD_RATE),
        .CHANNELS   (CHANNELS),
        .DEPTH      (DEPTH),
        .DIV_WIDTH  (DIV_WIDTH)
    ) dut (
        .i_clk      (r_clk),
        .i_rst_n    (r_rst_n),
        .i_probe    (w_probe),
        .i_uart_rx  (r_uart_rx),
        .o_uart_tx  (w_uart_tx)
    );

    always #(CLK_PERIOD/2) r_clk = ~r_clk; // Generate clock

    // ----- Scoreboard -----
    integer errors = 0;
    integer checks = 0;

    reg [7:0] r_buf [0:DEPTH-1]; // Bytes recovered from the last dump
    reg       r_dump_ok;         // All DEPTH bytes arrived
    integer   r_stop_errs;       // Framing errors seen during the last dump
    integer   r_dump_start;      // $time of the first start bit of the last dump
    integer   r_edge_time;       // $time of the most recent start bit

    task check(input i_cond, input [MSG_W-1:0] i_msg);
    begin
        checks = checks + 1;
        if (i_cond === 1'b1)
        begin
            $display("[%0t] PASS: %0s", $time, i_msg);
        end
        else
        begin
            errors = errors + 1;
            $display("[%0t] FAIL: %0s", $time, i_msg);
        end
    end
    endtask

    // ----- Host UART Model -----

    // Drive one byte into the DUT's receiver: start, 8 data LSB-first, stop
    task uart_send_byte(input [7:0] i_data);
        integer i;
    begin
        r_uart_rx = 1'b0;             // Start
        #(BIT_PERIOD);
        for (i = 0; i < 8; i = i + 1)
        begin
            r_uart_rx = i_data[i];
            #(BIT_PERIOD);
        end
        r_uart_rx = 1'b1;             // Stop
        #(BIT_PERIOD);
        #(2 * BIT_PERIOD);            // Idle gap so the receiver settles
    end
    endtask

    // Wait for a start bit and sample the frame at the center
    task uart_recv_byte(input integer i_timeout_ns, output [7:0] o_data, output o_ok);
        integer   i;
        integer   waited;
        reg [7:0] data;
    begin
        waited = 0;
        while (w_uart_tx !== 1'b0 && waited < i_timeout_ns)
        begin
            #(CLK_PERIOD);
            waited = waited + CLK_PERIOD;
        end

        if (w_uart_tx !== 1'b0)
        begin
            o_ok   = 1'b0;
            o_data = 8'h00;
        end
        else
        begin
            r_edge_time = $time;
            #(BIT_PERIOD + BIT_PERIOD/2); // Skip the start bit, land mid bit 0
            for (i = 0; i < 8; i = i + 1)
            begin
                data[i] = w_uart_tx;
                #(BIT_PERIOD);
            end

            if (w_uart_tx !== 1'b1)       // Mid stop bit, must be high
            begin
                r_stop_errs = r_stop_errs + 1;
            end
            #(BIT_PERIOD/2);              // Out to the end of the frame

            o_data = data;
            o_ok   = 1'b1;
        end
    end
    endtask

    // ----- Host Commands -----
    task cmd_set_div(input [DIV_WIDTH-1:0] i_div);
    begin
        uart_send_byte(CMD_SET_DIV);
        uart_send_byte(i_div[7:0]);   // LSB first
        uart_send_byte(i_div[15:8]);
        uart_send_byte(i_div[23:16]);
    end
    endtask

    task cmd_set_mask(input [7:0] i_mask);
    begin
        uart_send_byte(CMD_SET_MASK);
        uart_send_byte(i_mask);
    end
    endtask

    task cmd_set_value(input [7:0] i_value);
    begin
        uart_send_byte(CMD_SET_VALUE);
        uart_send_byte(i_value);
    end
    endtask

    task cmd_set_mode(input i_edge_mode);
    begin
        uart_send_byte(CMD_SET_MODE);
        uart_send_byte({7'd0, i_edge_mode});
    end
    endtask

    task cmd_arm();
    begin
        uart_send_byte(CMD_ARM);
    end
    endtask

    // ----- Capture Helpers -----

    // Collect a whole capture into r_buf
    task dump_capture();
        integer   i;
        reg [7:0] data;
        reg       ok;
    begin
        r_dump_ok    = 1'b1;
        r_stop_errs  = 0;
        r_dump_start = 0;

        for (i = 0; i < DEPTH; i = i + 1)
        begin
            uart_recv_byte((i == 0) ? FIRST_BYTE_TIMEOUT_NS : BYTE_TIMEOUT_NS, data, ok);
            if (!ok)
            begin
                r_dump_ok = 1'b0;
                $display("[%0t] INFO: dump stalled after %0d of %0d bytes", $time, i, DEPTH);
                i = DEPTH; // Abandon the rest of the frame
            end
            else
            begin
                r_buf[i] = data;
                if (i == 0) r_dump_start = r_edge_time;
            end
        end
    end
    endtask

    // A trigger-immediately capture can start streaming before the ARM frame has finished on the wire (UART idle delay)
    task arm_and_dump();
    begin
        fork
            cmd_arm();      // Stimulus thread
            dump_capture(); // Monitoring thread
        join                // Wait until all threads have finished
    end
    endtask

    // Fail if the DUT transmits anything over the next i_clocks
    task expect_idle(input integer i_clocks, input [MSG_W-1:0] i_msg);
        integer i;
        reg     quiet;
    begin
        quiet = 1'b1;
        for (i = 0; i < i_clocks; i = i + 1)
        begin
            @(posedge r_clk);
            if (w_uart_tx !== 1'b1) quiet = 1'b0;
        end
        check(quiet, i_msg);
    end
    endtask

    // Check if captured samples came back in the right order with the right spacing
    function all_stepped(input [7:0] i_step);
        integer k;
        reg     ok;
    begin
        ok = 1'b1;
        for (k = 1; k < DEPTH; k = k + 1)
        begin
            if (r_buf[k] !== ((r_buf[k-1] + i_step) & 8'hFF))
            begin
                if (ok) $display("[%0t] INFO: sample %0d is %02h, expected %02h",
                                 $time, k, r_buf[k], (r_buf[k-1] + i_step) & 8'hFF);
                ok = 1'b0;
            end
        end
        all_stepped = ok;
    end
    endfunction

    // Probes held steady, so every stored sample should read back the same
    function all_equal(input [7:0] i_value);
        integer k;
        reg     ok;
    begin
        ok = 1'b1;
        for (k = 0; k < DEPTH; k = k + 1)
        begin
            if (r_buf[k] !== i_value)
            begin
                if (ok) $display("[%0t] INFO: sample %0d is %02h, expected %02h",
                                 $time, k, r_buf[k], i_value);
                ok = 1'b0;
            end
        end
        all_equal = ok;
    end
    endfunction

    // ----- Main Test Sequence -----
    integer t_arm;
    integer expected_ns;

    initial
    begin
        // Reset
        r_rst_n = 1'b0;
        repeat (4) @(posedge r_clk);
        r_rst_n = 1'b1;
        repeat (4) @(posedge r_clk);

        // Test 1: idle behaviour
        check(w_uart_tx === 1'b1, "tx line idles high out of reset");
        expect_idle(200, "no capture is sent before ARM");

        // Test 2: unknown opcodes are ignored
        uart_send_byte(8'hAA);
        uart_send_byte(8'h55);
        expect_idle(200, "unknown opcodes produce no response");

        // Test 3: capture at full rate
        r_use_counter = 1'b1; // Probes count up once per clock
        cmd_set_div(24'd0);   // Sample every clock
        cmd_set_mask(8'h00);  // Match everything, trigger immediately
        cmd_set_value(8'h00);
        cmd_set_mode(1'b0);   // Level

        arm_and_dump();
        check(r_dump_ok,         "capture 1: all DEPTH bytes received");
        check(r_stop_errs == 0,  "capture 1: no framing errors");
        check(all_stepped(8'd1), "capture 1: samples are consecutive at div=0");
        expect_idle(200,         "capture 1: line goes idle after exactly DEPTH bytes");

        // Test 4: sample rate divider
        cmd_set_div(24'd259);
        t_arm = $time;
        arm_and_dump();

        expected_ns = DEPTH * 260 * CLK_PERIOD;
        check(r_dump_ok,         "capture 2: all DEPTH bytes received");
        check(all_stepped(8'd4), "capture 2: samples are 260 clocks apart at div=259");
        check((r_dump_start - t_arm) >= expected_ns &&
              (r_dump_start - t_arm) <= expected_ns + 20 * BIT_PERIOD,
                                 "capture 2: capture duration matches the divider");

        // Test 5: level trigger holds off until the condition is true
        r_use_counter = 1'b0;
        r_probe_hold  = 8'h00; // Channel 0 low, so the trigger cannot match
        cmd_set_div(24'd0);
        cmd_set_mask(8'h01);
        cmd_set_value(8'h01);
        cmd_set_mode(1'b0); // Level
        cmd_arm();

        expect_idle(500, "level trigger: armed but idle while ch0 is low");

        r_probe_hold = 8'h01; // Condition becomes true
        dump_capture();
        check(r_dump_ok,          "level trigger: capture ran once ch0 went high");
        check(all_equal(8'h01),   "level trigger: every sample reads 01");

        // Test 6: edge trigger needs a transition into the condition
        // Probes are left at 01, which already matches, so edge mode must wait
        cmd_set_mode(1'b1); // Edge
        cmd_arm();

        expect_idle(500, "edge trigger: no fire on an already-matching probe");

        r_probe_hold = 8'h00; // Non-matching sample primes the trigger
        repeat (20) @(posedge r_clk);
        r_probe_hold = 8'h01; // Transition into the condition fires it

        dump_capture();
        check(r_dump_ok,        "edge trigger: capture ran on the transition");
        check(all_equal(8'h01), "edge trigger: every sample reads 01");

        // Test 7: re-arm after a completed capture
        r_probe_hold = 8'h5A;
        cmd_set_mask(8'h00); // Trigger-immediately
        cmd_set_mode(1'b0);

        arm_and_dump();
        check(r_dump_ok,        "re-arm: third capture completed");
        check(r_stop_errs == 0, "re-arm: no framing errors");
        check(all_equal(8'h5A), "re-arm: buffer holds the new probe value");

        // ----- Summary -----
        $display("----------------------------------------");
        $display("Checks run : %0d", checks);
        $display("Failures   : %0d", errors);
        if (errors == 0) $display("RESULT: ALL TESTS PASSED");
        else             $display("RESULT: %0d TEST(S) FAILED", errors);
        $display("----------------------------------------");
        $finish;
    end

    // ----- Watchdog Safety Timeout -----
    initial
    begin
        #(TIMEOUT_NS);
        $display("[%0t] FAIL: testbench hung", $time);
        $finish;
    end

    // ----- Waveform Output -----
    initial
    begin
        $dumpfile("la_tb.vcd");
        $dumpvars(0, la_tb); // Log entire hierarchy
    end

endmodule
