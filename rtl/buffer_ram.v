/**
 * @file    buffer_ram.v
 * @brief   Capture buffer for the logic analyzer (simple dual port, single clock).
 *
 * Write side  : append-only, driven by the sampler strobe.
 * Read side   : plain addressed read for the UART dump FSM.
 */

module buffer_ram #(
    parameter integer CHANNELS   = 8,            // Bits per sample (one per probe channel)
    parameter integer DEPTH      = 8192,         // Samples held (multiple of one of M9K's size configurations to avoid waste)
    parameter integer ADDR_WIDTH = $clog2(DEPTH) // Bits needed to index DEPTH samples (13 for the default 8192-deep buffer)
)(
    input  wire                  i_clk,
    input  wire                  i_rst_n,   // Active-low synchronous reset

    // ----- Write port (capture) -----
    input  wire                  i_clear,   // Rewind the write pointer, arm for a new capture
    input  wire                  i_wr_en,   // Tie to the sampler's o_sample_valid
    input  wire [CHANNELS-1:0]   i_wr_data, // Tie to the sampler's o_sample
    output wire                  o_full,    // Ignore writes once buffer holds DEPTH samples
    output wire [ADDR_WIDTH:0]   o_count,   // Samples currently stored

    // ----- Read port (UART dump) -----
    input  wire [ADDR_WIDTH-1:0] i_rd_addr, // Address driven by the UART dump FSM to read back a stored sample
    output reg  [CHANNELS-1:0]   o_rd_data  // Valid 1 clock after i_rd_addr
);

    // Map memory array into physical M9K block RAM components on FPGA
    (* ramstyle = "M9K" *) reg [CHANNELS-1:0] r_mem [0:DEPTH-1]; 

    reg [ADDR_WIDTH-1:0] r_wr_addr;
    reg [ADDR_WIDTH:0]   r_count;

    assign o_full  = (r_count == DEPTH);
    assign o_count = r_count;

    wire w_do_write = i_wr_en && !o_full;

    // ----- Pointer -----
    always @(posedge i_clk)
    begin
        if (!i_rst_n || i_clear)
        begin
            r_wr_addr <= 0;
            r_count   <= 0;
        end
        else if (w_do_write)
        begin
            r_wr_addr <= r_wr_addr + 1;
            r_count   <= r_count + 1;
        end
    end

    // ----- Memory -----
    always @(posedge i_clk)
    begin
        if (w_do_write)
        begin
            r_mem[r_wr_addr] <= i_wr_data;
        end
        o_rd_data <= r_mem[i_rd_addr]; // Unconditional read to match Quartus Prime's M9K template for synthesis
    end

endmodule