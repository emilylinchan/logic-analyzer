#!/usr/bin/env python3
"""
la_host.py - host-side capture and plotting for the DE10-Lite logic analyzer.

Speaks the byte protocol implemented in la_top.v:

    0x01 <b0> <b1> <b2> SET_DIV : sample rate = CLK_FREQ / (div + 1), div LSB-first
    0x02 <mask>      SET_MASK   : 1 = channel participates in the trigger
    0x03 <value>     SET_VALUE  : expected level on masked channels
    0x04 <mode>      SET_MODE   : bit0, 0 = level trigger, 1 = edge trigger
    0x05             ARM        : begin capture; DEPTH bytes stream back when full

Examples
--------
Free-running capture at 1 MS/s (mask 0 matches everything, fires immediately):
    python la_host.py -p COM3 --rate 1e6

Trigger on a falling edge on channel 0 at 2 MS/s (i.e. catch a UART start bit):
    python la_host.py -p COM3 --rate 2e6 --mask 0x01 --value 0x00 --edge
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt

# ----- Configuration -----
CLK_FREQ = 50_000_000
DEPTH    = 8192
CHANNELS = 8
DIV_MAX  = 0xFFFFFF     # 3 byte max value for FPGA register
TRIGGER_MARGIN_S = 10.0 # Extra time beyond the fill window to wait for the trigger to fire
OUTPUT_DIR = Path("output") # Destination folder for saved PNG/CSV files


CMD_SET_DIV   = 0x01
CMD_SET_MASK  = 0x02
CMD_SET_VALUE = 0x03
CMD_SET_MODE  = 0x04
CMD_ARM       = 0x05

# ----- Helper Functions -----
def rate_to_div(rate):
    """Returns the nearest dividier for a requested sample rate and the actual rate achieved."""
    div = round(CLK_FREQ / rate) - 1
    div = max(0, min(DIV_MAX, div))
    return div, CLK_FREQ / (div + 1)

def configure(ser, div, mask, value, edge):
    """Send configuration over UART to FPGA."""
    ser.write(bytes([CMD_SET_DIV, div & 0xFF, (div >> 8) & 0xFF, (div >> 16) & 0xFF]))
    ser.write(bytes([CMD_SET_MASK, mask & 0xFF]))
    ser.write(bytes([CMD_SET_VALUE, value & 0xFF]))
    ser.write(bytes([CMD_SET_MODE, 1 if edge else 0]))
    ser.flush()
    
def capture(ser, depth, rate):
    """Arm, wait for the trigger, then read back the buffer."""
    ser.reset_input_buffer()
    ser.write(bytes([CMD_ARM]))
    ser.flush()

    buf = bytearray()
    fill_time = depth / rate # Time for the FPGA to fill its buffer before it streams back
    trigger_wait = fill_time + TRIGGER_MARGIN_S # Budget for trigger and acquisition
    deadline = time.time() + trigger_wait

    # Data capture logic
    while len(buf) < depth and time.time() < deadline:
        chunk = ser.read(depth - len(buf))
        if chunk:
            buf.extend(chunk)
            deadline = time.time() + 2 # Max gap allowed between UART bytes during dump
       
    # Error handling - trigger timeout    
    if not buf:
        raise RuntimeError(
            f"Trigger never fired within {trigger_wait:.0f} s. "
            f"Check the mask/value, or run with --mask 0 to capture immediately."
        )
        
    # Error handling - partial data
    if len(buf) < depth:
        print(f"WARNING: only got {len(buf)} of {depth} bytes.", file=sys.stderr)
    
    return bytes(buf)

def unpack(raw):
    """Returns a (n_samples, CHANNELS) uint8 array where column i is channel i's logic level over time."""
    arr_1d = np.frombuffer(raw, dtype=np.uint8) # Raw byte samples
    arr_2d = arr_1d[:, None]                    # Add a new axis so each sample is in its own row
    return np.unpackbits(arr_2d, axis=1, bitorder="little")

def time_scale(t_max):
    """Return a readable display unit for a time span given in seconds."""
    if t_max < 1e-6:
        return 1e9, "ns"
    if t_max < 1e-3:
        return 1e6, "us"
    if t_max < 1.0:
        return 1e3, "ms"
    return 1.0, "s"

def plot(bits, rate, channels, title, save):
    """Render a step-style waveform plot of the captured samples."""
    n = bits.shape[0]        # Samples (y-axis)
    t = np.arange(n) / rate  # Time    (x-axis)
    scale, unit = time_scale(t[-1] if n else 1.0)
    ts = t * scale
    
    fig, ax = plt.subplots(figsize=(13, 1.0 + 0.7 * len(channels)))
    colours = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    
    for row, ch in enumerate(channels):
        y = bits[:, ch] * 0.75 + row # Scale signal amplitude and offset for plot
        ax.step(ts, y, where="post", linewidth=1.2, color=colours[ch % len(colours)])
        ax.axhline(row, color="0.9", linewidth=0.5, zorder=0)
        
    ax.set_yticks([r + 0.375 for r in range(len(channels))])
    ax.set_yticklabels([f"D{ch}" for ch in channels])
    ax.set_ylim(-0.4, len(channels) - 0.1)
    ax.set_xlim(0, ts[-1] if n > 1 else 1)
    ax.set_xlabel(f"Time ({unit})")
    ax.set_title(title)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()
    
    if save:
        fig.savefig(save, dpi=150)
        print(f"Saved {save}")
    plt.show()
    
# ----- Main -----
def main():
    parser = argparse.ArgumentParser(description=__doc__, 
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("-p", "--port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("-b", "--baud", type=int, default=1_000_000,
                        help="must match BAUD_RATE in la_top.v (default: 1_000_000)")
    parser.add_argument("--rate", type=float, default=1e6, help="sample rate in Hz (default: 1e6)")
    parser.add_argument("--mask", type=lambda x: int(x, 0), default=0x00,
                        help="trigger mask, 1 = channel participates (default: 0 = fire immediately)")
    parser.add_argument("--value", type=lambda x: int(x, 0), default=0x00,
                        help="trigger value on masked channels (default: 0)")
    parser.add_argument("--edge", action="store_true",
                        help="require a non-matching sample first, so the trigger lands on the edge")
    parser.add_argument("--channels", default=",".join(str(i) for i in range(CHANNELS)),
                        help="channels to plot, e.g. 0,1,4 (default: all)")
    parser.add_argument("--depth", type=int, default=DEPTH, help="must match DEPTH in la_top.v")
    parser.add_argument("--save", help="write the plot to this PNG (saved in the output folder)")
    parser.add_argument("--csv", help="write the raw samples to this CSV (saved in the output folder)")
    
    args = parser.parse_args()
    
    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found")
        for port in ports:
            print(f"{port.device:<20} {port.description}")
        return
    
    if not args.port:
        parser.error("--port is requred (use --list to find it)")
        
    channels = [int(c) for c in args.channels.split(",") if c.strip() != ""]
    if any(ch < 0 or ch >= CHANNELS for ch in channels):
        parser.error(f"Channels must be 0-{CHANNELS-1}")

    # Resolve output paths inside the output folder, creating it if needed
    if args.save or args.csv:
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    save_path = OUTPUT_DIR / args.save if args.save else None
    csv_path  = OUTPUT_DIR / args.csv  if args.csv  else None

    div, actual = rate_to_div(args.rate)
    window = args.depth / actual
    print(f"sample rate : {actual:,.0f} Hz (div={div})")
    print(f"window      : {window * 1e3:.3f} ms for {args.depth} samples")
    print(f"trigger     : mask=0x{args.mask:02X} value=0x{args.value:02X} "
          f"mode={'edge' if args.edge else 'level'}")
    
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(0.05) # Let USB-serial bridge settle
        configure(ser, div, args.mask, args.value, args.edge)
        print("Armed, sampling data...")
        raw = capture(ser, args.depth, actual)
        
    bits = unpack(raw)
    print(f"Captured {bits.shape[0]} samples")
    
    if csv_path:
        header = "time_s," + ",".join(f"d{ch}" for ch in channels)
        t = (np.arange(bits.shape[0]) / actual)[:, None]
        np.savetxt(csv_path, np.hstack([t, bits[:, channels]]),
                   delimiter=",", header=header, comments="", fmt="%.9g")
        print(f"Saved {csv_path}")
    
    title = (f"DE10-Lite logic analyzer  -  {actual:,.0f} S/s  -  "
                f"{bits.shape[0]} samples  -  {window * 1e3:.2f} ms")
    plot(bits, actual, channels, title, save_path)
    
if __name__=="__main__":
    main()

        