#!/usr/bin/env python3
"""
la_host.py - host-side capture and plotting for the DE10-Lite logic analyzer.

Speaks the byte protocol implemented in la_top.v:

    0x01 <lo> <hi>   SET_DIV    sample rate = CLK_FREQ / (div + 1)
    0x02 <mask>      SET_MASK   1 = channel participates in the trigger
    0x03 <value>     SET_VALUE  expected level on masked channels
    0x04 <mode>      SET_MODE   bit0: 0 = level, 1 = edge
    0x05             ARM        DEPTH bytes stream back once the buffer fills

Examples
--------
List serial ports:
    python la_host.py --list

Free-running capture at 1 MS/s (mask 0 matches everything, fires immediately):
    python la_host.py -p COM3 --rate 1e6

Trigger on a falling edge on channel 0 (i.e. catch a UART start bit):
    python la_host.py -p COM3 --rate 2e6 --mask 0x01 --value 0x00 --edge

Only plot the channels you care about, and save the result:
    python la_host.py -p COM3 --channels 0,1,4 --save capture.png --csv capture.csv
"""

import argparse
import sys
import time

import numpy as np
import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt

# ----- Must match the parameters in la_top.v -----
CLK_FREQ = 50_000_000
DEPTH = 8192
CHANNELS = 8
DIV_MAX = 0xFFFF 
CMD_SET_DIV = 0x01
CMD_SET_MASK = 0x02
CMD_SET_VALUE = 0x03
CMD_SET_MODE = 0x04
CMD_ARM = 0x05


def rate_to_div(rate):
    """Nearest divider for a requested sample rate, plus the rate actually achieved."""
    div = round(CLK_FREQ / rate) - 1
    div = max(0, min(DIV_MAX, div))
    return div, CLK_FREQ / (div + 1)


def ping(ser):
    ser.reset_input_buffer()
    ser.write(bytes([CMD_PING]))
    ser.flush()
    reply = ser.read(1)
    if reply != PING_REPLY:
        raise RuntimeError(
            f"no response to ping (got {reply!r}). Check the port, the baud rate, "
            f"and that TX/RX are crossed between the FPGA and the USB-serial module."
        )


def configure(ser, div, mask, value, edge):
    ser.write(bytes([CMD_SET_DIV, div & 0xFF, (div >> 8) & 0xFF]))
    ser.write(bytes([CMD_SET_MASK, mask & 0xFF]))
    ser.write(bytes([CMD_SET_VALUE, value & 0xFF]))
    ser.write(bytes([CMD_SET_MODE, 1 if edge else 0]))
    ser.flush()


def capture(ser, depth, trigger_wait):
    """Arm, wait for the trigger, then read back the buffer."""
    ser.reset_input_buffer()
    ser.write(bytes([CMD_ARM]))
    ser.flush()

    buf = bytearray()
    deadline = time.time() + trigger_wait
    streaming = False

    while len(buf) < depth and time.time() < deadline:
        chunk = ser.read(depth - len(buf))
        if chunk:
            if not streaming:
                streaming = True
                print("triggered, receiving...")
            buf.extend(chunk)
            # Once bytes are flowing, a gap means the dump stalled, not that
            # we are still waiting on the trigger.
            deadline = time.time() + 2.0

    if not buf:
        raise RuntimeError(
            f"trigger never fired within {trigger_wait:.0f} s. "
            f"Check the mask/value, or run with --mask 0 to capture immediately."
        )
    if len(buf) < depth:
        print(f"warning: got {len(buf)} of {depth} bytes; plotting the partial capture",
              file=sys.stderr)
    return bytes(buf)


def unpack(raw):
    """Expand raw capture bytes into a per-channel waveform array.

    Each byte in `raw` is one sample with all CHANNELS probe bits packed
    together (bit 0 = channel 0, per the WIDTH-bit i_probe convention on the
    FPGA side). Returns a (n_samples, CHANNELS) uint8 array where column i
    is channel i's logic level over time.
    """
    arr_1d = np.frombuffer(raw, dtype=np.uint8)
    arr_2d = arr_1d[:, None]
    return np.unpackbits(arr_2d, axis=1, bitorder="little")


def time_scale(t_max):
    """Pick a display unit for a time span given in seconds.

    Given the largest time value to be plotted, returns (factor, unit_label)
    such that t_max * factor is a human-readable number in that unit
    (e.g. 2.5e-3 -> (1e3, "ms"), displayed as 2.5 ms).
    """
    if t_max < 1e-6:
        return 1e9, "ns"
    if t_max < 1e-3:
        return 1e6, "us"
    if t_max < 1.0:
        return 1e3, "ms"
    return 1.0, "s"


def plot(bits, rate, channels, title, save):
    n = bits.shape[0]
    t = np.arange(n) / rate
    scale, unit = time_scale(t[-1] if n else 1.0)
    ts = t * scale

    fig, ax = plt.subplots(figsize=(13, 1.0 + 0.7 * len(channels)))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for row, ch in enumerate(channels):
        y = bits[:, ch] * 0.75 + row
        # 'post' is the correct step style: the level holds until the next sample.
        ax.step(ts, y, where="post", linewidth=1.2, color=colors[ch % len(colors)])
        ax.axhline(row, color="0.9", linewidth=0.5, zorder=0)

    ax.set_yticks([r + 0.375 for r in range(len(channels))])
    ax.set_yticklabels([f"D{ch}" for ch in channels])
    ax.set_ylim(-0.4, len(channels) - 0.1)
    ax.set_xlim(0, ts[-1] if n > 1 else 1)
    ax.set_xlabel(f"time ({unit})   -   t=0 is the trigger point")
    ax.set_title(title)
    ax.grid(axis="x", alpha=0.3)
    fig.tight_layout()

    if save:
        fig.savefig(save, dpi=150)
        print(f"saved {save}")
    plt.show()


def summarize(bits, rate, channels):
    print(f"\n{'ch':>3}  {'edges':>7}  {'high %':>7}  {'min pulse':>12}")
    for ch in channels:
        col = bits[:, ch]
        edges = int(np.count_nonzero(np.diff(col)))
        high = 100.0 * col.mean()
        if edges:
            runs = np.diff(np.flatnonzero(np.diff(col)))
            shortest = (runs.min() if runs.size else 1) / rate
            # A minimum run of one sample means you are at or past the rate limit.
            note = f"{shortest * 1e9:,.0f} ns" + (" (!)" if shortest * rate <= 1 else "")
        else:
            note = "-"
        print(f"{ch:>3}  {edges:>7}  {high:>6.1f}%  {note:>12}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list", action="store_true", help="list serial ports and exit")
    p.add_argument("-p", "--port", help="serial port, e.g. COM3 or /dev/ttyUSB0")
    p.add_argument("-b", "--baud", type=int, default=1_000_000,
                   help="must match BAUD_RATE in la_top.v (default: 1000000)")
    p.add_argument("--rate", type=float, default=1e6, help="sample rate in Hz (default: 1e6)")
    p.add_argument("--mask", type=lambda x: int(x, 0), default=0x00,
                   help="trigger mask, 1 = channel participates (default: 0 = fire immediately)")
    p.add_argument("--value", type=lambda x: int(x, 0), default=0x00,
                   help="trigger value on masked channels (default: 0)")
    p.add_argument("--edge", action="store_true",
                   help="require a non-matching sample first, so the trigger lands on the edge")
    p.add_argument("--channels", default=",".join(str(i) for i in range(CHANNELS)),
                   help="channels to plot, e.g. 0,1,4 (default: all)")
    p.add_argument("--depth", type=int, default=DEPTH, help="must match DEPTH in la_top.v")
    p.add_argument("--wait", type=float, default=10.0,
                   help="seconds to wait for the trigger (default: 10)")
    p.add_argument("--save", help="write the plot to this PNG")
    p.add_argument("--csv", help="write the raw samples to this CSV")
    p.add_argument("--no-plot", action="store_true", help="capture only, skip the window")
    args = p.parse_args()

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("no serial ports found")
        for port in ports:
            print(f"{port.device:<20} {port.description}")
        return

    if not args.port:
        p.error("--port is required (use --list to find it)")

    channels = [int(c) for c in args.channels.split(",") if c.strip() != ""]
    if any(ch < 0 or ch >= CHANNELS for ch in channels):
        p.error(f"channels must be 0..{CHANNELS - 1}")

    div, actual = rate_to_div(args.rate)
    window = args.depth / actual
    print(f"sample rate : {actual:,.0f} Hz (div={div})")
    print(f"window      : {window * 1e3:.3f} ms for {args.depth} samples")
    print(f"trigger     : mask=0x{args.mask:02X} value=0x{args.value:02X} "
          f"mode={'edge' if args.edge else 'level'}")

    # A read timeout well under the trigger wait keeps the loop responsive.
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(0.05)  # let the USB-serial bridge settle after opening
        ping(ser)
        print("link ok")
        configure(ser, div, args.mask, args.value, args.edge)
        print("armed, waiting for trigger...")
        raw = capture(ser, args.depth, args.wait)

    bits = unpack(raw)
    print(f"captured {bits.shape[0]} samples")
    summarize(bits, actual, channels)

    if args.csv:
        header = "time_s," + ",".join(f"d{ch}" for ch in channels)
        t = (np.arange(bits.shape[0]) / actual)[:, None]
        np.savetxt(args.csv, np.hstack([t, bits[:, channels]]),
                   delimiter=",", header=header, comments="", fmt="%.9g")
        print(f"saved {args.csv}")

    if not args.no_plot:
        title = (f"DE10-Lite logic analyzer  -  {actual:,.0f} S/s  -  "
                 f"{bits.shape[0]} samples  -  {window * 1e3:.2f} ms")
        plot(bits, actual, channels, title, args.save)


if __name__ == "__main__":
    main()