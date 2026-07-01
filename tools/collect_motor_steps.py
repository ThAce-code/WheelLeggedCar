#!/usr/bin/env python3
"""
Collect CYT4BB7 motor step-response data from the debug UART.

The firmware is expected to stream VOFA JustFloat frames with 8 float channels:
time_ms, mode, target_motor_rpm, left_motor_rpm, right_motor_rpm, left_duty, right_duty,
feedback_online.

Example:
    python tools/collect_motor_steps.py --port COM6 --sequence 300:3,500:3,800:3
"""

import argparse
import csv
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None


VOFA_TAIL = b"\x00\x00\x80\x7f"
FLOAT_COUNT = 8
PAYLOAD_LEN = FLOAT_COUNT * 4
FRAME_LEN = PAYLOAD_LEN + len(VOFA_TAIL)
CSV_FIELDS = [
    "pc_time_s",
    "sample_index",
    "command_duty",
    "time_ms",
    "mode",
    "target_motor_rpm",
    "left_motor_rpm",
    "right_motor_rpm",
    "left_duty",
    "right_duty",
    "feedback_online",
]


def parse_sequence(text):
    steps = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" in item:
            duty_text, duration_text = item.split(":", 1)
            duration_s = float(duration_text)
        else:
            duty_text = item
            duration_s = 3.0
        duty = int(duty_text)
        if duration_s <= 0.0:
            raise ValueError("step duration must be positive")
        steps.append((duty, duration_s))
    if not steps:
        raise ValueError("sequence is empty")
    return steps


def write_command(port, command):
    data = (command + "\n").encode("ascii")
    port.write(data)
    port.flush()


def pop_frames(rx_buffer):
    frames = []
    while True:
        tail_index = rx_buffer.find(VOFA_TAIL)
        if tail_index < 0:
            if len(rx_buffer) > (FRAME_LEN * 4):
                del rx_buffer[:-FRAME_LEN]
            break

        if tail_index >= PAYLOAD_LEN:
            payload_start = tail_index - PAYLOAD_LEN
            payload = bytes(rx_buffer[payload_start:tail_index])
            try:
                frames.append(struct.unpack("<8f", payload))
            except struct.error:
                pass
            del rx_buffer[:tail_index + len(VOFA_TAIL)]
        else:
            del rx_buffer[:tail_index + len(VOFA_TAIL)]
    return frames


def collect_step(port, writer, rx_buffer, duty, duration_s, settle_s, sample_index):
    write_command(port, f"D,{duty}")
    step_start = time.monotonic()
    keep_from = step_start + settle_s
    end_time = step_start + duration_s

    print(f"step D,{duty} for {duration_s:.2f}s")

    while time.monotonic() < end_time:
        chunk = port.read(512)
        if chunk:
            rx_buffer.extend(chunk)
            for frame in pop_frames(rx_buffer):
                now = time.monotonic()
                if now < keep_from:
                    continue
                writer.writerow(
                    {
                        "pc_time_s": f"{now:.6f}",
                        "sample_index": sample_index,
                        "command_duty": duty,
                        "time_ms": f"{frame[0]:.3f}",
                        "mode": f"{frame[1]:.3f}",
                        "target_motor_rpm": f"{frame[2]:.3f}",
                        "left_motor_rpm": f"{frame[3]:.3f}",
                        "right_motor_rpm": f"{frame[4]:.3f}",
                        "left_duty": f"{frame[5]:.3f}",
                        "right_duty": f"{frame[6]:.3f}",
                        "feedback_online": f"{frame[7]:.3f}",
                    }
                )
                sample_index += 1
    return sample_index


def main():
    parser = argparse.ArgumentParser(description="Collect motor open-loop step-response CSV data.")
    parser.add_argument("--port", default="COM6", help="Debug UART port, default: COM6")
    parser.add_argument("--baud", type=int, default=460800, help="Debug UART baudrate, default: 460800")
    parser.add_argument(
        "--sequence",
        default="300:3,500:3,800:3,1000:3,1500:3,2000:3",
        help="Comma-separated duty:duration_s list, default: 300:3,500:3,800:3,1000:3,1500:3,2000:3",
    )
    parser.add_argument("--settle", type=float, default=0.2, help="Seconds to skip after each command")
    parser.add_argument("--between-stop", type=float, default=1.0, help="STOP seconds between steps")
    parser.add_argument("--out", default="data/motor_step.csv", help="Output CSV path")
    args = parser.parse_args()

    if serial is None:
        print("pyserial is not installed. Run: python -m pip install pyserial", file=sys.stderr)
        return 2

    steps = parse_sequence(args.sequence)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rx_buffer = bytearray()
    sample_index = 0

    with serial.Serial(args.port, args.baud, timeout=0.02) as port, out_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()

        write_command(port, "STOP")
        time.sleep(args.between_stop)
        port.reset_input_buffer()

        for duty, duration_s in steps:
            sample_index = collect_step(port, writer, rx_buffer, duty, duration_s, args.settle, sample_index)
            csv_file.flush()
            write_command(port, "STOP")
            time.sleep(args.between_stop)

    print(f"saved {sample_index} samples to {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
