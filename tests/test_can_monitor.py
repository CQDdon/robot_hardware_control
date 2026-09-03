#!/usr/bin/env python3
"""
LIVE CAN MONITOR
Công cụ giám sát lưu lượng CAN bus theo thời gian thực (real-time).
Hỗ trợ cả slcan (CANable / USB-CAN COM port / ttyACM) và socketcan (can0).
"""

import argparse
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import can
from rich.console import Console
from rich.live import Live
from rich.table import Table


# ============================================================
# OPTIONAL DECODERS
#
# Khi đã biết ID nào chứa velocity/speed thì thêm ở đây.
#
# Format:
# CAN_ID: ("name", byte_offset, struct_format, scale)
#
# Ví dụ:
#   <f  = float32 little-endian
#   <h  = int16 little-endian
#   <H  = uint16 little-endian
#   <i  = int32 little-endian
#   <B  = uint8
# ============================================================

DECODERS = {
    0x751: ("Speed_kmh", 0, "<I", 25.0 / 3696.0),
    0x753: ("Battery_V", 4, "<I", 0.01),
    # 0x500: ("RelayCmd", 0, "<B", 1),
    # 0x580: ("RelayStatus", 0, "<B", 1),
    # 0x009: ("ODrive0_Pos", 0, "<f", 1.0),
    # 0x029: ("ODrive1_Pos", 0, "<f", 1.0),
}


@dataclass
class FrameState:
    data: bytes = b""
    previous: bytes = b""
    dlc: int = 0
    count: int = 0
    last_seen: float = 0.0
    arrivals: deque = field(default_factory=lambda: deque(maxlen=5000))
    is_extended: bool = False


frames = {}
lock = threading.Lock()


class Receiver(can.Listener):
    def on_message_received(self, msg):
        now = time.monotonic()

        key = (msg.is_extended_id, msg.arbitration_id)

        with lock:
            if key not in frames:
                frames[key] = FrameState()

            f = frames[key]

            f.previous = f.data
            f.data = bytes(msg.data)
            f.dlc = msg.dlc
            f.count += 1
            f.last_seen = now
            f.is_extended = msg.is_extended_id

            f.arrivals.append(now)

            # Chỉ giữ timestamp trong 1 giây gần nhất
            while f.arrivals and now - f.arrivals[0] > 1.0:
                f.arrivals.popleft()


def format_data(current, previous):
    result = []

    for i, b in enumerate(current):
        changed = (
            len(previous) > 0
            and (i >= len(previous) or b != previous[i])
        )

        if changed:
            result.append(f"[bold black on yellow]{b:02X}[/]")
        else:
            result.append(f"{b:02X}")

    return " ".join(result)


def decode_value(can_id, data):
    if can_id not in DECODERS:
        return ""

    name, offset, fmt, scale = DECODERS[can_id]

    size = struct.calcsize(fmt)

    if offset + size > len(data):
        return f"{name}: ?"

    value = struct.unpack_from(fmt, data, offset)[0]
    value *= scale

    if isinstance(value, float):
        return f"{name}: {value:.4f}"

    return f"{name}: {value}"


def make_table(max_rows):
    now = time.monotonic()

    table = Table(
        title="LIVE CAN MONITOR — newest frame first",
        expand=True
    )

    table.add_column("ID", style="cyan", no_wrap=True)
    table.add_column("Hz", justify="right")
    table.add_column("Age", justify="right")
    table.add_column("Count", justify="right")
    table.add_column("DLC", justify="right")
    table.add_column("DATA")
    table.add_column("Decoded", style="green")

    with lock:
        snapshot = list(frames.items())

    # Frame vừa xuất hiện gần nhất luôn nằm trên đầu
    snapshot.sort(key=lambda item: item[1].last_seen, reverse=True)

    for (extended, can_id), f in snapshot[:max_rows]:

        age_ms = (now - f.last_seen) * 1000

        hz = len([
            t for t in f.arrivals
            if now - t <= 1.0
        ])

        if extended:
            id_text = f"{can_id:08X} EXT"
        else:
            id_text = f"{can_id:03X}"

        if age_ms < 100:
            age_text = f"[bold green]{age_ms:6.1f} ms[/]"
        elif age_ms < 500:
            age_text = f"[yellow]{age_ms:6.1f} ms[/]"
        else:
            age_text = f"[dim]{age_ms:6.0f} ms[/]"

        table.add_row(
            id_text,
            str(hz),
            age_text,
            str(f.count),
            str(f.dlc),
            format_data(f.data, f.previous),
            decode_value(can_id, f.data),
        )

    return table


def main():
    parser = argparse.ArgumentParser(description="Live CAN bus monitor with colorized diffs")

    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Cổng kết nối CAN (Ví dụ: /dev/ttyACM0, can0, hoặc COM5 trên Windows). Mặc định: /dev/ttyACM0"
    )

    parser.add_argument(
        "--interface",
        default="slcan",
        choices=["slcan", "socketcan"],
        help="Loại CAN interface: 'slcan' (CANable, USB-Serial) hoặc 'socketcan' (can0, vcan0). Mặc định: slcan"
    )

    parser.add_argument(
        "--bitrate",
        default=500000,
        type=int,
        help="CAN bitrate (Mặc định: 500000)"
    )

    parser.add_argument(
        "--baudrate",
        default=115200,
        type=int,
        help="Baudrate serial của slcan (Mặc định: 115200)"
    )

    parser.add_argument(
        "--rows",
        type=int,
        default=30,
        help="Số ID tối đa hiển thị (Mặc định: 30)"
    )

    args = parser.parse_args()

    # Tự động chuyển sang socketcan nếu port là canX / vcanX
    if args.port.startswith("can") or args.port.startswith("vcan"):
        args.interface = "socketcan"

    console = Console()

    console.print(
        f"[bold cyan]Opening CAN on [green]{args.port}[/] "
        f"([yellow]{args.interface}[/]) @ [magenta]{args.bitrate}[/] bit/s...[/]"
    )

    if args.interface == "socketcan":
        bus = can.Bus(
            interface="socketcan",
            channel=args.port,
            bitrate=args.bitrate,
        )
    else:
        bus = can.Bus(
            interface="slcan",
            channel=args.port,
            bitrate=args.bitrate,
            tty_baudrate=args.baudrate,
        )

    listener = Receiver()
    notifier = can.Notifier(bus, [listener])

    try:
        with Live(
            make_table(args.rows),
            console=console,
            refresh_per_second=10,
            screen=True
        ) as live:

            while True:
                live.update(make_table(args.rows))
                time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        notifier.stop()
        bus.shutdown()

        console.print("\n[bold red]Stopped.[/]")


if __name__ == "__main__":
    main()

