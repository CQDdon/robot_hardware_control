#!/usr/bin/env python3
"""
SELECTED CAN IDs — UNIQUE PAYLOADS INSPECTOR
Công cụ theo dõi và phân tích chi tiết payload (dạng HEX và BIT) của từng CAN ID cụ thể.
Hỗ trợ cả slcan (CANable / USB-Serial) và socketcan (can0).
"""

import argparse
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass

import can
from rich.console import Console
from rich.live import Live
from rich.table import Table


@dataclass
class PacketInfo:
    count: int
    first_seen: float
    last_seen: float


@dataclass
class IdInfo:
    packets: OrderedDict


can_ids = OrderedDict()
total_frames = 0
lock = threading.Lock()
WATCH_IDS = set()


def data_to_hex(data: bytes):
    return " ".join(f"{b:02X}" for b in data)


def data_to_bits(data: bytes):
    return " ".join(f"{b:08b}" for b in data)


class CanListener(can.Listener):
    def on_message_received(self, msg):
        global total_frames

        if msg.arbitration_id not in WATCH_IDS:
            return

        now = time.monotonic()
        payload = bytes(msg.data)
        can_id = msg.arbitration_id

        with lock:
            total_frames += 1

            if can_id not in can_ids:
                can_ids[can_id] = IdInfo(
                    packets=OrderedDict()
                )

            packets = can_ids[can_id].packets

            if payload not in packets:
                packets[payload] = PacketInfo(
                    count=1,
                    first_seen=now,
                    last_seen=now,
                )
            else:
                packets[payload].count += 1
                packets[payload].last_seen = now


def format_age(age_ms):
    if age_ms < 100:
        return f"[green]{age_ms:.1f} ms[/]"
    elif age_ms < 1000:
        return f"[yellow]{age_ms:.1f} ms[/]"
    else:
        return f"[dim]{age_ms / 1000:.1f} s[/]"


def make_table():
    now = time.monotonic()

    table = Table(
        title="SELECTED CAN IDs — UNIQUE PAYLOADS",
        expand=True,
    )

    table.add_column("ID", style="cyan", no_wrap=True)
    table.add_column("#", justify="right")
    table.add_column("Counter", justify="right")
    table.add_column("Last", justify="right")
    table.add_column("HEX", no_wrap=True)
    table.add_column("BITS", no_wrap=True)

    with lock:
        snapshot = [
            (can_id, list(info.packets.items()))
            for can_id, info in can_ids.items()
        ]
        current_total = total_frames

    for can_id, packets in snapshot:
        for index, (payload, info) in enumerate(packets, start=1):
            age_ms = (now - info.last_seen) * 1000

            table.add_row(
                f"{can_id:03X}" if index == 1 else "",
                str(index),
                str(info.count),
                format_age(age_ms),
                data_to_hex(payload),
                data_to_bits(payload),
            )

        table.add_section()

    table.caption = (
        f"Watching: "
        + ", ".join(f"0x{x:X}" for x in sorted(WATCH_IDS))
        + f"    Frames received: {current_total}"
    )

    return table


def parse_can_id(text):
    try:
        if text.lower().startswith("0x"):
            return int(text, 16)
        return int(text, 16)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"CAN ID không hợp lệ: {text}"
        )


def main():
    global WATCH_IDS

    parser = argparse.ArgumentParser(
        description="Theo dõi các unique payloads (HEX/BITS) của các CAN ID chỉ định."
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Cổng kết nối CAN (Ví dụ: /dev/ttyACM0, can0, hoặc COM5 trên Windows). Mặc định: /dev/ttyACM0",
    )

    parser.add_argument(
        "--interface",
        default="slcan",
        choices=["slcan", "socketcan"],
        help="Loại CAN interface: 'slcan' hoặc 'socketcan'. Mặc định: slcan",
    )

    parser.add_argument(
        "--bitrate",
        default=500000,
        type=int,
        help="CAN bitrate (Mặc định: 500000)",
    )

    parser.add_argument(
        "--baudrate",
        default=115200,
        type=int,
        help="Baudrate serial của slcan (Mặc định: 115200)",
    )

    parser.add_argument(
        "--ids",
        required=True,
        nargs="+",
        type=parse_can_id,
        help="Các CAN ID cần theo dõi (dạng Hex), ví dụ: --ids 500 580 18A hoặc 0x500 0x580",
    )

    args = parser.parse_args()

    # Tự động nhận diện socketcan nếu port là canX / vcanX
    if args.port.startswith("can") or args.port.startswith("vcan"):
        args.interface = "socketcan"

    WATCH_IDS = set(args.ids)

    console = Console()

    console.print(
        f"[bold cyan]Opening CAN on [green]{args.port}[/] "
        f"([yellow]{args.interface}[/]) @ [magenta]{args.bitrate}[/] bit/s...[/]"
    )
    console.print(
        "Watching CAN IDs: "
        + ", ".join(f"[cyan]0x{x:X}[/]" for x in sorted(args.ids))
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

    listener = CanListener()
    notifier = can.Notifier(bus, [listener])

    try:
        with Live(
            make_table(),
            console=console,
            refresh_per_second=10,
            screen=True,
        ) as live:

            while True:
                live.update(make_table())
                time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    finally:
        notifier.stop()
        bus.shutdown()
        console.print("\n[bold red]Stopped.[/]")


if __name__ == "__main__":
    main()
