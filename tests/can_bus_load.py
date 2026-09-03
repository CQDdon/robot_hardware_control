#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN Bus Bandwidth / Load Monitor
=================================
Chương trình giám sát và tính toán phần trăm tải băng thông (% Bus Load)
của từng cổng CAN (mặc định 500 kbps) theo thời gian thực trên Linux SocketCAN.

Công thức tính toán (Chuẩn CAN 2.0A / CAN 2.0B):
- Khung chuẩn (Standard 11-bit ID):
    Bits = 47 (Fixed Overhead) + (8 * DLC) + Stuffing_Bits
    Stuffing_Bits trung bình ≈ (34 + 8 * DLC) / 5
    => Với DLC = 8: ≈ 130 bits / frame.
- Khung mở rộng (Extended 29-bit ID):
    Bits = 67 (Fixed Overhead) + (8 * DLC) + Stuffing_Bits
    Stuffing_Bits trung bình ≈ (54 + 8 * DLC) / 5
    => Với DLC = 8: ≈ 154 bits / frame.

Tỷ lệ tải băng thông (% Bus Load):
    Bus Load (%) = (Tổng số bits trong chu kỳ đo / (Bitrate * Khoảng thời gian)) * 100%

Tác giả: CQDdon & Antigravity
"""

import argparse
import glob
import os
import socket
import struct
import sys
import threading
import time
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

try:
    from rich.console import Console
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.layout import Layout
    HAVE_RICH = True
except ImportError:
    HAVE_RICH = False


# ============================================================
# CẤU HÌNH & HẰNG SỐ
# ============================================================
DEFAULT_BITRATE = 500_000  # 500 kbps (mặc định)
CAN_EFF_FLAG = 0x80000000  # Extended Frame Flag (EFF)
CAN_RTR_FLAG = 0x40000000  # Remote Transmission Request Flag
CAN_ERR_FLAG = 0x20000000  # Error Frame Flag
CAN_SFF_MASK = 0x000007FF  # 11-bit standard ID mask
CAN_EFF_MASK = 0x1FFFFFFF  # 29-bit extended ID mask


def calculate_frame_bits(can_id_raw: int, dlc: int) -> Tuple[int, int, bool]:
    """
    Tính số bit vật lý trên đường truyền CAN cho 1 frame.
    
    Trả về:
        (total_bits, clean_can_id, is_extended)
    """
    is_extended = bool(can_id_raw & CAN_EFF_FLAG)
    is_rtr = bool(can_id_raw & CAN_RTR_FLAG)
    is_err = bool(can_id_raw & CAN_ERR_FLAG)

    if is_extended:
        clean_id = can_id_raw & CAN_EFF_MASK
    else:
        clean_id = can_id_raw & CAN_SFF_MASK

    if is_err:
        # Error frame: 6 dominant bits + 8 recessive delimiter + 3 IFS = 17 bits
        return 17, clean_id, is_extended

    data_bytes = 0 if is_rtr else max(0, min(8, dlc))
    data_bits = data_bytes * 8

    if not is_extended:
        # Standard 11-bit frame
        stuffed_field = 34 + data_bits
        unstuffed_field = 13
        stuff_bits = stuffed_field // 5
        total_bits = 34 + data_bits + unstuffed_field + stuff_bits
    else:
        # Extended 29-bit frame
        stuffed_field = 54 + data_bits
        unstuffed_field = 13
        stuff_bits = stuffed_field // 5
        total_bits = 54 + data_bits + unstuffed_field + stuff_bits

    return total_bits, clean_id, is_extended


# ============================================================
# LỚP THEO DÕI TỪNG CỔNG CAN
# ============================================================
class CanInterfaceMonitor:
    def __init__(self, iface: str, bitrate: int):
        self.iface = iface
        self.bitrate = bitrate
        self.running = True
        self.online = False
        self.sock: Optional[socket.socket] = None

        # Thống kê tích lũy
        self.total_frames = 0
        self.total_bits = 0
        self.peak_load_pct = 0.0

        # Thống kê trong cửa sổ hiện tại (Window statistics)
        self.lock = threading.Lock()
        self.window_frames = 0
        self.window_bits = 0
        self.window_id_counts: Dict[int, int] = defaultdict(int)
        self.window_id_bits: Dict[int, int] = defaultdict(int)

        # Kết quả tính toán chu kỳ gần nhất
        self.last_fps = 0.0
        self.last_kbps = 0.0
        self.last_load_pct = 0.0
        self.last_top_ids: List[Tuple[int, int, float, float]] = []
        self.last_update_time = time.time()

        # Thread đọc SocketCAN
        self.thread = threading.Thread(target=self._worker, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

    def _open_socket(self) -> bool:
        try:
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.settimeout(0.5)
            s.bind((self.iface,))
            self.sock = s
            self.online = True
            return True
        except Exception:
            self.sock = None
            self.online = False
            return False

    def _worker(self):
        while self.running:
            if not self.online:
                if not self._open_socket():
                    time.sleep(1.0)
                    continue

            try:
                # Đọc frame CAN chuẩn 16 bytes: <I (4 bytes ID), B (1 byte DLC), 3 bytes padding, 8 bytes data
                frame_data = self.sock.recv(16)
                if len(frame_data) < 16:
                    continue

                can_id_raw, can_dlc = struct.unpack_from("<IB", frame_data, 0)
                bits, clean_id, is_ext = calculate_frame_bits(can_id_raw, can_dlc)

                with self.lock:
                    self.window_frames += 1
                    self.window_bits += bits
                    self.window_id_counts[clean_id] += 1
                    self.window_id_bits[clean_id] += bits
                    self.total_frames += 1
                    self.total_bits += bits

            except socket.timeout:
                continue
            except OSError:
                self.online = False
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                self.sock = None
                time.sleep(1.0)

    def compute_window(self, dt: float):
        """Tính toán phần trăm tải và tốc độ trong chu kỳ dt giây vừa qua."""
        with self.lock:
            frames = self.window_frames
            bits = self.window_bits
            id_counts = dict(self.window_id_counts)
            id_bits = dict(self.window_id_bits)

            # Reset window
            self.window_frames = 0
            self.window_bits = 0
            self.window_id_counts.clear()
            self.window_id_bits.clear()

        if dt <= 0:
            dt = 1.0

        fps = frames / dt
        kbps = (bits / dt) / 1000.0
        load_pct = (bits / (self.bitrate * dt)) * 100.0

        if load_pct > self.peak_load_pct:
            self.peak_load_pct = load_pct

        # Top 5 ID chiếm băng thông cao nhất
        top_ids = []
        for cid, b in sorted(id_bits.items(), key=lambda item: item[1], reverse=True)[:5]:
            c_cnt = id_counts.get(cid, 0)
            c_kbps = (b / dt) / 1000.0
            c_pct = (b / (self.bitrate * dt)) * 100.0
            top_ids.append((cid, c_cnt, c_kbps, c_pct))

        self.last_fps = fps
        self.last_kbps = kbps
        self.last_load_pct = load_pct
        self.last_top_ids = top_ids
        self.last_update_time = time.time()


# ============================================================
# TỰ ĐỘNG PHÁT HIỆN CỔNG CAN
# ============================================================
def detect_can_interfaces() -> List[str]:
    """Tìm tất cả interface can hoặc vcan đang có trong hệ thống."""
    found = []
    for path in glob.glob("/sys/class/net/*"):
        iface = os.path.basename(path)
        type_file = os.path.join(path, "type")
        if os.path.exists(type_file):
            try:
                with open(type_file, "r") as f:
                    dev_type = int(f.read().strip())
                    if dev_type == 280:
                        found.append(iface)
            except Exception:
                pass
        elif iface.startswith(("can", "vcan")):
            found.append(iface)

    found = sorted(list(set(found)))
    return found


def make_bar(pct: float, width: int = 20) -> str:
    clamped = max(0.0, min(100.0, pct))
    fill_len = int(round((clamped / 100.0) * width))
    empty_len = width - fill_len
    return "█" * fill_len + "░" * empty_len


def get_load_color(pct: float) -> str:
    if pct >= 75.0:
        return "bold red"
    elif pct >= 50.0:
        return "bold yellow"
    elif pct >= 25.0:
        return "cyan"
    else:
        return "green"


# ============================================================
# HIỂN THỊ TERMINAL (RICH HOẶC ANSI)
# ============================================================
def render_rich_dashboard(monitors: List[CanInterfaceMonitor], interval: float) -> Panel:
    table = Table(title="[bold white]BẢNG TỔNG HỢP BĂNG THÔNG CỔNG CAN[/]", expand=True)
    table.add_column("Interface", style="bold cyan", justify="left")
    table.add_column("Bitrate", justify="right")
    table.add_column("Trạng thái", justify="center")
    table.add_column("Frames/s", justify="right")
    table.add_column("Tốc độ", justify="right")
    table.add_column("% Bus Load (Hiện tại)", justify="right")
    table.add_column("Thanh tải", justify="left", width=22)
    table.add_column("Peak Load", justify="right")
    table.add_column("Tổng Frames", justify="right")

    for mon in monitors:
        status_text = "[bold green]ONLINE[/]" if mon.online else "[bold red]OFFLINE[/]"
        color = get_load_color(mon.last_load_pct) if mon.online else "dim"
        bar_str = make_bar(mon.last_load_pct, 18) if mon.online else "░" * 18

        table.add_row(
            mon.iface,
            f"{mon.bitrate // 1000}k",
            status_text,
            f"{mon.last_fps:6.1f}" if mon.online else "-",
            f"{mon.last_kbps:6.1f} kbps" if mon.online else "-",
            f"[{color}]{mon.last_load_pct:6.2f} %[/]" if mon.online else "-",
            f"[{color}]{bar_str}[/]" if mon.online else f"[dim]{bar_str}[/]",
            f"{mon.peak_load_pct:6.2f} %" if mon.online else "-",
            f"{mon.total_frames:,}"
        )

    # Bảng chi tiết top ID
    detail_tables = []
    for mon in monitors:
        if not mon.online or not mon.last_top_ids:
            continue
        dtbl = Table(title=f"Top CAN IDs trên [{mon.iface}] (Chiếm băng thông)", expand=True)
        dtbl.add_column("CAN ID (Hex)", style="bold yellow", justify="center")
        dtbl.add_column("Msg / giây", justify="right")
        dtbl.add_column("Tốc độ (kbps)", justify="right")
        dtbl.add_column("Tải ID (%)", justify="right")

        for cid, cnt, kbps, pct in mon.last_top_ids:
            id_col = get_load_color(pct)
            dtbl.add_row(
                f"0x{cid:03X} ({cid})",
                f"{cnt}",
                f"{kbps:5.1f} kbps",
                f"[{id_col}]{pct:5.2f} %[/]"
            )
        detail_tables.append(dtbl)

    summary_layout = Layout()
    if detail_tables:
        summary_layout.split_column(
            Layout(table, name="overview"),
            Layout(detail_tables[0], name="details")
        )
    else:
        summary_layout.update(table)

    return Panel(
        summary_layout,
        title=f"[bold green]📊 CAN BUS BANDWIDTH MONITOR (Mặc định 500k bps | Chu kỳ: {interval:.1f}s)[/]",
        border_style="blue"
    )


def print_ansi_dashboard(monitors: List[CanInterfaceMonitor], interval: float):
    sys.stdout.write("\033[H\033[2J")  # Clear screen
    print("=" * 80)
    print(f"       CAN BUS BANDWIDTH MONITOR (Mặc định 500k bps | Cập nhật mỗi {interval:.1f}s)")
    print("=" * 80)
    print(f"{'Interface':<10} {'Bitrate':<8} {'Status':<10} {'Frames/s':<10} {'kbps':<10} {'Load (%)':<10} {'Peak (%)':<10} {'Total Frames'}")
    print("-" * 80)
    for mon in monitors:
        st = "ONLINE" if mon.online else "OFFLINE"
        if mon.online:
            bar = make_bar(mon.last_load_pct, 10)
            print(f"{mon.iface:<10} {mon.bitrate//1000}k     {st:<10} {mon.last_fps:9.1f} {mon.last_kbps:8.1f}k  {mon.last_load_pct:6.2f}% {bar}  {mon.peak_load_pct:6.2f}%   {mon.total_frames:,}")
        else:
            print(f"{mon.iface:<10} {mon.bitrate//1000}k     {st:<10} {'-':<10} {'-':<10} {'-':<10} {'-':<10}   {mon.total_frames:,}")
    print("=" * 80)
    print("Nhấn Ctrl+C để dừng.")


# ============================================================
# MAIN ENTRYPOINT
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description="Đo và tính toán phần trăm băng thông (%% Bus Load) của từng cổng CAN (mặc định 500 kbps)."
    )
    parser.add_argument(
        "interfaces",
        nargs="*",
        help="Danh sách các cổng CAN cần đo (ví dụ: can0 can1). Nếu để trống, tự động quét tất cả cổng can*."
    )
    parser.add_argument(
        "-b", "--bitrate",
        type=int,
        default=DEFAULT_BITRATE,
        help=f"Tốc độ baudrate của bus CAN (bps). Mặc định: {DEFAULT_BITRATE} (500k)"
    )
    parser.add_argument(
        "-i", "--interval",
        type=float,
        default=1.0,
        help="Khoảng thời gian tính toán và cập nhật lại %% băng thông (giây). Mặc định: 1.0s"
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Chỉ đo 1 chu kỳ rồi in kết quả và thoát."
    )

    args = parser.parse_args()

    # Xác định danh sách interface
    if args.interfaces:
        ifaces = args.interfaces
    else:
        detected = detect_can_interfaces()
        if detected:
            ifaces = detected
        else:
            # Mặc định theo hệ thống xe
            ifaces = ["can0", "can1"]

    print(f"Khởi động giám sát cổng CAN: {ifaces} với bitrate = {args.bitrate} bps...")

    monitors = [CanInterfaceMonitor(iface, args.bitrate) for iface in ifaces]
    for mon in monitors:
        mon.start()

    console = Console() if HAVE_RICH else None

    try:
        if args.once:
            time.sleep(args.interval)
            for mon in monitors:
                mon.compute_window(args.interval)
            if HAVE_RICH and console:
                console.print(render_rich_dashboard(monitors, args.interval))
            else:
                print_ansi_dashboard(monitors, args.interval)
            return

        if HAVE_RICH and console:
            with Live(render_rich_dashboard(monitors, args.interval), console=console, refresh_per_second=4) as live:
                last_calc = time.time()
                while True:
                    time.sleep(0.1)
                    now = time.time()
                    dt = now - last_calc
                    if dt >= args.interval:
                        for mon in monitors:
                            mon.compute_window(dt)
                        last_calc = now
                        live.update(render_rich_dashboard(monitors, args.interval))
        else:
            last_calc = time.time()
            while True:
                time.sleep(0.1)
                now = time.time()
                dt = now - last_calc
                if dt >= args.interval:
                    for mon in monitors:
                        mon.compute_window(dt)
                    last_calc = now
                    print_ansi_dashboard(monitors, args.interval)

    except KeyboardInterrupt:
        pass
    finally:
        for mon in monitors:
            mon.stop()
        print("\nĐã dừng giám sát băng thông CAN.")


if __name__ == "__main__":
    main()
