#!/usr/bin/env python3
"""
CAN1 VEHICLE TELEMETRY READER
Đọc và giải mã dữ liệu Pin (Battery Voltage - ID 0x753) và Tốc độ xe (Vehicle Speed - ID 0x751) từ đường can1.

Quy cách dữ liệu:
- ID 0x753 (Battery):
    - Đọc 4 byte sau (bytes [4..7]), định dạng little-endian uint32.
    - Giá trị decimal ~4000..5000 đại diện cho 40.00V .. 50.00V (đơn vị 0.01V, tức chia 100.0).
- ID 0x751 (Vehicle Speed):
    - Đọc 4 byte đầu (bytes [0..3]), định dạng little-endian uint32.
    - Giá trị raw từ 0 đến 4095 tương ứng với 0.0 đến 25.0 km/h (speed_kmh = raw / 4095.0 * 25.0).
"""

import argparse
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import can
from rich.align import Align
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

# Tùy chọn import ROS 2
HAS_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    from sensor_msgs.msg import BatteryState
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


@dataclass
class TelemetryState:
    # ID 0x753: Battery
    battery_raw: int = 0
    battery_voltage: float = 0.0
    battery_count: int = 0
    battery_last_seen: float = 0.0
    battery_data_hex: str = "--"
    battery_arrivals: deque = field(default_factory=lambda: deque(maxlen=100))

    # ID 0x751: Speed
    speed_raw: int = 0
    speed_kmh: float = 0.0
    speed_mps: float = 0.0
    speed_count: int = 0
    speed_last_seen: float = 0.0
    speed_data_hex: str = "--"
    speed_arrivals: deque = field(default_factory=lambda: deque(maxlen=100))


state = TelemetryState()
lock = threading.Lock()


class Can1TelemetryListener(can.Listener):
    def __init__(self, ros_node=None):
        super().__init__()
        self.ros_node = ros_node

    def on_message_received(self, msg: can.Message):
        now = time.monotonic()
        data = bytes(msg.data)
        can_id = msg.arbitration_id

        # ============================================================
        # ID 0x753: PIN / BATTERY VOLTAGE (4 byte sau)
        # ============================================================
        if can_id == 0x753 and len(data) >= 8:
            raw_battery = struct.unpack_from("<I", data, 4)[0]
            voltage = raw_battery / 100.0  # Ví dụ: 4964 -> 49.64V

            with lock:
                state.battery_raw = raw_battery
                state.battery_voltage = voltage
                state.battery_count += 1
                state.battery_last_seen = now
                state.battery_data_hex = " ".join(f"{b:02X}" for b in data)
                state.battery_arrivals.append(now)
                while state.battery_arrivals and now - state.battery_arrivals[0] > 1.0:
                    state.battery_arrivals.popleft()

            if self.ros_node:
                self.ros_node.publish_battery(voltage)

        # ============================================================
        # ID 0x751: TỐC ĐỘ XE / VEHICLE SPEED (4 byte đầu)
        # ============================================================
        elif can_id == 0x751 and len(data) >= 4:
            raw_speed = struct.unpack_from("<I", data, 0)[0]
            # Giới hạn an toàn trong khoảng 0..4095
            clamped_raw = min(max(raw_speed, 0), 4095)
            speed_kmh = (clamped_raw / 4095.0) * 25.0
            speed_mps = speed_kmh / 3.6

            with lock:
                state.speed_raw = raw_speed
                state.speed_kmh = speed_kmh
                state.speed_mps = speed_mps
                state.speed_count += 1
                state.speed_last_seen = now
                state.speed_data_hex = " ".join(f"{b:02X}" for b in data)
                state.speed_arrivals.append(now)
                while state.speed_arrivals and now - state.speed_arrivals[0] > 1.0:
                    state.speed_arrivals.popleft()

            if self.ros_node:
                self.ros_node.publish_speed(speed_kmh, speed_mps)


if HAS_ROS2:
    class Ros2TelemetryNode(Node):
        def __init__(self):
            super().__init__('can1_vehicle_telemetry_node')
            self.pub_bat_voltage = self.create_publisher(Float32, '/vehicle/sensor/battery_voltage', 10)
            self.pub_bat_state = self.create_publisher(BatteryState, '/vehicle/sensor/battery_state', 10)
            self.pub_speed_kmh = self.create_publisher(Float32, '/vehicle/sensor/speed_kmh', 10)
            self.pub_speed_mps = self.create_publisher(Float32, '/vehicle/sensor/speed_mps', 10)
            self.get_logger().info("ROS 2 CAN1 Telemetry Publishers initialized.")

        def publish_battery(self, voltage: float):
            v_msg = Float32()
            v_msg.data = float(voltage)
            self.pub_bat_voltage.publish(v_msg)

            b_msg = BatteryState()
            b_msg.header.stamp = self.get_clock().now().to_msg()
            b_msg.voltage = float(voltage)
            b_msg.present = True
            self.pub_bat_state.publish(b_msg)

        def publish_speed(self, speed_kmh: float, speed_mps: float):
            msg_kmh = Float32()
            msg_kmh.data = float(speed_kmh)
            self.pub_speed_kmh.publish(msg_kmh)

            msg_mps = Float32()
            msg_mps.data = float(speed_mps)
            self.pub_speed_mps.publish(msg_mps)


def build_dashboard(interface: str, bitrate: int):
    now = time.monotonic()
    with lock:
        bat_raw = state.battery_raw
        bat_v = state.battery_voltage
        bat_cnt = state.battery_count
        bat_age = (now - state.battery_last_seen) * 1000 if state.battery_last_seen > 0 else 99999
        bat_hz = len(state.battery_arrivals)
        bat_hex = state.battery_data_hex

        spd_raw = state.speed_raw
        spd_kmh = state.speed_kmh
        spd_mps = state.speed_mps
        spd_cnt = state.speed_count
        spd_age = (now - state.speed_last_seen) * 1000 if state.speed_last_seen > 0 else 99999
        spd_hz = len(state.speed_arrivals)
        spd_hex = state.speed_data_hex

    # Panel Battery
    bat_color = "green" if bat_v >= 46.0 else ("yellow" if bat_v >= 42.0 else "red")
    bat_age_str = f"[bold green]{bat_age:5.0f} ms[/]" if bat_age < 500 else f"[red]{bat_age:5.0f} ms (TIMEOUT)[/]"
    bat_text = (
        f"[bold {bat_color} text]Điện áp: {bat_v:6.2f} V[/]\n"
        f"Raw Dec: [cyan]{bat_raw}[/] (0x{bat_raw:08X})\n"
        f"Tần số: [magenta]{bat_hz} Hz[/] | Tổng gói: {bat_cnt}\n"
        f"Tuổi gói: {bat_age_str}\n"
        f"Raw Hex: [dim]{bat_hex}[/]"
    )
    p_battery = Panel(bat_text, title="[bold yellow]🔋 PIN / BATTERY (ID: 0x753)[/]", border_style=bat_color)

    # Panel Speed
    spd_age_str = f"[bold green]{spd_age:5.0f} ms[/]" if spd_age < 500 else f"[red]{spd_age:5.0f} ms (TIMEOUT)[/]"
    spd_text = (
        f"[bold cyan text]Vận tốc: {spd_kmh:5.2f} km/h[/]  ({spd_mps:4.2f} m/s)\n"
        f"Raw Dec: [cyan]{spd_raw}[/] / 4095 (Max: 25.0 km/h)\n"
        f"Tần số: [magenta]{spd_hz} Hz[/] | Tổng gói: {spd_cnt}\n"
        f"Tuổi gói: {spd_age_str}\n"
        f"Raw Hex: [dim]{spd_hex}[/]"
    )
    p_speed = Panel(spd_text, title="[bold blue]🏎️  VẬN TỐC XE (ID: 0x751)[/]", border_style="cyan")

    # Table tổng hợp
    table = Table(title=f"CAN1 LIVE TELEMETRY [{interface} @ {bitrate} bps]", expand=True)
    table.add_column("Thông số", style="bold white")
    table.add_column("Giá trị quy đổi", justify="right")
    table.add_column("Giá trị Raw", justify="right")
    table.add_column("Tần số (Hz)", justify="right")
    table.add_column("Trạng thái", justify="center")

    table.add_row(
        "🔋 Điện áp Pin (ID 0x753)",
        f"[{bat_color}]{bat_v:.2f} V[/]",
        str(bat_raw),
        f"{bat_hz} Hz",
        "OK" if bat_age < 1000 else "[red]NO SIGNAL[/]"
    )
    table.add_row(
        "🏎️  Tốc độ xe (ID 0x751)",
        f"[cyan]{spd_kmh:.2f} km/h[/] ({spd_mps:.2f} m/s)",
        f"{spd_raw}/4095",
        f"{spd_hz} Hz",
        "OK" if spd_age < 1000 else "[red]NO SIGNAL[/]"
    )

    layout = Layout()
    layout.split_column(
        Layout(table, size=6),
        Layout(name="cards", size=9)
    )
    layout["cards"].split_row(
        Layout(p_battery),
        Layout(p_speed)
    )
    return layout


def main():
    parser = argparse.ArgumentParser(description="CAN1 Vehicle Telemetry Reader (Battery 0x753 & Speed 0x751)")
    parser.add_argument("-i", "--interface", default="can1", help="CAN interface name (default: can1)")
    parser.add_argument("-b", "--bitrate", type=int, default=500000, help="Bitrate (default: 500000)")
    parser.add_argument("-t", "--type", default="socketcan", choices=["socketcan", "slcan"], help="Interface type")
    parser.add_argument("--ros", action="store_true", help="Enable ROS 2 topics publishing")
    args = parser.parse_args()

    ros_node = None
    if args.ros:
        if not HAS_ROS2:
            print("[ERROR] rclpy không khả dụng trong môi trường hiện tại.")
            sys.exit(1)
        rclpy.init()
        ros_node = Ros2TelemetryNode()
        # Chạy ROS 2 spin trong background thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        ros_thread.start()

    console = Console()
    console.print(f"[bold green]Đang kết nối tới interface [cyan]{args.interface}[/] ({args.type}) @ {args.bitrate} bps...[/]")

    try:
        if args.type == "socketcan":
            bus = can.Bus(channel=args.interface, interface="socketcan", bitrate=args.bitrate)
        else:
            bus = can.Bus(channel=args.interface, interface="slcan", bitrate=args.bitrate)
    except Exception as e:
        console.print(f"[bold red]Không thể mở cổng CAN {args.interface}: {e}[/]")
        console.print("[yellow]Gợi ý bật interface trên Jetson:[/]")
        console.print(f"  sudo ip link set {args.interface} type can bitrate {args.bitrate}")
        console.print(f"  sudo ip link set {args.interface} up")
        sys.exit(1)

    listener = Can1TelemetryListener(ros_node=ros_node)
    notifier = can.Notifier(bus, [listener])

    console.print("[bold green]Kết nối thành công! Đang lắng nghe ID 0x753 và 0x751... (Nhấn Ctrl+C để thoát)[/]")

    try:
        with Live(build_dashboard(args.interface, args.bitrate), refresh_per_second=10, console=console) as live:
            while True:
                time.sleep(0.1)
                live.update(build_dashboard(args.interface, args.bitrate))
    except KeyboardInterrupt:
        console.print("\n[yellow]Đang dừng chương trình...[/]")
    finally:
        notifier.stop()
        bus.shutdown()
        if ros_node:
            ros_node.destroy_node()
            rclpy.shutdown()
        console.print("[green]Đã đóng kết nối CAN1.[/]")


if __name__ == "__main__":
    main()

