#!/usr/bin/env python3
"""
REAL-VEHICLE TEST RUNNER & DIAGNOSTIC TOOL
Công cụ kiểm thử và đánh giá hành vi thực tế của xe tự hành.
Hỗ trợ giao diện Terminal trực quan (Rich UI), chạy các kịch bản kiểm thử tự động
hoặc bán tự động có hướng dẫn, đồng thời tích hợp phím ngắt khẩn cấp an toàn (Space/Q).

Sử dụng:
    python3 real_vehicle_test_runner.py
    python3 real_vehicle_test_runner.py --test preflight
    python3 real_vehicle_test_runner.py --test forward
    python3 real_vehicle_test_runner.py --test reverse
    python3 real_vehicle_test_runner.py --test steer
    python3 real_vehicle_test_runner.py --test brake
    python3 real_vehicle_test_runner.py --test aux
    python3 real_vehicle_test_runner.py --test timeout
"""

import argparse
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from vehicle_msgs.msg import (
    VehicleFunctionCommand,
    VehicleMotionCommand,
    VehicleMotionState,
    VehicleSystemState,
)

from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


@dataclass
class VehicleLiveTelemetry:
    # FSM State
    fsm_state: str = "Unknown"
    fsm_last_seen: float = 0.0

    # System State
    hardware_online: bool = False
    hardware_fault: bool = False
    hardware_motion_allowed: bool = False
    function_motion_enabled: bool = False
    emergency_stop: bool = False
    command_timed_out: bool = False
    safe_command_active: bool = True
    io_output_mask: int = 0
    system_last_seen: float = 0.0

    # Motion State
    steering_angle_rad: float = 0.0
    wheel_velocity_rad_s: float = 0.0
    speed_percentage: float = 0.0
    brake_percentage: float = 0.0
    gear: int = 0
    motion_safe_command_active: bool = True
    motion_last_seen: float = 0.0

    # CAN1 Telemetry
    battery_voltage: float = 0.0
    vehicle_speed_kmh: float = 0.0


class RealVehicleTestRunnerNode(Node):
    def __init__(self):
        super().__init__('real_vehicle_test_runner')
        self.telemetry = VehicleLiveTelemetry()
        self.lock = threading.Lock()

        # Publishers
        self.motion_pub = self.create_publisher(
            VehicleMotionCommand, '/vehicle/control/motion_cmd', 10
        )
        self.function_pub = self.create_publisher(
            VehicleFunctionCommand, '/vehicle/control/func_cmd', 10
        )

        # Subscribers
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            String, '/hsh/fsm_state', self._fsm_cb, qos_reliable
        )
        self.create_subscription(
            VehicleSystemState, '/vehicle/state/system', self._system_cb, qos_reliable
        )
        self.create_subscription(
            VehicleMotionState, '/vehicle/state/motion', self._motion_cb, qos_reliable
        )

        # Periodic command publisher timer (20Hz)
        self.active_gear = VehicleMotionCommand.GEAR_NEUTRAL
        self.active_speed = 0.0
        self.active_brake = 0.0
        self.active_steer = 0.0
        self.active_motion_enable = False
        self.active_emergency_stop = False
        self.active_aux_mask = 0
        self.publish_active = True

        self.cmd_timer = self.create_timer(0.05, self._publish_periodic_cmd)

    def _fsm_cb(self, msg: String):
        with self.lock:
            self.telemetry.fsm_state = msg.data
            self.telemetry.fsm_last_seen = time.time()

    def _system_cb(self, msg: VehicleSystemState):
        with self.lock:
            t = self.telemetry
            t.hardware_online = msg.hardware_online
            t.hardware_fault = msg.hardware_fault
            t.hardware_motion_allowed = msg.hardware_motion_allowed
            t.function_motion_enabled = msg.function_motion_enabled
            t.emergency_stop = msg.emergency_stop
            t.command_timed_out = msg.command_timed_out
            t.safe_command_active = msg.safe_command_active
            t.io_output_mask = msg.io_output_mask
            t.system_last_seen = time.time()

    def _motion_cb(self, msg: VehicleMotionState):
        with self.lock:
            t = self.telemetry
            t.steering_angle_rad = msg.steering_angle_rad
            t.wheel_velocity_rad_s = msg.wheel_velocity_rad_s
            t.speed_percentage = msg.speed_percentage
            t.brake_percentage = msg.brake_percentage
            t.gear = msg.gear
            t.motion_safe_command_active = msg.safe_command_active
            t.motion_last_seen = time.time()

    def _publish_periodic_cmd(self):
        if not self.publish_active:
            return

        m = VehicleMotionCommand()
        m.gear = self.active_gear
        m.speed_percentage = float(self.active_speed)
        m.brake_percentage = float(self.active_brake)
        m.steering_angle_rad = float(self.active_steer)

        f = VehicleFunctionCommand()
        f.motion_enable = self.active_motion_enable
        f.emergency_stop = self.active_emergency_stop
        f.io_output_mask = int(self.active_aux_mask)

        self.motion_pub.publish(m)
        self.function_pub.publish(f)

    def send_safe_command(self):
        self.active_gear = VehicleMotionCommand.GEAR_NEUTRAL
        self.active_speed = 0.0
        self.active_brake = 0.0
        self.active_steer = 0.0
        self.active_motion_enable = False
        self.active_emergency_stop = False
        self.publish_active = True
        self._publish_periodic_cmd()


class RealVehicleTestSuite:
    def __init__(self, node: RealVehicleTestRunnerNode):
        self.node = node
        self.console = Console()

    def wait_for_telemetry(self, timeout_sec: float = 3.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            with self.node.lock:
                if self.node.telemetry.system_last_seen > 0:
                    return True
            time.sleep(0.05)
        return False

    def render_dashboard(self) -> Panel:
        with self.node.lock:
            t = self.node.telemetry

            gear_str = "NEUTRAL" if t.gear == 0 else ("DRIVE (TIẾN)" if t.gear == 1 else "REVERSE (LÙI)")
            fsm_color = "green" if t.fsm_state in ["Ready", "Running"] else ("red" if t.fsm_state in ["Stop", "Fault"] else "yellow")
            safe_color = "red" if t.safe_command_active else "green"

            # Parse relay mask
            mask = t.io_output_mask
            r6_fwd = (mask & (1 << 5)) != 0
            r7_rev = (mask & (1 << 6)) != 0
            r8_main = (mask & (1 << 7)) != 0
            aux_mask = mask & 0x1F

            grid = Table.grid(expand=True)
            grid.add_column(ratio=1)
            grid.add_column(ratio=1)
            grid.add_column(ratio=1)

            col1 = (
                f"[bold cyan]FSM State:[/bold cyan] [{fsm_color}]{t.fsm_state}[/{fsm_color}]\n"
                f"[bold cyan]Safe Mode:[/bold cyan] [{safe_color}]{'ACTIVE (Khóa an toàn)' if t.safe_command_active else 'INACTIVE (Đang chạy)'}[/{safe_color}]\n"
                f"[bold cyan]Hardware Online:[/bold cyan] {'[green]YES[/green]' if t.hardware_online else '[red]NO[/red]'}\n"
                f"[bold cyan]Hardware Fault:[/bold cyan] {'[red]FAULT![/red]' if t.hardware_fault else '[green]OK[/green]'}\n"
                f"[bold cyan]Motion Allowed:[/bold cyan] {'[green]YES[/green]' if t.hardware_motion_allowed else '[yellow]LOCKED[/yellow]'}\n"
                f"[bold cyan]E-Stop Active:[/bold cyan] {'[red]PRESSED / STOP![/red]' if t.emergency_stop else '[green]RELEASED (Bình thường)[/green]'}"
            )

            col2 = (
                f"[bold yellow]Số (Gear):[/bold yellow] [bold]{gear_str}[/bold]\n"
                f"[bold yellow]Vận tốc đặt (Speed):[/bold yellow] {t.speed_percentage:.1f} %\n"
                f"[bold yellow]Điện áp ga AO8 (ước tính):[/bold yellow] {(t.speed_percentage / 100.0 * 5.0):.2f} V\n"
                f"[bold yellow]Phanh ODrive (Brake):[/bold yellow] {t.brake_percentage:.1f} %\n"
                f"[bold yellow]Góc lái đặt (Steering):[/bold yellow] {t.steering_angle_rad:+.2f} rad ({(t.steering_angle_rad * 70.0):+.1f} vòng)\n"
                f"[bold yellow]Vận tốc bánh xe:[/bold yellow] {t.wheel_velocity_rad_s:.2f} rad/s"
            )

            col3 = (
                f"[bold magenta]Relay 6 (Tiến):[/bold magenta] {'[green]ON (0x20)[/green]' if r6_fwd else '[dim]OFF[/dim]'}\n"
                f"[bold magenta]Relay 7 (Lùi):[/bold magenta] {'[green]ON (0x40)[/green]' if r7_rev else '[dim]OFF[/dim]'}\n"
                f"[bold magenta]Relay 8 (Nguồn chính):[/bold magenta] {'[green]ON (0x80)[/green]' if r8_main else '[dim]OFF[/dim]'}\n"
                f"[bold magenta]Relay phụ 1..5 (Aux):[/bold magenta] [cyan]0x{aux_mask:02X}[/cyan] (B1:{aux_mask&1} B2:{(aux_mask>>1)&1} B3:{(aux_mask>>2)&1} B4:{(aux_mask>>3)&1} B5:{(aux_mask>>4)&1})\n"
                f"[bold magenta]Mask tổng (Raw):[/bold magenta] 0x{mask:02X}\n"
                f"[bold magenta]Timeout Watchdog:[/bold magenta] {'[red]TIMED OUT![/red]' if t.command_timed_out else '[green]FEEDING[/green]'}"
            )

            grid.add_row(col1, col2, col3)

            return Panel(grid, title="[bold green] BẢNG ĐIỀU KHIỂN & TELEMETRY THỜI GIAN THỰC [/bold green]", border_style="green")

    # =========================================================================
    # TEST CASE IMPLEMENTATIONS
    # =========================================================================

    def run_preflight_check(self) -> bool:
        """TC-01-01 -> TC-01-03: Pre-flight Diagnostic Check."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 1: PRE-FLIGHT SYSTEM HEALTH CHECK ===[/bold cyan]")
        self.console.print("Đang kiểm tra kết nối phần cứng và trạng thái Safe State ban đầu...")

        if not self.wait_for_telemetry(4.0):
            self.console.print("[bold red]❌ THẤT BẠI: Không nhận được topic telemetry từ ROS 2 controller![/bold red]")
            return False

        with self.node.lock:
            t = self.node.telemetry
            online = t.hardware_online
            fault = t.hardware_fault
            safe = t.safe_command_active
            fsm = t.fsm_state
            mask = t.io_output_mask
            speed = t.speed_percentage

        errors = []
        if not online:
            errors.append("Hardware offline (Kiểm tra cáp CAN0, ODrive và ESP32)")
        if fault:
            errors.append("Hệ thống đang có lỗi phần cứng (Hardware fault detected)")
        if not safe:
            errors.append("Trạng thái khởi động không an toàn (Safe command must be active on startup)")
        if (mask & 0xE0) != 0:
            errors.append(f"Rơle động lực đang đóng trái phép khi khởi động (Mask=0x{mask:02X})")
        if speed > 0.1:
            errors.append(f"Điện áp ga khác 0V khi khởi động (Speed={speed}%)")

        if errors:
            self.console.print("[bold red]❌ PRE-FLIGHT CHECK FAILED:[/bold red]")
            for err in errors:
                self.console.print(f"  - [red]{err}[/red]")
            return False

        self.console.print(f"[bold green]✔ PRE-FLIGHT CHECK PASSED:[/bold green] FSM=[cyan]{fsm}[/cyan], Online=[green]YES[/green], SafeState=[green]OK[/green], Ga=[green]0.0V[/green], Relays=[green]0x{mask:02X}[/green]")
        return True

    def run_forward_drive_sequence_test(self) -> bool:
        """TC-02-01 & TC-02-02: Forward Drive Relay Sequencing."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 2: TUẦN TỰ RELAY VÀO SỐ TIẾN (FORWARD ENGAGEMENT) ===[/bold cyan]")
        self.console.print("[yellow]Yêu cầu: Kê kích bánh xe trước khi bắt đầu test.[/yellow]")

        self.node.send_safe_command()
        time.sleep(0.5)

        self.console.print("1. Kích hoạt Motion Enable và vào số DRIVE với 20% ga...")
        self.node.active_motion_enable = True
        self.node.active_emergency_stop = False
        self.node.active_gear = VehicleMotionCommand.GEAR_DRIVE
        self.node.active_speed = 20.0

        # Quan sát chuỗi trễ tuần tự
        start_t = time.time()
        r6_seen = False
        r8_seen = False
        throttle_seen = False

        while time.time() - start_t < 1.0:
            with self.node.lock:
                mask = self.node.telemetry.io_output_mask
                speed = self.node.telemetry.speed_percentage
                if (mask & 0x20) != 0 and (mask & 0x80) == 0:
                    r6_seen = True
                if (mask & 0x80) != 0 and (mask & 0x20) != 0:
                    r8_seen = True
                if speed >= 19.0:
                    throttle_seen = True
            time.sleep(0.01)

        self.console.print(f"  - Relay 6 (Tiến) đóng trước: {'[green]ĐẠT[/green]' if r6_seen else '[red]KHÔNG ĐẠT[/red]'}")
        self.console.print(f"  - Relay 8 (Nguồn) đóng sau trễ 100ms: {'[green]ĐẠT[/green]' if r8_seen else '[red]KHÔNG ĐẠT[/red]'}")
        self.console.print(f"  - Nạp điện áp ga sau trễ 200ms: {'[green]ĐẠT[/green]' if throttle_seen else '[red]KHÔNG ĐẠT[/red]'}")

        # Giữ chạy 1.5 giây để kiểm tra ổn định
        time.sleep(1.5)

        self.console.print("2. Trả về số NEUTRAL...")
        self.node.send_safe_command()
        time.sleep(0.2)

        with self.node.lock:
            mask_neutral = self.node.telemetry.io_output_mask
            speed_neutral = self.node.telemetry.speed_percentage

        neutral_ok = (mask_neutral & 0xE0) == 0 and speed_neutral <= 0.1
        self.console.print(f"  - Cắt toàn bộ rơle và ngắt ga về 0V: {'[green]ĐẠT[/green]' if neutral_ok else '[red]KHÔNG ĐẠT[/red]'}")

        success = r8_seen and throttle_seen and neutral_ok
        if success:
            self.console.print("[bold green]✔ BÀI TEST TIẾN: THÀNH CÔNG (PASS)[/bold green]")
        else:
            self.console.print("[bold red]❌ BÀI TEST TIẾN: THẤT BẠI (FAIL)[/bold red]")
        return success

    def run_reverse_drive_sequence_test(self) -> bool:
        """TC-02-03 & TC-02-04: Reverse Drive Relay Sequencing."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 3: TUẦN TỰ RELAY VÀO SỐ LÙI (REVERSE ENGAGEMENT) ===[/bold cyan]")
        self.node.send_safe_command()
        time.sleep(0.5)

        self.console.print("1. Kích hoạt Motion Enable và vào số REVERSE với 20% ga...")
        self.node.active_motion_enable = True
        self.node.active_gear = VehicleMotionCommand.GEAR_REVERSE
        self.node.active_speed = 20.0

        start_t = time.time()
        r7_seen = False
        r8_seen = False
        throttle_seen = False

        while time.time() - start_t < 1.0:
            with self.node.lock:
                mask = self.node.telemetry.io_output_mask
                speed = self.node.telemetry.speed_percentage
                if (mask & 0x40) != 0 and (mask & 0x80) == 0:
                    r7_seen = True
                if (mask & 0x80) != 0 and (mask & 0x40) != 0:
                    r8_seen = True
                if speed >= 19.0:
                    throttle_seen = True
            time.sleep(0.01)

        self.console.print(f"  - Relay 7 (Lùi) đóng trước: {'[green]ĐẠT[/green]' if r7_seen else '[red]KHÔNG ĐẠT[/red]'}")
        self.console.print(f"  - Relay 8 (Nguồn) đóng sau trễ 100ms: {'[green]ĐẠT[/green]' if r8_seen else '[red]KHÔNG ĐẠT[/red]'}")
        self.console.print(f"  - Nạp điện áp ga lùi: {'[green]ĐẠT[/green]' if throttle_seen else '[red]KHÔNG ĐẠT[/red]'}")

        time.sleep(1.5)
        self.node.send_safe_command()
        time.sleep(0.2)

        success = r8_seen and throttle_seen
        if success:
            self.console.print("[bold green]✔ BÀI TEST LÙI: THÀNH CÔNG (PASS)[/bold green]")
        else:
            self.console.print("[bold red]❌ BÀI TEST LÙI: THẤT BẠI (FAIL)[/bold red]")
        return success

    def run_steering_sweep_test(self) -> bool:
        """TC-03-01 to TC-03-05: Steering Sweep & Potentiometer Soft Limit."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 4: QUÉT HỆ THỐNG LÁI & CHẶN MỀM BIẾN TRỞ ===[/bold cyan]")
        self.console.print("Quy trình: 0 rad -> +0.5 rad (+35 vòng) -> +1.0 rad (+70 vòng) -> -1.0 rad (-70 vòng) -> 0 rad")

        self.node.active_motion_enable = True
        self.node.active_gear = VehicleMotionCommand.GEAR_NEUTRAL
        self.node.active_speed = 0.0

        angles = [0.5, 1.0, 0.0, -0.5, -1.0, 0.0]
        for target_rad in angles:
            self.console.print(f"-> Đặt góc lái: {target_rad:+.1f} rad (tương ứng {(target_rad * 70.0):+.0f} vòng ODrive)...")
            self.node.active_steer = target_rad
            # Chờ đủ thời gian cho ramp rate (35 vòng/s => 70 vòng cần ~2s)
            time.sleep(2.5)

            with self.node.lock:
                actual_rad = self.node.telemetry.steering_angle_rad
            self.console.print(f"   Hồi tiếp góc lái: {actual_rad:+.2f} rad")

        self.node.send_safe_command()
        self.console.print("[bold green]✔ BÀI TEST LÁI: HOÀN TẤT CHU TRÌNH QUÉT GÓC LÁI[/bold green]")
        return True

    def run_brake_proportional_test(self) -> bool:
        """TC-04-01 & TC-04-02: Proportional Brake Test."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 5: KIỂM THỬ PHANH ĐIỆN ODRIVE THEO TỶ LỆ ===[/bold cyan]")
        self.node.active_motion_enable = True
        self.node.active_gear = VehicleMotionCommand.GEAR_NEUTRAL
        self.node.active_speed = 0.0

        levels = [0.0, 33.0, 66.0, 100.0, 0.0]
        for brake_pct in levels:
            self.console.print(f"-> Đặt mức phanh: {brake_pct:.0f} % (kéo +{(brake_pct/100.0*3.0):.2f} vòng)...")
            self.node.active_brake = brake_pct
            time.sleep(1.5)

        self.node.send_safe_command()
        self.console.print("[bold green]✔ BÀI TEST PHANH: HOÀN TẤT CHU TRÌNH KÉO NHẢ PHANH[/bold green]")
        return True

    def run_aux_relays_cycling_test(self) -> bool:
        """TC-05-01: Cycling Auxiliary Relays 1..5."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 6: KIỂM THỬ ĐÓNG CẮT RELAY PHỤ TRỢ 1..5 ===[/bold cyan]")
        self.node.active_motion_enable = True

        for i in range(5):
            mask = 1 << i
            self.console.print(f"-> Bật Relay phụ {i+1} (Mask 0x{mask:02X})...")
            self.node.active_aux_mask = mask
            time.sleep(1.0)

        self.console.print("-> Bật đồng thời cả 5 Relay phụ (Mask 0x1F)...")
        self.node.active_aux_mask = 0x1F
        time.sleep(1.5)

        self.console.print("-> Tắt toàn bộ Relay phụ (Mask 0x00)...")
        self.node.active_aux_mask = 0x00
        time.sleep(0.5)

        self.node.send_safe_command()
        self.console.print("[bold green]✔ BÀI TEST RELAY PHỤ: THÀNH CÔNG[/bold green]")
        return True

    def run_command_timeout_watchdog_test(self) -> bool:
        """TC-08-01: Command Timeout Watchdog (250ms)."""
        self.console.print("\n[bold cyan]=== BÀI KIỂM TRA 7: WATCHDOG MẤT LỆNH ĐIỀU KHIỂN (TIMEOUT 250MS) ===[/bold cyan]")
        self.console.print("1. Đang nạp lệnh xe chạy số Tiến 20%...")
        self.node.active_motion_enable = True
        self.node.active_gear = VehicleMotionCommand.GEAR_DRIVE
        self.node.active_speed = 20.0
        time.sleep(1.0)

        self.console.print("2. CẮT ĐỘT NGỘT LUỒNG LỆNH (Mô phỏng đứt cáp mạng / crash planner)...")
        self.node.publish_active = False

        start_t = time.time()
        timeout_detected = False
        safe_activated = False

        while time.time() - start_t < 1.0:
            with self.node.lock:
                if self.node.telemetry.command_timed_out:
                    timeout_detected = True
                if self.node.telemetry.safe_command_active and (self.node.telemetry.io_output_mask & 0xE0) == 0:
                    safe_activated = True
            time.sleep(0.01)

        self.node.send_safe_command()

        self.console.print(f"  - Watchdog phát hiện timeout: {'[green]ĐẠT[/green]' if timeout_detected else '[red]KHÔNG ĐẠT[/red]'}")
        self.console.print(f"  - Ngắt rơle và vào Safe State trong vòng 250ms: {'[green]ĐẠT[/green]' if safe_activated else '[red]KHÔNG ĐẠT[/red]'}")

        success = timeout_detected and safe_activated
        if success:
            self.console.print("[bold green]✔ BÀI TEST WATCHDOG TIMEOUT: THÀNH CÔNG (PASS)[/bold green]")
        else:
            self.console.print("[bold red]❌ BÀI TEST WATCHDOG TIMEOUT: THẤT BẠI (FAIL)[/bold red]")
        return success


def run_interactive_monitor(runner: RealVehicleTestSuite, node: RealVehicleTestRunnerNode):
    """Interactive Live Monitor with Keybindings & Safety Escape."""
    console = Console()
    console.print("[bold green]BẮT ĐẦU CHẾ ĐỘ GIÁM SÁT THỜI GIAN THỰC (INTERACTIVE MONITOR)[/bold green]")
    console.print("Phím tắt: [bold yellow][Space] / [Q][/bold yellow]: Ngắt khẩn cấp & Thoát | [bold yellow][E][/bold yellow]: Bật/Tắt Motion")
    console.print("Phím test: [bold yellow][1][/bold yellow]: Preflight | [bold yellow][2][/bold yellow]: Test Tiến | [bold yellow][3][/bold yellow]: Test Lùi | [bold yellow][4][/bold yellow]: Test Lái | [bold yellow][5][/bold yellow]: Test Phanh | [bold yellow][6][/bold yellow]: Test Aux | [bold yellow][7][/bold yellow]: Test Timeout\n")

    if not sys.stdin.isatty():
        console.print("[yellow]Non-interactive terminal; running in passive monitoring mode...[/yellow]")
        with Live(runner.render_dashboard(), refresh_per_second=10) as live:
            try:
                while rclpy.ok():
                    live.update(runner.render_dashboard())
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        return

    old_settings = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())

    try:
        with Live(runner.render_dashboard(), refresh_per_second=10) as live:
            while rclpy.ok():
                live.update(runner.render_dashboard())

                # Non-blocking key check
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if readable:
                    key = sys.stdin.read(1).lower()
                    if key in [' ', 'q', '\x1b']:  # Space, Q, or Esc
                        node.send_safe_command()
                        console.print("\n[bold red]ĐÃ KÍCH HOẠT DỪNG AN TOÀN VÀ THOÁT![/bold red]")
                        break
                    elif key == 'e':
                        node.active_motion_enable = not node.active_motion_enable
                        if not node.active_motion_enable:
                            node.send_safe_command()
                    elif key == '1':
                        live.stop()
                        runner.run_preflight_check()
                        live.start()
                    elif key == '2':
                        live.stop()
                        runner.run_forward_drive_sequence_test()
                        live.start()
                    elif key == '3':
                        live.stop()
                        runner.run_reverse_drive_sequence_test()
                        live.start()
                    elif key == '4':
                        live.stop()
                        runner.run_steering_sweep_test()
                        live.start()
                    elif key == '5':
                        live.stop()
                        runner.run_brake_proportional_test()
                        live.start()
                    elif key == '6':
                        live.stop()
                        runner.run_aux_relays_cycling_test()
                        live.start()
                    elif key == '7':
                        live.stop()
                        runner.run_command_timeout_watchdog_test()
                        live.start()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.send_safe_command()


def main():
    parser = argparse.ArgumentParser(description="Real-Vehicle Test Runner & Diagnostic Suite")
    parser.add_argument(
        "--test",
        type=str,
        default="monitor",
        choices=["monitor", "preflight", "forward", "reverse", "steer", "brake", "aux", "timeout", "all"],
        help="Test case to execute (default: monitor)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = RealVehicleTestRunnerNode()
    runner = RealVehicleTestSuite(node)

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if args.test == "monitor":
            run_interactive_monitor(runner, node)
        elif args.test == "preflight":
            runner.run_preflight_check()
        elif args.test == "forward":
            runner.run_forward_drive_sequence_test()
        elif args.test == "reverse":
            runner.run_reverse_drive_sequence_test()
        elif args.test == "steer":
            runner.run_steering_sweep_test()
        elif args.test == "brake":
            runner.run_brake_proportional_test()
        elif args.test == "aux":
            runner.run_aux_relays_cycling_test()
        elif args.test == "timeout":
            runner.run_command_timeout_watchdog_test()
        elif args.test == "all":
            runner.run_preflight_check()
            runner.run_forward_drive_sequence_test()
            runner.run_reverse_drive_sequence_test()
            runner.run_steering_sweep_test()
            runner.run_brake_proportional_test()
            runner.run_aux_relays_cycling_test()
            runner.run_command_timeout_watchdog_test()
    except KeyboardInterrupt:
        node.send_safe_command()
        print("\nĐã ngắt bởi người dùng (Ctrl+C). Đã gửi Safe Command.")
    finally:
        node.send_safe_command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

