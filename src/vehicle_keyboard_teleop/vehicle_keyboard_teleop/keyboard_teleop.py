"""Non-blocking terminal teleoperation for the vehicle command interface."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node

from vehicle_msgs.msg import VehicleFunctionCommand, VehicleMotionCommand


class VehicleKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('vehicle_keyboard_teleop')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('throttle_step_percentage', 5.0)
        self.declare_parameter('max_throttle_percentage', 100.0)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.throttle_step = float(self.get_parameter('throttle_step_percentage').value)
        self.max_throttle = float(self.get_parameter('max_throttle_percentage').value)
        if self.publish_rate_hz <= 0.0 or self.throttle_step <= 0.0:
            raise ValueError('publish_rate_hz and all step sizes must be positive')
        if self.max_throttle < 0.0:
            raise ValueError('all limits must be non-negative')

        self.motion_pub = self.create_publisher(VehicleMotionCommand, '/vehicle/control/motion_cmd', 10)
        self.function_pub = self.create_publisher(VehicleFunctionCommand, '/vehicle/control/func_cmd', 10)
        self._terminal_settings = None
        self.running = True
        self._set_safe_state()
        self._enable_keyboard()
        self._print_help()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)

    def _set_safe_state(self):
        self.throttle = 0.0
        self.steering_velocity = 0.0
        # Position axis is released (IDLE) by default. Space selects one of
        # three relative offsets: current +10, +20, or +30 turns.
        self.position_level = 0
        self.gear = VehicleMotionCommand.GEAR_DRIVE
        self.motion_enable = False
        self.emergency_stop = False
        self.io_output_mask = 0

    def _enable_keyboard(self):
        if not sys.stdin.isatty():
            self.get_logger().warning('stdin không phải terminal; keyboard input bị tắt.')
            return
        self._terminal_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        if self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None

    def _print_help(self):
        print('\n=== Vehicle Keyboard Teleop (Controller Test API) ===')
        print('W/S: Ga (Throttle AO +/-) | A/D: Vận tốc đánh lái (Steer +/-) | C: Dừng lái (0 rad/s)')
        print('R: Tiến (Drive) | F: Lùi (Reverse) | N: Mo (Neutral) | Space: Phanh | Z: Nhả phanh')
        print('E: Bật/Tắt Motion | X: E-stop | Q: Thoát')
        print('1..5: Bật/Tắt Relay phụ 1..5; Relay 6 (Tới), 7 (Lùi), 8 (Nguồn) tự động điều khiển.\n')

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(value, high))

    def _read_key(self):
        if self._terminal_settings is None:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1) if readable else None

    def _handle_key(self, key):
        key = key.lower()
        if key == 'w':
            self.throttle = self._clamp(self.throttle + self.throttle_step, 0.0, self.max_throttle)
        elif key == 's':
            self.throttle = self._clamp(self.throttle - self.throttle_step, 0.0, self.max_throttle)
        elif key == 'a':
            self.steering_velocity = self._clamp(self.steering_velocity - 2.0, -20.0, 20.0)
        elif key == 'd':
            self.steering_velocity = self._clamp(self.steering_velocity + 2.0, -20.0, 20.0)
        elif key == 'c':
            self.steering_velocity = 0.0
        elif key == ' ':
            self.position_level = min(self.position_level + 1, 3)
        elif key == 'z':
            self.position_level = 0
        elif key == 'r':
            self.gear = VehicleMotionCommand.GEAR_DRIVE
        elif key == 'f':
            self.gear = VehicleMotionCommand.GEAR_REVERSE
        elif key == 'n':
            self.gear = VehicleMotionCommand.GEAR_NEUTRAL
        elif key == 'e':
            self.motion_enable = not self.motion_enable
        elif key == 'x':
            self.emergency_stop = True
            self.motion_enable = False
            self.throttle = 0.0
            self.steering_velocity = 0.0
        elif key in '12345':
            self.io_output_mask ^= 1 << (int(key) - 1)
        elif key in '678':
            print(f"\n[INFO] Relay {key} được tự động điều khiển theo Motion & Số (R/F/N), không cần thao tác tay.")
        elif key == '0':
            self.io_output_mask = 0
        elif key == 'q':
            self._set_safe_state()
            self.publish_commands()
            self.running = False
        else:
            return

    def publish_commands(self):
        motion = VehicleMotionCommand()
        motion.steering_angle_rad = float(self.steering_velocity)
        motion.speed_percentage = float(self.throttle)
        motion.brake_percentage = 100.0 * self.position_level / 3.0
        motion.gear = self.gear
        function = VehicleFunctionCommand()
        function.motion_enable = self.motion_enable
        function.emergency_stop = self.emergency_stop
        function.io_output_mask = self.io_output_mask
        self.motion_pub.publish(motion)
        self.function_pub.publish(function)

    def _display_state(self):
        gear_str = "DRIVE" if self.gear == VehicleMotionCommand.GEAR_DRIVE else (
            "REVERSE" if self.gear == VehicleMotionCommand.GEAR_REVERSE else "NEUTRAL"
        )
        text = (f'\r Gear={gear_str:<7} | '
                f'Ga={self.throttle:5.1f}% | '
                f'V_Lái={self.steering_velocity:+5.1f} rad/s | '
                f'Phanh=+{10 * self.position_level} turns | '
                f'Motion={"ON " if self.motion_enable else "OFF"} | '
                f'Estop={"ON " if self.emergency_stop else "OFF"} | '
                f'AuxRelay=[0x{self.io_output_mask & 0x1F:02X}]   ')
        sys.stdout.write(text)
        sys.stdout.flush()

    def _timer_callback(self):
        key = self._read_key()
        if key is not None:
            self._handle_key(key)
        self.publish_commands()
        self._display_state()

    def publish_safe_command(self):
        self._set_safe_state()
        self.publish_commands()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleKeyboardTeleop()
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_safe_command()
            node.restore_terminal()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nĐã gửi safe command và thoát.')


if __name__ == '__main__':
    main()
