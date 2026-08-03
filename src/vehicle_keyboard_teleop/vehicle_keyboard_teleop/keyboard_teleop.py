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
        self.declare_parameter('steering_step_rad', 0.10)
        self.declare_parameter('throttle_step_percentage', 5.0)
        self.declare_parameter('brake_step_percentage', 10.0)
        self.declare_parameter('max_steering_angle_rad', 1.57079632679)
        self.declare_parameter('max_throttle_percentage', 100.0)
        self.declare_parameter('max_brake_percentage', 100.0)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.steering_step = float(self.get_parameter('steering_step_rad').value)
        self.throttle_step = float(self.get_parameter('throttle_step_percentage').value)
        self.brake_step = float(self.get_parameter('brake_step_percentage').value)
        self.max_steering = float(self.get_parameter('max_steering_angle_rad').value)
        self.max_throttle = float(self.get_parameter('max_throttle_percentage').value)
        self.max_brake = float(self.get_parameter('max_brake_percentage').value)
        if self.publish_rate_hz <= 0.0 or min(self.steering_step, self.throttle_step, self.brake_step) <= 0.0:
            raise ValueError('publish_rate_hz and all step sizes must be positive')
        if min(self.max_steering, self.max_throttle, self.max_brake) < 0.0:
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
        self.steering = 0.0
        self.throttle = 0.0
        # The brake axis is released by default (IDLE).  It is a separate
        # actuator and must not follow the wheel command state.
        self.brake = 0.0
        self.gear = VehicleMotionCommand.GEAR_NEUTRAL
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
        print('\nVehicle keyboard teleop (publish 20 Hz mặc định)')
        print('W/S tốc độ bánh +/- | A/D steer trái/phải | Space tăng phanh | Z nhả phanh')
        print('R tiến | F lùi | N neutral | E bật/tắt motion | X E-stop | Q thoát an toàn')
        print('1..8: bật/tắt Q0..Q7; 0: tắt toàn bộ Q0..Q7. Output bị ép 0 khi ở safe state.\n')

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
            self.steering = self._clamp(self.steering + self.steering_step, -self.max_steering, self.max_steering)
        elif key == 'd':
            self.steering = self._clamp(self.steering - self.steering_step, -self.max_steering, self.max_steering)
        elif key == ' ':
            self.brake = self._clamp(self.brake + self.brake_step, 0.0, self.max_brake)
        elif key == 'z':
            self.brake = 0.0
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
            self.brake = self.max_brake
        elif key in '12345678':
            self.io_output_mask ^= 1 << (int(key) - 1)
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
        motion.steering_angle_rad = float(self.steering)
        motion.speed_percentage = float(self.throttle)
        motion.brake_percentage = float(self.brake)
        motion.gear = self.gear
        function = VehicleFunctionCommand()
        function.motion_enable = self.motion_enable
        function.emergency_stop = self.emergency_stop
        function.io_output_mask = self.io_output_mask
        self.motion_pub.publish(motion)
        self.function_pub.publish(function)

    def _display_state(self):
        gear = {VehicleMotionCommand.GEAR_NEUTRAL: 'N', VehicleMotionCommand.GEAR_DRIVE: 'D',
                VehicleMotionCommand.GEAR_REVERSE: 'R'}[self.gear]
        text = (f'\r steer={self.steering:+.2f} rad | throttle={self.throttle:5.1f}% | '
                f'brake={self.brake:5.1f}% | gear={gear} | motion={"ON " if self.motion_enable else "OFF"} | '
                f'estop={"ON " if self.emergency_stop else "OFF"} | Q=[{self.io_output_mask:02X}]   ')
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
