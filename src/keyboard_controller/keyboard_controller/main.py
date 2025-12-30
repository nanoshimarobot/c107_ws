#!/usr/bin/env python3
import os
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.declare_parameter('topic', '/cmd_vel')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('hold_timeout', 0.1)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.hold_timeout = self.get_parameter('hold_timeout').get_parameter_value().double_value

        self.vel_pub = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.twist = Twist()

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_w = 0.0  # angular.z

        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_w = 0.0

        self.mode = 'NONE'  # 'LINEAR' / 'ANGULAR' / 'NONE'

        # 端末設定退避 & raw 化
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setraw(self._stdin_fd)

        self.get_logger().info(
            "Hold-detection keyboard teleop started.\r\n"
            f"hold_timeout={self.hold_timeout:.2f}s\r\n"
            "Constraint: linear and angular are mutually exclusive (last input wins).\r\n"
            "Controls:\r\n"
            "  W/S : linear.x = +0.1 / -0.1 (hold)\r\n"
            "  A/D : linear.y = +0.1 / -0.1 (hold)\r\n"
            "  Left Arrow  : angular.z = +1.0 (hold)\r\n"
            "  Right Arrow : angular.z = -0.1 (hold)\r\n"
            "  Space : stop (all zeros)\r\n"
            "  Q : quit\r\n"
        )

    def destroy_node(self):
        try:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass
        super().destroy_node()

    def _kbhit(self) -> bool:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        return bool(rlist)

    def _read_key(self) -> str:
        ch1 = sys.stdin.read(1)
        if ch1 != '\x1b':
            return ch1

        if self._kbhit():
            ch2 = sys.stdin.read(1)
            if ch2 == '[' and self._kbhit():
                ch3 = sys.stdin.read(1)
                if ch3 == 'D':
                    return 'LEFT'
                if ch3 == 'C':
                    return 'RIGHT'
                if ch3 == 'A':
                    return 'UP'
                if ch3 == 'B':
                    return 'DOWN'
        return ''

    def _apply_key(self, key: str) -> bool:
        if not key:
            return False

        now = time.monotonic()
        k = key.lower() if len(key) == 1 else key

        if k == 'q':
            return True

        if k == ' ':
            # stop
            self.cmd_x = self.cmd_y = self.cmd_w = 0.0
            self.last_x = self.last_y = self.last_w = 0.0
            self.mode = 'NONE'
            return False

        is_linear_key = k in ['w', 'a', 's', 'd']
        is_angular_key = k in ['LEFT', 'RIGHT']

        if self.mode == 'ANGULAR' and k in ['c', 'd']:
            return False

        if is_linear_key:
            self.mode = 'LINEAR'
            self.cmd_w = 0.0
            self.last_w = 0.0

            if k == 'w':
                self.cmd_x = 0.1
                self.last_x = now
            elif k == 's':
                self.cmd_x = -0.1
                self.last_x = now
            elif k == 'a':
                self.cmd_y = 0.1
                self.last_y = now
            elif k == 'd':
                self.cmd_y = -0.1
                self.last_y = now
            return False

        if is_angular_key:
            self.mode = 'ANGULAR'
            self.cmd_x = 0.0
            self.cmd_y = 0.0
            self.last_x = 0.0
            self.last_y = 0.0

            if k == 'LEFT':
                self.cmd_w = 1.0
                self.last_w = now
            elif k == 'RIGHT':
                self.cmd_w = -0.1
                self.last_w = now
            return False

        return False

    def _apply_hold_timeout(self):
        now = time.monotonic()
        t = self.hold_timeout

        if self.mode == 'LINEAR':
            if self.last_x > 0.0 and (now - self.last_x) > t:
                self.cmd_x = 0.0
                self.last_x = 0.0

            if self.last_y > 0.0 and (now - self.last_y) > t:
                self.cmd_y = 0.0
                self.last_y = 0.0

            if self.cmd_x == 0.0 and self.cmd_y == 0.0:
                self.mode = 'NONE'

        elif self.mode == 'ANGULAR':
            if self.last_w > 0.0 and (now - self.last_w) > t:
                self.cmd_w = 0.0
                self.last_w = 0.0
                self.mode = 'NONE'

        else:
            self.cmd_x = 0.0
            self.cmd_y = 0.0
            self.cmd_w = 0.0
            self.last_x = 0.0
            self.last_y = 0.0
            self.last_w = 0.0

    def _on_timer(self):
        while self._kbhit():
            key = self._read_key()
            should_quit = self._apply_key(key)
            if should_quit:
                self.vel_pub.publish(Twist())
                rclpy.shutdown()
                return

        self._apply_hold_timeout()

        self.twist.linear.x = self.cmd_x
        self.twist.linear.y = self.cmd_y
        self.twist.angular.z = self.cmd_w

        self.vel_pub.publish(self.twist)


def main():
    rclpy.init()
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.vel_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
