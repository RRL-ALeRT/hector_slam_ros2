#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # Import the service definition
import serial
import time


class LEDControlNode(Node):
    def __init__(self):
        super().__init__('led_control_node')
        self.service = self.create_service(SetBool, 'arduino_lights', self.toggle_led_callback)
        self.arduino = serial.Serial('/dev/ttyUSB0', 9600)
        time.sleep(2)  # Wait for connection to establish
        self.get_logger().info('LED Control Service has started.')

    def toggle_led_callback(self, request, response):
        command = 'ON' if request.data else 'OFF'
        self.arduino.write((command + '\n').encode())
        response.success = True
        self.get_logger().info(f'Toggling LED: {"ON" if request.data else "OFF"}')
        return response


def main(args=None):
    rclpy.init(args=args)
    led_control_node = LEDControlNode()
    rclpy.spin(led_control_node)
    led_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
