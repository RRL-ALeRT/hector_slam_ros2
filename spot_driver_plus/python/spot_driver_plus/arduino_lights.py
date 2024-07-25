#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import threading
from rviz_2d_overlay_msgs.msg import OverlayText#
from std_msgs.msg import ColorRGBA

class LightControlService(Node):
    def __init__(self):
        super().__init__('light_control_service')
        self.srv = self.create_service(SetBool, 'set_light', self.set_light_callback)
        self.publisher_ = self.create_publisher(OverlayText, '/deck_text', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)
        self.create_timer(0.1, self.read_from_arduino)
        # self.read_thread = threading.Thread(target=self.read_from_arduino)
        # self.read_thread.start()
        self.command_light = False

    def set_light_callback(self, request, response):
        self.light_command = "ON" if request.data else "OFF"
        self.command_light = True
        response.success = True
        return response

    def read_from_arduino(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            self.publish_to_rviz(line.split(".")[0])

            if self.command_light:
                self.serial_port.write(self.light_command.encode())
                self.command_light = False

    def publish_to_rviz(self, text):
        msg = OverlayText()
        msg.horizontal_alignment = 0
        msg.vertical_alignment = 3
        msg.width = 500
        msg.height = 50
        msg.line_width = 2
        msg.text_size = 25.0
        color = ColorRGBA()
        color.r = 0.0
        color.g = 0.69
        color.b = 0.67
        color.a = 0.6
        msg.fg_color = color
        msg.text = text
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    light_control_service = LightControlService()
    rclpy.spin(light_control_service)
    light_control_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
