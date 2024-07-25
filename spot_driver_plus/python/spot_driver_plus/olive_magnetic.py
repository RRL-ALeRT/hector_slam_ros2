#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from visualization_msgs.msg import Marker
import math
from scipy.spatial.transform import Rotation as R

from rviz_2d_overlay_msgs.msg import OverlayText


class MagnetometerNode(Node):
    def __init__(self):
        super().__init__('magnetometer_node')

        # Declination data for EINDHOVEN, NETHERLANDS
        self.declination_degrees = 2 + 53 / 60  # Convert degrees and minutes to decimal degrees

        # Subscriptions to Magnetometer and IMU topics
        self.subscription_mag = self.create_subscription(
            MagneticField,
            '/olive/imu/id01/magnetometer',
            self.magnetometer_callback,
            1)

        # self.subscription_imu = self.create_subscription(
        #     Imu,
        #     '/olive/imu/id01/filtered_imu',
        #     self.imu_callback,
        #     1)
        
        self.deck_pub = self.create_publisher(OverlayText, '/deck_text', 1)

        self.callback_timer = self.create_timer(0.2, self.timer_callback)

        self.heading = 0.0  # Initialize heading
        self.heading_deg = 0.0  # Initialize heading in degrees
        self.direction = ""  # Initialize direction name
        self.magnitude = 0.0  # Initialize magnitude
        self.local_direction = ""

        # Orientation variables
        self.roll = 0.0
        self.pitch = 0.0

        # Parameters for smoothing
        self.alpha = 0.1  # Smoothing factor

        # Initialize quaternion variables
        self.current_orientation = None
        self.initial_orientation = None
        self.previous_mag_field = 0

    def imu_callback(self, msg):
        # Extract quaternion
        q = msg.orientation
        current_quaternion = R.from_quat([q.x, q.y, q.z, q.w])

        # Set initial orientation if not set
        if self.initial_orientation is None:
            self.initial_orientation = current_quaternion

        # Compute the relative rotation
        self.current_orientation = current_quaternion * self.initial_orientation.inv()

        # Extract roll and pitch from the quaternion
        euler_angles = self.current_orientation.as_euler('xyz')  # Check if 'xyz' is correct for your IMU
        self.roll = euler_angles[0]
        self.pitch = euler_angles[1]


    def magnetometer_callback(self, msg):
        self.mag_field = int(math.sqrt(msg.magnetic_field.x**2 + msg.magnetic_field.y**2 + msg.magnetic_field.z**2 ))
        
        # if self.current_orientation is None:
        #     return  # If orientation is not initialized, do not proceed

        # # Extract magnetic field values
        # mx = msg.magnetic_field.x
        # my = msg.magnetic_field.y
        # mz = msg.magnetic_field.z

        # # Apply roll and pitch corrections
        # cos_pitch = math.cos(self.pitch)
        # sin_pitch = math.sin(self.pitch)
        # cos_roll = math.cos(self.roll)
        # sin_roll = math.sin(self.roll)

        # # Corrected magnetometer readings in the sensor frame
        # self.bx = mx * cos_pitch + mz * sin_pitch
        # self.by = mx * sin_roll * sin_pitch + my * cos_roll - mz * sin_roll * cos_pitch
        # self.bz = -mx * sin_pitch + mz * cos_pitch  # Adjust for the sensor's frame orientation

        # # Calculate heading from the corrected magnetometer readings
        # self.local_heading = math.atan2(self.by, self.bx)

        # # Convert heading to degrees
        # self.local_heading_deg = math.degrees(self.local_heading)

        # # Adjust heading range to 0-360 degrees
        # self.local_heading_deg = (self.local_heading_deg + 360) % 360

        # # Get direction name
        # self.local_direction = self.get_direction_name(self.local_heading_deg)

        # # Debug information
        
    def timer_callback(self):
        if not hasattr(self, "mag_field"):
            return

        # magnitude = math.sqrt(self.bx**2 + self.by**2 + self.bz**2)
        # self.get_logger().info('Local Heading: {:.2f} deg, Local Direction: {}, Magnitude: {:.2f}'.format(self.local_heading_deg, self.local_direction, magnitude))
        # self.publish_marker()

        deck_msg = OverlayText()
        deck_msg.horizontal_alignment = 0
        deck_msg.vertical_alignment = 3
        deck_msg.width = 500
        deck_msg.height = 50
        deck_msg.line_width = 2
        deck_msg.text_size = 20.0
        deck_msg.fg_color.r = 0.0
        deck_msg.fg_color.g = 0.69
        deck_msg.fg_color.b = 0.67
        deck_msg.fg_color.a = 0.6

        # if self.local_direction == "":
        #     return
        # if magnitude < 70:
        #     self.deck_pub.publish(deck_msg)
        #     return

        # if "N" in self.local_direction:
        #     deck_msg.text = "North"
        # elif "S" in self.local_direction:
        #     deck_msg.text = "South"
        if self.previous_mag_field != self.mag_field:
            self.previous_mag_field = self.mag_field
            deck_msg.text = f"{self.mag_field}"
            self.deck_pub.publish(deck_msg)

    def get_direction_name(self, heading_deg):
        self.directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        index = int((heading_deg + 22.5) // 45) % 8
        return self.directions[index]

def main(args=None):
    rclpy.init(args=args)
    magnetometer_node = MagnetometerNode()
    try:
        rclpy.spin(magnetometer_node)
    except KeyboardInterrupt:
        pass  # allow Ctrl-C to end spin()

    magnetometer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
