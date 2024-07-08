#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

try:
    from world_info_msgs.msg import BoundingBox, BoundingBoxArray

    CAMERA_TOPIC = "/kinova_color"
    show_windows = False
except ImportError:
    show_windows = True


class MotionDetectorAdaptative(Node):

    def __init__(self, threshold=25):
        super().__init__('motion_detector_adaptative')
        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.listener_callback,
            1)
        
        self.draw_box_pub = self.create_publisher(BoundingBoxArray, f"{CAMERA_TOPIC}/bb", 1)

        self.bridge = CvBridge()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.show = show_windows
        self.frame = None

        self.gray_frame = None
        self.average_frame = None
        self.absdiff_frame = None
        self.previous_frame = None

        self.surface = None
        self.currentsurface = 0
        self.currentcontours = None
        self.threshold = threshold
        self.trigger_time = 0  # Hold timestamp of the last detection

        if show_windows:
            cv2.namedWindow("Image")
            cv2.createTrackbar("Detection threshold: ", "Image", self.threshold, 100, self.onChange)

    def onChange(self, val):  # callback when the user changes the detection threshold
        self.threshold = val
        self.get_logger().info(f"Threshold set to: {val}")

    def listener_callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        if self.frame is None:
            self.frame = frame
            self.gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.average_frame = frame.astype('float')
            self.surface = frame.shape[0] * frame.shape[1]

        self.processImage(frame)
        current_frame = frame.copy()
        instant = time.time()  # Get timestamp of the frame

        if self.somethingHasMoved():
            self.trigger_time = instant  # Update the trigger_time
            if instant > time.time() + 10:  # Wait 5 seconds after the webcam start for luminosity adjusting etc.
                self.get_logger().info("Something is moving!")

        if self.show:
            cv2.drawContours(current_frame, self.currentcontours, -1, (0, 0, 255), 2)
            cv2.imshow("Image", current_frame)
            if cv2.waitKey(1) & 0xFF == 27:  # Break if user enters 'Esc'.
                rclpy.shutdown()

        else:
            draw_box_array = BoundingBoxArray()
            draw_box_array.header = data.header

            for contour in self.currentcontours:
                draw_box = BoundingBox()
                x, y, w, h = cv2.boundingRect(contour)

                draw_box.cx = float(x)
                draw_box.cy = float(y)
                draw_box.name = "motion"
                draw_box.height = float(h)
                draw_box.width = float(w)
                draw_box_array.array.append(draw_box)

            self.draw_box_pub.publish(draw_box_array)

    def processImage(self, frame):
        cv2.GaussianBlur(frame, (21, 21), 0)

        if self.absdiff_frame is None:  # For the first time put values in difference, temp and moving_average
            self.absdiff_frame = frame.copy()
            self.previous_frame = frame.copy()
            cv2.accumulateWeighted(frame, self.average_frame, 0.05)
        else:
            cv2.accumulateWeighted(frame, self.average_frame, 0.05)  # Compute the average

        self.previous_frame = cv2.convertScaleAbs(self.average_frame)
        self.absdiff_frame = cv2.absdiff(frame, self.previous_frame)
        self.gray_frame = cv2.cvtColor(self.absdiff_frame, cv2.COLOR_BGR2GRAY)
        _, self.gray_frame = cv2.threshold(self.gray_frame, 50, 255, cv2.THRESH_BINARY)

        self.gray_frame = cv2.dilate(self.gray_frame, None, iterations=15)  # to get object blobs
        self.gray_frame = cv2.erode(self.gray_frame, None, iterations=10)

    def somethingHasMoved(self):
        contours, _ = cv2.findContours(self.gray_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.currentcontours = contours  # Save contours

        for contour in contours:
            self.currentsurface += cv2.contourArea(contour)

        avg = (self.currentsurface * 100) / self.surface  # Calculate the average of contour area on the total size
        self.currentsurface = 0  # Put back the current surface to 0

        self.get_logger().info(f"Detected average motion: {avg:.2f}%, threshold: {self.threshold}")
        return avg > self.threshold


def main(args=None):
    rclpy.init(args=args)
    motion_detector = MotionDetectorAdaptative()
    rclpy.spin(motion_detector)
    motion_detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
 