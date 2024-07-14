#!/usr/bin/env python3

# git clone https://github.com/RRL-ALeRT/QReader_openvino
# cd QReader_openvino
# pip3 install -e .

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from qreader import QReader
import cv2
import numpy as np
from world_info_msgs.msg import BoundingBox, BoundingBoxArray
from copy import deepcopy

IMAGE_TOPICS = {
    "rs_front_color_optical_frame": "/rs_front/color/image_raw",
    "rs_left_color_optical_frame": "/rs_left/color/image_raw",
    "rs_right_color_optical_frame": "/rs_right/color/image_raw",
}
DEPTH_IMAGE_TOPICS = {
    "rs_front_color_optical_frame": "/rs_front/aligned_depth_to_color/image_raw",
    "rs_left_color_optical_frame": "/rs_left/aligned_depth_to_color/image_raw",
    "rs_right_color_optical_frame": "/rs_right/aligned_depth_to_color/image_raw",
}


class QRCodeProcessor(Node):
    def __init__(self):
        super().__init__('qr_code_processor')
        
        self.bounding_box_pubs_dict = {}
        for frame_name, image_topic in IMAGE_TOPICS.items():
            self.bounding_box_pub = self.create_publisher(BoundingBoxArray, f'{image_topic}/bb', 1)
            self.bounding_box_pubs_dict[frame_name] = self.bounding_box_pub

            self.create_subscription(Image, image_topic, self.listener_callback, 1)

        self.depth_bounding_box_pubs_dict = {}
        for frame_name, depth_image_topic in DEPTH_IMAGE_TOPICS.items():
            self.depth_bounding_box_pub = self.create_publisher(BoundingBoxArray, f'{depth_image_topic}/bb', 1)
            self.depth_bounding_box_pubs_dict[frame_name] = self.depth_bounding_box_pub

        self.br = CvBridge()
        self.qreader = QReader()

    def listener_callback(self, msg):
        cv_image = deepcopy(self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        detections = self.qreader.detect_and_decode(rgb_image, return_detections=True)

        bb_array_msg = BoundingBoxArray()
        bb_array_msg.header = msg.header
        bb_array_msg.type = "qr"

        for content, points in zip(detections[0], detections[1]):
            if content is None:
                continue

            width, height = points["wh"]
            cx, cy = points["cxcy"]

            bb_msg = BoundingBox()
            bb_msg.name = str(content)
            bb_msg.width = width
            bb_msg.height = height
            bb_msg.cx = cx
            bb_msg.cy = cy
    
            bb_array_msg.array.append(bb_msg)

        self.bounding_box_pubs_dict[msg.header.frame_id].publish(bb_array_msg)
        self.depth_bounding_box_pubs_dict[msg.header.frame_id].publish(bb_array_msg)

def main(args=None):
    rclpy.init(args=args)
    qr_code_processor = QRCodeProcessor()
    rclpy.spin(qr_code_processor)
    qr_code_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
