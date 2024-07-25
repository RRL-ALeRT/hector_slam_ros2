#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from world_info_msgs.msg import BoundingBox, BoundingBoxArray
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
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


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        if not self.has_parameter("model"):
            self.declare_parameter("model", "hazmat")
        self.model_type = self.get_parameter("model").value
        
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
        pkg_dir = get_package_share_directory("spot_driver_plus")
        self.yolo = YOLO(f"{pkg_dir}/yolov8n/{self.model_type}_openvino_model", task="detect")  # Initialize YOLO model

    def listener_callback(self, msg):
        cv_image = deepcopy(self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
        # rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Predict with YOLO model
        results = self.yolo.predict(cv_image, verbose=False)

        bb_array_msg = BoundingBoxArray()
        bb_array_msg.header = msg.header
        bb_array_msg.type = self.model_type

        for result in results[0]:
            cx, cy, width, height = result.boxes.xywh.cpu()[0]

            bb_msg = BoundingBox()
            bb_msg.name = result.names[result.boxes.cls.cpu()[0].item()]
            confidence = float(result.boxes.conf.cpu()[0].item())
            if confidence < 0.8:
                continue
            bb_msg.confidence = confidence
            bb_msg.width = float(width)
            bb_msg.height = float(height)
            bb_msg.cx = float(cx)
            bb_msg.cy = float(cy)

            bb_array_msg.array.append(bb_msg)

        # Publish bounding box array
        self.bounding_box_pubs_dict[msg.header.frame_id].publish(bb_array_msg)
        self.depth_bounding_box_pubs_dict[msg.header.frame_id].publish(bb_array_msg)


def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    rclpy.spin(object_detector)
    object_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
