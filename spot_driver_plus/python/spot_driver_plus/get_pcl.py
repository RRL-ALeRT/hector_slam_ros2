#!/usr/bin/env python3

import time
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient, build_image_request, ImageDataError, depth_image_to_pointcloud
from bosdyn.client.exceptions import UnableToConnectToRobotError, TimedOutError, RpcError
from bosdyn.api import image_pb2
from spot_driver.ros_helpers import get_from_env_and_fall_back_to_param

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField

# FRAMES = ['frontleft', 'frontright', 'left', 'right', 'back', 'head']
FRAMES = ['back']


def transformation_matrix(tr_rt):
    tr = np.array([tr_rt[:3]])
    r = R.from_quat(tr_rt[3:]).as_matrix()
    r_tr = np.concatenate((r, tr.T), axis=1)
    return np.concatenate((r_tr, np.array([[0,0,0,1]])), axis=0)


def getImageMsg(data):
    """Takes the image and camera data and populates the necessary ROS messages
    Args:
        data: Image proto
    Returns:
        (tuple):
            * CameraInfo: message to define the state and config of the camera that took the image
    """
    camera_info_msg = CameraInfo()
    camera_info_msg.distortion_model = "plumb_bob"

    camera_info_msg.d.append(0)
    camera_info_msg.d.append(0)
    camera_info_msg.d.append(0)
    camera_info_msg.d.append(0)
    camera_info_msg.d.append(0)

    camera_info_msg.k[1] = 0
    camera_info_msg.k[3] = 0
    camera_info_msg.k[6] = 0
    camera_info_msg.k[7] = 0
    camera_info_msg.k[8] = 1

    camera_info_msg.r[0] = 1
    camera_info_msg.r[1] = 0
    camera_info_msg.r[2] = 0
    camera_info_msg.r[3] = 0
    camera_info_msg.r[4] = 1
    camera_info_msg.r[5] = 0
    camera_info_msg.r[6] = 0
    camera_info_msg.r[7] = 0
    camera_info_msg.r[8] = 1

    camera_info_msg.p[1] = 0
    camera_info_msg.p[3] = 0
    camera_info_msg.p[4] = 0
    camera_info_msg.p[7] = 0
    camera_info_msg.p[8] = 0
    camera_info_msg.p[9] = 0
    camera_info_msg.p[10] = 1
    camera_info_msg.p[11] = 0

    camera_info_msg.header.frame_id = data.shot.frame_name_image_sensor
    camera_info_msg.height = data.shot.image.rows
    camera_info_msg.width = data.shot.image.cols

    camera_info_msg.k[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.k[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.k[4] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.k[5] = data.source.pinhole.intrinsics.principal_point.y

    camera_info_msg.p[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.p[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.p[5] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.p[6] = data.source.pinhole.intrinsics.principal_point.y

    return camera_info_msg


def camera_static_transforms(image_data, total_frames_done):
    """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
    transforms. This is the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
    never change, but they may be calibrated slightly differently for each robot so we need to generate the
    transforms at runtime.
    Args:
    image_data: Image protobuf data from the wrapper
    """
    static_tf = []
    frames_done = []
    
    for frame_name in image_data.shot.transforms_snapshot.child_to_parent_edge_map:
        if frame_name not in FRAMES or frame_name in total_frames_done:
            continue
        transform = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
        static_tf.append(transform_msg(transform.parent_frame_name, frame_name,
                                    transform.parent_tform_child))
        frames_done.append(frame_name)
        
    return static_tf, frames_done


def transform_msg(parent_frame, child_frame, transform):
    """Populates a TransformStamped message
    Args:
        time: The time of the transform
        parent_frame: The parent frame of the transform
        child_frame: The child_frame_id of the transform
        transform: A transform to copy into a StampedTransform object. Should have position (x,y,z) and rotation (x,
        y,z,w) members
    Returns:
        TransformStamped message
    """
    new_tf = TransformStamped()
    
    new_tf.header.frame_id = parent_frame
    new_tf.child_frame_id = child_frame
    new_tf.transform.translation.x = transform.position.x
    new_tf.transform.translation.y = transform.position.y
    new_tf.transform.translation.z = transform.position.z
    new_tf.transform.rotation.x = transform.rotation.x
    new_tf.transform.rotation.y = transform.rotation.y
    new_tf.transform.rotation.z = transform.rotation.z
    new_tf.transform.rotation.w = transform.rotation.w

    return new_tf


def transform_points(points, camera_transform):
    # Transform the points using the camera transform
    transformed_points = np.hstack((points, np.ones((points.shape[0], 1))))
    transformed_points = np.dot(camera_transform, transformed_points.T).T[:, :3]
    
    return transformed_points


class GetPCL(Node):
    def __init__(self) -> None:
        # ROS2 init
        super().__init__('get_images')

        self._username = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_USERNAME", self, "username", "user")
        self._password = get_from_env_and_fall_back_to_param("BOSDYN_CLIENT_PASSWORD", self, "password", "password")
        self._robot_hostname = get_from_env_and_fall_back_to_param("SPOT_IP", self, "hostname", "10.0.0.3")

        # We don't need to publish infos for depth images as they're already aligned to visual frame

        self.bridge = CvBridge()
        self.logger = self.get_logger()

        # Create robot object with an image client.
        sdk = bosdyn.client.create_standard_sdk('image_depth_plus_visual')
        self._robot = sdk.create_robot(self._robot_hostname)
        
        self.authenticate()

        self.clock = Clock()

        self.image_client = self._robot.ensure_client(ImageClient.default_service_name)

        self.cameras = FRAMES[:5]
        self.sources = []
        for cam in self.cameras:
            self.sources.append(cam + '_depth_in_visual_frame')
        self._image_requests = [build_image_request(source, quality_percent=10, image_format=image_pb2.Image.FORMAT_RAW)
                                for source in self.sources]
        
        # self._image_requests = [build_image_request('back_fisheye_image', image_format=image_pb2.Image.FORMAT_RAW)]
        self.camera_tf = {}

        self.camera_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.done_transforms = [] # Appends when new child frame detail is available
        self.camera_tf_msgs = []
        self.camera_info_msgs = {}

        # Only start publishing once we get all static transforms and camera info details
        while len(self.done_transforms) < len(FRAMES):
           self.get_camera_info_tf_msgs()

        self.camera_transforms = {}
        for cam_tf in self.camera_tf_msgs:
            self.camera_transforms[cam_tf.child_frame_id] = \
                transformation_matrix([cam_tf.transform.translation.x, cam_tf.transform.translation.y, cam_tf.transform.translation.z,
                    cam_tf.transform.rotation.x, cam_tf.transform.rotation.y, cam_tf.transform.rotation.z, cam_tf.transform.rotation.w])

        self.pcl_pub = self.create_publisher(PointCloud2, 'spot_pcl', 1)

        publish_images = False
        self.declare_parameter('publish_images', value=publish_images)
        publish_images = self.get_parameter('publish_images').value
        # Todo: if publish_images:
        while rclpy.ok():
            # pass
            self.run()

    def create_point_cloud_msg(self, points):
        # Create a PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'body'
        msg.width = points.shape[0]
        msg.height = 1
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        
        return msg

    def authenticate(self):
        try:
            self._robot.authenticate(self._username, self._password)
            self._robot.start_time_sync()
        except bosdyn.client.RpcError as err:
            self._logger.error("Failed to communicate with robot: %s", err)
            self._valid = False
            return

    def get_camera_info_tf_msgs(self):
        try:
            image_responses = self.image_client.get_image(self._image_requests)

            for response in image_responses:
                static_cam_tfs, done_transforms = camera_static_transforms(response, self.done_transforms)

                for tf, done_tf in zip(static_cam_tfs, done_transforms):
                    if done_tf not in self.done_transforms:
                        self.done_transforms.append(done_tf)
                        self.camera_tf_msgs.append(tf)
                    
                    if done_tf in self.cameras:
                        self.camera_info_msgs[done_tf] = getImageMsg(response)

            # Static camera transforms
            for msg in self.camera_tf_msgs:
                msg.header.stamp.sec = self.clock.now().to_msg().sec
                msg.header.stamp.nanosec = self.clock.now().to_msg().nanosec
                self.camera_static_transform_broadcaster.sendTransform(msg)

        except (TimedOutError, ImageDataError):
            self.logger.info('Error: failed to get images.')
            return

    def run(self):
        try:
            image_responses = self.image_client.get_image_from_sources(self.sources)
        except (TimedOutError, ImageDataError):
            self.logger.info('Error: failed to get images.')
            return
        
        i = 0
        pcl = {}
        for r, c in zip(image_responses, self.cameras):
            pcl[i] = transform_points(depth_image_to_pointcloud(r), self.camera_transforms[c])
            i += 1

        match len(FRAMES):
            case 1:
                self.pcl_pub.publish(self.create_point_cloud_msg(pcl[0]))
            case 2:
                self.pcl_pub.publish(self.create_point_cloud_msg(np.concatenate((pcl[0], pcl[1]), axis=0)))
            case 3:
                self.pcl_pub.publish(self.create_point_cloud_msg(np.concatenate((pcl[0], pcl[1], pcl[2]), axis=0)))
            case 4:
                self.pcl_pub.publish(self.create_point_cloud_msg(np.concatenate((pcl[0], pcl[1], pcl[2], pcl[3]), axis=0)))
            case 5:
                self.pcl_pub.publish(self.create_point_cloud_msg(np.concatenate((pcl[0], pcl[1], pcl[2], pcl[3], pcl[4]), axis=0)))


def main(args = None):
    rclpy.init(args=args)
    pcl_node = GetPCL()

    try:
        rclpy.spin(pcl_node)
    except KeyboardInterrupt:
        pass
    finally:
        pcl_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
