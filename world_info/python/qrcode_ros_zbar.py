#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from world_info_msgs.msg import WorldInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from pyzbar.pyzbar import decode


def get_qr_pose(image, squareLength, K, D):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect with zxingcpp
    results = decode(gray)
    if not len(results):
        return None,None,None,None

    all_text = []
    all_corners = []
    all_pose_t = []
    all_pose_R = []
    for result in results:
        if not str(result.type) == 'QRCODE':
            continue

        # Get corners
        corners = result.polygon
        corners = np.array([
            [corners[0][0],corners[0][1]],
            [corners[1][0],corners[1][1]],
            [corners[2][0],corners[2][1]],
            [corners[3][0],corners[3][1]]
        ], np.float32)

        # Define the reference frame (e.g. the camera frame)
        ref_points = np.array([
            [ squareLength / 2,  squareLength / 2, 0],
            [ squareLength / 2, -squareLength / 2, 0],
            [-squareLength / 2, -squareLength / 2, 0],
            [-squareLength / 2,  squareLength / 2, 0]
        ], np.float32)

        # Estimate the pose of the square using the corner points and the reference frame
        retval, rvec, tvec = cv2.solvePnP(ref_points, corners, K, D, cv2.SOLVEPNP_IPPE_SQUARE)

        # Extract the rotation matrix from the rvec vector
        rot_mat, _ = cv2.Rodrigues(rvec)

        # The pose of the square is given by the transformation matrix [R|t]
        pose = np.hstack((rot_mat, tvec))

        # The position of the square can be extracted from the pose matrix as follows:
        position = pose[:3, 3]
        
        # The orientation of the square can be extracted using the rot_mat matrix as follows - green up, blue front:
        r = R.from_euler('xyz', [np.pi/2, 0, -np.pi/2])
        pose_R = np.matmul(r.as_matrix(), rot_mat)
        quat = R.from_matrix(pose_R).as_quat()

        all_text.append(str(result.data))
        all_corners.append(corners)
        all_pose_t.append(position)
        all_pose_R.append(quat)

    return all_text,all_corners,all_pose_t,all_pose_R


class QrCode(Node):
    def __init__(self) -> None:
        super().__init__('qrcode_node')

        self.tfb_ = TransformBroadcaster(self)
        
        self.bridge = CvBridge()
        self.create_subscription(Image,"/Spot/kinect_color", self.spot_kinect, 10)
        self.world_info_pub = self.create_publisher(WorldInfo,"/world_info_sub", 1)
        self.world_info_msg = WorldInfo()

    def spot_kinect(self,msg:Image) -> None:
        kinect_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        squareLength = 0.665

        K = np.array([
            [-292.8780354739923, 0.0, 160.0],
            [0.0, 292.8780354739923, 95.0],
            [0.0, 0.0, 1.0]
        ])
        D = np.array([0.,0.,0.,0.,0.])

        all_text,all_corners,all_pose_t,all_pose_R = get_qr_pose(kinect_img, squareLength, K, D)
        if all_text is None:
            # show the output image
            cv2.imshow("kinect", kinect_img)
            cv2.waitKey(1)
            return

        for text,corners,pose_t,pose_R in zip(all_text,all_corners,all_pose_t,all_pose_R):
            tfs = TransformStamped()

            tfs.header.stamp = self.get_clock().now().to_msg()

            tfs._header.frame_id = "kinect"
            tfs._child_frame_id = text
            tfs.transform.translation.x = float(pose_t[2])
            tfs.transform.translation.y = float(pose_t[0])
            tfs.transform.translation.z = float(-pose_t[1])
            tfs.transform.rotation.x = pose_R[0]
            tfs.transform.rotation.y = pose_R[1]
            tfs.transform.rotation.z = pose_R[2]
            tfs.transform.rotation.w = pose_R[3]
            self.tfb_.sendTransform(tfs)

            # extract the bounding box (x, y)-coordinates for the qrcode
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the qrcode detection
            cv2.line(kinect_img, ptA, ptB, (0, 255, 0), 2)
            cv2.line(kinect_img, ptB, ptC, (0, 255, 0), 2)
            cv2.line(kinect_img, ptC, ptD, (0, 255, 0), 2)
            cv2.line(kinect_img, ptD, ptA, (0, 255, 0), 2)
            # draw top left corner of the qrcode
            cv2.circle(kinect_img, (int(corners[0][0]), int(corners[0][1])), 5, (0, 0, 255), -1)
            # draw the center (x, y)-coordinates of the qrcode
            cX = int((corners[0][0]+corners[1][0]+corners[2][0]+corners[3][0])/4)
            cY = int((corners[0][1]+corners[1][1]+corners[2][1]+corners[3][1])/4)
            cv2.circle(kinect_img, (cX, cY), 5, (0, 0, 255), -1)

        # show the output image
        cv2.imshow("kinect", kinect_img)
        cv2.waitKey(1)
        

def main(args = None):
    rclpy.init(args=args)
    qrcode_node = QrCode()

    try:
        rclpy.spin(qrcode_node)
    except KeyboardInterrupt:
        qrcode_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
