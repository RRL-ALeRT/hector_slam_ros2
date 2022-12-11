//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/utils.h"
#include "geometry_msgs/msg/point.hpp"

std::string p_map_frame_;
std::string p_base_footprint_frame_;
std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf2_ros::TransformBroadcaster* tfB_;
geometry_msgs::msg::TransformStamped transform_, ts_robot_pose_transform_, ts_height_transform;

tf2::Quaternion robot_pose_quaternion_;
geometry_msgs::msg::Transform robot_pose_transform_, height_transform;

tf2::Quaternion tmp_;
tf2::Quaternion orientation_quaternion_;

sensor_msgs::msg::Imu last_imu_msg_;
sensor_msgs::msg::Imu fused_imu_msg_;
nav_msgs::msg::Odometry odom_msg_;
geometry_msgs::msg::Point robot_pose_position_;
geometry_msgs::msg::PoseStamped last_pose_msg_;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fused_imu_publisher_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

size_t callback_count_;

// #ifndef TF_MATRIX3x3_H
// typedef btScalar tfScalar;
// namespace tf { typedef btMatrix3x3 Matrix3x3; }
// #endif

void imuMsgCallback(const sensor_msgs::msg::Imu& imu_msg)
{
  callback_count_++;

  tf2::Quaternion tmp_(
    imu_msg.orientation.x,
    imu_msg.orientation.y,
    imu_msg.orientation.z,
    imu_msg.orientation.w);

  tf2Scalar imu_yaw, imu_pitch, imu_roll;
  tf2::Matrix3x3(tmp_).getRPY(imu_roll, imu_pitch, imu_yaw);

  geometry_msgs::msg::TransformStamped transform;
  // transform.setIdentity();
  tf2::Quaternion quat;

  quat.setRPY(imu_roll, imu_pitch, 0.0);

  if (true){
    transform.transform.rotation.x = tmp_.x();
    transform.transform.rotation.y = tmp_.y();
    transform.transform.rotation.z = tmp_.z();
    transform.transform.rotation.w = tmp_.w();
    transform.header.frame_id = p_base_stabilized_frame_;
    transform.header.stamp = imu_msg.header.stamp;
    transform.child_frame_id = p_base_frame_;
    tfB_->sendTransform(transform);
  }

  tf2Scalar pose_yaw, pose_pitch, pose_roll;

  if (last_pose_msg_.header.stamp.sec != 0){
    tf2::Quaternion tmp_(
      last_pose_msg_.pose.orientation.x,
      last_pose_msg_.pose.orientation.y,
      last_pose_msg_.pose.orientation.z,
      last_pose_msg_.pose.orientation.w);

    tf2::Matrix3x3(tmp_).getRPY(pose_roll, pose_pitch, pose_yaw);
  }else{
    pose_yaw = 0.0;
  }

  orientation_quaternion_.setRPY(imu_roll, imu_pitch, pose_yaw);

  fused_imu_msg_.header.stamp = imu_msg.header.stamp;

  fused_imu_msg_.orientation.x = orientation_quaternion_.x();
  fused_imu_msg_.orientation.y = orientation_quaternion_.y();
  fused_imu_msg_.orientation.z = orientation_quaternion_.z();
  fused_imu_msg_.orientation.w = orientation_quaternion_.w();

  fused_imu_publisher_->publish(fused_imu_msg_);

  //If no pose message received, yaw is set to 0.
  //@TODO: Check for timestamp of pose and disable sending if too old
  if (last_pose_msg_.header.stamp.sec != 0){
    if ( (callback_count_ % 5) == 0){
      odom_msg_.header.stamp = imu_msg.header.stamp;
      odom_msg_.pose.pose.orientation = fused_imu_msg_.orientation;
      odom_msg_.pose.pose.position = last_pose_msg_.pose.position;

      odometry_publisher_->publish(odom_msg_);
    }
  }
};

void poseMsgCallback(const geometry_msgs::msg::PoseStamped& pose_msg)
{

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.resize(2);

  robot_pose_transform_.rotation.x = pose_msg.pose.orientation.x;
  robot_pose_transform_.rotation.y = pose_msg.pose.orientation.y;
  robot_pose_transform_.rotation.z = pose_msg.pose.orientation.z;
  robot_pose_transform_.rotation.w = pose_msg.pose.orientation.w;
  robot_pose_transform_.translation.x = pose_msg.pose.position.x;
  robot_pose_transform_.translation.y = pose_msg.pose.position.y;
  robot_pose_transform_.translation.z = pose_msg.pose.position.z;
  ts_robot_pose_transform_.transform = robot_pose_transform_;
  ts_robot_pose_transform_.header.stamp = pose_msg.header.stamp;
  ts_robot_pose_transform_.header.frame_id = p_map_frame_;
  ts_robot_pose_transform_.child_frame_id = p_base_footprint_frame_;

  // height_transform.setIdentity();
  // height_transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  ts_height_transform.transform = height_transform;
  ts_height_transform.header.stamp = pose_msg.header.stamp;
  ts_height_transform.header.frame_id = p_base_footprint_frame_;
  ts_height_transform.child_frame_id = p_base_stabilized_frame_;

  transforms[0] = ts_robot_pose_transform_;
  transforms[1] = ts_height_transform;

  tfB_->sendTransform(transforms);

  // Perform simple estimation of vehicle altitude based on orientation
  if (last_pose_msg_.header.stamp.sec){
    tf2::Vector3 plane_normal = tf2::Matrix3x3(orientation_quaternion_) * tf2::Vector3(0.0, 0.0, 1.0);

    tf2::Vector3 last_position;
    last_position[0] = last_pose_msg_.pose.position.x;
    last_position[1] = last_pose_msg_.pose.position.y;
    last_position[2] = last_pose_msg_.pose.position.z;

    double height_difference =
        (-plane_normal.getX() * (robot_pose_position_.x - last_position.getX())
         -plane_normal.getY() * (robot_pose_position_.y - last_position.getY())
         +plane_normal.getZ() * last_position.getZ()) / last_position.getZ();
  }

  last_pose_msg_ = pose_msg;

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hector_imu_tools");

  p_map_frame_ = node->declare_parameter("map_frame", "map");
  p_base_footprint_frame_ = node->declare_parameter("base_footprint_frame", "base_footprint");
  p_base_stabilized_frame_ = node->declare_parameter("base_stabilized_frame", "base_stabilized");
  p_base_frame_ = node->declare_parameter("base_frame", "base_link");

  fused_imu_msg_.header.frame_id = p_base_stabilized_frame_;
  odom_msg_.header.frame_id = "map";

  tfB_ = new tf2_ros::TransformBroadcaster(node);

  fused_imu_publisher_ = node->create_publisher<sensor_msgs::msg::Imu>("/fused_imu",1);
  odometry_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("/state", 1);

  auto imu_subscriber = node->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, imuMsgCallback);
  auto pose_subscriber = node->create_subscription<geometry_msgs::msg::PoseStamped>("/pose", 10, poseMsgCallback);

  callback_count_ = 0;

  rclcpp::spin(node);

  delete tfB_;

  return 0;
}
