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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/utils.h"

rclcpp::Node::SharedPtr node;
std::string p_base_stabilized_frame_;
std::string p_base_frame_;
tf2_ros::TransformBroadcaster* tfB_;
geometry_msgs::msg::TransformStamped transform_;

// #ifndef TF_MATRIX3x3_H
//   typedef btScalar tf2Scalar;
//   namespace tf { typedef btMatrix3x3 Matrix3x3; }
// #endif skpawar1305

void imuMsgCallback(const sensor_msgs::msg::Imu& imu_msg)
{
  tf2::Quaternion tmp_(
    imu_msg.orientation.x,
    imu_msg.orientation.y,
    imu_msg.orientation.z,
    imu_msg.orientation.w);

  tf2Scalar yaw, pitch, roll;
  tf2::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

  tmp_.setRPY(roll, pitch, 0.0);

  transform_.transform.rotation.x = tmp_.x();
  transform_.transform.rotation.y = tmp_.y();
  transform_.transform.rotation.z = tmp_.z();
  transform_.transform.rotation.w = tmp_.w();

  transform_.header.stamp = imu_msg.header.stamp;

  tfB_->sendTransform(transform_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("imu_attitude_to_tf_node");

  p_base_stabilized_frame_ = node->declare_parameter("base_stabilized_frame", "base_stabilized");
  p_base_frame_ = node->declare_parameter("base_frame", "base_link");
  
  tfB_ = new tf2_ros::TransformBroadcaster(node);
  transform_.transform.translation.x = 0.0;
  transform_.transform.translation.y = 0.0;
  transform_.transform.translation.z = 0.0;
  transform_.header.frame_id = p_base_stabilized_frame_;
  transform_.child_frame_id = p_base_frame_;

  auto imu_subscriber = node->create_subscription<sensor_msgs::msg::Imu>("imu_topic", 10, imuMsgCallback);

  rclcpp::spin(node);

  delete tfB_;

  return 0;
}
