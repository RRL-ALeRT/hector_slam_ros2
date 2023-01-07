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

#include "PoseInfoContainer.h"
// #include "boost/array.hpp"

void PoseInfoContainer::update(const Eigen::Vector3f& slamPose, const Eigen::Matrix3f& slamCov, const rclcpp::Time& stamp, const std::string& frame_id)
{
  //Fill stampedPose
  std_msgs::msg::Header& header = stampedPose_.header;
  header.stamp = stamp;
  header.frame_id = frame_id;

  // geometry_msgs::msg::Pose& pose = stampedPose_.pose;
  // pose.position.x = slamPose.x();
  // pose.position.y = slamPose.y();

  // pose.orientation.w = cos(slamPose.z()*0.5f);
  // pose.orientation.z = sin(slamPose.z()*0.5f);

  stampedPose_.header.stamp = stamp;
  stampedPose_.header.frame_id = frame_id;

  stampedPose_.pose.position.x = slamPose.x();
  stampedPose_.pose.position.y = slamPose.y();

  stampedPose_.pose.orientation.w = cos(slamPose.z()*0.5f);
  stampedPose_.pose.orientation.z = sin(slamPose.z()*0.5f);

  //Fill covPose
  geometry_msgs::msg::PoseWithCovarianceStamped covPose;
  covPose_.header = header;
  covPose_.pose.pose = stampedPose_.pose;

  // boost::array<double, 36>& cov(covPose_.pose.covariance);

  covPose_.pose.covariance[0] = static_cast<double>(slamCov(0,0));
  covPose_.pose.covariance[7] = static_cast<double>(slamCov(1,1));
  covPose_.pose.covariance[35] = static_cast<double>(slamCov(2,2));

  double xyC = static_cast<double>(slamCov(0,1));
  covPose_.pose.covariance[1] = xyC;
  covPose_.pose.covariance[6] = xyC;

  double xaC = static_cast<double>(slamCov(0,2));
  covPose_.pose.covariance[5] = xaC;
  covPose_.pose.covariance[30] = xaC;

  double yaC = static_cast<double>(slamCov(1,2));
  covPose_.pose.covariance[11] = yaC;
  covPose_.pose.covariance[31] = yaC;

  //Fill tf tansform
  // tf2::poseMsgToTF(pose, poseTransform_);
  poseTransform_.header = header;
  poseTransform_.transform.translation.x = stampedPose_.pose.position.x;
  poseTransform_.transform.translation.y = stampedPose_.pose.position.y;
  poseTransform_.transform.translation.z = stampedPose_.pose.position.z;
  poseTransform_.transform.rotation.w = stampedPose_.pose.orientation.w;
  poseTransform_.transform.rotation.z = stampedPose_.pose.orientation.z;
}
