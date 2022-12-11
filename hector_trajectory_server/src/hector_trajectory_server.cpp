//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
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

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/create_timer.hpp>

#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "hector_nav_msgs/srv/get_robot_trajectory.hpp"
#include "hector_nav_msgs/srv/get_recovery_info.hpp"

#include <algorithm>

using namespace std;

bool comparePoseStampedStamps (const geometry_msgs::msg::PoseStamped& t1, const geometry_msgs::msg::PoseStamped& t2) {
  return (t1.header.stamp.sec < t2.header.stamp.sec && t1.header.stamp.nanosec < t2.header.stamp.nanosec);
}


/**
 * @brief Map generation node.
 */
class PathContainer
{
public:
  PathContainer(rclcpp::Node::SharedPtr node) : nh(node)
  {
    p_target_frame_name_ = nh->declare_parameter("target_frame_name", "map");
    p_source_frame_name_ = nh->declare_parameter("source_frame_name", "base_link");
    p_trajectory_update_rate_ = nh->declare_parameter("trajectory_update_rate", 4.0);
    p_trajectory_publish_rate_ = nh->declare_parameter("trajectory_publish_rate", 0.25);

    double tmp_val = 30;
    tf_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock(),
        tf2::durationFromSec(tmp_val));
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      rclcpp::node_interfaces::get_node_base_interface(nh),
      rclcpp::node_interfaces::get_node_timers_interface(nh));
    tf_->setCreateTimerInterface(timer_interface);
    tfL_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    waitForTf();

    sys_cmd_sub_ = nh->create_subscription<std_msgs::msg::String>("syscommand", 1, std::bind(&PathContainer::sysCmdCallback, this, std::placeholders::_1));
    trajectory_pub_ = nh->create_publisher<nav_msgs::msg::Path>("trajectory", 1);

    trajectory_provider_service_ = nh->create_service<hector_nav_msgs::srv::GetRobotTrajectory>("trajectory",
      std::bind(&PathContainer::trajectoryProviderCallBack, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    recovery_info_provider_service_ = nh->create_service<hector_nav_msgs::srv::GetRecoveryInfo>("trajectory_recovery_info",
      std::bind(&PathContainer::recoveryInfoProviderCallBack, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    last_reset_time_ = nh->get_clock()->now();

    update_trajectory_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(1.0 / p_trajectory_update_rate_),
      std::bind(&PathContainer::trajectoryUpdateTimerCallback, this));
    publish_trajectory_timer_ = rclcpp::create_timer(nh, nh->get_clock(), rclcpp::Duration::from_seconds(1.0 / p_trajectory_publish_rate_),
      std::bind(&PathContainer::publishTrajectoryTimerCallback, this));

    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;

    trajectory_.trajectory.header.frame_id = p_target_frame_name_;
  }

  void waitForTf()
  {
    auto start = nh->get_clock()->now().seconds();
    RCLCPP_INFO(nh->get_logger(), "Waiting for tf transform data between frames %s and %s to become available", p_target_frame_name_.c_str(), p_source_frame_name_.c_str() );

    bool transform_successful = false;

    while (!transform_successful){
      transform_successful = tf_->canTransform(p_target_frame_name_, p_source_frame_name_, rclcpp::Time(), rclcpp::Duration::from_seconds(0.5));
      if (transform_successful) break;

      auto now = nh->get_clock()->now().seconds();

      if (now-start > 20.0){
        RCLCPP_WARN_ONCE(nh->get_logger(), "No transform between frames %s and %s available after %f seconds of waiting. This warning only prints once.", p_target_frame_name_.c_str(), p_source_frame_name_.c_str(), (now-start));
      }
      
      if (!rclcpp::ok()) return;
      rclcpp::sleep_for(1000ms);
    }

    auto end = nh->get_clock()->now().seconds();
    RCLCPP_INFO(nh->get_logger(), "Finished waiting for tf, waited %f seconds", (end-start));
  }


  void sysCmdCallback(const std_msgs::msg::String& sys_cmd)
  {
    if (sys_cmd.data == "reset")
    {
      last_reset_time_ = nh->get_clock()->now();
      trajectory_.trajectory.poses.clear();
      trajectory_.trajectory.header.stamp = nh->get_clock()->now();
    }
  }

  void addCurrentTfPoseToTrajectory()
  {
    // pose_source_.header.stamp = rclcpp::Time(0);
    geometry_msgs::msg::PoseStamped pose_out;
    geometry_msgs::msg::TransformStamped source_to_target;

    if (tf_->canTransform(p_target_frame_name_, pose_source_.header.frame_id, nh->get_clock()->now(), rclcpp::Duration::from_seconds(0.5)))
    {
      source_to_target = tf_->lookupTransform(pose_source_.header.frame_id, p_target_frame_name_, pose_source_.header.stamp);
      tf2::Transform t_source_to_target(
        tf2::Matrix3x3(tf2::Quaternion(
          source_to_target.transform.rotation.x,source_to_target.transform.rotation.y,source_to_target.transform.rotation.z,source_to_target.transform.rotation.w)),
        tf2::Vector3(source_to_target.transform.translation.x, source_to_target.transform.translation.y, source_to_target.transform.translation.z)
      );

      auto t_position = t_source_to_target.inverse()*(tf2::Vector3(pose_source_.pose.position.x, pose_source_.pose.position.y, pose_source_.pose.position.z));
      auto t_orientation = t_source_to_target.inverse()*(tf2::Quaternion(pose_source_.pose.orientation.x, pose_source_.pose.orientation.y, pose_source_.pose.orientation.z, pose_source_.pose.orientation.w));
      pose_out.pose.position.x = t_position.getX();
      pose_out.pose.position.y = t_position.getY();
      pose_out.pose.position.z = t_position.getZ();
      pose_out.pose.orientation.x = t_orientation.getX();
      pose_out.pose.orientation.y = t_orientation.getY();
      pose_out.pose.orientation.z = t_orientation.getZ();
      pose_out.pose.orientation.w = t_orientation.getW();
      pose_out.header.stamp = nh->get_clock()->now();
      pose_out.header.frame_id = pose_source_.header.frame_id;

      if (trajectory_.trajectory.poses.size() != 0){
        //Only add pose to trajectory if it's not already stored
        if (pose_out.header.stamp != trajectory_.trajectory.poses.back().header.stamp){
          trajectory_.trajectory.poses.push_back(pose_out);
        }
      }else{
        trajectory_.trajectory.poses.push_back(pose_out);
      }
      trajectory_.trajectory.header.stamp = pose_out.header.stamp;

    }
    else
    {
      RCLCPP_INFO(nh->get_logger(), "lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", pose_source_.header.frame_id.c_str(), p_target_frame_name_.c_str());
      return;
    }
  }

  void trajectoryUpdateTimerCallback()
  {
    // try{
    addCurrentTfPoseToTrajectory();
    // }catch(tf2::TransformException e)
    // {
    //   RCLCPP_WARN(nh->get_logger(), "Trajectory Server: Transform from %s to %s failed: %s \n", p_target_frame_name_.c_str(), pose_source_.header.frame_id.c_str(), e.what() );
    // }
  }

  void publishTrajectoryTimerCallback()
  {
    trajectory_pub_->publish(trajectory_.trajectory);
  }

  bool trajectoryProviderCallBack(const std::shared_ptr<rmw_request_id_t> request_header,
                                  const std::shared_ptr<hector_nav_msgs::srv::GetRobotTrajectory::Request> req, 
                                  std::shared_ptr<hector_nav_msgs::srv::GetRobotTrajectory::Response> res)
  {
    *res = getTrajectory();
    return true;
  }

  inline const hector_nav_msgs::srv::GetRobotTrajectory_Response getTrajectory() const
  {
    return trajectory_;
  }

  bool recoveryInfoProviderCallBack(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<hector_nav_msgs::srv::GetRecoveryInfo::Request> req, 
                                   std::shared_ptr<hector_nav_msgs::srv::GetRecoveryInfo::Response> res)
  {
    const rclcpp::Time req_time = req->request_time;

    geometry_msgs::msg::PoseStamped tmp;
    tmp.header.stamp = req_time;

    std::vector<geometry_msgs::msg::PoseStamped> const & poses = trajectory_.trajectory.poses;

    if(poses.size() == 0)
    {
        RCLCPP_WARN(nh->get_logger(), "Failed to find trajectory leading out of radius %f"
                 " because no poses, i.e. no inverse trajectory, exists.", req->request_radius);
        return false;
    }

    //Find the robot pose in the saved trajectory
    std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it
            = std::lower_bound(poses.begin(), poses.end(), tmp, comparePoseStampedStamps);

    //If we didn't find the robot pose for the desired time, add the current robot pose to trajectory
    if (it == poses.end()){
      addCurrentTfPoseToTrajectory();
      it = poses.end();
      --it;
    }

    std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it_start = it;

    const geometry_msgs::msg::Point& req_coords ((*it).pose.position);

    double dist_sqr_threshold = req->request_radius * req->request_radius;

    double dist_sqr = 0.0;

    //Iterate backwards till the start of the trajectory is reached or we find a pose that's outside the specified radius
    while (it != poses.begin() && dist_sqr < dist_sqr_threshold){
      const geometry_msgs::msg::Point& curr_coords ((*it).pose.position);

      dist_sqr = (req_coords.x - curr_coords.x) * (req_coords.x - curr_coords.x) +
                 (req_coords.y - curr_coords.y) * (req_coords.y - curr_coords.y);

      --it;
    }

    if (dist_sqr < dist_sqr_threshold){
      RCLCPP_INFO(nh->get_logger(), "Failed to find trajectory leading out of radius %f", req->request_radius);
      return false;
    }

    std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it_end = it;

    res->req_pose = *it_start;
    res->radius_entry_pose = *it_end;

    std::vector<geometry_msgs::msg::PoseStamped>& traj_out_poses = res->trajectory_radius_entry_pose_to_req_pose.poses;

    res->trajectory_radius_entry_pose_to_req_pose.poses.clear();
    res->trajectory_radius_entry_pose_to_req_pose.header = res->req_pose.header;

    for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it_tmp = it_start; it_tmp != it_end; --it_tmp){
      traj_out_poses.push_back(*it_tmp);
    }

    return true;
  }

  //parameters
  std::string p_target_frame_name_;
  std::string p_source_frame_name_;
  double p_trajectory_update_rate_;
  double p_trajectory_publish_rate_;

  // Zero pose used for transformation to target_frame.
  geometry_msgs::msg::PoseStamped pose_source_;

  rclcpp::Node::SharedPtr nh;

  rclcpp::Service<hector_nav_msgs::srv::GetRobotTrajectory>::SharedPtr trajectory_provider_service_;
  rclcpp::Service<hector_nav_msgs::srv::GetRecoveryInfo>::SharedPtr recovery_info_provider_service_;

  rclcpp::TimerBase::SharedPtr update_trajectory_timer_;
  rclcpp::TimerBase::SharedPtr publish_trajectory_timer_;


  //rclcpp::Subscription<>::SharedPtr pose_update_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sys_cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr  trajectory_pub_;

  hector_nav_msgs::srv::GetRobotTrajectory::Response trajectory_;

  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tfL_{nullptr};

  rclcpp::Time last_reset_time_;
  rclcpp::Time last_pose_save_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("hector_trajectory_server");

  PathContainer pc(node);

  rclcpp::spin(node);

  return 0;
}
