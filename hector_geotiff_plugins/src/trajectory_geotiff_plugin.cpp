//=================================================================================================
// Copyright (c) 2012, Gregor Gebhardt, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
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

#include "hector_geotiff/map_writer_interface.h"
#include "hector_geotiff/map_writer_plugin_interface.h"

#include "rclcpp/rclcpp.hpp"
#include "hector_nav_msgs/srv/get_robot_trajectory.hpp"

#include <fstream>
#include <chrono>

namespace hector_geotiff_plugins
{

using namespace hector_geotiff;

class TrajectoryMapWriter : public MapWriterPluginInterface
{
public:
  TrajectoryMapWriter();
  virtual ~TrajectoryMapWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Client<hector_nav_msgs::srv::GetRobotTrajectory>::SharedPtr service_client_;

protected:
  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  int path_color_r_;
  int path_color_g_;
  int path_color_b_;
};

TrajectoryMapWriter::TrajectoryMapWriter()
    : initialized_(false)
{}

TrajectoryMapWriter::~TrajectoryMapWriter()
{}

void TrajectoryMapWriter::initialize(const std::string& name)
{
  nh_ = std::make_shared<rclcpp::Node>("trajectory_geotiff_plugin");
  std::string service_name_;

  service_name_ = nh_->declare_parameter("service_name", "trajectory");
  path_color_r_ = nh_->declare_parameter("path_color_r", 120);
  path_color_g_ = nh_->declare_parameter("path_color_g", 0);
  path_color_b_ = nh_->declare_parameter("path_color_b", 240);

  service_client_ = nh_->create_client<hector_nav_msgs::srv::GetRobotTrajectory>(service_name_);

  initialized_ = true;
  this->name_ = name;
  RCLCPP_INFO(nh_->get_logger(), "Successfully initialized hector_geotiff MapWriter plugin %s.", name_.c_str());
}

void TrajectoryMapWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    auto srv_path = std::make_shared<hector_nav_msgs::srv::GetRobotTrajectory::Request>();
    if (!service_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(nh_->get_logger(), "Cannot draw trajectory, service GetRobotTrajectory failed");
      return;
    }
    auto result = service_client_->async_send_request(srv_path);
    
    std::vector<geometry_msgs::msg::PoseStamped>& traj_vector (result.get()->trajectory.poses);

    size_t size = traj_vector.size();

    std::vector<Eigen::Vector2f> pointVec;
    pointVec.resize(size);

    for (size_t i = 0; i < size; ++i){
      const geometry_msgs::msg::PoseStamped& pose (traj_vector[i]);

      pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
    }

    if (size > 0){
      //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
      Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
      interface->drawPath(startVec, pointVec, path_color_r_, path_color_g_, path_color_b_);
    }
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hector_geotiff_plugins::TrajectoryMapWriter, hector_geotiff::MapWriterPluginInterface)
