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

#include "hector_geotiff/geotiff_writer.h"
#include "hector_geotiff/map_writer_plugin_interface.h"

#include <sstream>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>

#include <pluginlib/class_loader.hpp>

#include <memory>
#include <boost/algorithm/string.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <std_msgs/msg/string.hpp>
#include <hector_nav_msgs/srv/get_robot_trajectory.hpp>
#include "world_info_msgs/msg/world_info_array.hpp"

#include <QApplication>

using namespace std;

namespace hector_geotiff{

/**
 * @brief Map generation node.
 */
class MapGenerator
{
public:
  MapGenerator(std::shared_ptr<rclcpp::Node> node)
    : node_ (node)
    , geotiff_writer_(false)
    , running_saved_map_num_(0)
  {
    p_map_file_path_ = node_->declare_parameter("map_file_path", ".");
    geotiff_writer_.setMapFilePath(p_map_file_path_);
    geotiff_writer_.setUseUtcTimeSuffix(true);

    p_map_file_base_name_ = node_->declare_parameter("map_file_base_name", "");
    p_draw_background_checkerboard_ = node_->declare_parameter("draw_background_checkerboard", true);
    p_draw_free_space_grid_ = node_->declare_parameter("draw_free_space_grid", true);

    sys_cmd_sub_ = node_->create_subscription<std_msgs::msg::String>("syscommand", 1, std::bind(&MapGenerator::sysCmdCallback, this, std::placeholders::_1));
    world_info_sub_ = node_->create_subscription<world_info_msgs::msg::WorldInfoArray>("world_info_array", 1, std::bind(&MapGenerator::saveMapInfoCallback, this, std::placeholders::_1));

    map_service_client_ = node_->create_client<nav_msgs::srv::GetMap>("dynamic_map");
    //object_service_client_ = n_.serviceClient<worldmodel_msgs::GetObjectModel>("worldmodel/get_object_model");
    path_service_client_ = node_->create_client<hector_nav_msgs::srv::GetRobotTrajectory>("trajectory");

    auto p_geotiff_save_period = node_->declare_parameter("geotiff_save_period", 30.0);

    if(p_geotiff_save_period > 0.0){
      //publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), &PathContainer::publishTrajectoryTimerCallback, this, false);;
      map_save_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(int(p_geotiff_save_period)),
        std::bind(&MapGenerator::timerSaveGeotiffCallback, this));
    }

    p_plugin_list_ = node_->declare_parameter("plugins", "");

    std::vector<std::string> plugin_list;
    boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));

    //We always have at least one element containing "" in the string list
    if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0)){
      plugin_loader_ = std::make_unique<pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>>("hector_geotiff", "hector_geotiff::MapWriterPluginInterface");

      for (size_t i = 0; i < plugin_list.size(); ++i){
        try
        {
          auto tmp = plugin_loader_->createSharedInstance(plugin_list[i]);
          tmp->initialize(plugin_loader_->getName(plugin_list[i]));
          plugin_vector_.push_back(tmp);
        }
        catch(pluginlib::PluginlibException& ex)
        {
          RCLCPP_ERROR(node_->get_logger(), "The plugin failed to load for some reason. Error: %s", ex.what());
        }
      }
    }else{
      RCLCPP_INFO(node_->get_logger(), "No plugins loaded for geotiff node");
    }

    RCLCPP_INFO(node_->get_logger(), "Geotiff node started");
  }

  ~MapGenerator() = default;

  void map_queue_async_request()
  {
    while (!map_service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "map service not available, waiting again...");
    }
    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    // We give the async_send_request() method a callback that will get executed once the response
    // is received.
    // This way we can return immediately from this method and allow other work to be done by the
    // executor in `spin` while waiting for the response.
    using ServiceResponseFuture =
      rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        map = result.get()->map;
        if (path.size() > 0)
          this->writeGeotiff(false);
      };
    auto future_result = map_service_client_->async_send_request(request, response_received_callback);
  }

  void path_queue_async_request()
  {
    while (!path_service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "path service not available, waiting again...");
    }
    auto request = std::make_shared<hector_nav_msgs::srv::GetRobotTrajectory::Request>();

    // We give the async_send_request() method a callback that will get executed once the response
    // is received.
    // This way we can return immediately from this method and allow other work to be done by the
    // executor in `spin` while waiting for the response.
    using ServiceResponseFuture =
      rclcpp::Client<hector_nav_msgs::srv::GetRobotTrajectory>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        path = result.get()->trajectory.poses;
        if (map.data.size() > 0)
          this->writeGeotiff(false);
      };
    auto future_result = path_service_client_->async_send_request(request, response_received_callback);
  }

  void writeGeotiff(bool completed)
  {
    auto start_time = node_->get_clock()->now().seconds();

    if (map.data.size() > 0 && path.size() > 0)
    {
      RCLCPP_INFO(node_->get_logger(), "GeotiffNode: Map service called successfully");

      std::string map_file_name = p_map_file_base_name_;
      std::string competition_name;
      std::string team_name;
      std::string mission_name;
      std::string postfix;
      // if (n_.getParamCached("/competition", competition_name) && !competition_name.empty()) map_file_name = map_file_name + "_" + competition_name;
      // if (n_.getParamCached("/team", team_name)               && !team_name.empty())        map_file_name = map_file_name + "_" + team_name;
      // if (n_.getParamCached("/mission", mission_name)         && !mission_name.empty())     map_file_name = map_file_name + "_" + mission_name;
      // if (pn_.getParamCached("map_file_postfix", postfix)     && !postfix.empty())          map_file_name = map_file_name + "_" + postfix;
      // if (map_file_name.substr(0, 1) == "_") map_file_name = map_file_name.substr(1);
      if (map_file_name.empty()) map_file_name = "GeoTiffMap";
      geotiff_writer_.setMapFileName(map_file_name);
      bool transformSuccess = geotiff_writer_.setupTransforms(map);

      if(!transformSuccess){
        RCLCPP_INFO(node_->get_logger(), "Couldn't set map transform");
        return;
      }

      geotiff_writer_.setupImageSize();

      if (p_draw_background_checkerboard_){
        geotiff_writer_.drawBackgroundCheckerboard();
      }

      geotiff_writer_.drawMap(map, p_draw_free_space_grid_);
     
      for (int i = 0; i < wi_array.array.size(); i++) {
        geotiff_writer_.drawObjectOfInterest(Eigen::Vector2f(
          wi_array.array[i].pose.position.x, wi_array.array[i].pose.position.y),
          wi_array.array[i].num, Eigen::Vector3f(240,10,10), "CIRCLE");
      }

      geotiff_writer_.drawCoords();

      geotiff_writer_.completed_map_ = completed;

      RCLCPP_INFO(node_->get_logger(), "Writing geotiff plugins");
      for (size_t i = 0; i < plugin_vector_.size(); ++i){
        plugin_vector_[i]->draw(&geotiff_writer_);
      }

      RCLCPP_INFO(node_->get_logger(), "Writing geotiff");

      /**
        * No Victims for now, first  agree on a common standard for representation
        */
      /*
      if (req_object_model_){
        worldmodel_msgs::GetObjectModel srv_objects;
        if (object_service_client_.call(srv_objects))
        {
          // ROS_INFO("GeotiffNode: Object service called successfully");

          const worldmodel_msgs::ObjectModel& objects_model (srv_objects.response.model);

          size_t size = objects_model.objects.size();

          unsigned int victim_num  = 1;

          for (size_t i = 0; i < size; ++i){
            const worldmodel_msgs::Object& object (objects_model.objects[i]);

            if (object.state.state == worldmodel_msgs::ObjectState::CONFIRMED){
              geotiff_writer_.drawVictim(Eigen::Vector2f(object.pose.pose.position.x,object.pose.pose.position.y),victim_num);
              victim_num++;
            }
          }
        }
        else
        {
          // ROS_ERROR("Failed to call objects service");
        }
      }
      */

      // ROS_INFO("GeotiffNode: Path service called successfully");

      auto traj_vector = path;
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
        geotiff_writer_.drawPath(startVec, pointVec);
      }
      
      geotiff_writer_.writeGeotiffImage(completed);
      running_saved_map_num_++;

      auto elapsed_time = node_->get_clock()->now().seconds() - start_time;

      RCLCPP_INFO(node_->get_logger(), "GeoTiff created in %f seconds", elapsed_time);
      
      map = empty_map;
      path = empty_path;
    }
  }

  void timerSaveGeotiffCallback()
  {
    map_queue_async_request();
    path_queue_async_request();
  }

  void sysCmdCallback(const std_msgs::msg::String& sys_cmd)
  {
    if ( !(sys_cmd.data == "savegeotiff")){
      return;
    }

    this->writeGeotiff(true);
  }

  void saveMapInfoCallback(const world_info_msgs::msg::WorldInfoArray& array)
  {
    RCLCPP_INFO(node_->get_logger(), "Got mesg from World Info");
    wi_array = array;
    this->writeGeotiff(false);
  }

  std::string p_map_file_path_;
  std::string p_map_file_base_name_;
  std::string p_plugin_list_;
  bool p_draw_background_checkerboard_;
  bool p_draw_free_space_grid_;

  std::unique_ptr<pluginlib::ClassLoader<hector_geotiff::MapWriterPluginInterface>> plugin_loader_;
  // std::vector<boost::shared_ptr<hector_geotiff::MapWriterPluginInterface> > plugin_vector_;
  std::vector<std::shared_ptr<hector_geotiff::MapWriterPluginInterface>> plugin_vector_;
  
  rclcpp::Node::SharedPtr node_;

  GeotiffWriter geotiff_writer_;

  unsigned int running_saved_map_num_;

  std::string start_dir_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sys_cmd_sub_;
  rclcpp::Subscription<world_info_msgs::msg::WorldInfoArray>::SharedPtr world_info_sub_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_service_client_;
  rclcpp::Client<hector_nav_msgs::srv::GetRobotTrajectory>::SharedPtr path_service_client_;
  rclcpp::TimerBase::SharedPtr map_save_timer_;
  world_info_msgs::msg::WorldInfoArray wi_array;

  nav_msgs::msg::OccupancyGrid map, empty_map;
  std::vector<geometry_msgs::msg::PoseStamped> path, empty_path;
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("geotiff_node");
  hector_geotiff::MapGenerator mg(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

