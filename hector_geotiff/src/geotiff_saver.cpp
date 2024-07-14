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

#include <cstdio>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/srv/get_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <world_info_msgs/msg/world_info_array.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hector_nav_msgs/srv/get_robot_trajectory.hpp>

#include <QApplication>

using namespace std::chrono_literals;
using GetRT = hector_nav_msgs::srv::GetRobotTrajectory;

class GeotiffSaver
{
  public:
    GeotiffSaver(rclcpp::Node::SharedPtr node, std::string lap_name, std::string team_name) : node_(node), lap_name_(lap_name), team_name_(team_name) {
      world_info_sub_ = node_->create_subscription<world_info_msgs::msg::WorldInfoArray>("/world_info_array", 1, std::bind(&GeotiffSaver::worldInfoCallback, this, std::placeholders::_1));

      map_1m_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "projected_map_1m", 1, std::bind(&GeotiffSaver::map1mCallback, this, std::placeholders::_1));

      map_2m_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "projected_map_2m", 1, std::bind(&GeotiffSaver::map2mCallback, this, std::placeholders::_1));

      path_service_client_ = node_->create_client<GetRT>("/get_robot_trajectory");

      // Call on_timer function every second
      timer_ = node_->create_wall_timer(0.1s, std::bind(&GeotiffSaver::on_timer, this));

      path_queue_async_request();

      node_end_time_sec = node->get_clock()->now().seconds() + 5;
    }


  private:
    void on_timer()
    {
      if (node_->get_clock()->now().seconds() < node_end_time_sec && (pointVec.size() == 0 || wi_array.array.size() == 0)) return;

      if (map_1m_loaded && map_2m_loaded) {
        saveMap1m();
        saveMap2m();
        saveCSV();
        rclcpp::shutdown();
      }
    }

    //////////Path_queue_ !!!!!!!!!!!
    void path_queue_async_request()
    {
    for (long unsigned int i=5; i>0; i++) {
      if (!path_service_client_->wait_for_service(1s))
      {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN_STREAM(node_->get_logger(), "path service not available, retrying " << i << " more times");
      }
      else break;
    }
    auto request = std::make_shared<GetRT::Request>();

    using ServiceResponseFuture = rclcpp::Client<GetRT>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        path = result.get()->trajectory.poses;

        for (auto& pose : path)
        {
          Eigen::Vector2f point;
          point[0] = pose.pose.position.x;
          point[1] = pose.pose.position.y;
          pointVec.push_back(point);
        }

        path_loaded = true;
      };

    auto future_result = path_service_client_->async_send_request(request, response_received_callback);
    };

    void worldInfoCallback(const world_info_msgs::msg::WorldInfoArray::SharedPtr array)
    {
      if (wi_array.array.size() > 0) return;
      
      wi_array = *array;
    };

    void map1mCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
    {
      if (map_1m_loaded) return;
      map_1m = *map_msg;
      map_1m_loaded = true;
    };

    void map2mCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
    {
      if (map_2m_loaded) return;
      map_2m = *map_msg;
      map_2m_loaded = true;
    };

    void saveMap1m () {
      RCLCPP_INFO_ONCE(node_->get_logger(), "1m Map loaded.");
      hector_geotiff::GeotiffWriter geotiff_writer(false);

      map_name_ = "RoboCup2024-" + team_name_ + "-lap" + lap_name_ + "_1m";
      geotiff_writer.setMapFileName(map_name_);
      geotiff_writer.setupTransforms(map_1m);
      geotiff_writer.setupImageSize();
      geotiff_writer.drawBackgroundCheckerboard();
      geotiff_writer.drawMap(map_1m);
      geotiff_writer.drawCoords();

      if (pointVec.size() > 0)
      {
        startVec[0] = pointVec[0][0];
        startVec[1] = pointVec[0][1];
        geotiff_writer.drawPath(startVec, pointVec, 120, 0, 140);
      }

      int k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "hazmat" && wi.pose.position.z == 1.0)
        {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(255,100,30), "DIAMOND", 0);
        }
        k++;
      }

      k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "qr" && wi.pose.position.z == 1.0) {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(255,100,30), "CIRCLE", 0);
        }
        k++;
      }
      //////////////////////////////// object
      for (auto wi: wi_array.array) {
        if (wi.type == "object" && wi.pose.position.z == 1.0) {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(10, 240, 10), "DIAMOND", 0);
        }
        k++;
      }
      k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "victim") {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(240,10,10), "CIRCLE", 0);
        }
        k++;
      }

      geotiff_writer.writeGeotiffImage(true);
    };

    void saveMap2m () {
      RCLCPP_INFO_ONCE(node_->get_logger(), "2m Map loaded.");
      hector_geotiff::GeotiffWriter geotiff_writer(false);

      map_name_ = "RoboCup2024-" + team_name_ + "-lap" + lap_name_;
      geotiff_writer.setMapFileName(map_name_ +  + "_2m");
      geotiff_writer.setupTransforms(map_2m);
      geotiff_writer.setupImageSize();
      geotiff_writer.drawBackgroundCheckerboard();
      geotiff_writer.drawMap(map_2m);
      geotiff_writer.drawCoords();

      if (pointVec.size() > 0)
      {
        startVec[0] = pointVec[0][0];
        startVec[1] = pointVec[0][1];
        geotiff_writer.drawPath(startVec, pointVec, 120, 0, 140);
      }

      int k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "hazmat" &&  wi.pose.position.z == 2.0)
        {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(255,100,30), "DIAMOND", 0);
        }
        k++;
      }

      k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "qr" &&  wi.pose.position.z == 2.0) {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(240,10,10), "CIRCLE", 0);
        }
        k++;
      }
      /////////////object
      for (auto wi: wi_array.array) {
        if (wi.type == "object" && wi.pose.position.z == 2.0) {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(10, 240, 10), "DIAMOND", 0);
        }
        k++;
      }

      k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "victim") {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(240,10,10), "CIRCLE", 0);
        }
        k++;
      }

      geotiff_writer.writeGeotiffImage(true);
    };

    void saveCSV() {
      std::ofstream myfile;

      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);

      std::ostringstream oss;
      oss << std::put_time(&tm, "%H:%M:%S");
      std::string time_str = oss.str();

      myfile.open(map_name_ + "_pois_" + time_str + ".csv");
      myfile << "\"pois\"" << "\n" << "\"1.2\"" << "\n" << "\"" << team_name_ << "\"" << "\n" << "\"Germany\"" << "\n";

      myfile << wi_array.start_time;

      myfile << "\"" + lap_name_ + "\"" << "\n\n";
      myfile << "id,time,text,x,y,z,robot,mode,type";

      std::vector<std::string> final_world_data;
      final_world_data.resize(wi_array.array.size());

      for (int i = 0; i < final_world_data.size(); i++)
      {
        final_world_data[wi_array.id_array[i]] = "\n" +
                              std::to_string(wi_array.id_array[i]) + "," +
                              wi_array.time_array[i] + "," +
                              wi_array.array[i].num + "," +
                              std::to_string(wi_array.array[i].pose.position.x) + "," +
                              std::to_string(wi_array.array[i].pose.position.y) + "," +
                              std::to_string(wi_array.array[i].pose.position.z) + "," +
                              wi_array.robot_array[i] + "," +
                              wi_array.mode_array[i] + "," +
                              wi_array.array[i].type;
      }

      for (auto wd: final_world_data)
      {
        myfile << wd;
      }

      myfile.close();
    };

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_1m_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_2m_sub_;
    rclcpp::Subscription<world_info_msgs::msg::WorldInfoArray>::SharedPtr world_info_sub_;
    rclcpp::Client<GetRT>::SharedPtr path_service_client_;
    
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    int node_end_time_sec = 0;
    
    nav_msgs::msg::OccupancyGrid map_1m;
    nav_msgs::msg::OccupancyGrid map_2m;
    Eigen::Vector3f startVec;
    std::vector<Eigen::Vector2f> pointVec;

    world_info_msgs::msg::WorldInfoArray wi_array;
    std::vector<geometry_msgs::msg::PoseStamped> path, empty_path;
    std::string map_name_ = "";
    std::string lap_name_ = "";
    std::string team_name_ = "";
    bool map_loaded = false;
    bool map_1m_loaded = false;
    bool map_2m_loaded = false;
    bool path_loaded = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("geotiff_saver");

  // Retrieve the (non-option) argument:
  std::string lap_name = "final";
  if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
  {
    std::cerr << "ros2 run hector_geotiff geotiff_saver <lap_name>" << std::endl;
    return -1;
  }
  else
  {
    lap_name = argv[argc-1];
  }

  std::string team_name = "ALeRT";
  GeotiffSaver gs(node, lap_name, team_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
