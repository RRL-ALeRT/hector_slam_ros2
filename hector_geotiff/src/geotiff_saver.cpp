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
#include "world_info_msgs/msg/world_info_array.hpp"
#include <tf2/utils.h>

#include <QApplication>


class GeotiffSaver
{
  public:
    GeotiffSaver(rclcpp::Node::SharedPtr node, std::string mission_name) : node_(node), mission_name_(mission_name) {
      world_info_sub_ = node_->create_subscription<world_info_msgs::msg::WorldInfoArray>("world_info_array", 1, std::bind(&GeotiffSaver::worldInfoCallback, this, std::placeholders::_1));
      
      path_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/slam_toolbox/graph_visualization", 1, std::bind(&GeotiffSaver::pathCallback, this, std::placeholders::_1));

      map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1, std::bind(&GeotiffSaver::mapCallback, this, std::placeholders::_1));
    }


  private:
    void pathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr path)
    {
      auto marker_array = path->markers;
      int num_markers = marker_array.size();

      for (int i = 0; i < num_markers; i++) {
        if (marker_array[i].ns == "slam_toolbox") {
        
          pointVec.push_back(Eigen::Vector2f(marker_array[i].pose.position.x, marker_array[i].pose.position.y));

          RCLCPP_INFO_ONCE(node_->get_logger(), "Path loaded.");
        }
      }
      path_loaded = true;
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
    {
      map = *map_msg;
      map_loaded = true;
    };

    void worldInfoCallback(const world_info_msgs::msg::WorldInfoArray::SharedPtr array)
    {
      wi_array = *array;
      if (map_loaded && path_loaded) {
        saveMap();
        saveCSV();
        rclcpp::shutdown();
      }
    }

    void saveMap () {
      RCLCPP_INFO_ONCE(node_->get_logger(), "Map loaded.");
      hector_geotiff::GeotiffWriter geotiff_writer(false);

      startVec[0] = pointVec[0][0];
      startVec[1] = pointVec[0][1];

      map_name_ = "RoboCup2023-ALeRT-" + mission_name_;
      geotiff_writer.setMapFileName(map_name_);
      geotiff_writer.setupTransforms(map);
      geotiff_writer.setupImageSize();
      geotiff_writer.drawBackgroundCheckerboard();
      geotiff_writer.drawMap(map);
      geotiff_writer.drawCoords();
      geotiff_writer.drawPath(startVec, pointVec, 120, 0, 140);

      int k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "hazmat")
        {
          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(255,100,30), "DIAMOND", 0);
        }
        k++;
      }

      k = 0;
      for (auto wi: wi_array.array) {
        if (wi.type == "qr") {

          tf2::Quaternion q_tf(wi.pose.orientation.x,wi.pose.orientation.y,wi.pose.orientation.z,wi.pose.orientation.w);
          q_tf.inverse(); // tag to map quaternion
          tf2::Matrix3x3 m(q_tf); // Convert quaternion to matrix
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw); // Get Euler angles

          roll *= 180/M_PI;
          pitch *= 180/M_PI;
          yaw *= 180/M_PI;

          RCLCPP_INFO_STREAM(node_->get_logger(), roll << " " << pitch << " " << yaw);

          geotiff_writer.drawObjectOfInterest(Eigen::Vector2f(
            wi.pose.position.x, wi.pose.position.y),
            std::to_string(wi_array.id_array[k]), Eigen::Vector3f(181,101,29), "HALF_CIRCLE", 0);
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

      myfile.open(map_name_ + "_pois_ " + time_str + ".csv");
      myfile << "\"pois\"" << "\n" << "\"1.2\"" << "\n" << "\"ALeRT\"" << "\n" << "\"Germany\"" << "\n";

      myfile << wi_array.start_time;

      myfile << "\"" + mission_name_ + "\"" << "\n\n";
      myfile << "id,time,text,x,y,z,robot,mode,type";

      std::vector<std::string> final_world_data;
      final_world_data.resize(wi_array.array.size());

      for (int i = 0; i < final_world_data.size(); i++)
        final_world_data[wi_array.id_array[i]-1] = "\n" + // ids are starting from 1, hence id_array[i]-1. this conveniently sorts our data
                              std::to_string(wi_array.id_array[i]) + "," +
                              wi_array.time_array[i] + "," +
                              wi_array.array[i].num + "," +
                              std::to_string(wi_array.array[i].pose.position.x) + "," +
                              std::to_string(wi_array.array[i].pose.position.y) + "," +
                              std::to_string(wi_array.array[i].pose.position.z) + "," +
                              wi_array.robot_array[i] + "," +
                              wi_array.mode_array[i] + "," +
                              wi_array.array[i].type;

      for (auto wd: final_world_data)
        myfile << wd;

      myfile.close();
    };

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<world_info_msgs::msg::WorldInfoArray>::SharedPtr world_info_sub_;

    rclcpp::TimerBase::SharedPtr status_timer_;

    nav_msgs::msg::OccupancyGrid map;
    Eigen::Vector3f startVec;
    std::vector<Eigen::Vector2f> pointVec;

    world_info_msgs::msg::WorldInfoArray wi_array;
    std::string map_name_ = "";
    std::string mission_name_ = "";
    bool map_loaded = false;
    bool path_loaded = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("geotiff_saver");

  // Retrieve the (non-option) argument:
    std::string mission_name = "";
  if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
  {
    std::cerr << "ros2 run hector_geotiff geotiff_saver <Mission_name>" << std::endl;
    return -1;
  }
  else
  {
    mission_name = argv[argc-1];
  }

  GeotiffSaver gs(node, mission_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
