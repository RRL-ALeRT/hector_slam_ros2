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
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/srv/get_map.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <QApplication>

using namespace std;
using std::placeholders::_1;


class GeotiffSaver
{
  public:
    GeotiffSaver(rclcpp::Node::SharedPtr node, string map_name) : node_(node), map_name_(map_name) {
      path_sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/slam_toolbox/graph_visualization", 1, std::bind(&GeotiffSaver::pathCallback, this, _1));

      map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 1, std::bind(&GeotiffSaver::mapCallback, this, _1));
    }

  private:
    void pathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr path)
    {
      auto marker_array = path->markers;
      int num_markers = marker_array.size();
      for (int i = 0; i < num_markers; i++) {
        auto num_points = marker_array[i].points.size();

        if (marker_array[i].ns == "slam_toolbox_edges" && num_points > 0) {
          startVec[0] = marker_array[i].points[0].x;
          startVec[1] = marker_array[i].points[0].y;

          pointVec.resize(num_points);
          for (long unsigned int j = 0; j < num_points; j++) {
            pointVec[j] = Eigen::Vector2f(marker_array[i].points[j].x, marker_array[i].points[j].y);
          }

          RCLCPP_INFO(node_->get_logger(), "Path loaded.");
          path_loaded = true;

          if (map_loaded)
            saveMap ();
        }
      }
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
    {
      map = *map_msg;
      map_loaded = true;
      if (path_loaded)
        saveMap ();
    };

    void saveMap () {
        RCLCPP_INFO(node_->get_logger(), "Map loaded.");
        hector_geotiff::GeotiffWriter geotiff_writer(false);

        geotiff_writer.setMapFileName(map_name_);
        geotiff_writer.setupTransforms(map);
        geotiff_writer.setupImageSize();
        geotiff_writer.drawBackgroundCheckerboard();
        geotiff_writer.drawMap(map);
        geotiff_writer.drawCoords();
        geotiff_writer.drawPath(startVec, pointVec);

        geotiff_writer.writeGeotiffImage(true);
        rclcpp::shutdown();
    };

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    nav_msgs::msg::OccupancyGrid map;
    Eigen::Vector3f startVec;
    std::vector<Eigen::Vector2f> pointVec;

    string map_name_ = "";
    bool map_loaded = false;
    bool path_loaded = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("geotiff_saver");

  string map_name = "RoboCup2023-FHAachen-Mission1";

  GeotiffSaver gs(node, map_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
