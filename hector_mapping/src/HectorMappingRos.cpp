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

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#include "tf2/convert.h"
#include "tf2_ros/create_timer_ros.h"

#include "boost/lexical_cast.hpp"

// #ifndef TF_SCALAR_H
//   typedef btScalar tf2Scalar;
// #endif

using std::placeholders::_1;

HectorMappingRos::HectorMappingRos(rclcpp::Node::SharedPtr node)
  : node_(node)
  , debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(true)
  , pause_scan_processing_(false)
{
  double tmp_val = 30;
  tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock(),
      tf2::durationFromSec(tmp_val));
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rclcpp::node_interfaces::get_node_base_interface(node_),
    rclcpp::node_interfaces::get_node_timers_interface(node_));
  tf_->setCreateTimerInterface(timer_interface);
  tfL_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  tfB_ = new tf2_ros::TransformBroadcaster(node_);

  std::string mapTopic_ = "map";

  p_pub_drawings = node_->declare_parameter("pub_drawings", false);
  p_pub_debug_output_ = node_->declare_parameter("pub_debug_output", false);
  p_pub_map_odom_transform_ = node_->declare_parameter("pub_map_odom_transform", true);
  p_pub_odometry_ = node_->declare_parameter("pub_odometry", true);
  p_advertise_map_service_ = node_->declare_parameter("advertise_map_service", true);
  p_scan_subscriber_queue_size_ = node_->declare_parameter("scan_subscriber_queue_size", 5);

  p_map_resolution_= node_->declare_parameter("map_resolution", 0.025);
  p_map_size_= node_->declare_parameter("map_size", 1024);
  p_map_start_x_= node_->declare_parameter("map_start_x", 0.5);
  p_map_start_y_= node_->declare_parameter("map_start_y", 0.5);
  p_map_multi_res_levels_ = node_->declare_parameter("map_multi_res_levels", 3);

  p_update_factor_free_ = node_->declare_parameter("update_factor_free", 0.4);
  p_update_factor_occupied_ = node_->declare_parameter("update_factor_occupied", 0.9);

  p_map_update_distance_threshold_ = node_->declare_parameter("map_update_distance_thresh", 0.4);
  p_map_update_angle_threshold_ = node_->declare_parameter("map_update_angle_thresh", 0.9);

  p_scan_topic_ = node_->declare_parameter("scan_topic", "/scan");
  p_sys_msg_topic_ = node_->declare_parameter("sys_msg_topic", "syscommand");
  p_pose_update_topic_ = node_->declare_parameter("pose_update_topic", "poseupdate");

  p_use_tf_scan_transformation_ = node_->declare_parameter("use_tf_scan_transformation", false);
  p_use_tf_pose_start_estimate_ = node_->declare_parameter("use_tf_pose_start_estimate", false);
  p_map_with_known_poses_ = node_->declare_parameter("map_with_known_poses", false);

  p_base_frame_ = node_->declare_parameter("base_frame", "base_link");
  p_map_frame_ = node_->declare_parameter("map_frame", "map");
  p_odom_frame_ = node_->declare_parameter("odom_frame", "odom");

  p_pub_map_scanmatch_transform_ = node_->declare_parameter("pub_map_scanmatch_transform", true);
  p_tf_map_scanmatch_transform_frame_name_ = node_->declare_parameter("tf_map_scanmatch_transform_frame_name", "Velodyne_Puck");

  p_timing_output_ = node_->declare_parameter("output_timing", false);

  p_map_pub_period_ = node_->declare_parameter("map_pub_period", 2.0);

  double tmp;
  tmp = node_->declare_parameter("laser_min_dist", 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  tmp = node_->declare_parameter("laser_max_dist", 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  tmp = node_->declare_parameter("laser_z_min_value", -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  tmp = node_->declare_parameter("laser_z_max_value", 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    RCLCPP_INFO(node_->get_logger(), "HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings(node_);
  }

  if(p_pub_debug_output_)
  {
    RCLCPP_INFO(node_->get_logger(), "HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider(node_);
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopicStr, 1);
    tmp.mapMetadataPublisher_ = node_->create_publisher<nav_msgs::msg::MapMetaData>(mapMetaTopicStr, 1);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_->create_service<nav_msgs::srv::GetMap>("dynamic_map", std::bind(&HectorMappingRos::mapCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_->publish(mapPubContainer[i].map_.map.info);
    }
  }

  // Initialize services
  reset_map_service_ = node_->create_service<std_srvs::srv::Trigger>("reset_map", std::bind(&HectorMappingRos::resetMapCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  restart_hector_service_ = node_->create_service<hector_nav_msgs::srv::ResetMapping>("restart_mapping_with_new_pose", std::bind(&HectorMappingRos::restartHectorCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  toggle_scan_processing_service_ = node_->create_service<std_srvs::srv::SetBool>("pause_mapping", std::bind(&HectorMappingRos::pauseMapCallback, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  RCLCPP_INFO(node_->get_logger(), "HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  scanSubscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(p_scan_topic_, qos, std::bind(&HectorMappingRos::scanCallback, this, _1));
  sysMsgSubscriber_ = node_->create_subscription<std_msgs::msg::String>(p_sys_msg_topic_, 2, std::bind(&HectorMappingRos::sysMsgCallback, this, _1));

  poseUpdatePublisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(p_pose_update_topic_, 1);
  posePublisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("slam_out_pose", 1);

  scan_point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("slam_cloud",1);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ =
    node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&HectorMappingRos::initialPoseCallback, this, std::placeholders::_1));

  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  lastMapPublishTime = rclcpp::Time(0,0);
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;
}

void HectorMappingRos::scanCallback(const sensor_msgs::msg::LaserScan& scan)
{

  if (pause_scan_processing_)
  {
    return;
  }

  if (hectorDrawings)
  {
    hectorDrawings->setTime(scan.header.stamp);
  }

  auto start_time = node_->get_clock()->now().seconds();
  if (!p_use_tf_scan_transformation_)
  {
    // If we are not using the tf tree to find the transform between the base frame and laser frame,
    // then just convert the laser scan to our data container and process the update based on our last
    // pose estimate
    this->rosLaserScanToDataContainer(scan, laserScanContainer, slamProcessor->getScaleToMap());
    slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());
  }
  else
  {
    // If we are using the tf tree to find the transform between the base frame and laser frame,
    // let's get that transform
    geometry_msgs::msg::TransformStamped laser_transform;
    if (tf_->canTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, rclcpp::Duration::from_seconds(0.5)))
    {
      laser_transform = tf_->lookupTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), scan.header.frame_id.c_str());
      return;
    }

    // Convert the laser scan to point cloud
    projector_.projectLaser(scan, laser_point_cloud_, 30.0);

    // Publish the point cloud if there are any subscribers
    if (scan_point_cloud_publisher_->get_subscription_count() > 0)
    {
      scan_point_cloud_publisher_->publish(laser_point_cloud_);
    }

    // Convert the point cloud to our data container
    this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform, laserScanContainer, slamProcessor->getScaleToMap());

    // Now let's choose the initial pose estimate for our slam process update
    Eigen::Vector3f start_estimate(Eigen::Vector3f::Zero());
    if (initial_pose_set_)
    {
      // User has requested a pose reset
      initial_pose_set_ = false;
      start_estimate = initial_pose_;
    }
    else if (p_use_tf_pose_start_estimate_)
    {
      // Initial pose estimate comes from the tf tree
      if (tf_->canTransform(p_map_frame_, p_base_frame_, scan.header.stamp, rclcpp::Duration::from_seconds(0.5)))
      {
        geometry_msgs::msg::TransformStamped stamped_pose;

        stamped_pose = tf_->lookupTransform(p_map_frame_, p_base_frame_, scan.header.stamp);

        tf2::Quaternion tmp_(
          stamped_pose.transform.rotation.x,
          stamped_pose.transform.rotation.y,
          stamped_pose.transform.rotation.z,
          stamped_pose.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

        start_estimate = Eigen::Vector3f(stamped_pose.transform.translation.x, stamped_pose.transform.translation.y, yaw);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
        start_estimate = slamProcessor->getLastScanMatchPose();
      }
    }
    else
    {
      // If none of the above, the initial pose is simply the last estimated pose
      start_estimate = slamProcessor->getLastScanMatchPose();
    }

    // If "p_map_with_known_poses_" is enabled, we assume that start_estimate is precise and doesn't need to be refined
    if (p_map_with_known_poses_)
    {
      slamProcessor->update(laserScanContainer, start_estimate, true);
    }
    else
    {
      slamProcessor->update(laserScanContainer, start_estimate);
    }
  }

  // If the debug flag "p_timing_output_" is enabled, print how long this last iteration took
  if (p_timing_output_)
  {
    auto duration = node_->get_clock()->now().seconds() - start_time;
    RCLCPP_INFO(node_->get_logger(), "HectorSLAM Iter took: %f milliseconds", duration );
  }

  // If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
  if (p_map_with_known_poses_)
  {
    return;
  }

  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

  // Publish pose with and without covariances
  poseUpdatePublisher_->publish(poseInfoContainer_.getPoseWithCovarianceStamped());
  posePublisher_->publish(poseInfoContainer_.getPoseStamped());

  // Publish odometry if enabled
  if(p_pub_odometry_)
  {
    nav_msgs::msg::Odometry tmp;
    tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

    tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
    tmp.child_frame_id = p_base_frame_;
    odometryPublisher_->publish(tmp);
  }

  // Publish the map->odom transform if enabled
  if (p_pub_map_odom_transform_)
  {
    geometry_msgs::msg::TransformStamped odom_to_base;

    if (tf_->canTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, rclcpp::Duration::from_seconds(0.5)))
    {
      odom_to_base = tf_->lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "lookupTransform %s to %s timed out. Could not transform map into odom_frame.", p_odom_frame_.c_str(), p_base_frame_.c_str());
      return;
    }

    auto map_to_base = poseInfoContainer_.getTfTransform();

    tf2::Transform t_odom_to_base(
      tf2::Matrix3x3(tf2::Quaternion(
        odom_to_base.transform.rotation.x,odom_to_base.transform.rotation.y,odom_to_base.transform.rotation.z,odom_to_base.transform.rotation.w)),
      tf2::Vector3(odom_to_base.transform.translation.x, odom_to_base.transform.translation.y, odom_to_base.transform.translation.z)
    );
    tf2::Transform t_map_to_base(
      tf2::Matrix3x3(tf2::Quaternion(
        map_to_base.transform.rotation.x,map_to_base.transform.rotation.y,map_to_base.transform.rotation.z,map_to_base.transform.rotation.w)),
      tf2::Vector3(map_to_base.transform.translation.x, map_to_base.transform.translation.y, map_to_base.transform.translation.z)
    );

    geometry_msgs::msg::TransformStamped map_to_odom_;
    auto t_map_to_odom = t_map_to_base * t_odom_to_base.inverse();
    map_to_odom_.header.stamp = scan.header.stamp;
    map_to_odom_.transform.rotation.x = t_map_to_odom.getRotation().x();
    map_to_odom_.transform.rotation.y = t_map_to_odom.getRotation().y();
    map_to_odom_.transform.rotation.z = t_map_to_odom.getRotation().z();
    map_to_odom_.transform.rotation.w = t_map_to_odom.getRotation().w();
    map_to_odom_.transform.translation.x = t_map_to_odom.getOrigin().x();
    map_to_odom_.transform.translation.y = t_map_to_odom.getOrigin().y();
    map_to_odom_.transform.translation.z = t_map_to_odom.getOrigin().z();
    map_to_odom_.header.stamp = scan.header.stamp;
    map_to_odom_.header.frame_id = p_map_frame_;
    map_to_odom_.child_frame_id = p_odom_frame_;

    tfB_->sendTransform(map_to_odom_);
  }

  // Publish the transform from map to estimated pose (if enabled)
  if (p_pub_map_scanmatch_transform_)
  {
    auto map_to_scanmatch_ = poseInfoContainer_.getTfTransform();
    map_to_scanmatch_.child_frame_id = p_tf_map_scanmatch_transform_frame_name_;

    tfB_->sendTransform(map_to_scanmatch_);
  }
}

void HectorMappingRos::sysMsgCallback(const std_msgs::msg::String& string)
{
  RCLCPP_INFO(node_->get_logger(), "HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    RCLCPP_INFO(node_->get_logger(), "HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::mapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<nav_msgs::srv::GetMap::Request> req, 
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> resp)
{
  RCLCPP_INFO(node_->get_logger(), "HectorSM Map service called");
  *resp = mapPubContainer[0].map_;
  return true;
}

bool HectorMappingRos::resetMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                        const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  RCLCPP_INFO(node_->get_logger(), "HectorSM Reset map service called");
  slamProcessor->reset();
  return true;
}

bool HectorMappingRos::restartHectorCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                             const std::shared_ptr<hector_nav_msgs::srv::ResetMapping::Request> req, 
                                             std::shared_ptr<hector_nav_msgs::srv::ResetMapping::Response> resp)
{
  // Reset map
  RCLCPP_INFO(node_->get_logger(), "HectorSM Reset map");
  slamProcessor->reset();

  // Reset pose
  this->resetPose(req->initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success
  return true;
}

bool HectorMappingRos::pauseMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                             const std::shared_ptr<std_srvs::srv::SetBool::Request> req, 
                                             std::shared_ptr<std_srvs::srv::SetBool::Response> resp)
{
  this->toggleMappingPause(req->data);
  resp->success = true;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, rclcpp::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::srv::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_->publish(map_.map);
}

void HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::msg::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::msg::PointCloud2& pointCloud2, const geometry_msgs::msg::TransformStamped& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  sensor_msgs::msg::PointCloud pointCloud;
  sensor_msgs::convertPointCloud2ToPointCloud(pointCloud2, pointCloud);

  size_t size = pointCloud.points.size();
  // RCLCPP_INFO(node_->get_logger(), "size: %d", size);

  dataContainer.clear();

  tf2::Vector3 laserPos (laserTransform.transform.translation.x, laserTransform.transform.translation.y, laserTransform.transform.translation.z);
  dataContainer.setOrigo(Eigen::Vector2f(laserTransform.transform.translation.x, laserTransform.transform.translation.y)*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {
    const geometry_msgs::msg::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf2::Quaternion q(
        laserTransform.transform.rotation.x,
        laserTransform.transform.rotation.y,
        laserTransform.transform.rotation.z,
        laserTransform.transform.rotation.w);
      tf2::Vector3 v(laserTransform.transform.translation.x,laserTransform.transform.translation.y,laserTransform.transform.translation.z);
      tf2::Transform laserTransform_(q, v);

      tf2::Vector3 pointPosBaseFrame(laserTransform_ * tf2::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::srv::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  rclcpp::Rate r(1.0 / map_pub_period);
  while(rclcpp::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    auto mapTime = node_->get_clock()->now();
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::msg::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg)
{
  this->resetPose(msg.pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    RCLCPP_INFO(node_->get_logger(), "[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    RCLCPP_INFO(node_->get_logger(), "[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::msg::Pose &pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
  RCLCPP_INFO(node_->get_logger(), "[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}
