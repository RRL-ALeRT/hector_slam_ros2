
#include "rclcpp/rclcpp.hpp"

#include "world_info_msgs/msg/world_info.hpp"
#include "world_info_msgs/msg/world_info_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

rclcpp::Node::SharedPtr node;
world_info_msgs::msg::WorldInfo info;
rclcpp::Publisher<world_info_msgs::msg::WorldInfoArray>::SharedPtr wi_pub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;

visualization_msgs::msg::MarkerArray marker_array, empty_marker_array;
world_info_msgs::msg::WorldInfoArray wi_vector, empty_wi_vector;

std::unique_ptr<tf2_ros::Buffer> tf_;
std::shared_ptr<tf2_ros::TransformListener> tfL_{nullptr};

int id_count = 1;

struct PoseCount
{
  world_info_msgs::msg::WorldInfo info;
  int count;
  int id;
  std::string time;
  std::string robot;
  std::string mode;
};

std::unordered_map<std::string, PoseCount> apriltag_dict;
std::unordered_map<std::string, PoseCount> victim_dict;
std::unordered_map<std::string, PoseCount> qr_dict;

std::unordered_map<std::string, std::vector<PoseCount>> multiple_hazmat;

void transform_pose(const rclcpp::Node::SharedPtr tp_node, world_info_msgs::msg::WorldInfo& in, std::string target_frame) {
  auto source_to_target = tf_->lookupTransform(in.header.frame_id, target_frame, in.header.stamp, rclcpp::Duration::from_seconds(0.5));
  tf2::Transform t_source_to_target(
    tf2::Matrix3x3(tf2::Quaternion(
      source_to_target.transform.rotation.x,source_to_target.transform.rotation.y,source_to_target.transform.rotation.z,source_to_target.transform.rotation.w)),
    tf2::Vector3(source_to_target.transform.translation.x, source_to_target.transform.translation.y, source_to_target.transform.translation.z)
  );

  auto t_position = t_source_to_target.inverse()*(tf2::Vector3(in.pose.position.x, in.pose.position.y, in.pose.position.z));
  auto t_orientation = t_source_to_target.inverse()*(tf2::Quaternion(in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w));
  in.pose.position.x = t_position.getX();
  in.pose.position.y = t_position.getY();
  in.pose.position.z = t_position.getZ();
  in.pose.orientation.x = t_orientation.getX();
  in.pose.orientation.y = t_orientation.getY();
  in.pose.orientation.z = t_orientation.getZ();
  in.pose.orientation.w = t_orientation.getW();
  in.header.frame_id = target_frame;
}

void add_to_mean(geometry_msgs::msg::Pose& mean, const geometry_msgs::msg::Pose newPose, int& numPoses) {
  // Update position mean
  mean.position.x = (mean.position.x * numPoses + newPose.position.x) / (numPoses + 1);
  mean.position.y = (mean.position.y * numPoses + newPose.position.y) / (numPoses + 1);
  mean.position.z = (mean.position.z * numPoses + newPose.position.z) / (numPoses + 1);

  // Update orientation mean
  mean.orientation.x = (mean.orientation.x * numPoses + newPose.orientation.x) / (numPoses + 1);
  mean.orientation.y = (mean.orientation.y * numPoses + newPose.orientation.y) / (numPoses + 1);
  mean.orientation.z = (mean.orientation.z * numPoses + newPose.orientation.z) / (numPoses + 1);
  mean.orientation.w = (mean.orientation.w * numPoses + newPose.orientation.w) / (numPoses + 1);

  // Normalize the quaternion
  double norm = sqrt(
      mean.orientation.x * mean.orientation.x
    + mean.orientation.y * mean.orientation.y
    + mean.orientation.z * mean.orientation.z
    + mean.orientation.w * mean.orientation.w);
  mean.orientation.x /= norm;
  mean.orientation.y /= norm;
  mean.orientation.z /= norm;
  mean.orientation.w /= norm;

  numPoses++;
}

bool within_one_meter_range(const geometry_msgs::msg::Pose mean, const geometry_msgs::msg::Pose newPose) {
  if (std::abs(mean.position.x - newPose.position.x) > 1.0)
    if (std::abs(mean.position.y - newPose.position.y) > 1.0)
      if (std::abs(mean.position.z - newPose.position.z) > 1.0) return false;
  return true;
}

void receive_info(const world_info_msgs::msg::WorldInfo::SharedPtr msg)
{
  info = *msg;
  if (info.header.frame_id == "kinect_color") info.header.frame_id = "kinect"; // for webots

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%H:%M:%S");
  std::string time_str = oss.str();

  // APRILTAGS
  // Save whatever info we have in apriltag_dict(unordered_map) wrt map
  if (info.type == "apriltag") {

    if (apriltag_dict.find(info.num) != apriltag_dict.end())
      if (apriltag_dict[info.num].count >= 1000)
        return;

    try {
      transform_pose(node, info, "map");
    }
    catch (tf2::ConnectivityException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::ExtrapolationException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::LookupException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    
    if (apriltag_dict.find(info.num) == apriltag_dict.end())
      apriltag_dict[info.num] = PoseCount{info, 1, id_count++, time_str, std::string("Spot"), std::string("T")}; // First time
    else
      add_to_mean(apriltag_dict[info.num].info.pose, info.pose, apriltag_dict[info.num].count);
  }

  // QR
  // Save whatever info we have in qr_dict(unordered_map) wrt map
  if (info.type == "qr") {

    if (qr_dict.find(info.num) != qr_dict.end())
      if (qr_dict[info.num].count >= 1000)
        return;

    try {
      transform_pose(node, info, "map");
    }
    catch (tf2::ConnectivityException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::ExtrapolationException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::LookupException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    
    if (qr_dict.find(info.num) == qr_dict.end())
      qr_dict[info.num] = PoseCount{info, 1, id_count++, time_str, std::string("Spot"), std::string("T")}; // First time
    else
      add_to_mean(qr_dict[info.num].info.pose, info.pose, qr_dict[info.num].count);
  }

  // HAZMAT (FOR UNIQUE HAZMAT SYMBOLS)
  // Save whatever info we have in hazmat_dict(unordered_map) wrt map
  if (info.type == "hazmat") {
    
    try {
      transform_pose(node, info, "map");
    }
    catch (tf2::ConnectivityException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::ExtrapolationException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::LookupException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }

    bool hazmat_found = false;
    if (multiple_hazmat.find(info.num) != multiple_hazmat.end()) {
      for (auto single_hazmat: multiple_hazmat[info.num]) {
        if (within_one_meter_range(single_hazmat.info.pose, info.pose)) {
          if (single_hazmat.count <= 1000) {
            add_to_mean(single_hazmat.info.pose, info.pose, single_hazmat.count);
          }
          hazmat_found = true;
          break;
        }
      }
    }
    if (!hazmat_found) {
      PoseCount single_hazmat_dict;
      single_hazmat_dict = PoseCount{info, 1, id_count++, time_str, std::string("Spot"), std::string("T")}; // First time

      multiple_hazmat[info.num].push_back(single_hazmat_dict);
    }
  }

  // VICTIM (ONLY FOR SINGLE VICTIM)
  // Save whatever info we have in victim_dict(unordered_map) wrt map
  if (info.type == "victim") {

    if (victim_dict.find(info.num) != victim_dict.end())
      if (victim_dict[info.num].count >= 1000)
        return;

    try {
      transform_pose(node, info, "map");
    }
    catch (tf2::ConnectivityException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::ExtrapolationException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    catch (tf2::LookupException& e) {
      RCLCPP_WARN(node->get_logger(), e.what());
      return;
    }
    
    if (victim_dict.find(info.num) == victim_dict.end())
      victim_dict[info.num] = PoseCount{info, 1, id_count++, time_str, std::string("Spot"), std::string("T")}; // First time
    else
      add_to_mean(victim_dict[info.num].info.pose, info.pose, victim_dict[info.num].count);
  }

  // get rviz2 markers and tag_locations from various dicts
  marker_array = empty_marker_array;
  wi_vector = empty_wi_vector;

  for (auto it = apriltag_dict.begin(); it != apriltag_dict.end(); ++it) {
    uint32_t shape = visualization_msgs::msg::Marker::CUBE;
    visualization_msgs::msg::Marker marker;
    marker.header = it->second.info.header;
    marker.header.frame_id = "map";
    marker.ns = "apriltag";
    marker.id = marker_array.markers.size()+1;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = it->second.info.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker_array.markers.push_back(marker);

    wi_vector.array.push_back(it->second.info);
    wi_vector.id_array.push_back(it->second.id);
    wi_vector.time_array.push_back(it->second.time);
    wi_vector.robot_array.push_back(it->second.robot);
    wi_vector.mode_array.push_back(it->second.mode);
  }
  for (auto it = qr_dict.begin(); it != qr_dict.end(); ++it) {
    uint32_t shape = visualization_msgs::msg::Marker::CUBE;
    visualization_msgs::msg::Marker marker;
    marker.header = it->second.info.header;
    marker.header.frame_id = "map";
    marker.ns = "qr";
    marker.id = marker_array.markers.size()+1;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = it->second.info.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker_array.markers.push_back(marker);

    wi_vector.array.push_back(it->second.info);
    wi_vector.id_array.push_back(it->second.id);
    wi_vector.time_array.push_back(it->second.time);
    wi_vector.robot_array.push_back(it->second.robot);
    wi_vector.mode_array.push_back(it->second.mode);
  }
  for (auto single_hazmat_vector = multiple_hazmat.begin(); single_hazmat_vector != multiple_hazmat.end(); ++single_hazmat_vector) {
    for (auto it: single_hazmat_vector->second)
    {
      uint32_t shape = visualization_msgs::msg::Marker::CUBE;
      visualization_msgs::msg::Marker marker;
      marker.header = it.info.header;
      marker.header.frame_id = "map";
      marker.ns = "hazmat";
      marker.id = marker_array.markers.size()+1;
      marker.type = shape;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = it.info.pose;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker_array.markers.push_back(marker);

      wi_vector.array.push_back(it.info);
      wi_vector.id_array.push_back(it.id);
      wi_vector.time_array.push_back(it.time);
      wi_vector.robot_array.push_back(it.robot);
      wi_vector.mode_array.push_back(it.mode);
    }
  }
  for (auto it = victim_dict.begin(); it != victim_dict.end(); ++it) {
    uint32_t shape = visualization_msgs::msg::Marker::CUBE;
    visualization_msgs::msg::Marker marker;
    marker.header = it->second.info.header;
    marker.header.frame_id = "map";
    marker.ns = "victim";
    marker.id = marker_array.markers.size()+1;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = it->second.info.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker_array.markers.push_back(marker);

    wi_vector.array.push_back(it->second.info);
    wi_vector.id_array.push_back(it->second.id);
    wi_vector.time_array.push_back(it->second.time);
    wi_vector.robot_array.push_back(it->second.robot);
    wi_vector.mode_array.push_back(it->second.mode);
  }

  markers_pub->publish(marker_array);
  wi_pub->publish(wi_vector);
}

void statusTimer()
{
  markers_pub->publish(marker_array);
  wi_pub->publish(wi_vector);
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("world_info_server");

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream csv_start_time;
  csv_start_time << "\"" << std::put_time(&tm, "%Y-%m-%d") << "\"" << "\n" << "\"" << std::put_time(&tm, "%H:%M:%S") << "\"" << "\n";
  empty_wi_vector.start_time = csv_start_time.str();
  wi_vector = empty_wi_vector;

  auto sub = node->create_subscription<world_info_msgs::msg::WorldInfo>("world_info_sub", 1, &receive_info);
  wi_pub = node->create_publisher<world_info_msgs::msg::WorldInfoArray>("world_info_array", 1);
  markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);
  auto status_timer_ = node->create_wall_timer(std::chrono::milliseconds(10), &statusTimer);

  double tmp_val = 30;
  tf_ = std::make_unique<tf2_ros::Buffer>(node->get_clock(),
      tf2::durationFromSec(tmp_val));
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    rclcpp::node_interfaces::get_node_base_interface(node),
    rclcpp::node_interfaces::get_node_timers_interface(node));
  tf_->setCreateTimerInterface(timer_interface);
  tfL_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
