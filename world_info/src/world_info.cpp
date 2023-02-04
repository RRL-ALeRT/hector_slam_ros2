
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
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
world_info_msgs::msg::WorldInfoArray wi_vector;
visualization_msgs::msg::MarkerArray marker_array;
rclcpp::Publisher<world_info_msgs::msg::WorldInfoArray>::SharedPtr wi_pub;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;

geometry_msgs::msg::PoseStamped wrt_kinect;

std::unique_ptr<tf2_ros::Buffer> tf_;
std::shared_ptr<tf2_ros::TransformListener> tfL_{nullptr};

geometry_msgs::msg::PoseStamped transform_pose(rclcpp::Node::SharedPtr tp_node, geometry_msgs::msg::PoseStamped in, std::string target_frame) {
  geometry_msgs::msg::PoseStamped out;
  if (tf_->canTransform(target_frame, in.header.frame_id, tp_node->get_clock()->now(), rclcpp::Duration::from_seconds(0.5)))
  {
    auto source_to_target = tf_->lookupTransform(in.header.frame_id, target_frame, in.header.stamp);
    tf2::Transform t_source_to_target(
      tf2::Matrix3x3(tf2::Quaternion(
        source_to_target.transform.rotation.x,source_to_target.transform.rotation.y,source_to_target.transform.rotation.z,source_to_target.transform.rotation.w)),
      tf2::Vector3(source_to_target.transform.translation.x, source_to_target.transform.translation.y, source_to_target.transform.translation.z)
    );

    auto t_position = t_source_to_target.inverse()*(tf2::Vector3(in.pose.position.x, in.pose.position.y, in.pose.position.z));
    auto t_orientation = t_source_to_target.inverse()*(tf2::Quaternion(in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w));
    out.pose.position.x = t_position.getX();
    out.pose.position.y = t_position.getY();
    out.pose.position.z = t_position.getZ();
    out.pose.orientation.x = t_orientation.getX();
    out.pose.orientation.y = t_orientation.getY();
    out.pose.orientation.z = t_orientation.getZ();
    out.pose.orientation.w = t_orientation.getW();
    out.header.stamp = in.header.stamp;
    out.header.frame_id = target_frame;
  }
  return out;
}

void receive_info(const world_info_msgs::msg::WorldInfo::SharedPtr msg)
{
  info = *msg;
  if (info.type == "apriltag") {
    // Check in reverse order if incoming tag is already loaded.
    for(int i=wi_vector.array.size()-1; i>=0; i--) {
      if (wi_vector.array[i].type == info.type && wi_vector.array[i].num == info.num)
        return;
    }
    info.header.frame_id = "kinect";
    wrt_kinect.header = info.header;
    wrt_kinect.pose = info.pose;

    auto wrt_map = transform_pose(node, wrt_kinect, "map");
    if (!wrt_map.header.stamp.sec)
      return;

    info.header = wrt_map.header;
    info.pose = wrt_map.pose;

    uint32_t shape = visualization_msgs::msg::Marker::CUBE;
    visualization_msgs::msg::Marker marker;
    marker.header = info.header;
    marker.header.frame_id = "map";
    marker.ns = "apriltag";
    marker.id = wi_vector.array.size()+1;
    marker.type = shape;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = info.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker_array.markers.push_back(marker);
  }
  markers_pub->publish(marker_array);
  wi_vector.array.push_back(info);
  wi_pub->publish(wi_vector);
}


int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("world_info_server");

  auto sub = node->create_subscription<world_info_msgs::msg::WorldInfo>("world_info_sub", 1, &receive_info);
  wi_pub = node->create_publisher<world_info_msgs::msg::WorldInfoArray>("world_info_array", 1);
  markers_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);

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
