#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/srv/empty.hpp>
#include <hector_nav_msgs/srv/get_robot_trajectory.hpp>

#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using Empty = std_srvs::srv::Empty;
using GetRobotTrajectory = hector_nav_msgs::srv::GetRobotTrajectory;
using namespace std::placeholders;

geometry_msgs::msg::TransformStamped invertTransform(const geometry_msgs::msg::TransformStamped& input_transform) {
    tf2::Transform tf_transform;
    tf2::fromMsg(input_transform.transform, tf_transform);

    // Invert the transformation
    tf_transform = tf_transform.inverse();

    geometry_msgs::msg::TransformStamped output_transform;
    output_transform.header = input_transform.header;
    output_transform.header.frame_id = input_transform.header.frame_id;
    output_transform.child_frame_id = input_transform.child_frame_id;

    // Convert the inverted transform back to geometry_msgs::msg::TransformStamped
    tf2::toMsg(tf_transform, output_transform.transform);

    return output_transform;
}

class FrameListener : public rclcpp::Node
{
public:
  explicit FrameListener(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("map_path_node", options)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    travelled_path_pub = create_publisher<visualization_msgs::msg::Marker>("travelled_path", 1);

    odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 1, std::bind(&FrameListener::odom_cb, this, _1));

    reset_server = create_service<Empty>("reset_travelled_path", std::bind(&FrameListener::reset_map_frame_handle_service, this, _1, _2, _3));

    robot_trajectory_server = create_service<GetRobotTrajectory>("get_robot_trajectory", std::bind(&FrameListener::handle_robot_trajectory_service, this, _1, _2, _3));

    if (!has_parameter("odom_frame")) declare_parameter("odom_frame", odom_frame);
    get_parameter("odom_frame", odom_frame);

    transform_timer_ = create_wall_timer(500ms, std::bind(&FrameListener::query_transform, this));
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto last_pose = msg->pose.pose;
    auto last_stamp = msg->header.stamp;
    map_vision_static_transform(last_pose, last_stamp);
    tf2::doTransform(last_pose, last_pose, map_vision_tf2);
    add_to_path(last_pose);
  }

  void reset_map_frame_handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Empty::Request> request,
    const std::shared_ptr<Empty::Response> response)
  {
    (void)request_header;
    map_frame_published = false;
    poses.clear();
    RCLCPP_INFO(get_logger(), "travelled path resetted");
  }

  void handle_robot_trajectory_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRobotTrajectory::Request> request,
    const std::shared_ptr<GetRobotTrajectory::Response> response)
  {
    (void)request_header;

    for (auto& pose: poses) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose = pose;
      response->trajectory.poses.push_back(pose_stamped);
    }
    RCLCPP_INFO(get_logger(), "sent path");
  }

  void map_vision_static_transform(const geometry_msgs::msg::Pose& last_pose, const rclcpp::Time& last_stamp)
  {
    if (map_frame_published) return;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = last_stamp;
    t.header.frame_id = "map";
    t.child_frame_id = odom_frame;

    t.transform.translation.x = last_pose.position.x;
    t.transform.translation.y = last_pose.position.y;
    t.transform.translation.z = last_pose.position.z - ground_plane_vertical_distance;

    auto q = tf2::Quaternion{
      last_pose.orientation.x,
      last_pose.orientation.y,
      last_pose.orientation.z,
      last_pose.orientation.w
    };

    tf2::Matrix3x3 m;
    m.setRotation(q);

    tf2Scalar roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf2::Quaternion q_only_yaw;
    q_only_yaw.setRPY(0, 0, yaw);

    t.transform.rotation.x = q_only_yaw.getX();
    t.transform.rotation.y = q_only_yaw.getY();
    t.transform.rotation.z = q_only_yaw.getZ();
    t.transform.rotation.w = q_only_yaw.getW();

    auto t_inverted = invertTransform(t);

    tf_static_broadcaster_->sendTransform(t_inverted);

    map_vision_tf2 = t_inverted;

    map_frame_published = true;
  }

  bool points_are_far_enough(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
  {
    if (std::abs(p1.x - p2.x) > 0.05) return true;
    if (std::abs(p1.y - p2.y) > 0.05) return true;
    if (std::abs(p1.z - p2.z) > 0.05) return true;
    return false;
  }

  void add_to_path(const geometry_msgs::msg::Pose& pose)
  {
    if (poses.size() == 0) {
      poses.push_back(pose);
    }
    else if (points_are_far_enough(pose.position, poses.back().position)) {
      poses.push_back(pose);
    }

    // Create a new marker for the updated portion of the path
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.type = path_marker.LINE_STRIP;
    path_marker.action = path_marker.ADD;
    path_marker.scale.x = 0.05;
    path_marker.color.a = 0.6;
    path_marker.color.r = 0.1;
    path_marker.color.g = 0.6;
    path_marker.color.b = 0.8;

    // Add the new points to the marker
    for (auto i = std::max(0, int(poses.size() - 100)); i < poses.size(); ++i) {
      path_marker.points.push_back(poses[i].position);
    }

    // Publish the updated path marker
    travelled_path_pub->publish(path_marker);
  }

  void query_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("gpe", "body", tf2::TimePointZero);
      ground_plane_vertical_distance = transform.transform.translation.z;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Could not transform 'gpe' to 'body': %s", ex.what());
    }
  }

  geometry_msgs::msg::TransformStamped map_vision_tf2;
  std::string odom_frame = "vision";
  bool map_frame_published = false;
  float ground_plane_vertical_distance = 0.092;

  rclcpp::TimerBase::SharedPtr map_frame_timer_{nullptr};
  rclcpp::TimerBase::SharedPtr path_timer_{nullptr};
  rclcpp::TimerBase::SharedPtr transform_timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr travelled_path_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

  rclcpp::Service<Empty>::SharedPtr reset_server;
  rclcpp::Service<GetRobotTrajectory>::SharedPtr robot_trajectory_server;

  std::vector<geometry_msgs::msg::Pose> poses;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}