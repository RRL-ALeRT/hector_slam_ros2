#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/empty.hpp>
#include <hector_nav_msgs/srv/get_robot_trajectory.hpp>

#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using Empty = std_srvs::srv::Empty;
using GetRT = hector_nav_msgs::srv::GetRobotTrajectory;
using namespace std::placeholders;

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

    reset_server = create_service<Empty>("reset_map_frame_and_travelled_path", std::bind(&FrameListener::map_frame_and_path_reset_handle_service, this, _1, _2, _3));
    
    robot_trajectory_server = create_service<GetRT>("get_robot_trajectory", std::bind(&FrameListener::handle_robot_trajectory_service, this, _1, _2, _3));

    // Call on_timer function every second
    timer_ = create_wall_timer(0.1s, std::bind(&FrameListener::on_timer, this));

    if (!has_parameter("base_frame")) declare_parameter("base_frame", base_frame);
    get_parameter("base_frame", base_frame);

    if (!has_parameter("odom_frame")) declare_parameter("odom_frame", odom_frame);
    get_parameter("odom_frame", odom_frame);
  }

private:
  void map_frame_and_path_reset_handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Empty::Request> request,
    const std::shared_ptr<Empty::Response> response)
  {
    (void)request_header;
    map_frame_published = false;
    positions.clear();
    last_published_index = 0;
    RCLCPP_INFO(get_logger(), "travelled path resetted");
  }

  void handle_robot_trajectory_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<GetRT::Request> request,
    const std::shared_ptr<GetRT::Response> response)
  {
    (void)request_header;

    for (auto& p: positions) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.pose.position = p;
      response->trajectory.poses.push_back(pose_stamped);
    }
    RCLCPP_INFO(get_logger(), "sent path");
  }

  bool points_are_far_enough(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
  {
    if (std::abs(p1.x - p2.x) > 0.1) return true;
    if (std::abs(p1.y - p2.y) > 0.05) return true;
    if (std::abs(p1.z - p2.z) > 0.05) return true;
    return false;
  }

  void on_timer()
  {
    if (!map_frame_published) {
      try {
        std::string toFrameRel = base_frame;
        std::string fromFrameRel = odom_frame;
        t = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero);
        t.header.frame_id = "map";
        t.child_frame_id = odom_frame;
        t.transform.translation.z -= 0.094;

        auto q = tf2::Quaternion{t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w};
        tf2::Matrix3x3 m;
        m.setRotation(q);

        tf2Scalar roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion q_only_way;
        q_only_way.setRPY(0, 0, yaw);

        t.transform.rotation.x = q_only_way.getX();
        t.transform.rotation.y = q_only_way.getY();
        t.transform.rotation.z = q_only_way.getZ();
        t.transform.rotation.w = q_only_way.getW();

        map_frame_published = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(get_logger(), "%s", ex.what());
        return;
      }
    }
    tf_static_broadcaster_->sendTransform(t);

    try {
      std::string toFrameRel = "map";
      std::string fromFrameRel = base_frame;
      auto t_map_base = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);

      geometry_msgs::msg::Point position;
      position.x = t_map_base.transform.translation.x;
      position.y = t_map_base.transform.translation.y;
      position.z = t_map_base.transform.translation.z;

      if (positions.size() == 0) {
        positions.push_back(position);
      }
      else if (points_are_far_enough(position, positions.back())) {
        positions.push_back(position);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(get_logger(), "%s", ex.what());
      return;
    }

    if (previous_sub_count != travelled_path_pub->get_subscription_count()) {
      previous_sub_count = travelled_path_pub->get_subscription_count();
      last_published_index = 0;
    }

    if (positions.size() > last_published_index + 2) {
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
      for (auto i = std::max(0, static_cast<int>(positions.size() - 100)); i < positions.size(); ++i) {
        path_marker.points.push_back(positions[i]);
      }

      // Publish the updated path marker
      travelled_path_pub->publish(path_marker);

      // Update the last published index
      last_published_index = positions.size();
    }
  }

  geometry_msgs::msg::TransformStamped t, t_map_body;
  std::string base_frame = "body";
  std::string odom_frame = "vision";
  bool map_frame_published = false;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr travelled_path_pub;
  rclcpp::Service<Empty>::SharedPtr reset_server;
  rclcpp::Service<GetRT>::SharedPtr robot_trajectory_server;

  std::vector<geometry_msgs::msg::Point> positions;

  // Keep track of the index of the last published position
  unsigned long int last_published_index = 0;

  int previous_sub_count = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}