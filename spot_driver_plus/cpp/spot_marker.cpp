#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "interactive_markers/interactive_marker_server.hpp"

class MyInteractiveMarker : public rclcpp::Node
{
public:
  MyInteractiveMarker()
  : Node("interactive_marker_node")
  {
    // Set up publisher for pose messages
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/body_pose", 1);

    // Set up interactive marker server
    interactive_marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_marker_server", this);
    interactive_marker_server_->applyChanges();

    // Create interactive marker
    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = "body";
    marker.name = "my_marker";
    marker.description = "Interactive Marker";

    // Create control for visual representation
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeCubeMarker());
    marker.controls.push_back(control);

    // Create control for roll
    visualization_msgs::msg::InteractiveMarkerControl roll_control;
    roll_control.orientation.w = 1.0;
    roll_control.orientation.x = 1.0;
    roll_control.orientation.y = 0.0;
    roll_control.orientation.z = 0.0;
    roll_control.name = "roll";
    roll_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(roll_control);

    // Create control for pitch
    visualization_msgs::msg::InteractiveMarkerControl pitch_control;
    pitch_control.orientation.w = 1.0;
    pitch_control.orientation.x = 0.0;
    pitch_control.orientation.y = 1.0;
    pitch_control.orientation.z = 0.0;
    pitch_control.name = "pitch";
    pitch_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(pitch_control);

    // Create control for yaw
    visualization_msgs::msg::InteractiveMarkerControl yaw_control;
    yaw_control.orientation.w = 1.0;
    yaw_control.orientation.x = 0.0;
    yaw_control.orientation.y = 0.0;
    yaw_control.orientation.z = 1.0;
    yaw_control.name = "yaw";
    yaw_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(yaw_control);

    // Create control for height
    visualization_msgs::msg::InteractiveMarkerControl height_control;
    height_control.orientation.w = 1.0;
    height_control.orientation.x = 0.0;
    height_control.orientation.y = 1.0;
    height_control.orientation.z = 0.0;
    height_control.name = "height";
    height_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(height_control);

    // Add marker to server
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, std::bind(&MyInteractiveMarker::markerCallback, this, std::placeholders::_1));

    // Apply changes and start server
    interactive_marker_server_->applyChanges();
  }

  visualization_msgs::msg::Marker makeCubeMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;

    return marker;
  }

private:
  // Callback function for interactive marker
  void markerCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    // Publish pose
    pose_publisher_->publish(feedback->pose);

    // For smoothness
    rclcpp::Rate(10).sleep();
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyInteractiveMarker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
