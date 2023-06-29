#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("map_vision_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("/map_to_body", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      0.5s, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = "body";
    std::string toFrameRel = "vision";

    if (!tf2_published) {
      try {
        t = tf_buffer_->lookupTransform(
          fromFrameRel, toFrameRel,
          tf2::TimePointZero);
        tf2_published = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
        return;
      }
    }

    t.header.frame_id = "map";
    t.child_frame_id = "vision";
    tf_static_broadcaster_->sendTransform(t);

    try {
      t_map_body = tf_buffer_->lookupTransform(
        "body", "map",
        tf2::TimePointZero);
      pub->publish(t_map_body);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "%s", ex.what());
      return;
    }
  }

  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::TransformStamped t_map_body;

  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub;

  bool tf2_published = false;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}