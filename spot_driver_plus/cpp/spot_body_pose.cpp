#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "gpp_action_examples_interface/srv/spot_body_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class BodyPosePublisher : public rclcpp::Node
{
public:
  BodyPosePublisher() : Node("body_pose_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/body_pose", 1);
    service_ = this->create_service<gpp_action_examples_interface::srv::SpotBodyPose>("body_pose_service", std::bind(&BodyPosePublisher::handle_service_request, this, _1, _2));
  }

private:
  void handle_service_request(const std::shared_ptr<gpp_action_examples_interface::srv::SpotBodyPose::Request> request,
                              std::shared_ptr<gpp_action_examples_interface::srv::SpotBodyPose::Response> response)
  {
    // Publish the pose to the topic
    publisher_->publish(request->pose);
    RCLCPP_INFO(this->get_logger(), "Published body pose to topic /body_pose");
    
    // Set the response message to success
    response->success = true;
  }

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::Service<gpp_action_examples_interface::srv::SpotBodyPose>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
