#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
#include <world_info_msgs/srv/get_median_depth_xyz.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <numeric>
#include <vector>

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor descr;

  descr.description = description;
  descr.read_only = read_only;

  return descr;
}

namespace world_info
{

using GetMedianDepthXYZ = world_info_msgs::srv::GetMedianDepthXYZ;

class RSDepth : public rclcpp::Node
{
  public:
    explicit RSDepth(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rs_depth", options),
      // topics
      // sub_cam(image_transport::create_camera_subscription(this, "image_rect",
      //   std::bind(&RSDepth::onCamera, this, std::placeholders::_1, std::placeholders::_2),
      //   declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      sub_cam(image_transport::create_subscription(this, "/Spot/kinect_range",
        std::bind(&RSDepth::onCamera, this, std::placeholders::_1),
        declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data))
    {
      median_service = create_service<GetMedianDepthXYZ>("get_median_depth_xyz", 
              std::bind(&RSDepth::median_depth_xyz, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));
    }

  private:
    // void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
    //               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) # webots camera and info_msgs are not synced
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img)
    {
      // Convert the image message to a cv::Mat object
      try
      {
          frame =  cv_bridge::toCvShare(msg_img, "32FC1")->image;
          latest_msg_time = msg_img->header.stamp.sec;
      }
      catch (cv_bridge::Exception &e)
      {
          RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }      
    }
    
    void median_depth_xyz(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<GetMedianDepthXYZ::Request> request,
      const std::shared_ptr<GetMedianDepthXYZ::Response> response)
    {
      (void)request_header;
      if (std::abs(request->header.stamp.sec - latest_msg_time) > 1) {
        RCLCPP_ERROR(get_logger(), "last depth image received is not within 1 sec of requested image");
        return;
      }
      
      std::vector<float> depth_values;
      if (request->x.size() == 2) {
        // within a bounding box
        for (int i = request->x[0]; i < request->x[1]; i++) {
          for (int j = request->y[0]; j < request->y[1]; j++) {
            depth_values.push_back(frame.at<float>(j,i)); // row num, column num
          }
        }
      }
      else {
        // within a mask
        for (int x: request->x) {
          for (int y: request->y) {
            depth_values.push_back(frame.at<float>(y,x));
          }
        }
      }

      removeInfAndZeroValues(depth_values);
      float median_depth = findMedian(depth_values);

      // Intrinsic parameters of the depth camera
      float fx = 212.1688885346757;
      float fy = 212.1688885346757;
      float cx = 212;
      float cy = 120;
      
      // Get a depth frame and a pixel coordinate
      float pixel_x = std::accumulate(request->x.begin(), request->x.end(), 0.0f) / request->x.size();
      float pixel_y = std::accumulate(request->y.begin(), request->y.end(), 0.0f) / request->y.size();

      // Convert the pixel coordinates to 3D world coordinates
      float x, y, z;
      pixel_to_point(pixel_x, pixel_y, median_depth, fx, fy, cx, cy, x, y, z);

      response->x = x;
      response->y = y;
      response->z = z;
    }

    // Convert pixel coordinates to 3D world coordinates
    void pixel_to_point(float x, float y, float depth_value, float fx, float fy, float cx, float cy, float& x_out, float& y_out, float& z_out) {
      x_out = (x - cx) * depth_value / fx;
      y_out = (y - cy) * depth_value / fy;
      z_out = depth_value;
    }

    void removeInfAndZeroValues(std::vector<float>& vec) {
      vec.erase(std::remove_if(vec.begin(), vec.end(), [](float f) {
        return f == std::numeric_limits<float>::infinity() || f == 0.0f;
      }), vec.end());
    }

    float findMedian(std::vector<float>& vec) {
      auto n = vec.size();
      auto middle = vec.begin() + n / 2;
      std::nth_element(vec.begin(), middle, vec.end());
      if (n % 2 == 0) {
        auto left_middle = std::max_element(vec.begin(), middle);
        return (*left_middle + *middle) / 2.0f;
      } else {
        return *middle;
      }
    }

  const image_transport::Subscriber sub_cam;
  cv::Mat frame;
  int latest_msg_time;

  rclcpp::Service<GetMedianDepthXYZ>::SharedPtr median_service;

};

}  // namespace world_info

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::RSDepth)
