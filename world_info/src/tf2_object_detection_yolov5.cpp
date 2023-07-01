#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <world_info_msgs/msg/bounding_box.hpp>
#include <world_info_msgs/msg/bounding_box_array.hpp>
#include <world_info_msgs/msg/world_info.hpp>

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

class DetectObjectTf2 : public rclcpp::Node
{
public:
  explicit DetectObjectTf2(const std::string& depth_topic, const std::string& depth_info, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("detect_object_tf2", options),
    sub_cam(image_transport::create_subscription(this, depth_topic,
      std::bind(&DetectObjectTf2::onCamera, this, std::placeholders::_1),
      declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data))
  {
    bounding_box_depth_sub_ = create_subscription<world_info_msgs::msg::BoundingBoxArray>(depth_topic + "/bb", 1, std::bind(&DetectObjectTf2::median_depth_xyz, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(depth_info, 1, std::bind(&DetectObjectTf2::get_camera_info, this, std::placeholders::_1));

    world_info_pub_ = create_publisher<world_info_msgs::msg::WorldInfo>("/world_info", 1);
  
    // Initialize TransformBroadcaster
    tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    received_camera_info = false;
  }

private:
  void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img)
  {
    // Convert the image message to a cv::Mat object
    try
    {
      auto depth_frame =  cv_bridge::toCvShare(msg_img, msg_img->encoding)->image;
      if (msg_img->encoding == "16UC1")
      {
        depth_frame.convertTo(frame, CV_32FC1, 0.001);
      }
      else
      {
        frame = depth_frame;
      }

    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
  
  void get_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Access the intrinsic parameters
    fx = msg->k[0];
    fy = msg->k[4];
    cx = msg->k[2];
    cy = msg->k[5];
    received_camera_info = true;
    camera_info_sub_.reset();
  }

  void median_depth_xyz(const world_info_msgs::msg::BoundingBoxArray::SharedPtr msg)
  {
    if (!received_camera_info) return;

    for(auto& boundingbox: msg->array)
    {
      std::vector<float> depth_values;

      // within a bounding box
      float x_min, x_max, y_min, y_max;

      x_min = std::max(0, static_cast<int>(boundingbox.cx - boundingbox.width / 2));
      y_min = std::max(0, static_cast<int>(boundingbox.cy - boundingbox.height / 2));
      x_max = std::min(static_cast<int>(boundingbox.cx + boundingbox.width / 2), frame.size().width);
      y_max = std::min(static_cast<int>(boundingbox.cy + boundingbox.height / 2), frame.size().height);

      for (int i = x_min; i < x_max; i++)
      {
        for (int j = y_min; j < y_max; j++)
        {
          auto depth_value = frame.at<float>(j,i);
          if (0.2 < depth_value && depth_value < 3)
          {
            depth_values.push_back(depth_value); // row num, column num
          }
        }
      }

      if (depth_values.size() == 0)
      {
        continue;
      }
      float median_depth = findMedian(depth_values);
      
      // Get a depth frame and a pixel coordinate
      float pixel_x = (x_min + x_max) / 2;
      float pixel_y = (y_min + y_max) / 2;

      // Convert the pixel coordinates to 3D world coordinates
      float x, y, z;
      pixel_to_point(pixel_x, pixel_y, median_depth, fx, fy, cx, cy, x, y, z);

      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = msg->header;

      // Set the transform message fields
      tf_msg.child_frame_id = boundingbox.name;
      tf_msg.transform.translation.x = z;
      tf_msg.transform.translation.y = -x;
      tf_msg.transform.translation.z = -y;

      // Publish the transform message
      tfb_->sendTransform(tf_msg);

      // Publish the QR code poses
      world_info_msgs::msg::WorldInfo world_info_msg;

      world_info_msg.header = msg->header;
      world_info_msg.num = boundingbox.name;
      world_info_msg.pose.position.x = tf_msg.transform.translation.x;
      world_info_msg.pose.position.y = tf_msg.transform.translation.y;
      world_info_msg.pose.position.z = tf_msg.transform.translation.z;
      world_info_msg.type = msg->type;

      // Publish the WorldInfo message
      world_info_pub_->publish(world_info_msg);
    }
  }

  // Convert pixel coordinates to 3D world coordinates
  void pixel_to_point(float x, float y, float depth_value, double fx, double fy, double cx, double cy, float& x_out, float& y_out, float& z_out)
  {
    x_out = (x - cx) * depth_value / fx;
    y_out = (y - cy) * depth_value / fy;
    z_out = depth_value;
  }

  float findMedian(std::vector<float>& vec)
  {
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

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;

  const image_transport::Subscriber sub_cam;
  rclcpp::Subscription<world_info_msgs::msg::BoundingBoxArray>::SharedPtr bounding_box_depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;

  cv::Mat frame;

  std::string camera_frame_id;
  std::string depth_topic;

  // Intrinsic parameters of the depth camera
  double fx = 0;
  double fy = 0;
  double cx = 0;
  double cy = 0;
  bool received_camera_info;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string depth_topic;
  std::string depth_info;

  if (argc <= 1) // Insufficient arguments
  {
    RCLCPP_ERROR(rclcpp::get_logger("object_detection"), "Provide an aligned depth image and info topic names.");
    rclcpp::shutdown();
    return 0;
  }
  else
  {
    depth_topic = argv[1];
    depth_info = argv[2];
  }

  rclcpp::spin(std::make_shared<DetectObjectTf2>(depth_topic, depth_info));
  rclcpp::shutdown();
  return 0;
}
