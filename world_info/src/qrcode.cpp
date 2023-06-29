#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
#include "world_info_msgs/msg/world_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
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

class DetectQR : public rclcpp::Node
{
  public:
    explicit DetectQR(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("qrcode", options)
      // topics
      // sub_cam(image_transport::create_camera_subscription(this, "image_rect",
      //   std::bind(&DetectQR::onCamera, this, std::placeholders::_1, std::placeholders::_2),
      //   declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      // sub_cam(image_transport::create_subscription(this, "image_rect",
      //   std::bind(&DetectQR::onCamera, this, std::placeholders::_1),
      //   declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      // pub_qr(image_transport::create_publisher(this, "qr_detected"))
    {
        frame_id = "realsense";
        if (!has_parameter("frame_id"))
            declare_parameter("frame_id", frame_id);
        get_parameter("frame_id", frame_id);

        // Initialize TransformBroadcaster
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        cam_sub = create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", 1, std::bind(&DetectQR::onCamera, this, std::placeholders::_1));
        depth_sub = create_subscription<sensor_msgs::msg::Image>("/camera/aligned_depth_to_color/image_raw", 1, std::bind(&DetectQR::onDepthCamera, this, std::placeholders::_1));

        cam_info = create_subscription<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info", 1, std::bind(&DetectQR::onCameraInfo, this, std::placeholders::_1));

        // Create WorldInfo publisher
        world_info_pub_ = create_publisher<world_info_msgs::msg::WorldInfo>("/world_info_sub", 1);
    }

  private:
    // void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
    //               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) # webots camera and info_msgs are not synced
    void onCamera(const sensor_msgs::msg::Image::SharedPtr msg_img)
    {
      // Convert the image message to a cv::Mat object
      cv::Mat frame;
      try
      {
          frame =  cv_bridge::toCvShare(msg_img, "bgr8")->image;
      }
      catch (cv_bridge::Exception &e)
      {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
          return;
      }

      try
      {
        // Convert the image to grayscale
        cv::Mat image_gray;
        cv::cvtColor(frame, image_gray, cv::COLOR_BGR2GRAY);

        // Convert the OpenCV image to a zbar image
        int width = image_gray.cols;
        int height = image_gray.rows;
        uchar *raw = image_gray.data;
        zbar::Image image_zbar(width, height, "Y800", raw, width * height);

        scanner.scan(image_zbar);

        // Find QR codes in the image
        for (zbar::Image::SymbolIterator symbol = image_zbar.symbol_begin(); symbol != image_zbar.symbol_end(); ++symbol)
        {
          // Get the location of the corners
          int x2 = symbol->get_location_x(0);
          int y2 = symbol->get_location_y(0);
          int x3 = symbol->get_location_x(1);
          int y3 = symbol->get_location_y(1);
          int x4 = symbol->get_location_x(2);
          int y4 = symbol->get_location_y(2);
          int x1 = symbol->get_location_x(3);
          int y1 = symbol->get_location_y(3);

          std::vector<int> x_vec {x1,x2,x3,x4};
          std::vector<int> y_vec {y1,y2,y3,y4};

          int min_x = *std::min_element(x_vec.begin(), x_vec.end());
          int max_x = *std::max_element(x_vec.begin(), x_vec.end());
          int min_y = *std::min_element(y_vec.begin(), y_vec.end());
          int max_y = *std::max_element(y_vec.begin(), y_vec.end());
          
          float pose_x;
          float pose_y;
          float pose_z;

          median_depth_xyz(std::vector<float> {min_x, max_x}, std::vector<float> {min_y, max_y}, pose_x, pose_y, pose_z);

          if (std::isnan(pose_x) || std::isnan(pose_y) || std::isnan(pose_z))
              return;
          if (std::isinf(pose_x) || std::isinf(pose_y) || std::isinf(pose_z))
              return;

          geometry_msgs::msg::TransformStamped tf_msg;
          tf_msg.header.stamp = msg_img->header.stamp;
          tf_msg.header.frame_id = frame_id;

          // Set the transform message fields
          tf_msg.child_frame_id = symbol->get_data();
          tf_msg.transform.translation.x = pose_z;
          tf_msg.transform.translation.y = -pose_x;
          tf_msg.transform.translation.z = -pose_y;

          // Publish the transform message
          tfb_->sendTransform(tf_msg);

          // Publish the QR code poses
          world_info_msgs::msg::WorldInfo world_info_msg;

          world_info_msg.header.frame_id = frame_id;
          world_info_msg.header.stamp = msg_img->header.stamp;
          world_info_msg.num = symbol->get_data();
          world_info_msg.pose.position.x = tf_msg.transform.translation.x;
          world_info_msg.pose.position.y = tf_msg.transform.translation.y;
          world_info_msg.pose.position.z = tf_msg.transform.translation.z;
          world_info_msg.type = "qr";

          // Publish the WorldInfo message
          world_info_pub_->publish(world_info_msg);
        }
      }
      catch (cv::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    }

    void onDepthCamera(const sensor_msgs::msg::Image::SharedPtr msg_img)
    {
        if (fx == 0.0) {
            RCLCPP_INFO_ONCE(get_logger(), "waiting to read camera info");
            return;
        }
        RCLCPP_INFO_ONCE(get_logger(), "hazmat detection sub got first depth cam messsage");
        // Convert the image message to a cv::Mat object
        try
        {
            depth_frame = cv_bridge::toCvShare(msg_img, "32FC1")->image;
        }

        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }
    
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx = msg->k[0];
        cx = msg->k[2];
        fy = msg->k[4];
        cy = msg->k[5];
    }

    void median_depth_xyz(const std::vector<float> vec_x, const std::vector<float> vec_y, float& x, float& y, float& z)
    {
        std::vector<float> depth_values;
        if (vec_x.size() == 2) {
            // within a bounding box
            for (int i = vec_x[0]; i < vec_x[1]; i++) {
                for (int j = vec_y[0]; j < vec_y[1]; j++) {
                    float d = 0.001*depth_frame.at<float>(j,i);
                    if (0.01 < d < 4.0) {
                        depth_values.push_back(d); // row num, column num
                    }
                }
            }
        }
        else {
            // within a mask
            for (int x: vec_x) {
                for (int y: vec_y) {
                    float d = 0.001*depth_frame.at<float>(y,x);
                    if (0.01 < d < 4.0) {
                        depth_values.push_back(d); // row num, column num
                    }
                }
            }
        }

        float median_depth = findMedian(depth_values);
        
        // Get a depth frame and a pixel coordinate
        float pixel_x = std::accumulate(vec_x.begin(), vec_x.end(), 0.0f) / vec_x.size();
        float pixel_y = std::accumulate(vec_y.begin(), vec_y.end(), 0.0f) / vec_y.size();

        // Convert the pixel coordinates to 3D world coordinates
        pixel_to_point(pixel_x, pixel_y, median_depth, fx, fy, cx, cy, x, y, z);
        
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

  zbar::ImageScanner scanner;
  float square_length;
  std::vector<cv::Point3f> ref_points;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  const image_transport::Subscriber sub_cam;
  const image_transport::Publisher pub_qr;
  rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info;

  // Camera Intrinsics
  float fx = 0.0;
  float fy = 0.0;
  float cx = 0.0;
  float cy = 0.0;

  std::string frame_id;
  cv::Mat depth_frame;
};

}  // namespace world_info

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::DetectQR)
