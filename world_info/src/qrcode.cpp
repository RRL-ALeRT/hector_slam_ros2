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
    : Node("qrcode", options),
      // topics
      // sub_cam(image_transport::create_camera_subscription(this, "image_rect",
      //   std::bind(&DetectQR::onCamera, this, std::placeholders::_1, std::placeholders::_2),
      //   declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      sub_cam(image_transport::create_subscription(this, "image_rect",
        std::bind(&DetectQR::onCamera, this, std::placeholders::_1),
        declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      pub_qr(image_transport::create_publisher(this, "qr_detected"))
    {
        // Initialize TransformBroadcaster
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        square_length = 0.2;
        if(!has_parameter("qr_square_length"))
          declare_parameter("qr_square_length", square_length);
        get_parameter("qr_square_length", square_length);

        ref_points = {
          {-square_length / 2,  square_length / 2, 0},
          { square_length / 2,  square_length / 2, 0},
          { square_length / 2, -square_length / 2, 0},
          {-square_length / 2, -square_length / 2, 0}
        };

        // Create WorldInfo publisher
        world_info_pub_ = create_publisher<world_info_msgs::msg::WorldInfo>("/world_info_sub", 1);
    }

  private:
    // void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
    //               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) # webots camera and info_msgs are not synced
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img)
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

          std::vector<cv::Point2f> corners = {
            cv::Point2f(x1, y1),
            cv::Point2f(x2, y2),
            cv::Point2f(x3, y3),
            cv::Point2f(x4, y4)
          };

          // Define camera intrinsic parameters
          cv::Mat K = (cv::Mat_<double>(3, 3) << -292.878, 0, 160, 0, 292.878, 95, 0, 0, 1);
          cv::Mat D = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);

          // Estimate the pose of the square using the corner points and the reference frame
          cv::Mat rvec, tvec;
          cv::solvePnP(ref_points, corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

          // Extract the rotation matrix from the rvec vector
          cv::Mat rot_mat;
          cv::Rodrigues(rvec, rot_mat);

          // Adjust orientation for tf2
          cv::Mat r;
          euler_to_matrix(-M_PI/2, 0, -M_PI/2, r);
          rot_mat = r * rot_mat;

          // The position of the square can be extracted from the pose matrix as follows:
          cv::Point3f position(tvec.at<double>(0, 0),
                              tvec.at<double>(1, 0),
                              tvec.at<double>(2, 0));

          // Convert the rotation matrix to a quaternion
          geometry_msgs::msg::Quaternion quat;
          matrix_to_quat(rot_mat, quat);

          geometry_msgs::msg::TransformStamped tf_msg;
          tf_msg.header.stamp = msg_img->header.stamp;
          tf_msg.header.frame_id = "kinect";

          // Set the transform message fields
          tf_msg.child_frame_id = symbol->get_data();
          tf_msg.transform.translation.x = position.z;
          tf_msg.transform.translation.y = position.x;
          tf_msg.transform.translation.z = -position.y;
          tf_msg.transform.rotation = quat;

          // Publish the transform message
          tfb_->sendTransform(tf_msg);

          // Publish the QR code poses
          world_info_msgs::msg::WorldInfo world_info_msg;

          world_info_msg.header.stamp = msg_img->header.stamp;
          world_info_msg.num = symbol->get_data();
          world_info_msg.pose.position.x = tf_msg.transform.translation.x;
          world_info_msg.pose.position.y = tf_msg.transform.translation.y;
          world_info_msg.pose.position.z = tf_msg.transform.translation.z;
          world_info_msg.pose.orientation = quat;

          // Publish the WorldInfo message
          world_info_pub_->publish(world_info_msg);

          // Draw lines around the QR code
          cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
          cv::line(frame, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(0, 255, 0), 2);
          cv::line(frame, cv::Point(x3, y3), cv::Point(x4, y4), cv::Scalar(0, 255, 0), 2);
          cv::line(frame, cv::Point(x4, y4), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2);

          // Draw point at top left corner
          cv::circle(frame, cv::Point(x2, y2), 4, cv::Scalar(0, 0, 255), -1);
        }
      }
      catch (cv::Exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
      // Display the frame
      sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                  .toImageMsg();
      pub_qr.publish(*img_msg.get());
    }
    
    // Convert a rotation matrix to a quaternion
    void matrix_to_quat(const cv::Mat& rot_mat, geometry_msgs::msg::Quaternion& quat)
    {
      tf2::Matrix3x3 tf_rot_mat(rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
                                rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                                rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
      tf2::Quaternion tf_quat;
      tf_rot_mat.getRotation(tf_quat);
      quat.x = tf_quat.x();
      quat.y = tf_quat.y();
      quat.z = tf_quat.z();
      quat.w = tf_quat.w();
    }

    // Convert Euler angles (roll, pitch, yaw) to a rotation matrix
    void euler_to_matrix(double roll, double pitch, double yaw, cv::Mat& rot_mat)
    {
      tf2::Matrix3x3 tf_rot_mat;
      tf_rot_mat.setEulerYPR(yaw, pitch, roll);
      rot_mat = (cv::Mat_<double>(3, 3) << tf_rot_mat[0][0], tf_rot_mat[0][1], tf_rot_mat[0][2],
                tf_rot_mat[1][0], tf_rot_mat[1][1], tf_rot_mat[1][2],
                tf_rot_mat[2][0], tf_rot_mat[2][1], tf_rot_mat[2][2]);
    }

  zbar::ImageScanner scanner;
  float square_length;
  std::vector<cv::Point3f> ref_points;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  const image_transport::Subscriber sub_cam;
  const image_transport::Publisher pub_qr;
  rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;
};

}  // namespace world_info

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::DetectQR)
