#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

using namespace std;

namespace world_info
{

class DetectQR : public rclcpp::Node
{
  public:
    explicit DetectQR(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("qrcode", options)
    {
        // Initialize TransformBroadcaster
        tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Create image message subscriber
        auto callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void {
            this->spot_kinect(msg);
        };
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("/Spot/kinect_color", 10, callback);

        // Create WorldInfo publisher
        world_info_pub_ = this->create_publisher<world_info_msgs::msg::WorldInfo>("/world_info_sub", 1);
    }

  private:
    void spot_kinect(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the image message to a cv::Mat object
        cv::Mat kinect_img;
        try
        {
            kinect_img =  cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get the size of the QR code squares
        const float square_length = 0.665;

        // Convert the image to grayscale
        cv::Mat image_gray;
        cv::cvtColor(kinect_img, image_gray, cv::COLOR_BGR2GRAY);

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
          int x1 = symbol->get_location_x(0);
          int y1 = symbol->get_location_y(0);
          int x2 = symbol->get_location_x(1);
          int y2 = symbol->get_location_y(1);
          int x3 = symbol->get_location_x(2);
          int y3 = symbol->get_location_y(2);
          int x4 = symbol->get_location_x(3);
          int y4 = symbol->get_location_y(3);

          std::vector<cv::Point2f> corners = {
            cv::Point2f(x1, y1),
            cv::Point2f(x2, y2),
            cv::Point2f(x3, y3),
            cv::Point2f(x4, y4)
          };

          // Define the reference frame (e.g. the camera frame)
          std::vector<cv::Point3f> ref_points = {
            {-square_length / 2,  square_length / 2, 0},
            { square_length / 2,  square_length / 2, 0},
            { square_length / 2, -square_length / 2, 0},
            {-square_length / 2, -square_length / 2, 0}
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

          // The pose of the square is given by the transformation matrix [R|t]
          cv::Mat pose = cv::Mat::eye(4, 4, rot_mat.type());
          rot_mat.copyTo(pose(cv::Rect(0, 0, 3, 3)));
          tvec.copyTo(pose(cv::Rect(3, 0, 1, 3)));

          // The position of the square can be extracted from the pose matrix as follows:
          cv::Point3f position(pose.at<double>(0, 3), pose.at<double>(1, 3), pose.at<double>(2, 3));

          // The orientation of the square can be extracted using the rot_mat matrix as follows - green up, blue front:
          cv::Mat r;
          euler_to_matrix(M_PI/2, 0, -M_PI/2, r);

          cv::Mat pose_R = r * rot_mat;

          // Convert the rotation matrix to a quaternion
          geometry_msgs::msg::Quaternion quat;
          matrix_to_quat(pose_R, quat);

          geometry_msgs::msg::TransformStamped tf_msg;
          tf_msg.header.stamp = msg->header.stamp;
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

          world_info_msg.header.stamp = msg->header.stamp;
          world_info_msg.num = symbol->get_data();
          world_info_msg.pose.position.x = position.x;
          world_info_msg.pose.position.y = position.y;
          world_info_msg.pose.position.z = position.z;
          world_info_msg.pose.orientation = quat;

          // Publish the WorldInfo message
          world_info_pub_->publish(world_info_msg);

          // Draw lines around the QR code
          cv::line(kinect_img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
          cv::line(kinect_img, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(0, 255, 0), 2);
          cv::line(kinect_img, cv::Point(x3, y3), cv::Point(x4, y4), cv::Scalar(0, 255, 0), 2);
          cv::line(kinect_img, cv::Point(x4, y4), cv::Point(x1, y1), cv::Scalar(0, 255, 0), 2);

          // Draw point at top left corner
          cv::circle(kinect_img, cv::Point(x1, y1), 4, cv::Scalar(0, 0, 255), -1);
        }
        cv::imshow("qrcode", kinect_img);
        cv::waitKey(20);
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
    };

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

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;
};

}  // namespace world_info

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::DetectQR)
