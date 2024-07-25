#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <image_transport/image_transport.hpp>
#include "world_info_msgs/msg/world_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/quaternion.hpp>

#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include <cv_bridge/cv_bridge.h>

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

namespace world_info {

class DetectAruco : public rclcpp::Node
{
  public: 
    explicit DetectAruco(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("aruco", options),
      // topics
      // sub_cam(image_transport::create_camera_subscription(this, "image_rect",
      //   std::bind(&DetectAruco::onCamera, this, std::placeholders::_1, std::placeholders::_2),
      //   declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      sub_cam(image_transport::create_subscription(this, "image_rect",
        std::bind(&DetectAruco::onCamera, this, std::placeholders::_1),
        declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data)),
      pub_aruco(image_transport::create_publisher(this, "aruco_detected"))
    {
      // Initialize TransformBroadcaster
      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

      // Create WorldInfo publisher
      world_info_pub_ = create_publisher<world_info_msgs::msg::WorldInfo>("/world_info_sub", 1);

      // For pose estimation
      square_length = 0.2;
      if(!has_parameter("aruco_square_length"))
        declare_parameter("aruco_square_length", square_length);
      get_parameter("aruco_square_length", square_length);

      ref_points = {
        {-square_length / 2,  square_length / 2, 0},
        { square_length / 2,  square_length / 2, 0},
        { square_length / 2, -square_length / 2, 0},
        {-square_length / 2, -square_length / 2, 0}
      };

      // Create an Aruco detector
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      detectorParams = cv::aruco::DetectorParameters::create();
    }

    ~DetectAruco() {
      world_info_pub_.reset();
    }

  private:
    // void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
    //               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) # webots camera and info_msgs are not synced
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img)
    {
      // Create a camera matrix
      cv::Mat K = (cv::Mat_<double>(3, 3) <<
        292.8780354739923, 0., 160.0,
        0., 292.8780354739923, 95.0,
        0., 0., 1.
      );

      // Create distortion coefficients
      cv::Mat D = (cv::Mat_<double>(5, 1) <<
        0.0,0.0,0.0,0.0,0.0
      );

      // Convert the image message to a cv::Mat object
      cv::Mat frame;
      try {
        frame = cv_bridge::toCvShare(msg_img, "bgr8")->image;
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Detect Aruco tags in the frame and estimate the pose of the markers
      cv::aruco::detectMarkers(frame, dictionary, corners, ids, detectorParams);

      std::vector<geometry_msgs::msg::TransformStamped> tfs;

      for (long unsigned int i=0; i < ids.size(); i++) {
        // Estimate the pose of the square using the corner points and the reference frame
        cv::solvePnP(ref_points, corners[i], K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

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
        tf_msg.child_frame_id = "aruco_" + std::to_string(ids[i]);
        tf_msg.transform.translation.x = position.z;
        tf_msg.transform.translation.y = -position.x;
        tf_msg.transform.translation.z = -position.y;
        tf_msg.transform.rotation = quat;

        // Publish the QR code poses
        world_info_msgs::msg::WorldInfo world_info_msg;

        world_info_msg.header.stamp = msg_img->header.stamp;
        world_info_msg.name = std::to_string(ids[i]);
        world_info_msg.type = "aruco";
        world_info_msg.pose.position.x = tf_msg.transform.translation.x;
        world_info_msg.pose.position.y = tf_msg.transform.translation.y;
        world_info_msg.pose.position.z = tf_msg.transform.translation.z;
        world_info_msg.pose.orientation.w = quat.w;
        world_info_msg.pose.orientation.x = quat.x;
        world_info_msg.pose.orientation.y = quat.y;
        world_info_msg.pose.orientation.z = quat.z;

        // Publish the WorldInfo message
        world_info_pub_->publish(world_info_msg);

        // Publish the transform message
        tfs.push_back(tf_msg);

        // Draw the Aruco tags on the frame
        cv::aruco::drawDetectedMarkers(frame, corners, ids);

        // Draw the marker axes
        cv::aruco::drawAxis(frame, K, D, rvec,tvec, 0.1);
      }

      tf_broadcaster->sendTransform(tfs);

      // Display the frame
      sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                  .toImageMsg();
      pub_aruco.publish(*img_msg.get());
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

  std::vector<cv::Point3f> ref_points;

  float square_length;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::Mat rvec, tvec;
  cv::Mat frameCopy;

  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

  // const image_transport::CameraSubscriber sub_cam;
  const image_transport::Subscriber sub_cam;
  const image_transport::Publisher pub_aruco;
  rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

} // namespace world_info

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::DetectAruco)