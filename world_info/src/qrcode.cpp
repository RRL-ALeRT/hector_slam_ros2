#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <world_info_msgs/msg/bounding_box.hpp>
#include <world_info_msgs/msg/bounding_box_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
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

class DetectQR : public rclcpp::Node
{
  public:
    explicit DetectQR(const std::string& image_topic, const std::string& depth_topic, const std::string& camera_frame_id_, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("qrcode", options),
      image_topic_(image_topic),
      depth_topic_(depth_topic),
      sub_cam(image_transport::create_subscription(this, image_topic,
        std::bind(&DetectQR::onCamera, this, std::placeholders::_1),
        declare_parameter("image_transport", "raw", descr({}, true)), rmw_qos_profile_sensor_data))
    {
      camera_frame_id = camera_frame_id_;

      // Create WorldInfo publisher
      bounding_box_pub_ = create_publisher<world_info_msgs::msg::BoundingBoxArray>(image_topic_ + "/bb", 1);

      if (depth_topic_ != "")
      {
        bounding_box_depth_pub_ = create_publisher<world_info_msgs::msg::BoundingBoxArray>(depth_topic_ + "/bb", 1);
      }

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
    }

  private:
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

      world_info_msgs::msg::BoundingBoxArray bb_array_msg;
      bb_array_msg.type = "qr";

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

        world_info_msgs::msg::BoundingBox bb_msg;
        bb_msg.name = symbol->get_data();
        bb_msg.width = std::max(static_cast<int>(x1 - x2), std::max(static_cast<int>(x1 - x3), static_cast<int>(x1 - x4)));
        bb_msg.height = std::max(static_cast<int>(y2 - y1), std::max(static_cast<int>(y3 - y1), static_cast<int>(y4 - y1)));
        bb_msg.cx = ((x1 + x2 + x3 + x4) / 4) - bb_msg.width / 2;
        bb_msg.cy = (y1 + y2 + y3 + y4) / 4 - bb_msg.height / 2;
        bb_array_msg.array.push_back(bb_msg);
      }
      bb_array_msg.header.stamp = msg_img->header.stamp;
      bb_array_msg.header.frame_id = camera_frame_id == "" ? msg_img->header.frame_id : camera_frame_id;
      bounding_box_pub_->publish(bb_array_msg);
      if (depth_topic_ != "")
      {
        bounding_box_depth_pub_->publish(bb_array_msg);
      }
    }
    catch (cv::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  std::string image_topic_, depth_topic_, camera_frame_id;

  zbar::ImageScanner scanner;
  float square_length;
  std::vector<cv::Point3f> ref_points;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
  const image_transport::Subscriber sub_cam;
  
  rclcpp::Publisher<world_info_msgs::msg::BoundingBoxArray>::SharedPtr bounding_box_pub_;
  rclcpp::Publisher<world_info_msgs::msg::BoundingBoxArray>::SharedPtr bounding_box_depth_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string model_name = "";
  std::string image_topic = "";
  std::string depth_topic = "";
  std::string camera_frame_id = "";

  if (argc < 1) // Insufficient arguments
  {
    std::cerr << "Error: Provide a image topic name. If a 2nd argument of depth image is provided, then run tf2_object_detection_yolov5 as well. provide also a camera_frame_id after that\n";
    return 1;
  }
  else
  {
    // Read the second argument as image_topic
    image_topic = argv[1];

    // Check if the third argument exists and assign it to depth_topic
    if (argc >= 3)
    {
      depth_topic = argv[2];
      camera_frame_id = argv[3];
    }
  }

  std::cout << "Image Topic: " << image_topic << std::endl;
  std::cout << "Depth Topic: " << depth_topic << std::endl;
  std::cout << "Camera frame id: " << camera_frame_id << std::endl;

  rclcpp::spin(std::make_shared<DetectQR>(image_topic, depth_topic, camera_frame_id));
  rclcpp::shutdown();
  return 0;
}
