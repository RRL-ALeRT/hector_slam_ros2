#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

#include <opencv2/opencv.hpp>

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string &description, const bool &read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

class DetectMotion : public rclcpp::Node
{
public:
  explicit DetectMotion(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("motion", options),
  // topics
  sub_cam(image_transport::create_subscription(this, "/image_raw",
    std::bind(&DetectMotion::callbackImage, this, std::placeholders::_1),
    declare_parameter("image_transport", "raw", descr( {}, true)), rmw_qos_profile_sensor_data)),
  pub_motion(image_transport::create_publisher(this, "/motion_img"))
  {
    if (!has_parameter("motion_preset")) {
      declare_parameter("motion_preset", 0);
    }
    get_parameter("motion_preset", preset);
    RCLCPP_INFO_STREAM(get_logger(), "preset for: " << presetMap[preset]);
    if (!has_parameter("show_contours")) {
      declare_parameter("show_contours", show_contours);
    }
    get_parameter("show_contours", show_contours);
    if (!has_parameter("show_rectangles")) {
      declare_parameter("show_rectangles", show_rectangles);
    }
    get_parameter("show_rectangles", show_rectangles);
  }

  ~DetectMotion() {}

private:
  void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &img)
  {
    if (pub_motion.getNumSubscribers() == 0) {
      return;
    }

    cv::Mat frame;
    try
    {
      frame = cv_bridge::toCvShare(img, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    auto output = calculateContours(frame);

    if (contours_.size() > 0)
    {
      _motionDetected = true;
    }

    // Display the frame
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output)
                .toImageMsg();
    pub_motion.publish(*img_msg.get());
  }

  cv::Mat calculateContours(const cv::Mat& src)
  {
    get_parameter("motion_preset", preset);
    get_parameter("show_contours", show_contours);
    get_parameter("show_rectangles", show_rectangles);

    if (previous_preset != preset) {
      RCLCPP_INFO_STREAM(get_logger(), "preset for: " << presetMap[preset]);
      previous_preset = preset;
    }
    switch(preset)
    {
      case presets::rotating_disks:
        sobel_kernel_size = 0;
        sobel_scale       = 0.56;
        history           = 2;
        noise_reduction   = 57;
        bluring_factor    = 1;
        min_contour_size  = 15;
        merge_contours    = 19;
        clahe_clip_limit  = 2;
        clahe_grid_size   = 8;
        break;
      case presets::wibbely_wobbely_strings:
        sobel_kernel_size = 0;
        sobel_scale       = 1;
        history           = 2;
        bluring_factor    = 1;
        noise_reduction   = 100;
        merge_contours    = 18;
        min_contour_size  = 17;
        clahe_clip_limit  = 1.7;
        clahe_grid_size   = 6;
        break;
      case presets::far_away:
        sobel_kernel_size = 0;
        sobel_scale       = 0.6;
        history           = 7;
        noise_reduction   = 100;
        bluring_factor    = 0;
        min_contour_size  = 10;
        merge_contours    = 11;
        clahe_clip_limit  = 1.7;
        clahe_grid_size   = 7;
        break;
      case presets::vibration_tolerant:
        break;
    }

    cv::Mat     blurred, equalized, gray, edges;

    /* Remove noise and unnecsessary detail */
    const auto kernel_size = bluring_factor*2 + 1;
    cv::GaussianBlur(src, blurred, cv::Size(kernel_size,kernel_size),kernel_size,kernel_size);

    /* Convert to grayscale and enhance contrast */
    cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(clahe_clip_limit);
    clahe->setTilesGridSize(cv::Size(clahe_grid_size, clahe_grid_size));
    clahe->apply(gray, equalized);

    double scale = sobel_scale;
    int delta = 0;
    int ddepth = CV_16S;

    /// Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    cv::Sobel(equalized, grad_x, ddepth, 1, 0, 2 *sobel_kernel_size -1, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    cv::Sobel( equalized, grad_y, ddepth, 0, 1, 2 *sobel_kernel_size -1, scale, delta, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges );

    cv::Mat diff, comp, thresh;

    if(_buffer.size() == history)
    {
      double weight = 0.5;
      cv::subtract(edges, _buffer.back(), comp);  // difference between actual and previous frame
      if(_debug){
        cv::imshow("Difference_to_previous_frame", comp);
        cv::waitKey(30);
      }
      for(int i = _buffer.size() - 1; i > 0; --i) // differences in further history
      {
        cv::subtract(edges, _buffer[i-1], diff);  // difference between previous frames and actual frame
        cv::addWeighted(comp, weight, diff, 1-weight, 0, comp); // combine differences with weights
        weight *= 0.5;                                          // the older the frame, the less weigth it gets
      }
      if(_debug){
        cv::imshow("Weighted_sum_of_differences_in_history", comp);
        cv::waitKey(30);
      }
      /* Reduce background noise via threshold */
      auto cluster_distance_threshold = noise_reduction;
      cv::threshold(comp, thresh, cluster_distance_threshold, 255,cv::THRESH_BINARY);
      if(_debug){
        cv::imshow("Filtered_output_after_applying_threshold", thresh);
        cv::waitKey(30);
      }

      _buffer.push_back(edges);       // store latest frame in buffer
      _buffer.erase(_buffer.begin()); // remove one frame from buffer (oldest)

      /* Connect structures */
      cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(merge_contours, merge_contours));
      cv::dilate(thresh, thresh, kernel_erode);
      cv::erode( thresh, thresh, kernel_erode);
      if(_debug){
        cv::imshow("Output_after_dilating_and_eroding", thresh);
        cv::waitKey(30);
      }

      cv::findContours(thresh, _contours, _hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,  cv::Point(0,0));
      // remove contors with small size
      std::vector<std::vector<cv::Point> > filtered_contours;
      for(unsigned int i=0 ; i<_contours.size() ; ++i)
      {
        if(_contours[i].size() > min_contour_size)
          filtered_contours.push_back(_contours[i]);
      }

      /// Approximate contours to polygons + get bounding rects and circles
      std::vector<std::vector<cv::Point> > contours_poly( filtered_contours.size() );
      std::vector<cv::Rect> boundRect( filtered_contours.size() );
      std::vector<cv::Point2f>center( filtered_contours.size() );
      std::vector<float>radius( filtered_contours.size() );

      for( int i = 0; i < filtered_contours.size(); i++ )
      { 
        cv::approxPolyDP( cv::Mat(filtered_contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      }

      /// Draw polygonal contour + bonding rects
      for( int i = 0; i< filtered_contours.size(); i++ )
      {
        // Draw polygonal contours
        if(show_contours)
          cv::drawContours( src, contours_poly, i, green, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
        // Draw rectangles
        if(show_rectangles)
          cv::rectangle( src, boundRect[i].tl(), boundRect[i].br(), red, 2, 8, 0 );          
      }
      if(filtered_contours.size() > 0)
        cv::putText(src, "Motion detected!", place_text, font, font_scale, red, 1, cv::LINE_AA);

    }
    else if(_buffer.size() > history)
      _buffer.erase(_buffer.begin());
    else
      _buffer.push_back(edges);

    return src;
  }

  int preset;
  /**
   * @brief Enumerator for presets in reconfigure
   */
  enum presets {
    rotating_disks,
    wibbely_wobbely_strings,
    far_away,
    vibration_tolerant
  };
  std::map<int, std::string> presetMap = {
    {presets::rotating_disks, "rotating_disks"},
    {presets::wibbely_wobbely_strings, "wibbely_wobbely_strings"},
    {presets::far_away, "far_away"},
    {presets::vibration_tolerant, "vibration_tolerant"}
  };

  int previous_preset = 0;

  double sobel_kernel_size;
  double sobel_scale;
  int history;
  int noise_reduction;
  int bluring_factor;
  int min_contour_size;
  int merge_contours;
  int clahe_clip_limit;
  int clahe_grid_size;

  std::vector<cv::Vec4i>               _hierarchy;
  std::vector<std::vector<cv::Point> > _contours;       //!< contours for motion detection
  std::vector<cv::Mat>                 _buffer;

  cv::Point _centroid;                                  //!< centroid of motion
  cv::Rect  _box;                                       //!< bounding box around motion

  bool _motionDetected;                                  //!< true if motion detected
  bool _debug;                                           //!< true for debugging

  bool show_contours;
  bool show_rectangles;

  int font = cv::FONT_HERSHEY_PLAIN;
  double font_scale = 1.0;
  cv::Scalar red = cv::Scalar(0, 0, 255);
  cv::Scalar green = cv::Scalar(0, 255, 0);
  cv::Point place_text = cv::Point(10, 20);

  // const image_transport::CameraSubscriber sub_cam;
  const image_transport::Subscriber sub_cam;
  const image_transport::Publisher pub_motion;

  std::vector<std::vector<cv::Point>> contours_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<DetectMotion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
