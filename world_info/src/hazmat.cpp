#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <world_info_msgs/msg/world_info.hpp>
#include <world_info_msgs/srv/get_median_depth_xyz.hpp>

#include "tf2_ros/transform_broadcaster.h"

#include <ament_index_cpp/get_package_share_directory.hpp>


rcl_interfaces::msg::ParameterDescriptor
descr(const std::string &description, const bool &read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

namespace world_info
{
using GetMedianDepthXYZ = world_info_msgs::srv::GetMedianDepthXYZ;
using ServiceResponseFuture = rclcpp::Client<GetMedianDepthXYZ>::SharedFuture;

    struct Detection
    {
        int class_id;
        float confidence;
        cv::Rect box;
    };

    struct Resize
    {
        cv::Mat resized_image;
        int dw;
        int dh;
    };

    Resize resize_and_pad(cv::Mat &img, cv::Size new_shape)
    {
        float width = img.cols;
        float height = img.rows;
        float r = float(new_shape.width / std::max(width, height));
        int new_unpadW = int(round(width *r));
        int new_unpadH = int(round(height *r));
        Resize resize;
        cv::resize(img, resize.resized_image, cv::Size(new_unpadW, new_unpadH), 0, 0, cv::INTER_AREA);

        resize.dw = new_shape.width - new_unpadW;
        resize.dh = new_shape.height - new_unpadH;
        cv::Scalar color = cv::Scalar(100, 100, 100);
        cv::copyMakeBorder(resize.resized_image, resize.resized_image, 0, resize.dh, 0, resize.dw, cv::BORDER_CONSTANT, color);

        return resize;
    }

    class DetectHazmat: public rclcpp::Node
    {
        public:

        explicit DetectHazmat(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): Node("hazmat", options),
       	// topics
        sub_cam(image_transport::create_subscription(this, "image_rect",
            std::bind(&DetectHazmat::onCamera, this, std::placeholders::_1),
            declare_parameter("image_transport", "raw", descr( {}, true)), rmw_qos_profile_sensor_data)),
        pub_hazmat(image_transport::create_publisher(this, "hazmat_detected"))
        {
            // Initialize TransformBroadcaster
            tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

           	// Create WorldInfo publisher
            world_info_pub_ = create_publisher<world_info_msgs::msg::WorldInfo>("/world_info_sub", 1);

            SCORE_THRESHOLD = 0.2;
            NMS_THRESHOLD = 0.4;
            CONFIDENCE_THRESHOLD = 0.8;

            if (!has_parameter("hazmat_confidence_threshold"))
                declare_parameter("hazmat_confidence_threshold", CONFIDENCE_THRESHOLD);
            get_parameter("hazmat_confidence_threshold", CONFIDENCE_THRESHOLD);

            inference_mode = "CPU";
            if (!has_parameter("inference_mode"))
                declare_parameter("inference_mode", inference_mode);
            get_parameter("inference_mode", inference_mode);

            class_names = {"poison", "oxygen", "flammable", "flammable-solid", "corrosive", "dangerous", "non-flammable-gas", "organic-peroxide", "explosive", "radioactive", "inhalation-hazard", "spontaneously-combustible", "infectious-substance"};

            std::string package_share_directory = ament_index_cpp::get_package_share_directory("world_info");
            model = core.read_model(package_share_directory + "/weights/hazmat.onnx");

            median_xyz_client_ = create_client<GetMedianDepthXYZ>("get_median_depth_xyz");
            while (!median_xyz_client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(get_logger(), "service from rs_depth not available, waiting again...");
            }
        }

        ~DetectHazmat()
        {
            world_info_pub_.reset();
        }

        private:

        void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img)
        {
           	// Convert the image message to a cv::Mat object
            cv::Mat img;
            try
            {
                img = cv_bridge::toCvShare(msg_img, "bgr8")->image;
            }

            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            try {
                Resize res = resize_and_pad(img, cv::Size(640, 640));

                // Create tensor from image
                float *input_data = (float*) res.resized_image.data;

                if (first_run)
                {
                    // Inizialize Preprocessing for the model
                    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
                    // Specify input image format
                    ppp.input().tensor().set_element_type(ov::element::u8).set_layout("NHWC").set_color_format(ov::preprocess::ColorFormat::BGR);
                    // Specify preprocess pipeline to input image without resizing
                    ppp.input().preprocess().convert_element_type(ov::element::f32).convert_color(ov::preprocess::ColorFormat::RGB).scale({ 255., 255., 255. });
                    //  Specify model's input layout
                    ppp.input().model().set_layout("NCHW");
                    // Specify output results format
                    ppp.output().tensor().set_element_type(ov::element::f32);
                    // Embed above steps in the graph
                    model = ppp.build();
                    compiled_model = core.compile_model(model, inference_mode);

                    // Create an infer request for model inference 
                    infer_request = compiled_model.create_infer_request();

                    first_run = false;
                }

                ov::Tensor input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), input_data);
                infer_request.set_input_tensor(input_tensor);
                infer_request.infer();

                // Retrieve inference results 
                const ov::Tensor &output_tensor = infer_request.get_output_tensor();
                ov::Shape output_shape = output_tensor.get_shape();
                float *detections = output_tensor.data<float> ();

                // Postprocessing including NMS  
                std::vector<cv::Rect>boxes;
                std::vector<int> class_ids;
                std::vector<float> confidences;

                for (int i = 0; i < output_shape[1]; i++)
                {
                    float *detection = &detections[i *output_shape[2]];

                    float confidence = detection[4];
                    if (confidence >= CONFIDENCE_THRESHOLD)
                    {
                        float *classes_scores = &detection[5];
                        cv::Mat scores(1, output_shape[2] - 5, CV_32FC1, classes_scores);
                        cv::Point class_id;
                        double max_class_score;
                        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                        if (max_class_score>SCORE_THRESHOLD)
                        {
                            confidences.push_back(confidence);

                            class_ids.push_back(class_id.x);

                            float x = detection[0];
                            float y = detection[1];
                            float w = detection[2];
                            float h = detection[3];

                            float xmin = x - (w / 2);
                            float ymin = y - (h / 2);

                            boxes.push_back(cv::Rect(xmin, ymin, w, h));
                        }
                    }
                }

                std::vector<int> nms_result;
                cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
                std::vector<Detection> output;
                for (int i = 0; i < nms_result.size(); i++)
                {
                    Detection result;
                    int idx = nms_result[i];
                    result.class_id = class_ids[idx];
                    result.confidence = confidences[idx];
                    result.box = boxes[idx];
                    output.push_back(result);
                }

                // Print results and publish Figure with detections
                for (int i = 0; i < output.size(); i++)
                {
                    auto detection = output[i];
                    auto box = detection.box;
                    auto classId = detection.class_id;
                    auto confidence = detection.confidence;
                    float rx = (float) img.cols / (float)(res.resized_image.cols - res.dw);
                    float ry = (float) img.rows / (float)(res.resized_image.rows - res.dh);
                    box.x = rx *box.x;
                    box.y = ry *box.y;
                    box.width = rx *box.width;
                    box.height = ry *box.height;
                    RCLCPP_INFO_STREAM(get_logger(), "" << "Bbox" << i + 1 << ": Class: " << classId << " " <<
                        "Confidence: " << confidence << " Scaled coords:[ " <<
                        "cx: " << (float)(box.x + (box.width / 2)) << ", " <<
                        "cy: " << (float)(box.y + (box.height / 2)) << ", " <<
                        "w: " << (float) box.width << ", " <<
                        "h: " << (float) box.height << " ]");
                    float xmax = std::min(img.size().width - 1, box.x + box.width);
                    float ymax = std::min(img.size().height - 1, box.y + box.height);

                    auto request = std::make_shared<GetMedianDepthXYZ::Request>();
                    request->header = msg_img->header;
                    request->x = {box.x, xmax};
                    request->y = {box.y, ymax};

                    cv::rectangle(img, cv::Point(box.x, box.y), cv::Point(xmax, ymax), cv::Scalar(0, 255, 0), 3);
                    cv::putText(img, class_names[classId], cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

                    auto response_received_callback = [this,msg_img,classId](ServiceResponseFuture future) {
                        auto result = future.get();
                        if (std::isnan(result.get()->x) || std::isnan(result.get()->y) || std::isnan(result.get()->z))
                            return;
                        if (std::isinf(result.get()->x) || std::isinf(result.get()->y) || std::isinf(result.get()->z))
                            return;
                        if (result.get()->x == 0 & result.get()->y == 0 & result.get()->z == 0)
                            return;

                        geometry_msgs::msg::TransformStamped tf_msg;
                        tf_msg.header.stamp = msg_img->header.stamp;
                        tf_msg.header.frame_id = "kinect";

                        // Set the transform message fields
                        tf_msg.child_frame_id = class_names[classId];
                        tf_msg.transform.translation.x = result.get()->z;
                        tf_msg.transform.translation.y = -result.get()->x;
                        tf_msg.transform.translation.z = -result.get()->y;

                        // Publish the transform message
                        tfb_->sendTransform(tf_msg);

                        // Publish the QR code poses
                        world_info_msgs::msg::WorldInfo world_info_msg;

                        world_info_msg.header.stamp = msg_img->header.stamp;
                        world_info_msg.num = class_names[classId];
                        world_info_msg.pose.position.x = tf_msg.transform.translation.x;
                        world_info_msg.pose.position.y = tf_msg.transform.translation.y;
                        world_info_msg.pose.position.z = tf_msg.transform.translation.z;
                        world_info_msg.type = "hazmat";

                        // Publish the WorldInfo message
                        world_info_pub_->publish(world_info_msg);
                    };
                    auto future_result = median_xyz_client_->async_send_request(request, response_received_callback);
                }
            }
            catch (cv::Exception &e) {
                RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

           	// Display the frame
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(msg_img->header, "bgr8", img)
                .toImageMsg();
            pub_hazmat.publish(*img_msg.get());
        }

       	// const image_transport::CameraSubscriber sub_cam;
        const image_transport::Subscriber sub_cam;
        const image_transport::Publisher pub_hazmat;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;
        rclcpp::Client<GetMedianDepthXYZ>::SharedPtr median_xyz_client_;

        ov::Core core;
        std::shared_ptr<ov::Model>model;
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        std::string inference_mode;

        float SCORE_THRESHOLD;
        float NMS_THRESHOLD;
        float CONFIDENCE_THRESHOLD;
        std::vector<std::string> class_names;

        bool first_run = true;
    };

}	// namespace world_info
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::DetectHazmat)