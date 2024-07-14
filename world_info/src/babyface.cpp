#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <world_info_msgs/msg/world_info.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <world_info_msgs/srv/get_median_depth_xyz.hpp>

#include "tf2_ros/transform_broadcaster.h"

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

    cv::Mat letterbox(cv::Mat &img, std::vector<float> &paddings, std::vector<int> new_shape = { 640, 640 })
    {
       	// Get current image shape[height, width]
       	// Refer to https://github.com/ultralytics/yolov5/blob/master/utils/augmentations.py#L111

        int img_h = img.rows;
        int img_w = img.cols;

       	// Compute scale ratio(new / old) and target resized shape
        float scale = std::min(new_shape[1] *1.0 / img_h, new_shape[0] *1.0 / img_w);
        int resize_h = int(round(img_h *scale));
        int resize_w = int(round(img_w *scale));
        paddings[0] = scale;

       	// Compute padding
        int pad_h = new_shape[1] - resize_h;
        int pad_w = new_shape[0] - resize_w;

       	// Resize and pad image while meeting stride-multiple constraints
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

       	// divide padding into 2 sides
        float half_h = pad_h *1.0 / 2;
        float half_w = pad_w *1.0 / 2;
        paddings[1] = half_h;
        paddings[2] = half_w;

       	// Compute padding boarder
        int top = int(round(half_h - 0.1));
        int bottom = int(round(half_h + 0.1));
        int left = int(round(half_w - 0.1));
        int right = int(round(half_w + 0.1));

       	// Add border
        cv::copyMakeBorder(resized_img, resized_img, top, bottom, left, right, 0, cv::Scalar(114, 114, 114));

        return resized_img;
    }

    std::vector<cv::Scalar>colors = { cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
        cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255)
    };

    float sigmoid_function(float a)
    {
        float b = 1. / (1. + exp(-a));
        return b;
    }

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

    class DetectBabyface: public rclcpp::Node
    {
        public:

            explicit DetectBabyface(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): Node("babyface", options),
       	// topics
        sub_cam(image_transport::create_subscription(this, "image_rect",
            std::bind(&DetectBabyface::onCamera, this, std::placeholders::_1),
            declare_parameter("image_transport", "raw", descr( {}, true)), rmw_qos_profile_sensor_data)),
        pub_babyface(image_transport::create_publisher(this, "babyface_detected"))
        {
            // Initialize TransformBroadcaster
            tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

           	// Create WorldInfo publisher
            world_info_pub_ = create_publisher<world_info_msgs::msg::WorldInfo>("/world_info_sub", 1);

            median_xyz_client_ = create_client<GetMedianDepthXYZ>("get_median_depth_xyz");
            // while (!median_xyz_client_->wait_for_service(std::chrono::seconds(1))) {
            //     if (!rclcpp::ok()) {
            //         RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            //         return;
            //     }
            //     RCLCPP_INFO(get_logger(), "service from rs_depth not available, waiting again...");
            // }

            SCORE_THRESHOLD = 0.2;
            NMS_THRESHOLD = 0.4;
            CONFIDENCE_THRESHOLD = 0.7;
            num_classes = 1;

            if (!has_parameter("babyface_confidence_threshold"))
                declare_parameter("babyface_confidence_threshold", CONFIDENCE_THRESHOLD);
            get_parameter("babyface_confidence_threshold", CONFIDENCE_THRESHOLD);

            inference_mode = "CPU";
            if (!has_parameter("inference_mode"))
                declare_parameter("inference_mode", inference_mode);
            get_parameter("inference_mode", inference_mode);

            std::string package_share_directory = ament_index_cpp::get_package_share_directory("world_info");
            model = core.read_model(package_share_directory + "/weights/babyface_seg.onnx");
        }

        ~DetectBabyface()
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
                cv::RNG rng;
                cv::Mat masked_img;
                std::vector<float> paddings(3);	//scale, half_h, half_w
                cv::Mat resized_img = letterbox(img, paddings);	//resize to (640,640) by letterbox
                // BGR->RGB, u8(0-255)->f32(0.0-1.0), HWC->NCHW
                cv::Mat blob = cv::dnn::blobFromImage(resized_img, 1 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true);

                if (first_run)
                {
                    // Inizialize Preprocessing for the model
                    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(model);
                    model = ppp.build();
                    compiled_model = core.compile_model(model, inference_mode);

                    // Create an infer request for model inference 
                    infer_request = compiled_model.create_infer_request();

                    first_run = false;
                }

                ov::Tensor input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), blob.ptr(0));
                infer_request.set_input_tensor(input_tensor);
                infer_request.infer();

                // Retrieve inference results 
                auto detect = infer_request.get_output_tensor(0);
                auto detect_shape = detect.get_shape();
                // std::cout << "The shape of Detection tensor:"<< detect_shape << std::endl;
                auto proto = infer_request.get_output_tensor(1);
                auto proto_shape = proto.get_shape();
                // std::cout << "The shape of Proto tensor:" << proto_shape << std::endl;
                
                // --------- Do the Post Process
                // Detect Matrix: 25200 x 117  
                cv::Mat detect_buffer(detect_shape[1], detect_shape[2], CV_32F, detect.data());
                // Proto Matrix:  1x32x160x160 => 32 x 25600
                cv::Mat proto_buffer(proto_shape[1], proto_shape[2] *proto_shape[3], CV_32F, proto.data());

                // -------- Post-process the inference result -----------
                float nms_threshold = 0.5;
                std::vector<cv::Rect>boxes;
                std::vector<int> class_ids;
                std::vector<float> class_scores;
                std::vector<float> confidences;
                std::vector<cv::Mat>masks;
                // cx,cy,w,h,confidence,c1,c2,...c80
                float scale = paddings[0];
                for (int i = 0; i < detect_buffer.rows; i++)
                {
                    float confidence = detect_buffer.at<float> (i, 4);
                    if (confidence < CONFIDENCE_THRESHOLD)
                    {
                        continue;
                    }

                    cv::Mat classes_scores = detect_buffer.row(i).colRange(5, 5+num_classes);
                    cv::Point class_id;
                    double score;
                    cv::minMaxLoc(classes_scores, NULL, &score, NULL, &class_id);

                    // class score: 0~1
                    if (score>0.25)
                    {
                        cv::Mat mask = detect_buffer.row(i).colRange(5+num_classes, 5+num_classes+32);
                        float cx = detect_buffer.at<float> (i, 0);
                        float cy = detect_buffer.at<float> (i, 1);
                        float w = detect_buffer.at<float> (i, 2);
                        float h = detect_buffer.at<float> (i, 3);
                        int left = static_cast<int> ((cx - 0.5 *w - paddings[2]) / scale);
                        int top = static_cast<int> ((cy - 0.5 *h - paddings[1]) / scale);
                        int width = static_cast<int> (w / scale);
                        int height = static_cast<int> (h / scale);
                        cv::Rect box;
                        box.x = left;
                        box.y = top;
                        box.width = width;
                        box.height = height;

                        boxes.push_back(box);
                        class_ids.push_back(class_id.x);
                        class_scores.push_back(score);
                        confidences.push_back(confidence);
                        masks.push_back(mask);
                    }
                }

                // NMS
                std::vector<int> indices;
                cv::dnn::NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, nms_threshold, indices);
                cv::Mat rgb_mask = cv::Mat::zeros(img.size(), img.type());

                // -------- Visualize the detection results -----------
                for (size_t i = 0; i < indices.size(); i++)
                {
                    int index = indices[i];
                    int class_id = class_ids[index];
                    cv::Rect box = boxes[index];
                    int x1 = std::max(0, box.x);
                    int y1 = std::max(0, box.y);
                    int x2 = std::max(0, box.br().x);
                    int y2 = std::max(0, box.br().y);

                    cv::Mat m = masks[index] *proto_buffer;
                    for (int col = 0; col < m.cols; col++)
                    {
                        m.at<float> (0, col) = sigmoid_function(m.at<float> (0, col));
                    }

                    cv::Mat m1 = m.reshape(1, 160);	// 1x25600 -> 160x160

                    int mx1 = std::max(0, int((x1 *scale + paddings[2]) *0.25));
                    int mx2 = std::max(0, int((x2 *scale + paddings[2]) *0.25));
                    int my1 = std::max(0, int((y1 *scale + paddings[1]) *0.25));
                    int my2 = std::max(0, int((y2 *scale + paddings[1]) *0.25));
                    cv::Mat mask_roi = m1(cv::Range(my1, my2), cv::Range(mx1, mx2));
                    cv::Mat rm, det_mask;
                    cv::resize(mask_roi, rm, cv::Size(x2 - x1, y2 - y1));
                    for (int r = 0; r < rm.rows; r++)
                    {
                        for (int c = 0; c < rm.cols; c++)
                        {
                            float pv = rm.at<float> (r, c);
                            if (pv>0.5)
                            {
                                rm.at<float> (r, c) = 1.0;
                            }
                            else
                            {
                                rm.at<float> (r, c) = 0.0;
                            }
                        }
                    }

                    rm = rm *rng.uniform(0, 255);
                    rm.convertTo(det_mask, CV_8UC1);
                    if ((y1 + det_mask.rows) >= img.rows)
                    {
                        y2 = img.rows - 1;
                    }

                    if ((x1 + det_mask.cols) >= img.cols)
                    {
                        x2 = img.cols - 1;
                    }

                    cv::Mat mask = cv::Mat::zeros(cv::Size(img.cols, img.rows), CV_8UC1);
                    det_mask(cv::Range(0, y2 - y1), cv::Range(0, x2 - x1)).copyTo(mask(cv::Range(y1, y2), cv::Range(x1, x2)));
                    add(rgb_mask, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), rgb_mask, mask);

                    cv::rectangle(img, boxes[index], colors[class_id % 6], 2, 8);
                    std::string label = std::to_string(class_id) + ":" + std::to_string(class_scores[index]);
                    cv::putText(img, label, cv::Point(boxes[index].tl().x, boxes[index].tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, .5, colors[class_id % 6]);
                    cv::addWeighted(img, 0.8, rgb_mask, 0.2, 0, img);

                    // Mask to x-y points
                    auto request = std::make_shared<GetMedianDepthXYZ::Request>();
                    request->header = msg_img->header;
                    request->x = {box.x, box.x + box.width};
                    request->y = {box.y, box.y + box.y + box.height};
                    // for (int j = 0; j < mask.rows; j++) {
                    //     for (int i = 0; i < mask.cols; i++) {
                    //         if (mask.at<uchar>(i, j) != 0) {
                    //             request->x.push_back(i);
                    //             request->y.push_back(j);
                    //         }
                    //     }
                    // }
                    
                    auto response_received_callback = [this,msg_img](ServiceResponseFuture future) {
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
                        tf_msg.child_frame_id = "victim";
                        tf_msg.transform.translation.x = result.get()->z;
                        tf_msg.transform.translation.y = -result.get()->x;
                        tf_msg.transform.translation.z = -result.get()->y;

                        // Publish the transform message
                        tfb_->sendTransform(tf_msg);

                        // Publish the QR code poses
                        world_info_msgs::msg::WorldInfo world_info_msg;

                        world_info_msg.header.stamp = msg_img->header.stamp;
                        world_info_msg.num = "victim";
                        world_info_msg.pose.position.x = tf_msg.transform.translation.x;
                        world_info_msg.pose.position.y = tf_msg.transform.translation.y;
                        world_info_msg.pose.position.z = tf_msg.transform.translation.z;
                        world_info_msg.type = "victim";

                        // Publish the WorldInfo message
                        world_info_pub_->publish(world_info_msg);
                    };
                    // auto future_result = median_xyz_client_->async_send_request(request, response_received_callback);
                }
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            catch (cv::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv exception: %s", e.what());
                return;
            }


            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img)
                .toImageMsg();
            pub_babyface.publish(*img_msg.get());
        }

       	// const image_transport::CameraSubscriber sub_cam;
        const image_transport::Subscriber sub_cam;
        const image_transport::Publisher pub_babyface;

        rclcpp::Publisher<world_info_msgs::msg::WorldInfo>::SharedPtr world_info_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
        rclcpp::Client<GetMedianDepthXYZ>::SharedPtr median_xyz_client_;

        ov::Core core;
        std::shared_ptr<ov::Model>model;
        ov::CompiledModel compiled_model;
        ov::InferRequest infer_request;
        std::string inference_mode;

        float SCORE_THRESHOLD;
        float NMS_THRESHOLD;
        float CONFIDENCE_THRESHOLD;

        bool first_run = true;
        int num_classes;
    };

}	// namespace world_info
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(world_info::DetectBabyface)