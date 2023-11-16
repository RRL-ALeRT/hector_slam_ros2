#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

class DepthToPointcloudNode : public rclcpp::Node {
public:
    DepthToPointcloudNode()
        : Node("depth_to_pointcloud_node") {
        // Subscribe to the depth image topic
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth_image_topic", 10, std::bind(&DepthToPointcloudNode::depthCallback, this, std::placeholders::_1));

        // Create the publisher for the filtered point cloud
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud_topic", 10);

        // Subscribe to the camera info topic
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info_topic", 10, std::bind(&DepthToPointcloudNode::cameraInfoCallback, this, std::placeholders::_1));
    }

private:
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert depth image to a cv::Mat
        cv_bridge::CvImagePtr cv_image_ptr;
        try {
            cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat depth_image = cv_image_ptr->image;

        // Check if camera info is available
        if (!camera_info_) {
            RCLCPP_WARN(this->get_logger(), "Camera info not available. Skipping depth to point cloud conversion.");
            return;
        }

        // Create the camera model
        image_geometry::PinholeCameraModel camera_model;
        camera_model.fromCameraInfo(camera_info_);

        // Generate point cloud from depth image
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                pcl::PointXYZ point;

                // Compute the 3D point from the depth image
                float depth = depth_image.at<uint16_t>(v, u) * 0.001f;  // Convert to meters
                if (depth > 0.0f) {
                    cv::Point3d point_3d = camera_model.projectPixelTo3dRay(cv::Point2d(u, v));
                    point.x = point_3d.x * depth;
                    point.y = point_3d.y * depth;
                    point.z = point_3d.z * depth;
                    pcl_pointcloud->push_back(point);
                }
            }
        }

        // Apply voxel grid filtering using pcl
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(pcl_pointcloud);
        voxel_filter.setLeafSize(0.1, 0.1, 0.1);  // Set the voxel size
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.filter(*downsampled_cloud);

        // Apply statistical outlier removal to filter out noisy points
        int num_neighbors = 20; // Adjust the number of neighbors as needed
        double std_dev = 0.02;  // Adjust the standard deviation threshold as needed
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
        outlier_filter.setInputCloud(downsampled_cloud);
        outlier_filter.setMeanK(num_neighbors);
        outlier_filter.setStddevMulThresh(std_dev);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        outlier_filter.filter(*outlier_removed_cloud);

        // Convert pcl point cloud to ROS 2 PointCloud2 message
        sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*outlier_removed_cloud, *pointcloud_msg);
        pointcloud_msg->header = msg->header;

        // Publish the filtered point cloud
        pointcloud_pub_->publish(*pointcloud_msg);
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_ = msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthToPointcloudNode>());
    rclcpp::shutdown();
    return 0;
}
