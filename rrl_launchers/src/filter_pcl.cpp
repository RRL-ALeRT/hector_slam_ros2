#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

namespace rrl_launchers {

class FilteredPointCloud : public rclcpp::Node
{
public:
  explicit FilteredPointCloud(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("filtered_points", options)
  {
    pcl_filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 1);
    point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", 1, std::bind(&FilteredPointCloud::pointCloudCallback, this, std::placeholders::_1));
  }

  ~FilteredPointCloud() {
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (pcl_filtered_pub_->get_subscription_count() == 0)
    {
      // No subscribers, skip processing
      return;
    }

    // Convert ROS point cloud message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // // Apply statistical outlier removal to filter out noisy points
    // int num_neighbors = 20; // Adjust the number of neighbors as needed
    // double std_dev = 0.02;  // Adjust the standard deviation threshold as needed
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
    // outlier_filter.setInputCloud(downsampled_cloud);
    // outlier_filter.setMeanK(num_neighbors);
    // outlier_filter.setStddevMulThresh(std_dev);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // outlier_filter.filter(*outlier_removed_cloud);

    double radius = 0.1;    // Radius to search for neighbors
    int min_neighbors = 5;  // Minimum number of neighbors within radius to keep a point

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;  // Ensure this line uses RadiusOutlierRemoval
    outlier_filter.setInputCloud(pcl_cloud);
    outlier_filter.setRadiusSearch(radius);
    outlier_filter.setMinNeighborsInRadius(min_neighbors);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    outlier_filter.filter(*outlier_removed_cloud);

    // Convert the filtered PCL point cloud back to a ROS message
    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*outlier_removed_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = msg->header;

    // Publish the filtered point cloud
    pcl_filtered_pub_->publish(filtered_cloud_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_filtered_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
};

} // namespace rrl_launchers

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rrl_launchers::FilteredPointCloud)
