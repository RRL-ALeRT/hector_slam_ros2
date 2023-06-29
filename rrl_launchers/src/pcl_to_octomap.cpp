#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <std_srvs/srv/empty.hpp>

class OctoMapProcessingNode : public rclcpp::Node
{
public:
  OctoMapProcessingNode()
    : Node("octomap_processing_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "filtered_point_cloud", 1, std::bind(&OctoMapProcessingNode::pointCloudCallback, this, std::placeholders::_1));

    update_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&OctoMapProcessingNode::updateCallback, this));

    // Initialize the OctoMap
    octomap_ = std::make_shared<octomap::OcTree>(0.05); // Set the desired resolution of the OctoMap

    save_octomap_service_ = this->create_service<std_srvs::srv::Empty>(
      "save_octomap", std::bind(&OctoMapProcessingNode::saveOctoMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    filtered_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("filtered_octomap", 10);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    std::string fromFrameRel = "vision";
    std::string toFrameRel = "body";

    if (!tf2_published) {
      try {
        static_tf2 = tf_buffer_->lookupTransform(
          toFrameRel, fromFrameRel,
          tf2::TimePointZero);
        tf2_published = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
        return;
      }
    }

    static_tf2.header.frame_id = "map";
    static_tf2.child_frame_id = "vision";
    tf_static_broadcaster_->sendTransform(static_tf2);

    // Convert ROS point cloud message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    geometry_msgs::msg::TransformStamped t;

    fromFrameRel = msg->header.frame_id;
    toFrameRel = "map";
    try {
      t = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRel,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "%s", ex.what());
      return;
    }

    // Apply the transformation to each point in the input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(*pcl_cloud, *transformed_cloud, t);

    // Update the OctoMap with the point cloud
    for (const auto& point : transformed_cloud->points)
    {
      octomap_->updateNode(point.x, point.y, point.z, true); // Set occupied voxels
    }
  }

  void updateCallback()
  {
    // Prune the OctoMap
    octomap_->prune();

    // Create a new OctoMap that contains only the updated part
    octomap::OcTree updated_octomap(octomap_->getResolution());
  
    // Iterate over the leaf nodes of the OctoMap
    for (auto it = octomap_->begin_leafs(), end = octomap_->end_leafs(); it != end; ++it)
    {
      if (octomap_->isNodeOccupied(&(*it)))
      {
        // Add the occupied node to the updated OctoMap
        updated_octomap.updateNode(it.getKey(), true);
      }
    }

    // Publish the updated OctoMap as a ROS message
    auto updated_octomap_msg = std::make_shared<octomap_msgs::msg::Octomap>();
    octomap_msgs::binaryMapToMsg(updated_octomap, *updated_octomap_msg);
    updated_octomap_msg->header.frame_id = "map"; // Set the frame ID of the OctoMap
    updated_octomap_msg->header.stamp = this->now();
    filtered_octomap_pub_->publish(*updated_octomap_msg);
  }

  void saveOctoMapCallback(const std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response)
  {
    // Prune the OctoMap
    octomap_->prune();
    // Generate a file name based on the current date and time
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    char filename[100];
    std::strftime(filename, sizeof(filename), "octomap_%Y%m%d_%H%M%S.ot", timeinfo);
    // Save the OctoMap to a file
    octomap_->write(filename);
    RCLCPP_INFO(this->get_logger(), "OctoMap saved to: %s", filename);
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped static_tf2;
  bool tf2_published;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<octomap::OcTree> octomap_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr filtered_octomap_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_octomap_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctoMapProcessingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}