
#include "rclcpp/rclcpp.hpp"

#include "world_info_msgs/msg/world_info.hpp"
#include "world_info_msgs/msg/world_info_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "std_srvs/srv/empty.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using Empty = std_srvs::srv::Empty;
using namespace std::placeholders;
using namespace std::chrono_literals;

void add_to_mean(geometry_msgs::msg::Pose& mean, const geometry_msgs::msg::Pose newPose, int& numPoses)
{
  // Update position mean
  mean.position.x = (mean.position.x * numPoses + newPose.position.x) / (numPoses + 1);
  mean.position.y = (mean.position.y * numPoses + newPose.position.y) / (numPoses + 1);
  mean.position.z = (mean.position.z * numPoses + newPose.position.z) / (numPoses + 1);

  // Update orientation mean
  mean.orientation.x = (mean.orientation.x * numPoses + newPose.orientation.x) / (numPoses + 1);
  mean.orientation.y = (mean.orientation.y * numPoses + newPose.orientation.y) / (numPoses + 1);
  mean.orientation.z = (mean.orientation.z * numPoses + newPose.orientation.z) / (numPoses + 1);
  mean.orientation.w = (mean.orientation.w * numPoses + newPose.orientation.w) / (numPoses + 1);

  // Normalize the quaternion
  double norm = sqrt(
      mean.orientation.x * mean.orientation.x
    + mean.orientation.y * mean.orientation.y
    + mean.orientation.z * mean.orientation.z
    + mean.orientation.w * mean.orientation.w);
  mean.orientation.x /= norm;
  mean.orientation.y /= norm;
  mean.orientation.z /= norm;
  mean.orientation.w /= norm;

  numPoses++;
}

bool within_one_meter_range(const geometry_msgs::msg::Pose mean, const geometry_msgs::msg::Pose newPose)
{
  if (std::abs(mean.position.x - newPose.position.x) > 1.0) return false;
  if (std::abs(mean.position.y - newPose.position.y) > 1.0) return false;
  if (std::abs(mean.position.z - newPose.position.z) > 1.0) return false;
  return true;
}

void set_height_1or2(double& value)
{
  double difference1 = std::abs(value - 1);
  double difference2 = std::abs(value - 2);

  value = difference1 < difference2 ? 1 : 2;
}

class WorldInfo : public rclcpp::Node
{
public:
  explicit WorldInfo(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("world_info", options)
  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream csv_start_time;
    csv_start_time << "\"" << std::put_time(&tm, "%Y-%m-%d") << "\"" << "\n" << "\"" << std::put_time(&tm, "%H:%M:%S") << "\"" << "\n";
    empty_wi_vector.start_time = csv_start_time.str();
    wi_vector = empty_wi_vector;

    sub = create_subscription<world_info_msgs::msg::WorldInfo>("/world_info", 1, std::bind(&WorldInfo::receive_info, this, std::placeholders::_1));
    wi_pub = create_publisher<world_info_msgs::msg::WorldInfoArray>("/world_info_array", 1);
    markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 1);

    reset_world = create_service<Empty>("reset_world_info", std::bind(&WorldInfo::reset_world_cb, this, _1, _2, _3));

    // Call on_timer function every second
    timer_ = create_wall_timer(1s, std::bind(&WorldInfo::on_timer, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void on_timer()
  {
    if (wi_vector.array.size() > 0)
      wi_pub->publish(wi_vector);
  }

  void reset_world_cb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Empty::Request> request,
    const std::shared_ptr<Empty::Response> response)
  {
    (void)request_header;
    tag_collection.clear();
    RCLCPP_INFO(get_logger(), "world_info resetted");
  }

  void transform_pose(world_info_msgs::msg::WorldInfo& in, std::string target_frame)
  {
    auto source_to_target = tf_buffer_->lookupTransform(in.header.frame_id, target_frame, in.header.stamp, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform t_source_to_target(
      tf2::Matrix3x3(tf2::Quaternion(
        source_to_target.transform.rotation.x,source_to_target.transform.rotation.y,source_to_target.transform.rotation.z,source_to_target.transform.rotation.w)),
      tf2::Vector3(source_to_target.transform.translation.x, source_to_target.transform.translation.y, source_to_target.transform.translation.z)
    );

    auto t_position = t_source_to_target.inverse()*(tf2::Vector3(in.pose.position.x, in.pose.position.y, in.pose.position.z));
    auto t_orientation = t_source_to_target.inverse()*(tf2::Quaternion(in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w));
    in.pose.position.x = t_position.getX();
    in.pose.position.y = t_position.getY();
    in.pose.position.z = t_position.getZ();
    in.pose.orientation.x = t_orientation.getX();
    in.pose.orientation.y = t_orientation.getY();
    in.pose.orientation.z = t_orientation.getZ();
    in.pose.orientation.w = t_orientation.getW();
    in.header.frame_id = target_frame;
  }

  void receive_info(const world_info_msgs::msg::WorldInfo::SharedPtr msg)
  {
    if (msg->header.frame_id == "kinect_color") msg->header.frame_id = "kinect"; // for webots

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    std::string time_str = oss.str();

    try
    {
      transform_pose(*msg, "map");
    }
    catch (tf2::ConnectivityException& e)
    {
      RCLCPP_WARN(get_logger(), e.what());
      return;
    }
    catch (tf2::ExtrapolationException& e)
    {
      RCLCPP_WARN(get_logger(), e.what());
      return;
    }
    catch (tf2::LookupException& e)
    {
      RCLCPP_WARN(get_logger(), e.what());
      return;
    }

    std::string tag_name = msg->type + "_" + msg->num;

    bool tag_found = false;
    if (tag_collection.find(tag_name) != tag_collection.end())
    {
      for (auto tag: tag_collection[tag_name])
      {
        if (within_one_meter_range(tag.info.pose, msg->pose))
        {
          if (tag.count <= 100)
          {
            add_to_mean(tag.info.pose, msg->pose, tag.count);
          }
          tag_found = true;
          break;
        }
      }
    }
    if (!tag_found)
    {
      PoseCount tag_dict;
      tag_dict = PoseCount{*msg, 1, id_count, time_str, std::string("Spot"), std::string("A")}; // First time
      id_count++;
      tag_collection[tag_name].push_back(tag_dict);
    }

    // get rviz2 markers and tag_locations from various dicts
    marker_array = empty_marker_array;
    wi_vector = empty_wi_vector;

    for (auto tag_vector = tag_collection.begin(); tag_vector != tag_collection.end(); ++tag_vector)
    {
      for (const auto& it: tag_vector->second)
      {
        uint32_t shape = visualization_msgs::msg::Marker::CUBE;
        visualization_msgs::msg::Marker marker;
        marker.header = it.info.header;
        marker.header.frame_id = "map";
        marker.ns = it.info.type;
        marker.id = marker_array.markers.size()+1;
        marker.type = shape;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // For 1m/2m maps
        auto modified_info = it.info;
        set_height_1or2(modified_info.pose.position.z);

        marker.pose = modified_info.pose;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker_array.markers.push_back(marker);

        wi_vector.array.push_back(modified_info);
        wi_vector.id_array.push_back(it.id);
        wi_vector.time_array.push_back(it.time);
        wi_vector.robot_array.push_back(it.robot);
        wi_vector.mode_array.push_back(it.mode);
      }
    }

    markers_pub->publish(marker_array);
    wi_pub->publish(wi_vector);
  }

  world_info_msgs::msg::WorldInfo info;
  rclcpp::Subscription<world_info_msgs::msg::WorldInfo>::SharedPtr sub;
  rclcpp::Publisher<world_info_msgs::msg::WorldInfoArray>::SharedPtr wi_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  rclcpp::Service<Empty>::SharedPtr reset_world;

  visualization_msgs::msg::MarkerArray marker_array, empty_marker_array;
  world_info_msgs::msg::WorldInfoArray wi_vector, empty_wi_vector;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  int id_count = 0;

  struct PoseCount
  {
    world_info_msgs::msg::WorldInfo info;
    int count;
    int id;
    std::string time;
    std::string robot;
    std::string mode;
  };

  std::unordered_map<std::string, std::vector<PoseCount>> tag_collection;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldInfo>());
  rclcpp::shutdown();
  return 0;
}
