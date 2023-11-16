#include <chrono>
#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spot_msgs/msg/battery_state_array.hpp"

class BatteryState : public rclcpp::Node {
public:
  BatteryState()
      : Node("battery_screen"), first_time(true) {
    subscription_ = create_subscription<spot_msgs::msg::BatteryStateArray>(
        "/status/battery_states", 1,
        std::bind(&BatteryState::bs_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::seconds(20),
                               std::bind(&BatteryState::write_bs, this));
  }

private:
  void bs_callback(const spot_msgs::msg::BatteryStateArray::SharedPtr msg) {
    percent = static_cast<int>(msg->battery_states[0].charge_percentage);
    minutes = msg->battery_states[0].estimated_runtime.sec / 60;

    if (first_time) {
      first_time = false;
      write_bs();
    }
  }

  void write_bs() {
    if (!percent) {
      return;
    }

    std::ofstream file(std::getenv("HOME") + std::string("/spot_battery.txt"));
    if (file.is_open()) {
      file << "Spot: " << percent << "%, " << minutes << " min";
    }
  }

  rclcpp::Subscription<spot_msgs::msg::BatteryStateArray>::SharedPtr
      subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool first_time;
  int percent;
  int minutes;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto bs = std::make_shared<BatteryState>();
  rclcpp::spin(bs);
  rclcpp::shutdown();
  return 0;
}
