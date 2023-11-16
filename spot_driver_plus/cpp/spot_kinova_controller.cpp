#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <spot_msgs/srv/set_locomotion.hpp>
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/planning_options.hpp>

#include <tf2/utils.h>

const std::string JOY_TOPIC = "/joy";
const std::string SPOT_TWIST_TOPIC = "/cmd_vel";
const std::string SPOT_POSE_TOPIC = "/body_pose";
// const std::string KINOVA_TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string KINOVA_TWIST_TOPIC = "/twist_controller/commands";
using KinovaTwistType = geometry_msgs::msg::TwistStamped;
const std::string KINOVA_GRIPPER_VEL = "/twist_controller/gripper_vel";
const double GRIPPER_VELOCITY = 0.2;

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum AxisBluetooth
{
  LEFT_STICK_X_BT = 0,
  LEFT_STICK_Y_BT = 1,
  LEFT_TRIGGER_BT = 5,
  RIGHT_STICK_X_BT = 2,
  RIGHT_STICK_Y_BT = 3,
  RIGHT_TRIGGER_BT = 4,
  D_PAD_X_BT = 6,
  D_PAD_Y_BT = 7
};
enum AxisDeck
{
  LEFT_STICK_X_DECK = 0,
  LEFT_STICK_Y_DECK = 1,
  LEFT_TRIGGER_DECK = 9,
  RIGHT_STICK_X_DECK = 2,
  RIGHT_STICK_Y_DECK = 3,
  RIGHT_TRIGGER_DECK = 8
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 6,
  MENU = 7,
  XBOX = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10,
  HOME = 11
};
enum ButtonBluetooth
{
  A_BT = 0,
  B_BT = 1,
  X_BT = 3,
  Y_BT = 4,
  LEFT_BUMPER_BT = 6,
  RIGHT_BUMPER_BT = 7,
  CHANGE_VIEW_BT = 10,
  MENU_BT = 11,
  XBOX_BT = 8,
  LEFT_STICK_CLICK_BT = 13,
  RIGHT_STICK_CLICK_BT = 14,
  HOME_BT = 15
};
enum ButtonDeck
{
  A_DECK = 3,
  B_DECK = 4,
  X_DECK = 5,
  Y_DECK = 6,
  LEFT_BUMPER_DECK = 7,
  RIGHT_BUMPER_DECK = 8,
  CHANGE_VIEW_DECK = 11,
  MENU_DECK = 12,
  DECK = 13,
  LEFT_STICK_CLICK_DECK = 14,
  RIGHT_STICK_CLICK_DECK = 15,
  HOME_DECK = 2,
  D_PAD_X_L_DECK = 18,
  D_PAD_X_R_DECK = 19,
  D_PAD_Y_U_DECK = 16,
  D_PAD_Y_D_DECK = 17
};
enum Gait
{
  AUTO = 1,
  TROT = 2,
  CRAWL = 4,
  AMBLE = 5,
  JOG = 7,
  HOP = 8
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
const std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };

Gait getNextGait(Gait currentGait)
{
  switch (currentGait)
  {
    case AUTO: return TROT;
    case TROT: return CRAWL;
    case CRAWL: return AMBLE;
    case AMBLE: return JOG;
    case JOG: return HOP;
    case HOP: return AUTO;
    default: return AUTO; // Handle invalid or unknown values
  }
}

/** \brief // This converts a joystick axes and buttons array to a Twist message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A Twist message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmdKinova(const std::vector<float>& axes, const std::vector<int>& buttons,
                           geometry_msgs::msg::Twist& twist)
{
  // Ignore if buttons are pressed. 
  // skpawar1305: We'll simultaneously control gripper as well. Ignore buttons X and Y.
  // if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  if (buttons[A] || buttons[B] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    return false;
  }

  // The bread and butter: map buttons to twist commands
  twist.linear.x = axes[RIGHT_STICK_X];
  twist.linear.y = axes[RIGHT_STICK_Y];

  double lin_z_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_z_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist.linear.z = (lin_z_right + lin_z_left);

  twist.angular.x = - 1.5 * axes[LEFT_STICK_Y];
  twist.angular.y = 1.5 * axes[LEFT_STICK_X];

  double roll_positive = buttons[RIGHT_BUMPER];
  double roll_negative = - 1 * (buttons[LEFT_BUMPER]);
  twist.angular.z = 1.5 * (roll_positive + roll_negative);

  twist.linear.x = twist.linear.x / 5;
  twist.linear.y = twist.linear.y / 5;
  twist.linear.z = twist.linear.z / 5;

  twist.angular.x = twist.angular.x * (180 / M_PI) / 5;
  twist.angular.y = twist.angular.y * (180 / M_PI) / 5;
  twist.angular.z = twist.angular.z * (180 / M_PI) / 5;

  return true;
}

/** \brief // This converts a joystick axes and buttons array to a Twist message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A Twist message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmdSpot(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::shared_ptr<geometry_msgs::msg::Twist>& twist, const float& spot_speed_factor,
                     const Gait& MODE)
{
  // Ignore if buttons are pressed
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    return false;
  }

  // Use our variable speed factor only in AUTO Locomotion Mode
  float speed = MODE == AUTO ? spot_speed_factor : 0.4;

  // The bread and butter: map buttons to twist commands
  twist->linear.x = speed * axes[LEFT_STICK_Y];
  twist->linear.y = speed * axes[LEFT_STICK_X];
  twist->angular.z = axes[RIGHT_STICK_X];

  return true;
}

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param pose A Pose message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToPoseSpot(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::shared_ptr<geometry_msgs::msg::Pose>& pose, const float& height)
{
  // Ignore if buttons are pressed
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    return false;
  }

  pose->position.z = height;

  tf2::Quaternion q;
  q.setRPY(-axes[LEFT_STICK_X], axes[LEFT_STICK_Y], axes[RIGHT_STICK_X]);
  // The bread and butter: pose buttons to twist commands
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
  pose->orientation.w = q.w();
  
  return true;
}

void resetBodyPose(float& height, float& roll, float& pitch, float& yaw)
{
  height = 0.0;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
}

// For when the controller is connected with bluetooth
sensor_msgs::msg::Joy bluetooth_remapped(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
  sensor_msgs::msg::Joy bt_msg;

  bt_msg.header = msg->header;

  bt_msg.axes.push_back(msg->axes[LEFT_STICK_X_BT]);
  bt_msg.axes.push_back(msg->axes[LEFT_STICK_Y_BT]);
  bt_msg.axes.push_back(msg->axes[LEFT_TRIGGER_BT]);
  bt_msg.axes.push_back(msg->axes[RIGHT_STICK_X_BT]);
  bt_msg.axes.push_back(msg->axes[RIGHT_STICK_Y_BT]);
  bt_msg.axes.push_back(msg->axes[RIGHT_TRIGGER_BT]);
  bt_msg.axes.push_back(msg->axes[D_PAD_X_BT]);
  bt_msg.axes.push_back(msg->axes[D_PAD_Y_BT]);

  bt_msg.buttons.push_back(msg->buttons[A_BT]);
  bt_msg.buttons.push_back(msg->buttons[B_BT]);
  bt_msg.buttons.push_back(msg->buttons[X_BT]);
  bt_msg.buttons.push_back(msg->buttons[Y_BT]);
  bt_msg.buttons.push_back(msg->buttons[LEFT_BUMPER_BT]);
  bt_msg.buttons.push_back(msg->buttons[RIGHT_BUMPER_BT]);
  bt_msg.buttons.push_back(msg->buttons[CHANGE_VIEW_BT]);
  bt_msg.buttons.push_back(msg->buttons[MENU_BT]);
  bt_msg.buttons.push_back(msg->buttons[XBOX_BT]);
  bt_msg.buttons.push_back(msg->buttons[LEFT_STICK_CLICK_BT]);
  bt_msg.buttons.push_back(msg->buttons[RIGHT_STICK_CLICK_BT]);
  bt_msg.buttons.push_back(msg->buttons[HOME_BT]);

  return bt_msg;
}

// For when the controller is steam deck
sensor_msgs::msg::Joy deck_remapped(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
  sensor_msgs::msg::Joy deck_msg;

  deck_msg.header = msg->header;

  deck_msg.axes.push_back(msg->axes[LEFT_STICK_X_DECK]);
  deck_msg.axes.push_back(msg->axes[LEFT_STICK_Y_DECK]);
  deck_msg.axes.push_back(msg->axes[LEFT_TRIGGER_DECK]);
  deck_msg.axes.push_back(msg->axes[RIGHT_STICK_X_DECK]);
  deck_msg.axes.push_back(msg->axes[RIGHT_STICK_Y_DECK]);
  deck_msg.axes.push_back(msg->axes[RIGHT_TRIGGER_DECK]);
  // Keeping consistency with other controllers
  if (msg->buttons[D_PAD_X_L_DECK] == 1)
  {
    deck_msg.axes.push_back(1);
  }
  else if (msg->buttons[D_PAD_X_R_DECK] == 1)
  {
    deck_msg.axes.push_back(-1);
  }
  else
  {
    deck_msg.axes.push_back(0);
  }
  if (msg->buttons[D_PAD_Y_U_DECK] == 1)
  {
    deck_msg.axes.push_back(1);
  }
  else if (msg->buttons[D_PAD_Y_D_DECK] == 1)
  {
    deck_msg.axes.push_back(-1);
  }
  else
  {
    deck_msg.axes.push_back(0);
  }

  deck_msg.buttons.push_back(msg->buttons[A_DECK]);
  deck_msg.buttons.push_back(msg->buttons[B_DECK]);
  deck_msg.buttons.push_back(msg->buttons[X_DECK]);
  deck_msg.buttons.push_back(msg->buttons[Y_DECK]);
  deck_msg.buttons.push_back(msg->buttons[LEFT_BUMPER_DECK]);
  deck_msg.buttons.push_back(msg->buttons[RIGHT_BUMPER_DECK]);
  deck_msg.buttons.push_back(msg->buttons[CHANGE_VIEW_DECK]);
  deck_msg.buttons.push_back(msg->buttons[MENU_DECK]);
  deck_msg.buttons.push_back(msg->buttons[DECK]);
  deck_msg.buttons.push_back(msg->buttons[LEFT_STICK_CLICK_DECK]);
  deck_msg.buttons.push_back(msg->buttons[RIGHT_STICK_CLICK_DECK]);
  deck_msg.buttons.push_back(msg->buttons[HOME_DECK]);

  return deck_msg;
}

class MoveGroupActionClient
{
using MoveGroupAction = moveit_msgs::action::MoveGroup;
using JointConstraint = moveit_msgs::msg::JointConstraint;
using Constraints = moveit_msgs::msg::Constraints;
using MotionPlanRequest = moveit_msgs::msg::MotionPlanRequest;
using PlanningOptions = moveit_msgs::msg::PlanningOptions;

public:
  MoveGroupActionClient(const std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  {
    action_client_ = rclcpp_action::create_client<MoveGroupAction>(
      node_, "/move_action");
  }

  void send_goal(std::vector<double> target_angles)
  {
    auto goal_msg = MoveGroupAction::Goal();
    auto motion_plan_request = MotionPlanRequest();

    // Set up motion plan request and constraints
    motion_plan_request.workspace_parameters.header.stamp = node_->get_clock()->now();
    motion_plan_request.workspace_parameters.header.frame_id = "base_link";
    motion_plan_request.workspace_parameters.min_corner.x = -1.0;
    motion_plan_request.workspace_parameters.min_corner.y = -1.0;
    motion_plan_request.workspace_parameters.min_corner.z = -1.0;
    motion_plan_request.workspace_parameters.max_corner.x = 1.0;
    motion_plan_request.workspace_parameters.max_corner.y = 1.0;
    motion_plan_request.workspace_parameters.max_corner.z = 1.0;
    motion_plan_request.start_state.is_diff = true;

    Constraints constraints;
    // Add joint constraints
    for (size_t i = 0; i < target_angles.size(); ++i) {
      JointConstraint jc;
      jc.joint_name = "joint_" + std::to_string(i + 1);
      jc.position = target_angles[i] * M_PI / 180;
      jc.tolerance_above = 0.001;
      jc.tolerance_below = 0.001;
      jc.weight = 1.0;
      constraints.joint_constraints.push_back(jc);
    }
    motion_plan_request.goal_constraints.push_back(constraints);

    // Set planning options
    motion_plan_request.pipeline_id = "ompl";
    motion_plan_request.group_name = "manipulator";
    motion_plan_request.num_planning_attempts = 4;
    motion_plan_request.allowed_planning_time = 10.0;
    motion_plan_request.max_velocity_scaling_factor = 0.4;
    motion_plan_request.max_acceleration_scaling_factor = 0.4;
    motion_plan_request.max_cartesian_speed = 0.0;

    PlanningOptions planning_options;
    planning_options.plan_only = false;
    planning_options.look_around = true;
    planning_options.look_around_attempts = 5;
    planning_options.max_safe_execution_cost = 0.0;
    planning_options.replan = true;
    planning_options.replan_attempts = 4;
    planning_options.replan_delay = 0.1;
    // Fill in the goal message
    goal_msg.request = motion_plan_request;
    goal_msg.planning_options = planning_options;

    // Send the goal
    auto future_goal_handle = action_client_->async_send_goal(goal_msg);
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<MoveGroupAction>::SharedPtr action_client_;
};

class GripperGoalSender
{
public:
    GripperGoalSender(const std::shared_ptr<rclcpp::Node> node)
    {
       action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
              node, "/robotiq_gripper_controller/gripper_cmd");
    }

    void sendGoal(double position, double max_effort)
    {
        // Create a GripperCommand goal
        control_msgs::action::GripperCommand::Goal goal;
        goal.command.position = position;
        goal.command.max_effort = max_effort;

        // Send the goal to the action server
        auto send_goal_future = action_client_->async_send_goal(goal);
    }

private:
    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr action_client_;
};

class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("spot_controller", options)
  {
    // Setup pub/sub
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(JOY_TOPIC, 1,
                                                          [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
                                                            return joyCB(msg);
                                                          });

    pose_pub_spot_ = create_publisher<geometry_msgs::msg::Pose>(SPOT_POSE_TOPIC, 1);

    twist_pub_spot_ = create_publisher<geometry_msgs::msg::Twist>(SPOT_TWIST_TOPIC, 1);
    twist_pub_kinova_ = create_publisher<KinovaTwistType>(KINOVA_TWIST_TOPIC, 1);
    gripper_vel_kinova_ = create_publisher<std_msgs::msg::Float32>(KINOVA_GRIPPER_VEL, 1);

    spot_stand = create_client<std_srvs::srv::Trigger>("/stand");
    spot_sit = create_client<std_srvs::srv::Trigger>("/sit");

    spot_gait = create_client<spot_msgs::srv::SetLocomotion>("/locomotion_mode");

    kinova_controller_switch_ = create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  }

  void create_action_clients()
  {
    std::shared_ptr<rclcpp::Node> node_ptr = shared_from_this();

    gripper_goal_sender = std::make_shared<GripperGoalSender>(node_ptr);
    move_group = std::make_shared<MoveGroupActionClient>(node_ptr);
  }

  void gait(const Gait& MODE)
  {
    if (!spot_gait->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "locomotion service not available");
      return;
    }
    auto request = std::make_shared<spot_msgs::srv::SetLocomotion::Request>();
    request.get()->locomotion_mode = static_cast<int>(MODE);

    auto future_result = spot_gait->async_send_request(request);
  }

  void stand()
  {
    resetBodyPose(height, roll, pitch, yaw);
    locomotion_mode = AUTO;
    gait(locomotion_mode);
    is_sitting = false;
    spot_speed_factor = 0.5;

    auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
    pose_msg->position.z = height;
    pose_pub_spot_->publish(*pose_msg);

    if (!spot_stand->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "stand service not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = spot_stand->async_send_request(request);
  }

  void sit()
  {
    is_sitting = true;
    if (!spot_sit->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "sit service not available");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result = spot_sit->async_send_request(request);
  }

  void kinova_switch_controller(const std::string& control_type)
  {
    if (!kinova_controller_switch_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "kinova switch controller service not available");
      return;
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    if (control_type == "twist_controller")
    {
      request->activate_controllers.push_back("twist_controller");
      request->deactivate_controllers.push_back("joint_trajectory_controller");
      request->deactivate_controllers.push_back("robotiq_gripper_controller");
    }
    else if (control_type == "joint_trajectory_controller")
    {
      request->activate_controllers.push_back("joint_trajectory_controller");
      request->activate_controllers.push_back("robotiq_gripper_controller");
      request->deactivate_controllers.push_back("twist_controller");
    }
    request->strictness = 1;
    request->activate_asap = true;
    auto future_result = kinova_controller_switch_->async_send_request(request);
  }

  void gripper_action(const std::string& gripper_command)
  {
    if (gripper_command == "OPEN") gripper_goal_sender->sendGoal(0.0, 100.0);
    else if (gripper_command == "CLOSE") gripper_goal_sender->sendGoal(0.71, 100.0);
  }

  void spot_control(const sensor_msgs::msg::Joy& msg, std::vector<int> buttons)
  {
    // Following makes the buttons behave like onPress
    for (long unsigned int i = 0; i < buttons.size(); ++i)
    {
      if (buttons[i] == 1)
      {
        if (pressed_buttons.count(i) != 0)
        {
          buttons[i] = 0; // Set buttons[i] to 0 if already pressed
        }
        else
        {
          pressed_buttons.insert(i); // Add pressed button to set
        }
      }
      else if (buttons[i] == 0)
      {
        pressed_buttons.erase(i); // Remove released button from set
      }
    }

    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
    auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();

    if (!stance)
    {
      if (convertJoyToCmdSpot(msg.axes, buttons, twist_msg, spot_speed_factor, locomotion_mode))
      {
        if (!is_sitting)
        {
          twist_pub_spot_->publish(*twist_msg);
        }
      }
    }
    else
    {
      if (convertJoyToPoseSpot(msg.axes, buttons, pose_msg, height))
      {
        pose_pub_spot_->publish(*pose_msg);
        if (!is_sitting) {
          twist_pub_spot_->publish(*twist_msg);
        }
      }
    }

    if (buttons[Y])
    {
      stand();
    }
    else if (buttons[X])
    {
      sit();
    }
    else if (buttons[B])
    {
      locomotion_mode = msg.axes[RIGHT_TRIGGER] == -1 ? getNextGait(locomotion_mode) : AUTO;
      gait(locomotion_mode);
    }
    else if (buttons[A])
    {
      if (stance) stance = false;
      else if (!stance) stance = true;
    }
    else if (buttons[LEFT_BUMPER])
    {
      spot_speed_factor = std::max(spot_speed_factor - 0.125, 0.2);
    }
    else if (buttons[RIGHT_BUMPER])
    {
      spot_speed_factor = std::min(spot_speed_factor + 0.125, 1.0);
    }
    else if (msg.axes[D_PAD_Y] != 0)
    {
      height = msg.axes[D_PAD_Y] * 0.2;
      if (!stance) {
        pose_msg->position.z = height;
        pose_pub_spot_->publish(*pose_msg);
      }
    }
  }

  void kinova_control(const sensor_msgs::msg::Joy& msg, std::vector<int> buttons)
  {
    auto twist_msg = std::make_shared<KinovaTwistType>();
    twist_msg->header.frame_id = "base_link";
    twist_msg->header.stamp = get_clock()->now();

    if (twist_controller_running)
    {
      if (convertJoyToCmdKinova(msg.axes, buttons, twist_msg->twist))
      {
        twist_pub_kinova_->publish(*twist_msg);
      }

      // To control gripper when in twist mode
      auto gripper_vel = std::make_shared<std_msgs::msg::Float32>();
      if (buttons[Y])
      {
        gripper_vel->data = GRIPPER_VELOCITY;
      }
      else if (buttons[X])
      {
        gripper_vel->data = -GRIPPER_VELOCITY;
      }
      gripper_vel_kinova_->publish(*gripper_vel);
    }

    // Following makes the buttons behave like onPress
    for (long unsigned int i = 0; i < buttons.size(); ++i)
    {
      if (buttons[i] == 1)
      {
        if (pressed_buttons.count(i) != 0)
        {
          buttons[i] = 0; // Set buttons[i] to 0 if already pressed
        }
        else
        {
          pressed_buttons.insert(i); // Add pressed button to set
        }
      }
      else if (buttons[i] == 0)
      {
        pressed_buttons.erase(i); // Remove released button from set
      }
    }
    
    if (!twist_controller_running)
    {
      // To control gripper when in joint_trajectory mode
      if (buttons[Y])
      {
        gripper_action("OPEN");
      }
      else if (buttons[X])
      {
        gripper_action("CLOSE");
      }
      else if (buttons[B])
      {
        std::vector<double> joint_angles_degrees{0, 15, -130, 0, 55, 90};
        move_group->send_goal(joint_angles_degrees); // Home Position
      }
      else if (buttons[A])
      {
        std::vector<double> joint_angles_degrees{0, -106, -148, 0, -59.2, 90};
        move_group->send_goal(joint_angles_degrees); // Retract Position
      }
    }

    // Switch between joint_trajectory/twist controllers for Kinova
    if (!d_pad_pressed)
    {
      d_pad_pressed = true;
      if (msg.axes[D_PAD_Y] == 1.0)
      {
        RCLCPP_INFO(get_logger(), "Switching to twist_controller");
        twist_controller_running = true;
        kinova_switch_controller("twist_controller");
      }
      else if (msg.axes[D_PAD_Y] == -1.0)
      {
        RCLCPP_INFO(get_logger(), "Switching to joint_trajectory_controller");
        twist_controller_running = false;
        kinova_switch_controller("joint_trajectory_controller");
      }
    }
    if (msg.axes[D_PAD_Y] == 0.0)
    {
      d_pad_pressed = false;
    }
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& joy_msg)
  {  
    sensor_msgs::msg::Joy msg;

    if (joy_msg->buttons.size() == 16)
    { 
      msg = bluetooth_remapped(joy_msg);
    }
    else if (joy_msg->buttons.size() == 24)
    { 
      msg = deck_remapped(joy_msg);
    }
    else {
      msg = *joy_msg;
    }

    auto buttons = msg.buttons;

    if (buttons[CHANGE_VIEW] && !control_spot)
    {
      control_spot = true;
    }
    else if (buttons[MENU] && control_spot)
    {
      control_spot = false;
    }

    if (msg.axes[LEFT_TRIGGER] < 0 && msg.axes[RIGHT_TRIGGER] < 0 && !cutoff_controller_flag)
    {
      cutoff_controller_flag = true;
      cutoff_controller = !cutoff_controller;
    }
    else if (msg.axes[LEFT_TRIGGER] == 1 && msg.axes[RIGHT_TRIGGER] == 1)
    {
      cutoff_controller_flag = false;
    }
    if (cutoff_controller) return;

    control_spot ? spot_control(msg, buttons) : kinova_control(msg, buttons);
  }

private:
  bool cutoff_controller = true;
  bool cutoff_controller_flag = false;
  bool control_spot = true;
  bool stance = false;
  bool is_sitting = true;
  bool d_pad_pressed = false;
  bool twist_controller_running = false;
  float height, roll, pitch, yaw;
  float spot_speed_factor = 0.5;
  Gait locomotion_mode = AUTO;

  std::set<int> pressed_buttons; // Store pressed buttons

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_spot_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_spot_;
  rclcpp::Publisher<KinovaTwistType>::SharedPtr twist_pub_kinova_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_vel_kinova_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr spot_stand, spot_sit;
  rclcpp::Client<spot_msgs::srv::SetLocomotion>::SharedPtr spot_gait;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr kinova_controller_switch_;

  std::shared_ptr<GripperGoalSender> gripper_goal_sender;
  std::shared_ptr<MoveGroupActionClient> move_group;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<JoyToServoPub>(options);
  node->create_action_clients();
  rclcpp::spin(node);
  return 0;
}
