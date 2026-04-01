#include "wheel_loader_controller/simple_vehicle_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

class SimpleVehicleControllerFullState : public rclcpp::Node
{
public:
  SimpleVehicleControllerFullState()
  : Node("simple_vehicle_controller_fullstate"),
    update_rate_(20.0),
    current_velocity_(2.0),
    current_steering_(0.0),
    initial_velocity_(2.0),
    initial_steering_(0.0),
    velocity_step_(1.0),
    steering_step_(0.1),
    update_interval_sec_(3.0),
    max_velocity_(8.0),
    min_velocity_(0.0),
    max_steering_(0.5),
    min_steering_(-0.5),
    steering_direction_(1.0),
    first_tick_(true),
    last_update_time_sec_(0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Full-State Vehicle Controller (Velocity Limited)");

    // 13个关节都用上，防止出现死锁
    joint_names_ = {
      "FRwheel_revolute",         // 0
      "FLwheel_revolute",         // 1
      "boom_cylinder_revolute",   // 2
      "boom_pivot_revolute",      // 3
      "chassis_body_revolute",    // 4
      "boom_cylinder_prismatic",  // 5
      "rocker_bucket_revolute",   // 6
      "rocker_revolute",          // 7
      "RLwheel_revolute",         // 8
      "RRwheel_revolute",         // 9
      "rocker_link_revolute",     // 10
      "rocker_cylinder_revolute", // 11
      "bucket_cylinder_prismatic" // 12
    };

    // Parameters
    declare_parameter("update_rate", update_rate_);
    declare_parameter("initial_velocity", initial_velocity_);
    declare_parameter("initial_steering", initial_steering_);
    declare_parameter("velocity_step", velocity_step_);
    declare_parameter("steering_step", steering_step_);
    declare_parameter("update_interval_sec", update_interval_sec_);
    declare_parameter("max_velocity", max_velocity_);
    declare_parameter("min_velocity", min_velocity_);
    declare_parameter("max_steering", max_steering_);
    declare_parameter("min_steering", min_steering_);

    get_parameter("update_rate", update_rate_);
    get_parameter("initial_velocity", initial_velocity_);
    get_parameter("initial_steering", initial_steering_);
    get_parameter("velocity_step", velocity_step_);
    get_parameter("steering_step", steering_step_);
    get_parameter("update_interval_sec", update_interval_sec_);
    get_parameter("max_velocity", max_velocity_);
    get_parameter("min_velocity", min_velocity_);
    get_parameter("max_steering", max_steering_);
    get_parameter("min_steering", min_steering_);

    current_velocity_ = initial_velocity_;
    current_steering_ = initial_steering_;

    joint_cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);

    auto period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimpleVehicleControllerFullState::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Controller started");
    RCLCPP_INFO(this->get_logger(), "Initial velocity: %.2f rad/s", current_velocity_);
    RCLCPP_INFO(this->get_logger(), "Initial steering: %.2f rad", current_steering_);
    RCLCPP_INFO(this->get_logger(), "Update interval: %.2f sec", update_interval_sec_);
    RCLCPP_INFO(this->get_logger(), "Velocity range: [%.2f, %.2f]", min_velocity_, max_velocity_);
    RCLCPP_INFO(this->get_logger(), "Steering range: [%.2f, %.2f]", min_steering_, max_steering_);
    RCLCPP_INFO(this->get_logger(), "Steering mode: ping-pong sweep");
  }

private:
  void updateCommands(double now_sec)
  {
    if (first_tick_) {
      last_update_time_sec_ = now_sec;
      first_tick_ = false;
      return;
    }

    const double elapsed = now_sec - last_update_time_sec_;
    if (elapsed >= update_interval_sec_) {
      // 上限为8rad/s
      current_velocity_ += velocity_step_;
      current_velocity_ = std::clamp(current_velocity_, min_velocity_, max_velocity_);

      // 往复式
      current_steering_ += steering_direction_ * steering_step_;

      if (current_steering_ >= max_steering_) {
        current_steering_ = max_steering_;
        steering_direction_ = -1.0;
      } else if (current_steering_ <= min_steering_) {
        current_steering_ = min_steering_;
        steering_direction_ = 1.0;
      }

      last_update_time_sec_ = now_sec;

      RCLCPP_INFO(
        this->get_logger(),
        "[Update] velocity = %.2f rad/s | steering = %.2f rad (%.2f deg) | steering_dir = %.0f",
        current_velocity_,
        current_steering_,
        current_steering_ * 180.0 / M_PI,
        steering_direction_);
    }
  }

  void publishCommand()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.resize(joint_names_.size(), 0.0);
    msg.velocity.resize(joint_names_.size(), 0.0);
    msg.effort.clear();


    msg.velocity[0] = current_velocity_;  // FR
    msg.velocity[1] = current_velocity_;  // FL
    msg.velocity[8] = current_velocity_;  // RL
    msg.velocity[9] = current_velocity_;  // RR

    msg.position[4] = current_steering_;  

    joint_cmd_pub_->publish(msg);
  }

  void timerCallback()
  {
    // 时钟选择
    const double now_sec = now().seconds();

    // 判断/clock是否开启
    if (now_sec <= 0.0) {
      static bool warned = false;
      if (!warned) {
        RCLCPP_WARN(this->get_logger(), "ROS/sim time has not started yet. Waiting for /clock ...");
        warned = true;
      }
      return;
    }

    updateCommands(now_sec);
    publishCommand();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;

  double update_rate_;

  double current_velocity_;
  double current_steering_;

  double initial_velocity_;
  double initial_steering_;

  double velocity_step_;
  double steering_step_;
  double update_interval_sec_;

  double max_velocity_;
  double min_velocity_;
  double max_steering_;
  double min_steering_;

  double steering_direction_;

  bool first_tick_;
  double last_update_time_sec_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleVehicleControllerFullState>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}