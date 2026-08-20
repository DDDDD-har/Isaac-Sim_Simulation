#ifndef WHEEL_LOADER_CONTROLLER__SIMPLE_VEHICLE_CONTROLLER_HPP_
#define WHEEL_LOADER_CONTROLLER__SIMPLE_VEHICLE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <string>
#include <vector>

namespace wheel_loader_controller
{

class SimpleVehicleController : public rclcpp::Node
{
public:
  explicit SimpleVehicleController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareAndLoadParameters();
  void setupRosInterfaces();
  void timerCallback();

  void updateCommands(double now_sec);
  void publishWheelCommand();
  void publishSteeringCommand();

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> wheel_joint_names_;
  std::string steering_joint_name_;

  double update_rate_;

  double initial_velocity_;
  double initial_steering_;

  double current_velocity_;
  double current_steering_;

  double velocity_step_;
  double steering_step_;
  double update_interval_sec_;

  double max_velocity_;
  double min_velocity_;

  double max_steering_;
  double min_steering_;

  bool first_tick_;
  double last_update_time_sec_;
};

}  // namespace wheel_loader_controller

#endif  // WHEEL_LOADER_CONTROLLER__SIMPLE_VEHICLE_CONTROLLER_HPP_