#ifndef WHEEL_LOADER_CONTROLLER__CURVE_PATH_LATERAL_PID_TRACKER_HPP_
#define WHEEL_LOADER_CONTROLLER__CURVE_PATH_LATERAL_PID_TRACKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>
#include <fstream>

namespace wheel_loader_controller {

struct PathPoint {
    double x, y, yaw;
};

class CurvePathLateralPIDTracker : public rclcpp::Node {
public:
    explicit CurvePathLateralPIDTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~CurvePathLateralPIDTracker();

private:
    void declareAndLoadParameters();
    void setupRosInterfaces();
    void generateReferencePath();
    void controlLoopCallback();
    
    // 订阅回调
    void frontAxleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // 工具函数
    size_t findNearestPathPoint(double x, double y);
    double normalizeAngle(double angle) const;
    double quaternionToYaw(double x, double y, double z, double w) const;
    void publishVehicleCommand(double wheel_velocity, double articulation_angle);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr front_axle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool front_axle_received_{false}, joint_received_{false};
    double front_axle_x_{0.0}, front_axle_y_{0.0}, front_axle_yaw_{0.0};
    double current_articulation_angle_{0.0};

    // PID 核心变量
    double kp_{0.22}, ki_{0.005}, kd_{0.45}; 
    double integral_error_{0.0}, previous_error_{0.0}, i_limit_{0.2};
    bool first_run_{true};

    // 运动约束
    double target_velocity_{3.0};
    double max_steering_{0.4}, min_steering_{-0.4};
    double articulation_rate_limit_{0.06}; // 转向机构每帧允许的最大位移
    double last_phi_cmd_{0.0};

    // 路径参数
    double path_x_start_{0.0}, path_x_end_{150.0}, path_dx_{0.2};
    double path_amplitude_{2.8}, path_frequency_{0.07}, path_y_offset_{2.7581};
    bool stop_at_end_{true};
    double stop_dist_{1.5};

    std::vector<std::string> joint_names_;
    std::vector<PathPoint> reference_path_;
    size_t last_nearest_idx_{0};
    std::ofstream log_file_;
};
} 
#endif