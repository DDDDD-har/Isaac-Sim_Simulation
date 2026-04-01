#include "wheel_loader_controller/curve_path_lateral_pid_tracker.hpp"
#include <cmath>
#include <algorithm>
#include <iomanip>

namespace wheel_loader_controller {

CurvePathLateralPIDTracker::CurvePathLateralPIDTracker(const rclcpp::NodeOptions & options)
: Node("curve_path_lateral_pid_tracker", options) {
    
    // 初始化关节列表，必须严格对应 Isaac Sim rebuilt 图的顺序
    joint_names_ = {"FRwheel_revolute", "FLwheel_revolute", "boom_cylinder_revolute", 
                    "boom_pivot_revolute", "chassis_body_revolute", "boom_cylinder_prismatic", 
                    "rocker_bucket_revolute", "rocker_revolute", "RLwheel_revolute", 
                    "RRwheel_revolute", "rocker_link_revolute", "rocker_cylinder_revolute", 
                    "bucket_cylinder_prismatic"};

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "正在启动挖掘机横向 PID 控制系统，这次一定，我信！！！！！！！！...");

    declareAndLoadParameters();
    generateReferencePath();
    setupRosInterfaces();

    // 重要：初始化 CSV 文件并写入表头，确保列名与 Python 脚本一致
    log_file_.open("tracking_performance.csv", std::ios::out);
    if (log_file_.is_open()) {
        log_file_ << "time,x,y,ref_x,ref_y,err_y,phi_cmd\n";
        RCLCPP_INFO(this->get_logger(), "[文件] 已创建日志：tracking_performance.csv");
    }

    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), 
        std::bind(&CurvePathLateralPIDTracker::controlLoopCallback, this));

    RCLCPP_INFO(this->get_logger(), "系统就绪，开始闭环控制。");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void CurvePathLateralPIDTracker::controlLoopCallback() {
    if (!front_axle_received_ || !joint_received_) return;

    // 1. 路径匹配
    size_t idx = findNearestPathPoint(front_axle_x_, front_axle_y_);
    const auto & ref = reference_path_[idx];

    // 2. 自动停车判断
    double dist_to_goal = std::hypot(front_axle_x_ - reference_path_.back().x, front_axle_y_ - reference_path_.back().y);
    if (stop_at_end_ && dist_to_goal < stop_dist_) {
        publishVehicleCommand(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "[状态] 到达终点，触发自动停车。");
        if(log_file_.is_open()) log_file_.close();
        return;
    }

    // ========================================================
    // 核心算法：横向 PID 控制计算
    // ========================================================
    
    // 计算位置偏差 (世界坐标系)
    double dx = front_axle_x_ - ref.x;
    double dy = front_axle_y_ - ref.y;
    
    // 【核心】Frenet 转换：计算车辆相对于路径法线方向的横向偏差 e_y
    double e_y = -std::sin(ref.yaw) * dx + std::cos(ref.yaw) * dy;

    double dt = 0.05; // 采样周期

    // --- P 项：比例控制 ---
    // 提供即时的纠偏拉力，Kp 决定了车辆回中的积极程度
    double p_term = kp_ * e_y;

    // --- I 项：积分控制 ---
    // 累加历史误差，用于消除系统静差（如侧滑或执行器偏差）
    integral_error_ += e_y * dt;
    integral_error_ = std::clamp(integral_error_, -i_limit_, i_limit_); // 抗饱和
    double i_term = ki_ * integral_error_;

    // --- D 项：微分控制 ---
    // 预测偏差变化趋势，提供阻尼，对抗高速行驶下的惯性滞后
    double d_term = first_run_ ? 0.0 : kd_ * (e_y - previous_error_) / dt;
    previous_error_ = e_y;
    first_run_ = false;

    // 计算总期望铰接角 (负反馈原则)
    double phi_ref = -(p_term + i_term + d_term);
    phi_ref = std::clamp(phi_ref, min_steering_, max_steering_);

    // 【执行器保护】限制转向命令的变化率，防止转向机构动作过大导致仿真崩溃
    double delta_phi = std::clamp(phi_ref - last_phi_cmd_, -articulation_rate_limit_, articulation_rate_limit_);
    double phi_cmd = last_phi_cmd_ + delta_phi;
    last_phi_cmd_ = phi_cmd;

    // 3. 执行控制命令
    publishVehicleCommand(target_velocity_, phi_cmd);

    // 4. 实时调试信息打印 (每隔 1 秒打印一次)
    static int log_counter = 0;
    if (log_counter++ % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), "[运行中] 误差 e_y: %.3f m | 铰接角 cmd: %.3f rad", e_y, phi_cmd);
    }

    // 5. 写入 CSV 数据
    if (log_file_.is_open()) {
        log_file_ << std::fixed << std::setprecision(4) << this->now().seconds() << "," 
                  << front_axle_x_ << "," << front_axle_y_ << "," 
                  << ref.x << "," << ref.y << "," 
                  << e_y << "," << phi_cmd << "\n";
    }
}

void CurvePathLateralPIDTracker::declareAndLoadParameters() {
    this->declare_parameter("kp", 0.22);
    this->declare_parameter("ki", 0.005);
    this->declare_parameter("kd", 0.45);
    this->declare_parameter("target_velocity", 3.0);
    this->declare_parameter("path_amplitude", 2.8);
    this->declare_parameter("path_frequency", 0.07);
    this->declare_parameter("path_y_offset", 2.7581);
    this->declare_parameter("articulation_rate_limit", 0.06);

    this->get_parameter("kp", kp_);
    this->get_parameter("ki", ki_);
    this->get_parameter("kd", kd_);
    this->get_parameter("target_velocity", target_velocity_);
    this->get_parameter("path_amplitude", path_amplitude_);
    this->get_parameter("path_frequency", path_frequency_);
    this->get_parameter("path_y_offset", path_y_offset_);
    this->get_parameter("articulation_rate_limit", articulation_rate_limit_);

    RCLCPP_INFO(this->get_logger(), "[配置] 加载 PID: P=%.2f, I=%.3f, D=%.2f", kp_, ki_, kd_);
    RCLCPP_INFO(this->get_logger(), "[配置] 目标速度: %.2f | 转向限速: %.3f", target_velocity_, articulation_rate_limit_);
}

void CurvePathLateralPIDTracker::generateReferencePath() {
    reference_path_.clear();
    for (double x = path_x_start_; x <= path_x_end_; x += path_dx_) {
        PathPoint pt;
        pt.x = x;
        pt.y = path_y_offset_ + path_amplitude_ * std::sin(path_frequency_ * x);
        double dydx = path_amplitude_ * path_frequency_ * std::cos(path_frequency_ * x);
        pt.yaw = std::atan2(dydx, 1.0);
        reference_path_.push_back(pt);
    }
    RCLCPP_INFO(this->get_logger(), "[路径] 已生成参考轨迹，总点数: %zu", reference_path_.size());
}

// 其余回调与工具函数保持不变...
void CurvePathLateralPIDTracker::frontAxleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    front_axle_x_ = msg->pose.position.x; front_axle_y_ = msg->pose.position.y;
    front_axle_yaw_ = std::atan2(2.0*(msg->pose.orientation.w*msg->pose.orientation.z), 1.0-2.0*(msg->pose.orientation.z*msg->pose.orientation.z));
    front_axle_received_ = true;
}
void CurvePathLateralPIDTracker::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "chassis_body_revolute") { joint_received_ = true; break; }
    }
}
void CurvePathLateralPIDTracker::setupRosInterfaces() {
    front_axle_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/wheel_loader/front_axle_pose", 10, std::bind(&CurvePathLateralPIDTracker::frontAxleCallback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states_rebuilt", 10, std::bind(&CurvePathLateralPIDTracker::jointStateCallback, this, std::placeholders::_1));
    joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_command", 10);
}
size_t CurvePathLateralPIDTracker::findNearestPathPoint(double x, double y) {
    double min_d2 = 1e10; size_t nearest = last_nearest_idx_;
    size_t s = (last_nearest_idx_ > 50) ? last_nearest_idx_ - 50 : 0;
    size_t e = std::min(last_nearest_idx_ + 50, reference_path_.size() - 1);
    for (size_t i = s; i <= e; ++i) {
        double d2 = std::pow(x-reference_path_[i].x, 2) + std::pow(y-reference_path_[i].y, 2);
        if (d2 < min_d2) { min_d2 = d2; nearest = i; }
    }
    last_nearest_idx_ = nearest; return nearest;
}
void CurvePathLateralPIDTracker::publishVehicleCommand(double v, double phi) {
    sensor_msgs::msg::JointState cmd; cmd.header.stamp = this->now(); cmd.name = joint_names_;
    cmd.position.assign(joint_names_.size(), 0.0); cmd.velocity.assign(joint_names_.size(), 0.0);
    cmd.velocity[0]=v; cmd.velocity[1]=v; cmd.velocity[8]=v; cmd.velocity[9]=v; cmd.position[4]=phi;
    joint_cmd_pub_->publish(cmd);
}
CurvePathLateralPIDTracker::~CurvePathLateralPIDTracker() {}
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wheel_loader_controller::CurvePathLateralPIDTracker>());
    rclcpp::shutdown(); return 0;
}