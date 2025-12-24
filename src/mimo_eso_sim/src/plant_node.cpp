#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>
#include <random>

class PlantNode : public rclcpp::Node
{
public:
    PlantNode() : Node("plant_node"), gen_(rd_()), dist_(0.0, 1.0)
    {
        // 初始化物理参数
        mass_ = 1.5;
        inertia_ << 0.02, 0.02, 0.04; // 绕轴转动惯量 Jx, Jy, Jz

        vel_.setZero();
        ang_vel_.setZero();
        f_wind_last_.setZero();
        tau_wind_last_.setZero();

        // 发布者：速度、角速度、真实扰动真相
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/velocity", 10);
        ang_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/angular_velocity", 10);
        true_f_dist_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/true_force_dist", 10);
        true_tau_dist_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/true_torque_dist", 10);

        // 订阅者：控制力矩和推力
        thrust_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/control/thrust", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
            { f_ctrl_ << msg->x, msg->y, msg->z; });
        torque_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/control/torque", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
            { tau_ctrl_ << msg->x, msg->y, msg->z; });

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&PlantNode::simulation_step, this));
    }

private:
    void simulation_step()
    {
        double dt = 0.01;

        // 1. 生成随机风力和风力矩 (随机行走模型)
        Eigen::Vector3d f_wind, tau_wind;
        for (int i = 0; i < 3; ++i)
        {
            f_wind[i] = (1.0 - dt / 1.0) * f_wind_last_[i] + 2.0 * std::sqrt(2.0 * dt / 1.0) * dist_(gen_);
            tau_wind[i] = (1.0 - dt / 0.5) * tau_wind_last_[i] + 1.0 * std::sqrt(2.0 * dt / 0.5) * dist_(gen_);
        }
        f_wind_last_ = f_wind;
        tau_wind_last_ = tau_wind;

        // 2. 平动动力学 (F = m*a)
        // 加速度 = (空气阻力 + 控制推力 + 外部风力 + 重力) / 质量
        Eigen::Vector3d v_dot = (-0.1 * vel_ + f_ctrl_ + f_wind) / mass_ + Eigen::Vector3d(0, 0, -9.81);
        vel_ += v_dot * dt;

        // 3. 转动动力学 (Tau = J * alpha)
        // 角加速度 = (控制力矩 + 外部风力矩) / 转动惯量
        Eigen::Vector3d w_dot;
        w_dot.x() = (tau_ctrl_.x() + tau_wind.x()) / inertia_.x();
        w_dot.y() = (tau_ctrl_.y() + tau_wind.y()) / inertia_.y();
        w_dot.z() = (tau_ctrl_.z() + tau_wind.z()) / inertia_.z();
        ang_vel_ += w_dot * dt;

        // 4. 发布测量值
        publish_vec(vel_pub_, vel_);
        publish_vec(ang_vel_pub_, ang_vel_);
        publish_vec(true_f_dist_pub_, f_wind / mass_);                        // 发布单位质量的干扰加速度
        publish_vec(true_tau_dist_pub_, tau_wind.array() / inertia_.array()); // 发布单位惯量的角加速度干扰
    }

    void publish_vec(rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub, const Eigen::Vector3d &v)
    {
        auto m = geometry_msgs::msg::Vector3();
        m.x = v.x();
        m.y = v.y();
        m.z = v.z();
        pub->publish(m);
    }

    double mass_;
    Eigen::Vector3d inertia_;
    Eigen::Vector3d vel_, ang_vel_, f_ctrl_, tau_ctrl_, f_wind_last_, tau_wind_last_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> dist_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_, ang_vel_pub_, true_f_dist_pub_, true_tau_dist_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr thrust_sub_, torque_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlantNode>());
    rclcpp::shutdown();
    return 0;
}