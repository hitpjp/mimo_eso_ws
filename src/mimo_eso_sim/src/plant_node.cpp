#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>
#include <random>

class PlantNode : public rclcpp::Node
{
public:
    PlantNode() : Node("plant_node"), gen_(rd_()), dist_(0.0, 1.0)
    {
        // 物理参数
        mass_ = 1.5;
        inertia_ << 0.02, 0.02, 0.04;

        pos_.setZero();
        vel_.setZero();
        angle_.setZero();
        ang_vel_.setZero();
        f_wind_last_.setZero();
        tau_wind_last_.setZero();
        f_wind_dot_.setZero();
        tau_wind_dot_.setZero();

        // 发布者
        pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/position", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/velocity", 10);
        angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/angle", 10);
        ang_vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/angular_velocity", 10);
        true_f_dist_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/true_force_dist", 10);
        true_tau_dist_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/plant/true_torque_dist", 10);

        // 订阅控制量
        thrust_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/control/thrust", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
            { f_ctrl_ << msg->x, msg->y, msg->z; });
        torque_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/control/torque", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
            { tau_ctrl_ << msg->x, msg->y, msg->z; });

        // 设置频率为 1000Hz (1ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&PlantNode::simulation_step, this));
    }

private:
    void simulation_step()
    {
        double dt = 0.001;

        // 1. 生成二阶平滑随机干扰 (消除锯齿)
        for (int i = 0; i < 3; ++i)
        {
            // 驱动噪声强度
            double f_noise = 1.5 * dist_(gen_);
            double tau_noise = 0.8 * dist_(gen_);

            // 二阶滤波动态: x'' = -2*zeta*wn*x' - wn^2*x + noise
            f_wind_dot_[i] += (-4.0 * f_wind_dot_[i] - 4.0 * f_wind_last_[i] + f_noise) * dt;
            tau_wind_dot_[i] += (-6.0 * tau_wind_dot_[i] - 9.0 * tau_wind_last_[i] + tau_noise) * dt;

            f_wind_last_[i] += f_wind_dot_[i] * dt;
            tau_wind_last_[i] += tau_wind_dot_[i] * dt;
        }

        // 2. 平动动力学
        Eigen::Vector3d v_dot = (-0.1 * vel_ + f_ctrl_ + f_wind_last_) / mass_ + Eigen::Vector3d(0, 0, -9.81);
        vel_ += v_dot * dt;
        pos_ += vel_ * dt;

        // 3. 转动动力学
        Eigen::Vector3d w_dot;
        w_dot.x() = (tau_ctrl_.x() + tau_wind_last_.x()) / inertia_.x();
        w_dot.y() = (tau_ctrl_.y() + tau_wind_last_.y()) / inertia_.y();
        w_dot.z() = (tau_ctrl_.z() + tau_wind_last_.z()) / inertia_.z();
        ang_vel_ += w_dot * dt;
        angle_ += ang_vel_ * dt;

        // 4. 发布测量值
        publish_vec(pos_pub_, pos_);
        publish_vec(vel_pub_, vel_);
        publish_vec(angle_pub_, angle_);
        publish_vec(ang_vel_pub_, ang_vel_);
        publish_vec(true_f_dist_pub_, f_wind_last_ / mass_); // 发布加速度单位
        publish_vec(true_tau_dist_pub_, tau_wind_last_.array() / inertia_.array());
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
    Eigen::Vector3d inertia_, pos_, vel_, angle_, ang_vel_, f_ctrl_, tau_ctrl_;
    Eigen::Vector3d f_wind_last_, tau_wind_last_, f_wind_dot_, tau_wind_dot_;
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> dist_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pos_pub_, vel_pub_, angle_pub_, ang_vel_pub_, true_f_dist_pub_, true_tau_dist_pub_;
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