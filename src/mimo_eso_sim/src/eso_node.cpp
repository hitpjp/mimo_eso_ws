#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>

class VectorNESO
{
public:
    // 修改后的构造函数：直接接收 beta1, beta2, beta3
    // 参数说明：b1/b2/b3增益，ts步长，a1/a2非线性幂次，d线性区间，lpf滤波因子
    VectorNESO(double b1, double b2, double b3, double ts, double a1, double a2, double d, double lpf)
        : beta1_(b1), beta2_(b2), beta3_(b3), ts_(ts), alpha1_(a1), alpha2_(a2), delta_(d), lpf_factor_(lpf)
    {
        z1_.setZero();
        z2_.setZero();
        z3_.setZero();
        z3_lpf_.setZero();
    }

    void update(const Eigen::Vector3d &y_meas, const Eigen::Vector3d &bu)
    {
        Eigen::Vector3d e = z1_ - y_meas;

        auto fal = [](double err, double alpha, double delta)
        {
            if (std::abs(err) > delta)
                return std::pow(std::abs(err), alpha) * (err > 0 ? 1.0 : -1.0);
            return err / std::pow(delta, 1.0 - alpha);
        };

        Eigen::Vector3d fal_e1, fal_e2;
        for (int i = 0; i < 3; ++i)
        {
            fal_e1[i] = fal(e[i], alpha1_, delta_);
            fal_e2[i] = fal(e[i], alpha2_, delta_);
        }

        // 三阶状态更新方程
        z1_ += (z2_ - beta1_ * e) * ts_;
        z2_ += (z3_ - beta2_ * fal_e1 + bu) * ts_;
        z3_ += (-beta3_ * fal_e2) * ts_;

        // 限幅保护：防止数值爆炸（针对单位加速度）
        for (int i = 0; i < 3; ++i)
            z3_[i] = std::max(-50.0, std::min(50.0, z3_[i]));

        // 输出低通滤波：平滑估计出的干扰值
        z3_lpf_ = (1.0 - lpf_factor_) * z3_lpf_ + lpf_factor_ * z3_;
    }

    Eigen::Vector3d get_dist() const { return z3_lpf_; }

private:
    double beta1_, beta2_, beta3_, ts_, alpha1_, alpha2_, delta_, lpf_factor_;
    Eigen::Vector3d z1_, z2_, z3_, z3_lpf_;
};

class ESONode : public rclcpp::Node
{
public:
    ESONode() : Node("eso_node")
    {
        // --- 手动设置 beta1=100, beta2=300, beta3=1000 ---

        // 平动 ESO
        // 参数：b1, b2, b3, ts, a1, a2, delta, lpf
        trans_eso_ = std::make_unique<VectorNESO>(100.0, 300.0, 1000.0, 0.001, 0.8, 0.8, 0.15, 0.1);

        // 转动 ESO
        rot_eso_ = std::make_unique<VectorNESO>(100.0, 300.0, 1000.0, 0.001, 0.8, 0.8, 0.2, 0.08);

        // 订阅器
        p_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/plant/position", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { p_meas_ << msg->x, msg->y, msg->z; });
        v_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/plant/velocity", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { v_meas_ << msg->x, msg->y, msg->z; });
        a_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/plant/angle", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { a_meas_ << msg->x, msg->y, msg->z; });
        f_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/control/thrust", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { f_ctrl_ << msg->x, msg->y, msg->z; });
        tau_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/control/torque", 10,
                                                                          [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                          { tau_ctrl_ << msg->x, msg->y, msg->z; });

        // 发布器
        df_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/eso/force_dist", 10);
        dtau_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/eso/torque_dist", 10);

        // 定时器周期 1ms (1000Hz)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ESONode::on_timer, this));
    }

private:
    void on_timer()
    {
        // 平动 bu 计算 (质量 1.5kg, 阻尼系数 0.1)
        Eigen::Vector3d bu_trans = (f_ctrl_ / 1.5) + Eigen::Vector3d(0, 0, -9.81) - (0.1 * v_meas_ / 1.5);
        trans_eso_->update(p_meas_, bu_trans);

        // 转动 bu 计算 (惯量 Jx=0.02, Jy=0.02, Jz=0.04)
        Eigen::Vector3d bu_rot;
        bu_rot.x() = tau_ctrl_.x() / 0.02;
        bu_rot.y() = tau_ctrl_.y() / 0.02;
        bu_rot.z() = tau_ctrl_.z() / 0.04;
        rot_eso_->update(a_meas_, bu_rot);

        // 发布估计结果
        auto msg_f = geometry_msgs::msg::Vector3();
        msg_f.x = trans_eso_->get_dist().x();
        msg_f.y = trans_eso_->get_dist().y();
        msg_f.z = trans_eso_->get_dist().z();
        df_pub_->publish(msg_f);

        auto msg_tau = geometry_msgs::msg::Vector3();
        msg_tau.x = rot_eso_->get_dist().x();
        msg_tau.y = rot_eso_->get_dist().y();
        msg_tau.z = rot_eso_->get_dist().z();
        dtau_pub_->publish(msg_tau);
    }

    std::unique_ptr<VectorNESO> trans_eso_, rot_eso_;
    Eigen::Vector3d p_meas_ = Eigen::Vector3d::Zero(), v_meas_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_meas_ = Eigen::Vector3d::Zero(), f_ctrl_ = Eigen::Vector3d::Zero(), tau_ctrl_ = Eigen::Vector3d::Zero();
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr p_sub_, v_sub_, a_sub_, f_sub_, tau_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr df_pub_, dtau_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ESONode>());
    rclcpp::shutdown();
    return 0;
}