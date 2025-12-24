#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <Eigen/Dense>

class VectorNESO
{
public:
    VectorNESO(double wo, double ts) : ts_(ts)
    {
        beta1_ = 2.0 * wo;
        beta2_ = wo * wo;
        z1_.setZero();
        z2_.setZero();
    }

    void update(const Eigen::Vector3d &y_meas, const Eigen::Vector3d &bu)
    {
        Eigen::Vector3d e = z1_ - y_meas;
        Eigen::Vector3d fal_e;
        for (int i = 0; i < 3; ++i)
        {
            double delta = 0.005; // 减小线性区间提高微观精度
            fal_e[i] = (std::abs(e[i]) > delta) ? std::pow(std::abs(e[i]), 0.5) * (e[i] > 0 ? 1 : -1) : e[i] / std::pow(delta, 0.5);
        }
        z1_ += (z2_ - beta1_ * e + bu) * ts_;
        z2_ += (-beta2_ * fal_e) * ts_;
    }
    Eigen::Vector3d get_dist() const { return z2_; }

private:
    double beta1_, beta2_, ts_;
    Eigen::Vector3d z1_, z2_;
};

class ESONode : public rclcpp::Node
{
public:
    ESONode() : Node("eso_node")
    {
        trans_eso_ = std::make_unique<VectorNESO>(25.0, 0.01); // 提高带宽
        rot_eso_ = std::make_unique<VectorNESO>(40.0, 0.01);   // 姿态带宽要求更高

        v_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/plant/velocity", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { v_meas_ << msg->x, msg->y, msg->z; });
        w_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/plant/angular_velocity", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { w_meas_ << msg->x, msg->y, msg->z; });
        f_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/control/thrust", 10,
                                                                        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
                                                                        { f_ctrl_ << msg->x, msg->y, msg->z; });

        df_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/eso/force_dist", 10);
        dtau_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/eso/torque_dist", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ESONode::on_timer, this));
    }

private:
    void on_timer()
    {
        // 1. 平动 ESO 更新 (含重力、推力、阻尼补偿)
        Eigen::Vector3d bu_trans = (f_ctrl_ / 1.5) + Eigen::Vector3d(0, 0, -9.81) - 0.1 * v_meas_ / 1.5;
        trans_eso_->update(v_meas_, bu_trans);

        // 2. 转动 ESO 更新 (目前暂无控制力矩 tau_ctrl，故为 Zero)
        rot_eso_->update(w_meas_, Eigen::Vector3d::Zero());

        // 3. 发布
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
    Eigen::Vector3d v_meas_ = Eigen::Vector3d::Zero(), w_meas_ = Eigen::Vector3d::Zero(), f_ctrl_ = Eigen::Vector3d::Zero();
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr v_sub_, w_sub_, f_sub_;
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