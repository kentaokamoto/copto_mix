#include <cmath>
#include <chrono>
#include <copto_mix/mix_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <Eigen/Dense>
#include "std_msgs/msg/float32_multi_array.hpp"


using namespace  std::chrono_literals;
namespace copto_mix
{
MIXComponent::MIXComponent(const rclcpp::NodeOptions & options) : Node("copto_mix_node", options)
{
  M = Eigen::MatrixXd::Zero(4, 4); // mixing
  A = Eigen::MatrixXd::Zero(4, 4); // control u -> torque -> pwm
  
  T = Eigen::VectorXd::Zero(4); // pwm
  u = Eigen::VectorXd::Zero(4); // pid_control

  CTLsubscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/copto/ctl_val", 5, std::bind(&MIXComponent::CTLtopic_callback, this, std::placeholders::_1));

  PWMpublisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/copto/pwm", 5);

  timer_ = this->create_wall_timer(1ms, std::bind(&MIXComponent::update, this));
}

void MIXComponent::init_M()
{
  M << -l, l, l, -l,
        -l, -l, l, l,
        -c, c, -c, c,
        1, 1, 1, 1;

  A << a_r, 0, 0, 0,
        0, a_p, 0, 0,
        0, 0, a_y, 0,
        0, 0, 0, a_t;
}

// r p y t
void MIXComponent::CTLtopic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  u << msg->data[0], msg->data[1], msg->data[2], msg->data[3];
}

void MIXComponent::update()
{
  if(!is_init)
  {
    init_M();
    is_init = true;
  }

  T = M.inverse()*u;

  // r p y t
  std_msgs::msg::Float32MultiArray pwm;
  pwm.data[0] = T(0);
  pwm.data[1] = T(1);
  pwm.data[2] = T(2);
  pwm.data[3] = T(3);

  PWMpublisher_->publish(pwm);
}

}  // namespace copto_mix

RCLCPP_COMPONENTS_REGISTER_NODE(copto_mix::MIXComponent)