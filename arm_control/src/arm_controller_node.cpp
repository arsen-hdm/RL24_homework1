#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class MyControlNode : public rclcpp::Node
{
public:
  MyControlNode()
  : Node("joint_state_controller")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 7,
     std::bind(&MyControlNode::joint_state_callback, this, std::placeholders::_1));
     
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controller/command", 10);
    
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
      std::bind(&MyControlNode::timer_callback, this));       
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
     RCLCPP_INFO(this->get_logger(), "Current joint positions:");
     for (size_t i = 0; i < msg->name.size(); ++i)
     {
        RCLCPP_INFO(this->get_logger(), "Joint: '%s', Position: %f", msg->name[i].c_str(), msg->position[i]);
     }
  }
  
  void timer_callback()
  {
     auto command_msg = std_msgs::msg::Float64MultiArray();
     command_msg.data = {0.5, 1.0, 1.5, 2.0};
     publisher_->publish(command_msg);
     RCLCPP_INFO(this->get_logger(), "Publishing command: [%f, %f, %f, %f]", 
       command_msg.data[0], command_msg.data[1], command_msg.data[2], command_msg.data[3]);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyControlNode>());
  rclcpp::shutdown();
  return 0;
}
