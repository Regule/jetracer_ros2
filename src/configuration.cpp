#include "jetracer_ros2/configuration.hpp"

namespace jetracer_ros2
{

  void SerialConfig::declare_parameters(rclcpp::Node *node)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_PORT_NAME;
    node->declare_parameter(PARAM_PORT_NAME, DEFAULT_PORT_NAME, descriptor);
    descriptor.description = DESCRIPTION_BAUD_RATE;
    node->declare_parameter(PARAM_BAUD_RATE, DEFAULT_BAUD_RATE, descriptor);
  }

  void SerialConfig::update_parameters(rclcpp::Node *node)
  {
    this->port_name = node->get_parameter(PARAM_PORT_NAME).as_string();
    this->baud_rate = node->get_parameter(PARAM_BAUD_RATE).as_int();
  }

  void SerialConfig::print_config(rclcpp::Node *node)
  {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_PORT_NAME, this->port_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_BAUD_RATE, this->baud_rate);
  }

}  // namespace jetracer_ros2
