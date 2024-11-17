#include "jetracer_ros2/configuration.hpp"

namespace jetracer_ros2
{

  void SerialConfig::declare(rclcpp::Node *node)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_PORT_NAME;
    node->declare_parameter(PARAM_PORT_NAME, DEFAULT_PORT_NAME, descriptor);
    descriptor.description = DESCRIPTION_BAUD_RATE;
    node->declare_parameter(PARAM_BAUD_RATE, DEFAULT_BAUD_RATE, descriptor);
  }

  void SerialConfig::update(rclcpp::Node *node)
  {
    this->port_name = node->get_parameter(PARAM_PORT_NAME).as_string();
    this->baud_rate = node->get_parameter(PARAM_BAUD_RATE).as_int();
  }

  void SerialConfig::print(rclcpp::Node *node)
  {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_PORT_NAME, this->port_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_BAUD_RATE, this->baud_rate);
  }

  void PidConfig::declare(rclcpp::Node *node)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_PROPORTIONAL;
    node->declare_parameter(PARAM_PROPORTIONAL, DEFAULT_PROPORTIONAL, descriptor);
    descriptor.description = DESCRIPTION_INTEGRAL;
    node->declare_parameter(PARAM_INTEGRAL, DEFAULT_INTEGRAL, descriptor);
    descriptor.description = DESCRIPTION_DERIVATIVE;
    node->declare_parameter(PARAM_DERIVATIVE, DEFAULT_DERIVATIVE, descriptor);
  }

  void PidConfig::update(rclcpp::Node *node)
  {
    this->proportional = node->get_parameter(PARAM_PROPORTIONAL).as_int();
    this->integral = node->get_parameter(PARAM_INTEGRAL).as_int();
    this->derivative = node->get_parameter(PARAM_DERIVATIVE).as_int();
  }

  void PidConfig::print(rclcpp::Node *node)
  {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_PROPORTIONAL, this->proportional);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_INTEGRAL, this->integral);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_DERIVATIVE, this->derivative);
  }

  void JetRacerConfig::declare(rclcpp::Node *node)
  {
    serial.declare(node);
    pid.declare(node);
  }

  void JetRacerConfig::update(rclcpp::Node *node)
  {
    serial.update(node);
    pid.update(node);
  }

  void JetRacerConfig::print(rclcpp::Node *node)
  {
    serial.print(node);
    pid.print(node);
  }



}  // namespace jetracer_ros2
