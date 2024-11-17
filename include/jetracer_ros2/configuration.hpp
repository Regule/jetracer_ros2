#ifndef JETRACER_ROS2__CONFIGURATION_HPP_
#define JETRACER_ROS2__CONFIGURATION_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace jetracer_ros2
{

class SerialConfig
{
public:
  const char* PARAM_PORT_NAME = "port_name";
  const char* PARAM_BAUD_RATE = "baud_rate";

  const std::string DEFAULT_PORT_NAME = std::string("/dev/ttyACM0");
  const int DEFAULT_BAUD_RATE = 115200;

  const char* DESCRIPTION_PORT_NAME =
  "path to port file, for exmample \"/dev/ttyACM0\"";

public:
  std::string port_name;
  int baud_rate;

public:
  void declare_parameters(rclcpp::Node *node);
  void update_parameters(rclcpp::Node *node);
  void print_config(rclcpp::Node *node);
};

};  // namespace jetracer_ros2

#endif  // JETRACER_ROS2__CONFIGURATION_HPP_
