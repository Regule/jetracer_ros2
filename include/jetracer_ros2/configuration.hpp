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
  const char* DESCRIPTION_BAUD_RATE =
  "serial port baudrate";

public:
  std::string port_name;
  int baud_rate;

public:
  void declare(rclcpp::Node *node);
  void update(rclcpp::Node *node);
  void print(rclcpp::Node *node);
};

class PidConfig
{
public:
  const char* PARAM_PROPORTIONAL = "pid_proportional";
  const char* PARAM_INTEGRAL = "pid_integral";
  const char* PARAM_DERIVATIVE = "pid_derivative";

  int DEFAULT_PROPORTIONAL = 1;
  int DEFAULT_INTEGRAL = 1;
  int DEFAULT_DERIVATIVE = 1;


  const char* DESCRIPTION_PROPORTIONAL =
  "proportional factor of PID";
  const char* DESCRIPTION_INTEGRAL =
  "integral factor of PID";
  const char* DESCRIPTION_DERIVATIVE =
  "derivative factor of PID";

public:
  int proportional;
  int integral;
  int derivative;

public:
  void declare(rclcpp::Node *node);
  void update(rclcpp::Node *node);
  void print(rclcpp::Node *node);
};


class JetRacerConfig
{
public:
  SerialConfig serial;
  PidConfig pid;

public:
  void declare(rclcpp::Node *node);
  void update(rclcpp::Node *node);
  void print(rclcpp::Node *node);
};


}  // namespace jetracer_ros2

#endif  // JETRACER_ROS2__CONFIGURATION_HPP_
