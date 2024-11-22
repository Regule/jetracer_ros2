#ifndef JETRACER_ROS2__CONFIGURATION_HPP_
#define JETRACER_ROS2__CONFIGURATION_HPP_

#include <string>
#include <vector>
#include <sstream>
#include "rclcpp/rclcpp.hpp"

namespace jetracer_ros2
{

class SerialConfig
{
public:
  static constexpr const char* PARAM_PORT_NAME = "port_name";
  static constexpr const char* PARAM_BAUD_RATE = "baud_rate";
  static constexpr const char* PARAM_TIMEOUT_MS = "timeout_ms";

  static constexpr const char* DEFAULT_PORT_NAME = "/dev/ttyACM0";
  static constexpr int DEFAULT_BAUD_RATE = 115200;
  static constexpr int DEFAULT_TIMEOUT_MS = 1000;

  static constexpr const char* DESCRIPTION_PORT_NAME =
  "path to port file, for exmample \"/dev/ttyACM0\"";
  static constexpr const char* DESCRIPTION_BAUD_RATE =
  "serial port baudrate";
  static constexpr const char* DESCRIPTION_TIMEOUT_MS =
  "timeout in milliseconds";

public:
  std::string port_name;
  int baud_rate;
  int timeout_ms;

public:
  void declare(rclcpp::Node *node);
  void update(rclcpp::Node *node);
  void print(rclcpp::Node *node);
};

class PidConfig
{
public:
  static constexpr const char* PARAM_PROPORTIONAL = "pid_proportional";
  static constexpr const char* PARAM_INTEGRAL = "pid_integral";
  static constexpr const char* PARAM_DERIVATIVE = "pid_derivative";

  static constexpr int DEFAULT_PROPORTIONAL = 1;
  static constexpr int DEFAULT_INTEGRAL = 1;
  static constexpr int DEFAULT_DERIVATIVE = 1;


  static constexpr const char* DESCRIPTION_PROPORTIONAL =
  "proportional factor of PID";
  static constexpr const char* DESCRIPTION_INTEGRAL =
  "integral factor of PID";
  static constexpr const char* DESCRIPTION_DERIVATIVE =
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

class CalibrationConfig
{
public:
  static constexpr const char* PARAM_LINEAR_CORRECTION = "linear_correction";
  static constexpr const char* PARAM_SERVO_BIAS = "servo_bias";
  static constexpr const char* PARAM_CALIBRATION_COEFFICENTS = "callibration_coefficents";
 
  static constexpr float DEFAULT_LINEAR_CORRECTION = 1.0;
  static constexpr int DEFAULT_SERVO_BIAS = 1;
  static constexpr double DEFAULT_CALIBRATION_COEFFICENTS[4] = {-0.016073, 0.176183, -23.428084, 1500};
 

  static constexpr const char* DESCRIPTION_LINEAR_CORRECTION =
  "linear correction";
  static constexpr const char* DESCRIPTION_SERVO_BIAS =
  "servo bias";
  static constexpr const char* DESCRIPTION_CALIBRATION_COEFFICENTS =
  "Coefficient of quartic equation for Steering calibration";
 
public:
  float linear_correction;
  int servo_bias;
  std::vector<double> coefficents;

public:
  void declare(rclcpp::Node *node);
  void update(rclcpp::Node *node);
  void print(rclcpp::Node *node);
};


class JetRacerConfig
{
public:
  static constexpr const char* PARAM_PUBLISH_ODOMETRY_TRANSFORM = "publish_odometry_transform";

  static constexpr bool DEFAULT_PUBLISH_ODOMETRY_TRANSFORM = true;

  static constexpr const char* DESCRIPTION_PUBLISH_ODOMETRY_TRANSFORM =
  "publish odometry transform";

public:
  SerialConfig serial;
  PidConfig pid;
  CalibrationConfig calibration;
  bool publish_odometry_transform;

public:
  void declare(rclcpp::Node *node);
  void update(rclcpp::Node *node);
  void print(rclcpp::Node *node);
};


}  // namespace jetracer_ros2

#endif  // JETRACER_ROS2__CONFIGURATION_HPP_
