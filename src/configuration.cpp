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
    descriptor.description = DESCRIPTION_TIMEOUT_MS;
    node->declare_parameter(PARAM_TIMEOUT_MS, DEFAULT_TIMEOUT_MS, descriptor);
  }

  void SerialConfig::update(rclcpp::Node *node)
  {
    this->port_name = node->get_parameter(PARAM_PORT_NAME).as_string();
    this->baud_rate = node->get_parameter(PARAM_BAUD_RATE).as_int();
    this->timeout_ms = node->get_parameter(PARAM_TIMEOUT_MS).as_int();
  }

  void SerialConfig::print(rclcpp::Node *node)
  {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_PORT_NAME, this->port_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_BAUD_RATE, this->baud_rate);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_TIMEOUT_MS, this->timeout_ms);
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

  void CalibrationConfig::declare(rclcpp::Node *node)
  {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_LINEAR_CORRECTION;
    node->declare_parameter(PARAM_LINEAR_CORRECTION, DEFAULT_LINEAR_CORRECTION, descriptor);
    descriptor.description = DESCRIPTION_SERVO_BIAS;
    node->declare_parameter(PARAM_SERVO_BIAS, DEFAULT_SERVO_BIAS, descriptor);
    descriptor.description = DESCRIPTION_CALIBRATION_COEFFICENTS;
    // TODO: Addd limits so that the range of float (instead of double) will not be exceeded
    std::vector<double> default_calibration_coefficents(DEFAULT_CALIBRATION_COEFFICENTS, DEFAULT_CALIBRATION_COEFFICENTS+4);
    node->declare_parameter(PARAM_CALIBRATION_COEFFICENTS, default_calibration_coefficents, descriptor);
  }

  void CalibrationConfig::update(rclcpp::Node *node)
  {
    this->linear_correction = node->get_parameter(PARAM_LINEAR_CORRECTION).as_double();
    this->servo_bias = node->get_parameter(PARAM_SERVO_BIAS).as_int();
    this->coefficents = node->get_parameter(PARAM_CALIBRATION_COEFFICENTS).as_double_array();
  }

  void CalibrationConfig::print(rclcpp::Node *node)
  {
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_LINEAR_CORRECTION, this->linear_correction);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SERVO_BIAS, this->servo_bias);
    std::stringstream vector_description;
    vector_description << "[";
    for(size_t i=0; i<coefficents.size()-1; i++)
    {
      vector_description << coefficents[i] << " ";
    }
    vector_description << coefficents[coefficents.size()-1] << "]";
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_CALIBRATION_COEFFICENTS, vector_description.str().c_str());
  }
  

  void JetRacerConfig::declare(rclcpp::Node *node)
  {
    serial.declare(node);
    pid.declare(node);
    calibration.declare(node);

    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_PUBLISH_ODOMETRY_TRANSFORM;
    node->declare_parameter(PARAM_PUBLISH_ODOMETRY_TRANSFORM, DEFAULT_PUBLISH_ODOMETRY_TRANSFORM, descriptor);

  }

  void JetRacerConfig::update(rclcpp::Node *node)
  {
    serial.update(node);
    pid.update(node);
    calibration.update(node);
    this->publish_odometry_transform = node->get_parameter(PARAM_PUBLISH_ODOMETRY_TRANSFORM).as_bool();
  }

  void JetRacerConfig::print(rclcpp::Node *node)
  {
    serial.print(node);
    pid.print(node);
    calibration.print(node);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_PUBLISH_ODOMETRY_TRANSFORM, this->publish_odometry_transform?"true":"false");
  }



}  // namespace jetracer_ros2
