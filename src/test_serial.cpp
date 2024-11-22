#include <sstream>
#include "jetracer_ros2/configuration.hpp"
#include "serial/serial.h"

namespace jetracer_ros2
{

class SerialTestNode: public rclcpp::Node
{
public:
    SerialTestNode();

private:
    SerialConfig _config;
    std::unique_ptr<serial::Serial> _port;
};

SerialTestNode::SerialTestNode(): Node("serial_test")
{
    _config.declare(this);
    _config.update(this);
    _config.print(this);
    _port = std::make_unique<serial::Serial>(_config.port_name,
                                             _config.baud_rate,
                                             serial::Timeout::simpleTimeout(_config.timeout_ms)
                                             );
    if(!_port->isOpen())
    {
        std::stringstream error_description;
        error_description << "Unable to open port " << _config.port_name;
        throw std::runtime_error(error_description.str());
    }
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<jetracer_ros2::SerialTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}