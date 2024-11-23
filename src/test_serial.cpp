#include <sstream>
#include <ios>
#include "jetracer_ros2/configuration.hpp"
#include "serial/serial.h"

namespace jetracer_ros2
{

class SerialTestNode: public rclcpp::Node
{
public:
    SerialTestNode();
    ~SerialTestNode();

private:
    SerialConfig _config;
    std::unique_ptr<serial::Serial> _port;
};

SerialTestNode::SerialTestNode(): Node("serial_test")
{
    _config.declare(this);
    _config.update(this);
    _config.print(this);
    try
    {
        _port = std::make_unique<serial::Serial>(_config.port_name,
                                             _config.baud_rate,
                                             serial::Timeout::simpleTimeout(_config.timeout_ms)
                                             );
    }
    catch(const std::exception& e)
    {
        // Exception thrown by Serial class are very uninformative. I decided to repleace them
        // with exception that gives information on what port we tried to open.
        std::stringstream error_description;
        error_description << "Unable to open port " << _config.port_name;
        throw std::runtime_error(error_description.str());
    }
}

SerialTestNode::~SerialTestNode()
{
    try
    {
        _port->close();
    }
    catch(const std::exception& e)
    {
        RCLCPP_DEBUG(this->get_logger(), "Exception occured when closing serial port, this can happen - %s", e.what());
    }
    
}


}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    try
    {
        auto node = std::make_shared<jetracer_ros2::SerialTestNode>();
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("global"), "Node shutdown - %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}