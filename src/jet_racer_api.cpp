#include "jetracer_ros2/configuration.hpp"
#include "serial/serial.h"

namespace jetracer_ros2
{

class JetRacerApi
{
public:
    JetRacerApi(const std::string address, int baudrate);
    void write_params(int p,int i, int d, double linear_correction, int servo_bias);
    void write_coefficents(const std::vector<float> coefficents);
    void write_velocity(double x, double y, double yaw);

private:
    static constexpr const char _HEADER[] = {0xAA, 0x55}; 
    static constexpr const char _MSG_TYPE_PARAMS[] = {0x0F, 0x12};
    static constexpr const char _MSG_TYPE_COEFFICENTS[] = {0x15, 0x13};
    static constexpr const char _MSG_TYPE_VELOCITY[] = {0x0b, 0x11};

private:
    std::unique_ptr<serial::Serial> _port;

private:
    void write_to_data(std::vector<char> &data, size_t index, int value);
    void write_to_data(std::vector<char> &data, size_t index, double value);
};


class JetRacerApiNode: public rclcpp::Node
{
public:
    JetRacerApiNode();

};

JetRacerApiNode::JetRacerApiNode(): Node("jet_racer_api")
{
    JetRacerConfig conf;
    conf.declare(this);
    conf.update(this);
    conf.print(this);

}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<jetracer_ros2::JetRacerApiNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}