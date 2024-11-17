#include "jetracer_ros2/configuration.hpp"

namespace jetracer_ros2
{
class JetRacerApiNode: public rclcpp::Node
{
public:
    JetRacerApiNode();

};

JetRacerApiNode::JetRacerApiNode(): Node("jet_racer_api")
{
    SerialConfig conf;
    conf.declare_parameters(this);
    conf.update_parameters(this);
    conf.print_config(this);

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