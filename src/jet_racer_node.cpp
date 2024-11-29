#include "rclcpp/rclcpp.hpp"
#include "jetracer_ros2/configuration.hpp"

using namespace jetracer_ros2;

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

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<JetRacerApiNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}