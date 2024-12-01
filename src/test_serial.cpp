#include <sstream>
#include <ios>
#include "jetracer_ros2/configuration.hpp"
#include "serial/serial.h"
#include "jetracer_ros2/jet_racer_api.hpp"

using namespace jetracer_ros2;

//============= ORIGINAL CODE AS REFERENCE =================
uint8_t checksum(uint8_t* buf, size_t len)
{
  uint8_t sum = 0x00;
  for(size_t i=0;i<len;i++)
  {
    sum += *(buf + i);
  }
  return sum;
}

/*robot parameter sending function*/
std::vector<uint8_t> SetParams(int p,int i, int d, double linear_correction,int servo_bias) {
uint8_t tmp[15];
  tmp[0]  = 0xAA;
  tmp[1]  = 0x55;
  tmp[2]  = 0x0F;
  tmp[3]  = 0x12;
  tmp[4]  = (p>>8)&0xff;
  tmp[5]  = p&0xff;
  tmp[6]  = (i>>8)&0xff;
  tmp[7]  = i&0xff;
  tmp[8]  = (d>>8)&0xff;
  tmp[9]  = d&0xff;
  tmp[10] = (int16_t)((int16_t)(linear_correction*1000)>>8)&0xff;
  tmp[11] = (int16_t)(linear_correction*1000)&0xff;
  tmp[12] = ((int16_t)((int16_t)servo_bias)>>8)&0xff;
  tmp[13] = ((int16_t)servo_bias)&0xff;
  tmp[14] = checksum(tmp,14);
  return std::vector<uint8_t>(tmp, tmp+15);
}

//========================================================


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
    int p =1;
    int i=13; 
    int d=4;
    double linear_correction= 1.542;
    int servo_bias = 11;

    auto reference = SetParams(p,i,d,linear_correction,servo_bias);
    JetRacerDataPack data_pack_stream;
    data_pack_stream << (uint8_t)0xAA << (uint8_t)0x55 << (uint8_t)0x0F << (uint8_t)0x12 << p << i << d << linear_correction << servo_bias;
    auto data_pack = data_pack_stream.get_datapack();
    if(data_pack.size() != reference.size())
    {
        RCLCPP_FATAL(this->get_logger(), "Datapack size (%ld) is different from reference size (%ld)", data_pack.size(), reference.size());
        return;
    }

    bool error = false;
    for(size_t i=0; i<data_pack.size(); i++)
    {
        if(data_pack[i]!=reference[i])
        {
            error = true;
            RCLCPP_ERROR(this->get_logger(), "%x <--> %x  ERROR", data_pack[i], reference[i]);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "%x <--> %x  OK", data_pack[i], reference[i]);
        }
    }
    if(error)
    {
        RCLCPP_ERROR(this->get_logger(), "DISCREPANCY BETWEEN DATAPACK AND REFERENCE");
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




int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    try
    {
        auto node = std::make_shared<SerialTestNode>();
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("global"), "Node shutdown - %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}