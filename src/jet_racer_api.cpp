#include "jetracer_ros2/jet_racer_api.hpp"

namespace jetracer_ros2
{

JetRacerDataPack& JetRacerDataPack::operator<<(uint8_t value)
{
    _data.push_back(value);
    return *this;
}

JetRacerDataPack& JetRacerDataPack::operator<<(int value)
{
    *this << (uint8_t)((value>>8) & 0xff);
    *this << (uint8_t)(value & 0xff);
    return *this;
}

JetRacerDataPack& JetRacerDataPack::operator<<(double value)
{
  *this << (uint8_t)((int16_t)((int16_t)(value*1000)>>8)&0xff);
  *this << (uint8_t)((int16_t)(value*1000)&0xff);
  return *this;
}

char JetRacerDataPack::_calculate_checksum() const
{
    uint8_t sum = 0x00;
    for(char value: _data)
    {
        sum += value;
    }
    return sum;
}

std::vector<uint8_t> JetRacerDataPack::get_datapack() const
{
    std::vector<uint8_t> data{std::begin(_data), std::end(_data)};
    data.push_back(_calculate_checksum());
    return data;
}


JetRacerApi::JetRacerApi(const std::string address, int baudrate, int timeout_ms)
{
    _port = std::make_unique<serial::Serial>(address,
                                             baudrate,
                                             serial::Timeout::simpleTimeout(timeout_ms)
                                             );
}

void JetRacerApi::write_params(int p,int i, int d, double linear_correction, int servo_bias)
{
    JetRacerDataPack data_pack_stream;
    data_pack_stream << MSG_HEADER << MSG_TYPE_PARAMS << p << i << d << linear_correction << servo_bias;
    auto data_pack = data_pack_stream.get_datapack();
    _port->write(data_pack);
}

void JetRacerApi::write_coefficents(const std::vector<float> coefficents)
{
    JetRacerDataPack data_pack_stream;
    data_pack_stream << MSG_HEADER << MSG_TYPE_COEFFICENTS << coefficents;
    auto data_pack = data_pack_stream.get_datapack();
    _port->write(data_pack);
}

void JetRacerApi::write_velocity(double x, double y, double yaw)
{
    JetRacerDataPack data_pack_stream;
    data_pack_stream << MSG_HEADER << MSG_TYPE_VELOCITY << x << y << yaw;
    auto data_pack = data_pack_stream.get_datapack();
    _port->write(data_pack);
}


}