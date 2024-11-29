#include "jetracer_ros2/jet_racer_api.hpp"

namespace jetracer_ros2
{

JetRacerDataPack& JetRacerDataPack::operator<<(char value)
{
    _data.push_back(value);
    return *this;
}

JetRacerDataPack& JetRacerDataPack::operator<<(int value)
{
    *this << (char)((value>>8) & 0xff);
    *this << (char)(value & 0xff);
    return *this;
}

JetRacerDataPack& JetRacerDataPack::operator<<(double value)
{
  *this << (char)((int16_t)((int16_t)(value*1000)>>8)&0xff);
  *this << (char)((int16_t)(value*1000)&0xff);
  return *this;
}

char JetRacerDataPack::_calculate_checksum() const
{
    char sum = 0x00;
    for(char value: _data)
    {
        sum += value;
    }
    return sum;
}

std::vector<char> JetRacerDataPack::get_datapack() const
{
    std::vector<char> data{std::begin(_data), std::end(_data)};
    data.push_back(_calculate_checksum());
    return data;
}

}