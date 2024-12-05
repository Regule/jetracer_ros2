#ifndef JET_RACER_API_HPP
#define JET_RACER_API_HPP
#include <list>
#include <memory>
#include "serial/serial.h"

namespace jetracer_ros2
{

class JetRacerDataPack
{
public:
    JetRacerDataPack& operator<<(uint8_t value);
    JetRacerDataPack& operator<<(int value);
    JetRacerDataPack& operator<<(double value);
    template <typename data_type> JetRacerDataPack& operator<<(const std::vector<data_type> &value_vector);
    std::vector<uint8_t> get_datapack() const;

private:
    std::list<uint8_t> _data;
private:
    char _calculate_checksum() const;
};


const std::vector<uint8_t> MSG_HEADER = {(uint8_t)0xAA, (uint8_t)0x55}; 
const std::vector<uint8_t> MSG_TYPE_PARAMS = {(uint8_t)0x0F, (uint8_t)0x12};
const std::vector<uint8_t> MSG_TYPE_COEFFICENTS = {(uint8_t)0x15, (uint8_t)0x13};
const std::vector<uint8_t> MSG_TYPE_VELOCITY = {(uint8_t)0x0b, (uint8_t)0x11};


class JetRacerApi
{
public:
    JetRacerApi(const std::string address, int baudrate, int timeout_ms);
    void write_params(int p,int i, int d, double linear_correction, int servo_bias);
    void write_coefficents(const std::vector<float> coefficents);
    void write_velocity(double x, double y, double yaw);

private:
    std::unique_ptr<serial::Serial> _port;

};



template <typename data_type> JetRacerDataPack& JetRacerDataPack::operator<<(const std::vector<data_type> &value_vector)
{
    for(data_type value: value_vector)
    {
        *this << value;
    }
    return *this;
}

}

#endif // JET_RACER_API_HPP