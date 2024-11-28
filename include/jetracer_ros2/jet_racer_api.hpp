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
    JetRacerDataPack& operator<<(char value);
    JetRacerDataPack& operator<<(int value);
    JetRacerDataPack& operator<<(double value);
    template <typename data_type> JetRacerDataPack& operator<<(std::vector<data_type> value_vector);
    std::vector<char> get_datapack() const;

private:
    std::list<char> _data;
private:
    char _calculate_checksum() const;
};

/*
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
*/


template <typename data_type> JetRacerDataPack& JetRacerDataPack::operator<<(std::vector<data_type> value_vector)
{
    for(data_type value: value_vector)
    {
        *this << value;
    }
    return *this;
}

}

#endif // JET_RACER_API_HPP