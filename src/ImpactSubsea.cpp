#include "depth_port_manager/ImpactSubsea.hpp"

#include <boost/log/trivial.hpp>
#include <cstring>
#include <sstream>
#include <thread>
using namespace std::chrono_literals;

namespace depth_port_manager
{
    const std::string ImpactSubsea::ID1 = "ISDPT";

    int ImpactSubsea::ReadDataCheck(std::function<ssize_t(uint8_t *, int)> readFunc, char buffer[], int bufferSize)
    {
        readFunc((uint8_t *)buffer, 0);
        if (buffer[0] != '$') {return 0;};
        int index;

        for (index = 1; buffer[index - 1] != '\n' && index < bufferSize; index++)
        {
            readFunc((uint8_t *)buffer, index);
        }

        if (index >= bufferSize)
        {
            return -1;
        }

        buffer[index] = 0;

        if (strncmp(&buffer[1], ImpactSubsea::ID1.c_str(), ImpactSubsea::ID_SIZE) == 0)  // Add checksum verification
        {
            return index;
        }
        return -2;
    }

    DepthData ImpactSubsea::ParseData(std::string data)
    {
        DepthData returnVal;
        std::string tmp = "";
        try
        {
            std::stringstream ss(data);

            std::getline(ss, tmp, ',');  // Get the header of the message
            std::getline(ss, tmp, ',');  // Get the depth
            returnVal.depth = stof(tmp);

            std::getline(ss, tmp, ',');  // skip M
            std::getline(ss, tmp, ',');  // Get the pressure
            returnVal.press = stof(tmp);

            std::getline(ss, tmp, ',');  // skip B
            std::getline(ss, tmp, ',');  // Get the temperature
            returnVal.temp = stof(tmp);
        }
        catch (...)
        {
            BOOST_LOG_TRIVIAL(info) << "Depth sensor : Bad packet error";
        }

        return returnVal;
    }

    void ImpactSubsea::Tare(std::function<ssize_t(std::string)> writeFunc)
    {
        writeFunc("#tare\n");
        std::this_thread::sleep_for(0.1s);
    }
}  // namespace depth_port_manager
