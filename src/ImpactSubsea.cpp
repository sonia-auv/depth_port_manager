#include "depth_port_manager/ImpactSubsea.hpp"

#include <boost/log/trivial.hpp>
#include <cstring>
#include <sstream>
#include <thread>

#include "sonia_common_cpp/SerialConn.hpp"
using namespace std::chrono_literals;

namespace depth_port_manager
{
    const std::string ImpactSubsea::ID1 = "ISDPT";

    ImpactSubsea::ImpactSubsea(std::shared_ptr<sonia_common_cpp::IConnection> connection)
        : IDepthDevice(connection)
    {}

    int ImpactSubsea::ReadDataCheck(char buffer[], int bufferSize)
    {
        sonia_common_cpp::SerialTram tram;
        tram.size = 1;
        _conn->Read(tram);
        if (buffer[0] != '$')
        {
            return 0;
        };
        int index;

        for (index = 1; buffer[index - 1] != '\n' && index < bufferSize; index++)
        {
            tram.offset = index;
            _conn->Read(tram);
        }

        if (index >= bufferSize)
        {
            return -1;
        }

        buffer = (char *)tram.data.data();
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

    void ImpactSubsea::Tare()
    {
        sonia_common_cpp::SerialTram tram;
        std::string msg = "#tare\n";
        tram.data.assign(msg.begin(), msg.end());
        _conn->Transmit(tram);
        std::this_thread::sleep_for(0.1s);
    }
}  // namespace depth_port_manager
