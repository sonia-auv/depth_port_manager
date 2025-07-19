// #include <sstream>
#include "depth_port_manager/DepthProvider.hpp"

#include <boost/log/trivial.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace depth_port_manager
{
    DepthProvider::DepthProvider() : Node("depth_provider"), _serialConnection("/dev/DEPTH", B115200, true)
    {
        _depthPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/depth", 100);
        _pressPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/press", 100);
        _tempPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/temp", 100);

        _readThread = std::thread(std::bind(&DepthProvider::readSerialDevice, this));
        _sendThread = std::thread(std::bind(&DepthProvider::sendId1Register, this));

        _tareSrv = this->create_service<std_srvs::srv::Trigger>("/provider_depth/tare",
                                                                std::bind(&DepthProvider::tare, this, _1, _2));
    }

    DepthProvider::~DepthProvider()
    {
        _sendStopThread = true;
        _readStopThread = true;
    }

    bool DepthProvider::OpenPort()
    {
        bool res = this->_serialConnection.OpenPort();
        if (res)
        {
            _serialConnection.Flush();
        }
        return res;
    }

    void DepthProvider::readSerialDevice()
    {
        char buffer[BUFFER_SIZE];
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        while (!_readStopThread)
        {
            do
            {
                _serialConnection.ReadOnce((uint8_t *)buffer, 0);
            } while (buffer[0] != '$');

            int index;

            for (index = 1; buffer[index - 1] != '\n' && index < BUFFER_SIZE; index++)
            {
                _serialConnection.ReadOnce((uint8_t *)buffer, index);
            }

            if (index >= BUFFER_SIZE)
            {
                continue;
            }

            buffer[index] = 0;

            if (strncmp(&buffer[1], ID1, ID_SIZE) == 0)  // Add checksum verification
            {
                _id1String.push_back((std::string)buffer);
            }

        }  // end while
    }      // end read

    void DepthProvider::sendId1Register()
    {
        while (!_sendStopThread)
        {
            std::string tmp = "";

            while (!_id1String.empty())
            {
                try
                {
                    std::stringstream ss(_id1String.get_n_pop_front());

                    std::getline(ss, tmp, ',');  // Get the header of the message

                    std::getline(ss, tmp, ',');  // Get the depth
                    _depth.data = stof(tmp);
                    _depthPublisher->publish(_depth);

                    std::getline(ss, tmp, ',');  // skip M

                    std::getline(ss, tmp, ',');  // Get the pressure
                    _press.data = stof(tmp);
                    _pressPublisher->publish(_press);

                    std::getline(ss, tmp, ',');  // skip B

                    std::getline(ss, tmp, ',');  // Get the temperature
                    _temp.data = stof(tmp);
                    _tempPublisher->publish(_temp);
                }
                catch (...)
                {
                    BOOST_LOG_TRIVIAL(info) << "Depth sensor : Bad packet error";
                }
            }
        }
    }

    void DepthProvider::tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        _serialConnection.Transmit("#tare\n");
        std::this_thread::sleep_for(0.1s);
        response->success = true;
        response->message = "Depth Sensor tared";
        BOOST_LOG_TRIVIAL(info) << "Depth Sensor tare finished";
    }
}  // namespace depth_port_manager
