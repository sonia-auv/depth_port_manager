#include <sstream>
#include "boost/log/trivial.hpp"
#include "depth_port_manager/DepthProvider.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace depth_provider
{
    DepthProvider::DepthProvider()
        : Node("depth_provider"), _serialConnection("/dev/DEPTH", B115200, true)
    {
        depthPublisher_ = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/depth", 100);
        pressPublisher_ = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/press", 100);
        tempPublisher_ = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/temp", 100);

        read_thread = std::thread(std::bind(&DepthProvider::readSerialDevice, this));
        send_thread = std::thread(std::bind(&DepthProvider::sendId1Register, this));

        tare_srv = this->create_service<std_srvs::srv::Empty>("/provider_depth/tare", std::bind(&DepthProvider::tare, this, _1, _2));
    }

    DepthProvider::~DepthProvider()
    {
        _send_stop_thread = true;
        _read_stop_thread = true;
    }

    bool DepthProvider::OpenPort()
    {
        bool res = _serialConnection.OpenPort();
        if (res)
        {
            _serialConnection.Flush();
        }
        return res;
    }

    void DepthProvider::readSerialDevice()
    {
        char buffer[BUFFER_SIZE];

        while (!_read_stop_thread)
        {
            do
            {   
                _serialConnection.ReadOnce((uint8_t*)buffer,0);
            } while (buffer[0] != '$');

            int i;

            for (i = 1; buffer[i - 1] != '\n' && i < BUFFER_SIZE; i++)
            {
                _serialConnection.ReadOnce((uint8_t *)buffer, i);
            }

            if (i >= BUFFER_SIZE)
            {
                continue;
            }

            buffer[i] = 0;

            if(!strncmp(&buffer[1], ID1, 5)) // Add checksum verification
            {
                std::unique_lock<std::mutex> mlock(id1_mutex);
                id1_string = std::string(buffer);
                id1_cond.notify_one();
            }

        } // end while
    }     // end read

    void DepthProvider::sendId1Register()
    {
        while (!_send_stop_thread)
        {
            std::string tmp = "";

            std::unique_lock<std::mutex> mlock(id1_mutex);
            id1_cond.wait(mlock);

            try
            {
                if (!id1_string.empty())
                {
                    std::stringstream ss(id1_string);

                    std::getline(ss, tmp, ','); // Get the header of the message

                    std::getline(ss, tmp, ','); // Get the depth
                    depth_.data = stof(tmp);
                    depthPublisher_->publish(depth_);

                    std::getline(ss, tmp, ','); // skip M

                    std::getline(ss, tmp, ','); // Get the pressure
                    press_.data = stof(tmp);
                    pressPublisher_->publish(press_);

                    std::getline(ss, tmp, ','); // skip B
                    
                    std::getline(ss, tmp, ','); // Get the temperature
                    temp_.data = stof(tmp);
                    tempPublisher_->publish(temp_);
                }
            }
            catch(...)
            {
                BOOST_LOG_TRIVIAL(info)<<"Depth sensor : Bad packet error";
            }
            
            
        }
    }

    bool DepthProvider::tare(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        _serialConnection.Transmit("#tare\n");
        std::this_thread::sleep_for(0.1s);

        BOOST_LOG_TRIVIAL(info)<<"Depth Sensor tare finished";
        return true;
    }
} // end namespace