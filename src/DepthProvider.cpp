// #include <sstream>
#include "depth_port_manager/DepthProvider.hpp"

#include <boost/log/trivial.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace depth_port_manager
{
    DepthProvider::DepthProvider(std::shared_ptr<IDepthDevice> device) : Node("depth_provider"), _device(device)
    {
        // Setting Quality of service policy
        rclcpp::QoS qos_pub_info(10);
        qos_pub_info.reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile)
            .history(rclcpp::HistoryPolicy::KeepLast);

        _depthPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/depth", qos_pub_info);
        _pressPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/press", qos_pub_info);
        _tempPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/temp", qos_pub_info);

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

    void DepthProvider::readSerialDevice()
    {
        char buffer[BUFFER_SIZE];
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        while (!_readStopThread)
        {
            if (_device->ReadDataCheck(buffer, BUFFER_SIZE) > 0)
            {
                _id1String.push_back((std::string)buffer);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }  // end while
    }  // end read

    void DepthProvider::sendId1Register()
    {
        while (!_sendStopThread)
        {
            std::string tmp = "";

            while (!_id1String.empty())
            {
                DepthData data = _device->ParseData(_id1String.get_n_pop_front());

                std_msgs::msg::Float32 publishData;

                publishData.data = data.depth;
                _depthPublisher->publish(publishData);

                publishData.data = data.temp;
                _tempPublisher->publish(publishData);

                publishData.data = data.press;
                _pressPublisher->publish(publishData);
            }
        }
    }

    void DepthProvider::tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        _device->Tare();
        response->success = true;
        response->message = "Depth Sensor tared";
        BOOST_LOG_TRIVIAL(info) << "Depth Sensor tare finished";
    }
}  // namespace depth_port_manager
