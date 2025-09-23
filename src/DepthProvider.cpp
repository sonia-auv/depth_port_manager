// #include <sstream>
#include "depth_port_manager/DepthProvider.hpp"

#include <boost/log/trivial.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace depth_port_manager
{
    DepthProvider::DepthProvider(IDepthDevice& device, sonia_common_cpp::SerialConn& conn)
        : Node("depth_provider"), _device(device), _connection(conn)
    {
        //Setting Quality of service policy
        rclcpp::QoS qos_pub_info(10);
        qos_pub_info.reliability(rclcpp::ReliabilityPolicy::Reliable);

        _depthPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/depth", qos_pub_info);
        _pressPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/press", qos_pub_info);
        _tempPublisher = this->create_publisher<std_msgs::msg::Float32>("/provider_depth/temp", qos_pub_info);

        _readThread = std::thread(std::bind(&DepthProvider::readSerialDevice, this));
        _sendThread = std::thread(std::bind(&DepthProvider::sendId1Register, this));

        _tareSrv = this->create_service<std_srvs::srv::Trigger>("/provider_depth/tare", std::bind(&DepthProvider::tare, this, _1, _2));
    }

    DepthProvider::~DepthProvider()
    {
        _sendStopThread = true;
        _readStopThread = true;
        _mtxParser.unlock();
        _cvReaderParser.notify_all();
    }

    void DepthProvider::readSerialDevice()
    {
        char buffer[BUFFER_SIZE];
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        while (!_readStopThread)
        {
            if (_device.ReadDataCheck(
                    [&](uint8_t* pData, int offset) -> ssize_t { return _connection.ReadOnce(pData, offset); }, buffer,
                    BUFFER_SIZE) > 0)
            {
                _id1String.push_back((std::string)buffer);
                _cvReaderParser.notify_all();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }  // end while
    }// end read

    void DepthProvider::sendId1Register()
    {
        std::unique_lock<std::mutex> _lockParser(_mtxParser);
        while (!_sendStopThread)
        {
            _cvReaderParser.wait(_lockParser, [&] { return !_id1String.empty(); });
            DepthData data = _device.ParseData(_id1String.get_n_pop_front());

            std_msgs::msg::Float32 publishData;

            publishData.data = data.depth;
            _depthPublisher->publish(publishData);

            publishData.data = data.temp;
            _tempPublisher->publish(publishData);

            publishData.data = data.press;
            _pressPublisher->publish(publishData);
        }
    }

    void DepthProvider::tare(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        _device.Tare([&](std::string data) -> ssize_t { return _connection.Transmit(data); });
        response->success = true;
        response->message = "Depth Sensor tared";
        BOOST_LOG_TRIVIAL(info) << "Depth Sensor tare completed";
    }
}  // namespace depth_port_manager
