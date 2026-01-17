#pragma once

#include <stdio.h>

#include <sonia_common_cpp/SerialConn.hpp>
#include <sonia_common_cpp/SharedQueue.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "depth_port_manager/IDepthDevice.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depth_port_manager
{
    class DepthProvider : public rclcpp::Node
    {
        public:
        DepthProvider(std::shared_ptr<IDepthDevice> device);
        ~DepthProvider();

        private:
        void readSerialDevice();
        void sendId1Register();
        void tare(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        std::shared_ptr<IDepthDevice> _device;

        sonia_common_cpp::SharedQueue<std::string> _id1String;

        bool _readStopThread = false;
        std::thread _readThread;

        bool _sendStopThread = false;
        std::thread _sendThread;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _depthPublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pressPublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _tempPublisher;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _tareSrv;
        static const int BUFFER_SIZE = 4096;
        static const int ID_SIZE = 5;


        std::mutex _mtxParser;
        std::condition_variable _cvReaderParser;
        sonia_common_cpp::SharedQueue<uint8_t> _parseQueue;
    };
}  // namespace depth_port_manager