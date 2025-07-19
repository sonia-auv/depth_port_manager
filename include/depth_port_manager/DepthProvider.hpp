#pragma once

#include <stdio.h>

#include <sonia_common_cpp/SerialConn.hpp>
#include <sonia_common_cpp/SharedQueue.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#define ID1 "ISDPT"

namespace depth_port_manager
{
    class DepthProvider : public rclcpp::Node
    {
        public:
        DepthProvider();
        ~DepthProvider();
        bool OpenPort();

        private:
        void readSerialDevice();
        void sendId1Register();
        void tare(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        sonia_common_cpp::SharedQueue<std::string> _id1String;

        bool _readStopThread = false;
        std::thread _readThread;

        bool _sendStopThread = false;
        std::thread _sendThread;

        
        sonia_common_cpp::SerialConn _serialConnection;
        
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _depthPublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pressPublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _tempPublisher;
        
        std_msgs::msg::Float32 _depth;
        std_msgs::msg::Float32 _press;
        std_msgs::msg::Float32 _temp;
        
        
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _tareSrv;
        static const int BUFFER_SIZE = 4096;
        static const int ID_SIZE = 5;
    };
}  // namespace depth_port_manager