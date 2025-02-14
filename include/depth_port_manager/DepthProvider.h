#pragma once

#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sonia_common_cpp/SerialConn.hpp>

#include <stdio.h>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "SharedQueue.h"

#define ID1 "ISDPT"

namespace depth_provider
{
    class DepthProvider:public rclcpp::Node
    {
        public:
            DepthProvider();
            ~DepthProvider();
            bool OpenPort();
        private:            
            bool _read_stop_thread=false;
            std::thread read_thread;

            bool _send_stop_thread=false;
            std::thread send_thread;

            std::mutex id1_mutex;
            std::string id1_string = "";
            std::condition_variable id1_cond;

            void readSerialDevice();
            void sendId1Register();

            static const int BUFFER_SIZE= 4096;

            sonia_common_cpp::SerialConn _serialConnection;

            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr depthPublisher_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressPublisher_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tempPublisher_;

            std_msgs::msg::Float32 depth_;
            std_msgs::msg::Float32 press_;
            std_msgs::msg::Float32 temp_;

            bool tare(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
            rclcpp::Service<std_srvs::srv::Empty>::SharedPtr tare_srv;
    };
}