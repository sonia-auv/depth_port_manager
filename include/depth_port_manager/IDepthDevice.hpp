#pragma once

#include <stdio.h>

#include <cstdint>
#include <functional>
#include <string>
#include <memory>

#include "sonia_common_cpp/IConnection.hpp"

namespace depth_port_manager
{

    struct DepthData
    {
        float depth;
        float temp;
        float press;
        bool operator==(const DepthData& other) const
        {
            return depth == other.depth && temp == other.temp && press == other.press;
        }
    };

    class IDepthDevice
    {
        public:
        inline IDepthDevice(std::shared_ptr<sonia_common_cpp::IConnection> connection) : _conn(connection) {};
        virtual int ReadDataCheck(char buffer[], int bufferSize) = 0;
        virtual DepthData ParseData(std::string data) = 0;
        virtual void Tare() = 0;

        protected:
        std::shared_ptr<sonia_common_cpp::IConnection> _conn;
    };
}  // namespace depth_port_manager
