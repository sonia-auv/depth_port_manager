#pragma once

#include <stdio.h>

#include <cstdint>
#include <functional>
#include <string>

namespace depth_port_manager
{

    struct DepthData {
        float depth;
        float temp;
        float press;
        bool operator==(const DepthData& other) const {
            return depth == other.depth && temp == other.temp && press == other.press;
        }
    };

    class IDepthDevice
    {
        public:
        virtual int ReadDataCheck(std::function<ssize_t(uint8_t *, int)> readFunc, char buffer[], int bufferSize) = 0;
        virtual DepthData ParseData(std::string data) = 0;
        virtual void Tare(std::function<ssize_t(std::string)> writeFunc) = 0;
    };
}  // namespace depth_port_manager
