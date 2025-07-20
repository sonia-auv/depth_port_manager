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
    };

    class IDepthDevice
    {
        public:
        virtual bool ReadDataCheck(std::function<ssize_t(uint8_t *, int)> readFunc, char buffer[], int bufferSize) = 0;
        virtual DepthData ParseData(std::string data) = 0;
        virtual void Tare(std::function<ssize_t(std::string)> writeFunc) = 0;
    };
}  // namespace depth_port_manager
