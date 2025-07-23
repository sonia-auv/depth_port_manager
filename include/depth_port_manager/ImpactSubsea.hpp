#pragma once
#include <string>

#include "depth_port_manager/IDepthDevice.hpp"
namespace depth_port_manager
{
    class ImpactSubsea : public IDepthDevice
    {
        public:
        ImpactSubsea() = default;
        ~ImpactSubsea() = default;
        int ReadDataCheck(std::function<ssize_t(uint8_t *, int)> readFunc, char buffer[], int bufferSize) override;
        DepthData ParseData(std::string data) override;
        void Tare(std::function<ssize_t(std::string)> writeFunc) override;
        static const int ID_SIZE = 5;
        static const std::string ID1;
    };
}  // namespace depth_port_manager
