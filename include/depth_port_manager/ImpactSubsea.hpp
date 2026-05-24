#pragma once
#include <string>
#include <memory>

#include "depth_port_manager/IDepthDevice.hpp"

#include "sonia_common_cpp/SerialConn.hpp"

namespace depth_port_manager
{
    class ImpactSubsea : public IDepthDevice
    {
        public:
        explicit ImpactSubsea(std::shared_ptr<sonia_common_cpp::IConnection> connection);
        ~ImpactSubsea() = default;
        int ReadDataCheck(char buffer[], int bufferSize) override;
        DepthData ParseData(std::string data) override;
        void Tare() override;
        static const int ID_SIZE = 5;
        static const std::string ID1;
    };
}  // namespace depth_port_manager
