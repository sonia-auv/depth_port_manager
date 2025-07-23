#include <stdlib.h>

#include "depth_port_manager/DepthProvider.hpp"
#include "depth_port_manager/ImpactSubsea.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // Device and connection defined by env var.
    sonia_common_cpp::SerialConn conn("/dev/DEPTH", B115200, true);

    if (!conn.OpenPort())
    {
        RCLCPP_FATAL(rclcpp::get_logger("depth_port_manager"), "Could not open port...");
        return EXIT_FAILURE;
    }

    depth_port_manager::ImpactSubsea device;
    auto depth = std::make_shared<depth_port_manager::DepthProvider>(device, conn);

    rclcpp::spin(depth);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
