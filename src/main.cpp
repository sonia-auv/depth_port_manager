#include <stdlib.h>

#include "depth_port_manager/DepthProvider.hpp"
#include "depth_port_manager/ImpactSubsea.hpp"
#include "depth_port_manager/MS5837.hpp"
#include "sonia_common_cpp/I2CConn.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const char *env_var = std::getenv("AUV");
    if (env_var == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "AUV ENV Variable not set!!!");
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
    std::cout << "Got Env" << std::endl;
    std::shared_ptr<sonia_common_cpp::IConnection> conn;
    std::cout << "Create Empty vars" << std::endl;
    if (strcmp(env_var, "LITE1") == 0)
    {
        conn = std::make_shared<sonia_common_cpp::I2CConn>("/dev/i2c-8", 0x76);
        std::cout << "Created I2CConn" << std::endl;
    }
    else
    {
        conn = std::make_shared<sonia_common_cpp::SerialConn>("/dev/DEPTH", B115200, true);
    }

    if (!conn->OpenPort())
    {
        RCLCPP_FATAL(rclcpp::get_logger("depth_port_manager"), "Could not open port...");
        return EXIT_FAILURE;
    }
    
    std::shared_ptr<depth_port_manager::IDepthDevice> device;
    if (strcmp(env_var, "LITE1") == 0)
    {
        device = std::make_shared<depth_port_manager::MS5837>(conn);
        std::cout << "Create Filled Vars" << std::endl;
    }
    else
    {
        device = std::make_shared<depth_port_manager::ImpactSubsea>(conn);
    }
    // Device and connection defined by env var.


    auto depth = std::make_shared<depth_port_manager::DepthProvider>(device);

    rclcpp::spin(depth);

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
