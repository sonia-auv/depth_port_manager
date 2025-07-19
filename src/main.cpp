#include <stdlib.h>

#include "depth_port_manager/DepthProvider.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto depth = std::make_shared<depth_port_manager::DepthProvider>();

    if (!depth->OpenPort())
    {
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }
    rclcpp::spin(depth);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
