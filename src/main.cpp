#include "depth_port_manager/DepthProvider.h"
#include <stdlib.h>
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto depth= std::make_shared<depth_provider::DepthProvider>();
    
    if (!depth->OpenPort())
    {   
        std::cout << "Could not open port..." << std::endl;
        return EXIT_FAILURE;
    }
    rclcpp::spin(depth);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
