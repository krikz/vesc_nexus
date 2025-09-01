#include "vesc_nexus/vesc_nexus_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VescNexusNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
