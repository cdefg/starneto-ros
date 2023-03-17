#include "starneto_node.hpp"
#include <iostream>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StarnetoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}